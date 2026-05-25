#include <stdio.h>
#include <cstdint>
#include <ostream>

/*
    Decoder for Bresser Weather Center 6-in-1.

    Copyright (C) 2019 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#define SENSOR_TYPE_WEATHER0        0 // Weather Station
#define SENSOR_TYPE_WEATHER1        1 // Weather Station
#define SENSOR_TYPE_THERMO_HYGRO    2 // Thermo-/Hygro-Sensor
#define SENSOR_TYPE_POOL_THERMO     3 // Pool / Spa Thermometer
#define SENSOR_TYPE_SOIL            4 // Soil Temperature and Moisture (from 6-in-1 decoder)
#define SENSOR_TYPE_LEAKAGE         5 // Water Leakage
#define SENSOR_TYPE_AIR_PM          8 // Air Quality Sensor (Particle Matter)
#define SENSOR_TYPE_RAIN            9 // Professional Rain Gauge (from 5-in-1 decoder)
#define SENSOR_TYPE_LIGHTNING       9 // Lightning Sensor
#define SENSOR_TYPE_CO2             10 // CO2 Sensor
#define SENSOR_TYPE_HCHO_VOC        11 // Air Quality Sensor (HCHO and VOC)
#define SENSOR_TYPE_WEATHER3        12 // Weather Station (3-in-1)
#define SENSOR_TYPE_WEATHER8        13 // Weather Station (8-in-1)


// Sensor specific rain gauge overflow threshold (mm)
#define WEATHER0_RAIN_OV          1000
#define WEATHER1_RAIN_OV        100000


// Flags for controlling completion of reception in getData()
#define DATA_COMPLETE           0x1     // only completed slots (as opposed to partially filled)
#define DATA_TYPE               0x2     // at least one slot with specific sensor type
#define DATA_ALL_SLOTS          0x8     // all slots completed

// Flags for checking enabled decoders
#define DECODER_6IN1            0x01

// Message buffer size
#define MSG_BUF_SIZE            27

struct Weather {
    bool     temp_ok = false;         //!< temperature o.k. (only 6-in-1)
    bool     tglobe_ok = false;       //!< globe temperature o.k. (only 8-in-1)
    bool     humidity_ok = false;     //!< humidity o.k.
    bool     light_ok = false;        //!< light o.k. (only 7-in-1)
    bool     uv_ok = false;           //!< uv radiation o.k. (only 6-in-1)
    bool     wind_ok = false;         //!< wind speed/direction o.k. (only 6-in-1)
    bool     rain_ok = false;         //!< rain gauge level o.k.
    float    temp_c = 0.0;            //!< temperature in degC
    float    tglobe_c = 0.0;          //!< globe temperature in degC (only 8-in-1)
    float    light_klx = 0.0;         //!< Light KLux (only 7-in-1)
    float    light_lux = 0.0;         //!< Light lux (only 7-in-1)
    float    uv = 0.0;                //!< uv radiation (only 6-in-1 & 7-in-1)
    float    rain_mm = 0.0;           //!< rain gauge level in mm
    #ifdef WIND_DATA_FLOATINGPOINT   
    float    wind_direction_deg = 0.0;  //!< wind direction in deg
    float    wind_gust_meter_sec = 0.0; //!< wind speed (gusts) in m/s
    float    wind_avg_meter_sec = 0.0;  //!< wind speed (avg)   in m/s
    #endif
    #ifdef WIND_DATA_FIXEDPOINT
    // For LoRa_Serialization:
    //   fixed point integer with 1 decimal -
    //   saves two bytes compared to "RawFloat"
    uint16_t wind_direction_deg_fp1 = 0;  //!< wind direction in deg (fixed point int w. 1 decimal)
    uint16_t wind_gust_meter_sec_fp1 = 0; //!< wind speed (gusts) in m/s (fixed point int w. 1 decimal)
    uint16_t wind_avg_meter_sec_fp1 = 0;  //!< wind speed (avg)   in m/s (fixed point int w. 1 decimal)
    #endif
    uint8_t  humidity = 0;                //!< humidity in %
};

struct AirPM {
    uint16_t pm_1_0;                //!< air quality PM1.0 in µg/m³
    uint16_t pm_2_5;                //!< air quality PM2.5 in µg/m³
    uint16_t pm_10;                 //!< air quality PM10  in µg/m³
    bool     pm_1_0_init;           //!< measurement value invalid due to initialization
    bool     pm_2_5_init;           //!< measurement value invalid due to initialization
    bool     pm_10_init;            //!< measurement value invalid due to initialization
};


struct Sensor {
    uint32_t sensor_id;        //!< sensor ID (5-in-1: 1 byte / 6-in-1: 4 bytes / 7-in-1: 2 bytes)
    float    rssi;             //!< received signal strength indicator in dBm
    uint8_t  s_type;           //!< sensor type
    uint8_t  chan;             //!< channel
    uint8_t  decoder;          //!< decoder used
    bool     startup = false;  //!< startup after reset / battery change
    bool     battery_ok;       //!< battery o.k.
    bool     valid;            //!< data valid (but not necessarily complete)
    bool     complete;         //!< data is split into two separate messages is complete (only 6-in-1 WS)
    union {
        struct Weather      w;
        struct AirPM        pm;
    };

    Sensor ()
    {
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wclass-memaccess"
        memset(this, 0, sizeof(*this));
        #pragma GCC diagnostic pop
    };
};

typedef struct Sensor sensor_t;            //!< Shortcut for struct Sensor
std::vector<sensor_t> sensor;              //!< sensor data array
float   rssi = 0.0;                        //!< received signal strength indicator in dBm
uint8_t rxFlags;                           //!< receive flags (see getData())
uint8_t enDecoders = 0xFF;                 //!< enabled Decoders                     

// Radio message decoding status
typedef enum DecodeStatus {
    DECODE_INVALID, DECODE_OK, DECODE_PAR_ERR, DECODE_CHK_ERR, DECODE_DIG_ERR, DECODE_SKIP, DECODE_FULL
} DecodeStatus;

/*****************************************
 * FUNCTIONS
 *****************************************/
DecodeStatus decodeBresser6In1Payload(const uint8_t *msg, uint8_t msgSize)
{
    (void)msgSize;                                                                             // unused parameter - kept for consistency with other decoders; avoid warning
    int const moisture_map[] = {0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99}; // scale is 20/3

    // Per-message status flags
    bool temp_ok = false;
    bool humidity_ok = false;
    bool uv_ok = false;
    bool wind_ok = false;
    bool rain_ok = false;

    // LFSR-16 digest, generator 0x8810 init 0x5412
    int chkdgst = (msg[0] << 8) | msg[1];
    int digest = lfsr_digest16(&msg[2], 15, 0x8810, 0x5412);
    if (chkdgst != digest)
    {
        std::print("Digest check failed - [%02X] != [%02X]", chkdgst, digest);
        return DECODE_DIG_ERR;
    }
    // Checksum, add with carry
    int sum = add_bytes(&msg[2], 16); // msg[2] to msg[17]
    if ((sum & 0xff) != 0xff)
    {
        std::print("Checksum failed");
        return DECODE_CHK_ERR;
    }

    uint32_t id_tmp = ((uint32_t)msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | (msg[5]);
    uint8_t type_tmp = (msg[6] >> 4); // 1: weather station, 2: indoor?, 4: soil probe
    uint8_t chan_tmp = (msg[6] & 0x7);
    uint8_t flags = (msg[16] & 0x0f);
    DecodeStatus status;

    // Find appropriate slot in sensor data array and update <status>
    int slot = 0;

    if (status != DECODE_OK)
        return status;

    if (!sensor[slot].valid)
    {
        // Reset value after if slot is empty
        sensor[slot].w.temp_ok = false;
        sensor[slot].w.humidity_ok = false;
        sensor[slot].w.uv_ok = false;
        sensor[slot].w.wind_ok = false;
        sensor[slot].w.rain_ok = false;
    }
    sensor[slot].sensor_id = id_tmp;
    sensor[slot].s_type = type_tmp;
    sensor[slot].chan = chan_tmp;
    sensor[slot].decoder = DECODER_6IN1;
    sensor[slot].startup = ((msg[6] & 0x8) == 0) ? true : false; // s.a. #1214
    sensor[slot].battery_ok = (msg[13] >> 1) & 1;                // b[13] & 0x02 is battery_good, s.a. #1993

    // temperature, humidity(, uv) - shared with rain counter
    temp_ok = humidity_ok = (flags == 0);
    float temp = 0;
    if (temp_ok)
    {
        bool sign = (msg[13] >> 3) & 1;
        int temp_raw = (msg[12] >> 4) * 100 + (msg[12] & 0x0f) * 10 + (msg[13] >> 4);

        temp = ((sign) ? (temp_raw - 1000) : temp_raw) * 0.1f;

        // Correction for Bresser 3-in-1 Professional Wind Gauge / Anemometer, PN 7002531
        // The temperature range (as far as provided in other Bresser manuals) is -40...+60°C
        if (temp < -50.0)
        {
            temp = -temp_raw * 0.1f;
        }

        sensor[slot].w.temp_c = temp;
        sensor[slot].w.humidity = (msg[14] >> 4) * 10 + (msg[14] & 0x0f);

        // apparently ff01 or 0000 if not available, ???0 if valid, inverted BCD
        uv_ok = ((~msg[15] & 0xff) <= 0x99) && ((~msg[16] & 0xf0) <= 0x90);
        if (uv_ok)
        {
            int uv_raw = ((~msg[15] & 0xf0) >> 4) * 100 + (~msg[15] & 0x0f) * 10 + ((~msg[16] & 0xf0) >> 4);
            sensor[slot].w.uv = uv_raw * 0.1f;
        }
    }

    // int unk_ok  = (msg[16] & 0xf0) == 0xf0;
    // int unk_raw = ((msg[15] & 0xf0) >> 4) * 10 + (msg[15] & 0x0f);

    // invert 3 bytes wind speeds
    uint8_t _imsg7 = msg[7] ^ 0xff;
    uint8_t _imsg8 = msg[8] ^ 0xff;
    uint8_t _imsg9 = msg[9] ^ 0xff;

    wind_ok = (_imsg7 <= 0x99) && (_imsg8 <= 0x99) && (_imsg9 <= 0x99);
    if (wind_ok)
    {
        int gust_raw = (_imsg7 >> 4) * 100 + (_imsg7 & 0x0f) * 10 + (_imsg8 >> 4);
        int wavg_raw = (_imsg9 >> 4) * 100 + (_imsg9 & 0x0f) * 10 + (_imsg8 & 0x0f);
        int wind_dir_raw = ((msg[10] & 0xf0) >> 4) * 100 + (msg[10] & 0x0f) * 10 + ((msg[11] & 0xf0) >> 4);

#ifdef WIND_DATA_FLOATINGPOINT
        sensor[slot].w.wind_gust_meter_sec = gust_raw * 0.1f;
        sensor[slot].w.wind_avg_meter_sec = wavg_raw * 0.1f;
        sensor[slot].w.wind_direction_deg = wind_dir_raw * 1.0f;
#endif
#ifdef WIND_DATA_FIXEDPOINT
        sensor[slot].w.wind_gust_meter_sec_fp1 = gust_raw;
        sensor[slot].w.wind_avg_meter_sec_fp1 = wavg_raw;
        sensor[slot].w.wind_direction_deg_fp1 = wind_dir_raw * 10;
#endif
    }

    // rain counter, inverted 3 bytes BCD - shared with temp/hum
    uint8_t _imsg12 = msg[12] ^ 0xff;
    uint8_t _imsg13 = msg[13] ^ 0xff;
    uint8_t _imsg14 = msg[14] ^ 0xff;

    rain_ok = (flags == 1) && (type_tmp == 1);
    if (rain_ok)
    {
        int rain_raw = (_imsg12 >> 4) * 100000 + (_imsg12 & 0x0f) * 10000 + (_imsg13 >> 4) * 1000 + (_imsg13 & 0x0f) * 100 + (_imsg14 >> 4) * 10 + (_imsg14 & 0x0f);
        sensor[slot].w.rain_mm = rain_raw * 0.1f;
    }

    // Pool / Spa thermometer
    if (sensor[slot].s_type == SENSOR_TYPE_POOL_THERMO)
    {
        humidity_ok = false;
    }

    // The thermo hygro sensor and the soil moisture sensor might present valid readings but do not have the hardware
    if ((sensor[slot].s_type == SENSOR_TYPE_SOIL) || (sensor[slot].s_type == SENSOR_TYPE_THERMO_HYGRO))
    {
        wind_ok = 0;
        uv_ok = 0;
    }

    if (sensor[slot].s_type == SENSOR_TYPE_SOIL && temp_ok && sensor[slot].w.humidity >= 1 && sensor[slot].w.humidity <= 16)
    {
        humidity_ok = false;
        //sensor[slot].soil.moisture = moisture_map[sensor[slot].w.humidity - 1];
        //sensor[slot].soil.temp_c = temp;
    }

    // Update per-slot status flags
    sensor[slot].w.temp_ok |= temp_ok;
    sensor[slot].w.humidity_ok |= humidity_ok;
    sensor[slot].w.uv_ok |= uv_ok;
    sensor[slot].w.wind_ok |= wind_ok;
    sensor[slot].w.rain_ok |= rain_ok;
    std::print("Flags: Temp=%d  Hum=%d  Wind=%d  Rain=%d  UV=%d", temp_ok, humidity_ok, wind_ok, rain_ok, uv_ok);

    sensor[slot].valid = true;

    // Weather station data is split into two separate messages (except for Professional Wind Gauge)
    if (sensor[slot].s_type == SENSOR_TYPE_WEATHER1)
    {
        if (sensor[slot].w.temp_ok && sensor[slot].w.rain_ok)
        {
            sensor[slot].complete = true;
        }
    }
    else
    {
        sensor[slot].complete = true;
    }

    // Save rssi to sensor specific data set
    sensor[slot].rssi = rssi;

    const int i = slot;
    std::print("sensor[%d]: v=%d id=0x%08X t=%d c=%d", i, sensor[i].valid, (unsigned int)sensor[i].sensor_id, sensor[i].s_type, sensor[i].complete);
    return DECODE_OK;
}

uint16_t lfsr_digest16(uint8_t const message[], unsigned bytes, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k)
    {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i)
        {
            // fprintf(stderr, "key at bit %d : %04x\n", i, key);
            // if data bit is set then xor with key
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        result += message[i];
    }
    return result;
}


/*************************************
 * MAIN
***************************************/

int main() {
    printf("Hello, World!\n");
    return 0;
}   
