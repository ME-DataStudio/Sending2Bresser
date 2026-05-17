///////////////////////////////////////////////////////////////////////////////////////////////////
// Sending2Bresser.ino
//
// Example for BresserWeatherSensorReceiver - 
// Using getMessage() for non-blocking reception of a single data message.
//
// The data may be incomplete, because certain sensors need two messages to
// transmit a complete data set.
// Which sensor data is received in case of multiple sensors are in range
// depends on the timing of transmitter and receiver.  
//
// https://github.com/matthias-bs/BresserWeatherSensorReceiver
//
//
// created: 05/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20260517 Created from https://github.com/matthias-bs/BresserWeatherSensorReceiver
// https://github.com/matthias-bs/Bresser5in1-CC1101
//
// ToDo: 
// - 
//
///////////////////////////////////////////////////////////////////////////////////////////////////

/*
  RadioLib CC1101 Transmit to Address Example

  This example transmits packets using CC1101 FSK radio
  module. Packets can have 1-byte address of the
  destination node. After setting node address, this node
  will automatically filter out any packets that do not
  contain either node address or broadcast addresses.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#cc1101

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include <SPI.h>

//RP2040 pins SPI0
byte sck = 18;   
byte miso = 16;
byte mosi = 19;
byte ss = 17;
int gdo0 = 20;
int gdo2 = 21;

// CC1101 has the following connections:
// CS pin:    10
// GDO0 pin:  2
// RST pin:   unused
// GDO2 pin:  3 (optional)
CC1101 radio = new Module(ss, gdo0, RADIOLIB_NC, gdo2);

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

void setup() {
  Serial.begin(115200);
  SPI.setCS(ss);
  SPI.setMISO(miso);
  SPI.setMOSI(mosi);
  SPI.setSCK(sck);
  //SPI.begin(sck, miso, mosi, ss);
  Serial.printf("SPI.begin(SCK=%d, MISO=%d, MOSI=%d, CS=%d)\n", sck, miso, mosi, ss);

  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set node address
  // NOTE: Calling this method will automatically enable
  //       address filtering. CC1101 also allows to set
  //       number of broadcast address (0/1/2).
  //       The following sets one broadcast address 0x00.
  //       When setting two broadcast addresses, 0x00 and
  //       0xFF will be used.
  Serial.print(F("[CC1101] Setting node address ... "));
  state = radio.setNodeAddress(0x01, 1);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // address filtering can also be disabled
  // NOTE: Calling this method will also erase previously
  //       set node address
  /*
    Serial.print(F("[CC1101] Disabling address filtering ... "));
    state == radio.disableAddressFiltering();
    if(state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); }
    }
  */
}

void loop() {
  Serial.print(F("[CC1101] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to 255 characters long
  int state = radio.transmit("Hello World!");

  // you can also transmit byte array up to 255 bytes long
  // With some limitations see here: https://github.com/jgromes/RadioLib/discussions/1138
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 255 bytes
    Serial.println(F("too long!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }

  // wait for a second before transmitting again
  delay(1000);
}











