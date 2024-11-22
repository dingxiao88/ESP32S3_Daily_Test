/* Copyright 2016, 2018 David Conran
*  Copyright 2020 Sadid Rafsun Tulon
*
* An IR LED circuit *MUST* be connected to the ESP8266 on a pin
* as specified by kIrLed below.
*
* TL;DR: The IR LED needs to be driven by a transistor for a good result.
*
* Suggested circuit:
*     https://github.com/crankyoldgit/IRremoteESP8266/wiki#ir-sending
*
* Common mistakes & tips:
*   * Don't just connect the IR LED directly to the pin, it won't
*     have enough current to drive the IR LED effectively.
*   * Make sure you have the IR LED polarity correct.
*     See: https://learn.sparkfun.com/tutorials/polarity/diode-and-led-polarity
*   * Typical digital camera/phones can be used to see if the IR LED is flashed.
*     Replace the IR LED with a normal LED if you don't have a digital camera
*     when debugging.
*   * Avoid using the following pins unless you really know what you are doing:
*     * Pin 0/D3: Can interfere with the boot/program mode & support circuits.
*     * Pin 1/TX/TXD0: Any serial transmissions from the ESP8266 will interfere.
*     * Pin 3/RX/RXD0: Any serial transmissions to the ESP8266 will interfere.
*   * ESP-01 modules are tricky. We suggest you use a module with more GPIOs
*     for your first time. e.g. ESP-12 etc.
*/

#define DX_GREE   0
#define DX_COOLIX 1


#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

#if DX_GREE
#define SEND_GREE 0
#include <ir_Gree.h>
#elif DX_COOLIX
#define SEND_COOLIX 1
#include <ir_Coolix.h>
#endif


const uint16_t kIrLed = 8;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).  

#if DX_GREE
IRGreeAC ac(kIrLed);  // Set the GPIO to be used for sending messages.  //@-格力空调
#elif DX_COOLIX
IRCoolixAC ac(kIrLed);  // Set the GPIO to be used for sending messages.   //@-美的空调
#endif

unsigned char boot_key_lock = 0;

//@-美的空调
// Protocol  : COOLIX
// 11:17:59.243 -> Code      : 0xB2BFD0 (24 Bits)
// 11:17:59.243 -> Mesg Desc.: Power: On, Mode: 0 (Cool), Fan: 5 (Auto), Temp: 26C, Zone Follow: Off, Sensor Temp: Off
// 11:17:59.243 -> uint16_t rawData[199] = {4562, 4428,  672, 1568,  668, 438,  676, 1572,  670, 1570,  670, 442,  672, 444,  668, 1572,  668, 446,  670, 442,  668, 1574,  670, 438,  674, 440,  652, 1594,  644, 1596,  668, 440,  676, 1568,  672, 1570,  670, 438,  674, 1572,  670, 1568,  670, 1570,  670, 1570,  668, 1572,  670, 1570,  668, 444,  668, 1572,  668, 440,  672, 442,  674, 444,  648, 466,  670, 436,  654, 466,  672, 1568,  668, 1574,  668, 440,  674, 1570,  670, 446,  644, 468,  672, 434,  678, 438,  676, 440,  676, 432,  656, 1594,  668, 438,  674, 1572,  644, 1594,  670, 1570,  668, 1572,  668, 5162,  4566, 4424,  674, 1566,  674, 440,  672, 1568,  670, 1570,  670, 444,  674, 442,  670, 1568,  670, 444,  670, 444,  672, 1568,  670, 446,  668, 446,  670, 1570,  670, 1570,  670, 444,  666, 1572,  668, 1572,  668, 446,  668, 1572,  644, 1596,  668, 1572,  664, 1574,  642, 1598,  664, 1576,  642, 472,  642, 1598,  640, 474,  642, 472,  638, 478,  638, 476,  638, 474,  642, 474,  638, 1602,  638, 1600,  638, 478,  636, 1602,  638, 476,  636, 478,  636, 478,  636, 480,  634, 502,  614, 502,  610, 1628,  612, 502,  612, 1626,  612, 1628,  612, 1628,  612, 1630,  610};  // COOLIX B2BFD0
// 11:17:59.366 -> uint64_t data = 0xB2BFD0;

// 11:17:44.963 -> Protocol  : COOLIX
// 11:17:44.963 -> Code      : 0xB27BE0 (24 Bits)
// 11:17:44.963 -> Mesg Desc.: Power: Off
// 11:17:44.963 -> uint16_t rawData[199] = {4564, 4428,  670, 1570,  670, 394,  716, 1570,  670, 1570,  668, 442,  674, 444,  670, 1572,  670, 444,  670, 436,  674, 1572,  672, 438,  676, 434,  678, 1570,  670, 1570,  666, 444,  670, 1572,  670, 444,  670, 1570,  670, 1570,  668, 1570,  672, 1566,  672, 442,  646, 1594,  670, 1566,  670, 1570,  670, 444,  670, 444,  670, 438,  674, 440,  674, 1570,  666, 440,  676, 438,  676, 1568,  668, 1570,  668, 1572,  668, 438,  676, 438,  676, 446,  670, 436,  676, 446,  670, 440,  674, 446,  670, 436,  678, 1568,  668, 1570,  668, 1572,  668, 1570,  670, 1570,  670, 5160,  4566, 4428,  670, 1570,  670, 444,  672, 1568,  644, 1594,  672, 442,  670, 444,  674, 1564,  674, 440,  672, 442,  674, 1566,  672, 442,  672, 442,  674, 1568,  670, 1568,  672, 442,  672, 1566,  672, 442,  670, 1572,  668, 1570,  670, 1570,  668, 1572,  670, 446,  668, 1570,  668, 1572,  666, 1574,  668, 446,  666, 448,  644, 470,  644, 470,  642, 1598,  664, 448,  644, 472,  642, 1598,  638, 1598,  642, 1600,  640, 476,  638, 476,  638, 476,  638, 476,  640, 476,  638, 476,  638, 476,  638, 478,  636, 1604,  636, 1604,  634, 1604,  636, 1626,  612, 1626,  614};  // COOLIX B27BE0
// 11:17:45.080 -> uint64_t data = 0xB27BE0;

void printState() {
  // Display the settings.
  Serial.println("GREE A/C remote is in the following state:");
  Serial.printf("  %s\n", ac.toString().c_str());
  // Display the encoded IR sequence.

  #if DX_GREE
  unsigned char* ir_code = ac.getRaw();  //@-格力
  #elif DX_COOLIX
  uint32_t ir_code = ac.getRaw();        //@-美的
  #endif
  
  Serial.print("IR Code: 0x");
  #if DX_GREE
  for (uint8_t i = 0; i < kGreeStateLength; i++)   //@-格力
    Serial.printf("%02X", ir_code[i]);
  #elif DX_COOLIX
  Serial.printf("0x%X", ir_code);  //@-美的
  #endif  
  Serial.println();
}

void setup() {

  ac.begin();
  Serial.begin(115200);
  delay(200);

  //@-BOOT案件
  pinMode(0, INPUT);

  // Set up what we want to send. See ir_Gree.cpp for all the options.
  // Most things default to off.
  Serial.println("Default state of the remote.");
  printState();
  Serial.println("Setting desired state for A/C.");
  ac.on();
  // ac.setFan(3);
  ac.setFan(1);
  // kGreeAuto, kGreeDry, kGreeCool, kGreeFan, kGreeHeat
  // ac.setMode(kGreeCool);
  ac.setMode(kCoolixHeat);
  ac.setTemp(28);  // 16-30C
  // ac.setSwingVertical(true, kGreeSwingAuto);
  // ac.setXFan(false);
  // ac.setLight(false);
  // ac.setSleep(false);
  // ac.setTurbo(false);
}

void loop() {


    bool val = digitalRead(0);
    if ((val == LOW) && (boot_key_lock == 0))
    {
      boot_key_lock = 1;
      // Serial.println("boot key pressed");

      Serial.println("Sending IR command to A/C ...");
      ac.send();
      printState();
    }
    else if(val == HIGH)
    {
      boot_key_lock = 0;
    }
    

//   // Now send the IR signal.
// // #if SEND_GREE
//   Serial.println("Sending IR command to A/C ...");
//   ac.send();
// // #endif  // SEND_GREE
//   printState();
//   delay(5000);
}
