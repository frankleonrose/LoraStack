![Build Status](https://travis-ci.org/frankleonrose/LoraStack.svg?branch=master)

LoraStack is a high level library that uses the [MCCI LMiC LoRaWAN](https://github.com/frankleonrose/arduino-lorawan#node-projects) implementation
combined with a flexible [ParameterStore](https://github.com/frankleonrose/ParameterStore) to present a simple API to `provision`, `join`, and
`sendBytes` on [The Things Network](https://thethingsnetwork.org).

## Basic Use

```c++
#include <LoraStack.h>

#include <ParameterStore.h>
#include <RamStore.h>

#include <Timer.h>

// Lorawan Device ID, App ID, and App Key
const char *devEui = "0001020304050607";
const char *appEui = "0001020304050607";
const char *appKey = "000102030405060708090A0B0C0D0E0F";

#define MY_FREQUENCY_PLAN  // One of TTN_FP_EU868, TTN_FP_US915, TTN_FP_AS920_923, TTN_FP_AS923_925, TTN_FP_KR920_923

// Using RamStore for quick start. In a real device you
// would have EEPROM or FRAM or some other non-volatile
// store supported by ParameterStore. Or serialize RamStore
// to a file on an SD card.
const int STORE_SIZE = 2000;
RamStore<STORE_SIZE> byteStore;
ParameterStore gParameters(byteStore);

// Create lower level hardware interface with access to parameters
const Arduino_LoRaWAN::lmic_pinmap define_lmic_pins = {
  // Feather LoRa wiring with IO1 wired to D6
  .nss = 8, // Chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3, 6, LMIC_UNUSED_PIN},
};
LoraStack_LoRaWAN lorawan(define_lmic_pins, gParameters);
// Create high level interface to TTN
LoraStack ttn(lorawan, gParameters, MY_FREQUENCY_PLAN);

Timer gTimer;

void setup() {
  bool status = ttn.join(appEui, devEui, appKey);
  if (!status) {
      Log.Error(F("Failed to provision device!" CR));
      while (1);
  }

  // Setup timer to kick off send every 10 seconds
  gTimer.every(10 * 1000, [](){
    // Prepare payload of 1 byte to indicate LED status
    byte payload[1];
    payload[0] = (digitalRead(LED_BUILTIN) == HIGH) ? 1 : 0;

    // Send it off
    ttn.sendBytes(payload, sizeof(payload));
  });
}

void loop() {
  ttn.loop(); // Call often to do LoRaWAN work

  gTimer.update();

  delay(10); // Not too long!
}
```

## API

The LoraStack interface is modeled after the TheThingsNetwork interface defined in [TheThingsNetwork.h](https://github.com/TheThingsNetwork/arduino-device-lib/blob/master/src/TheThingsNetwork.h).

A critical difference is that the The Things Network library relies on the [Microchip RN2xx3 module](http://www.microchip.com/design-centers/wireless-connectivity/embedded-wireless/lora-technology).
This chip dramatically reduces the amount of code required to initialize the LoRaWAN system, because all
your code does is communicate with the module over a serial line. The module provides storage for
network layer parameters like Frame count. Also, the module
performs LoRaWAN tasks in parallel with your microprocessor which means that a sketch can
call `delay(10000)` after a `sendBytes` with impunity - LoRaWAN transmission and reception will continue
on the other chip.

In contrast, LoraStack is based on LMiC which runs the LoRaWAN code on the same processor. Therefore,
long `delay` calls are not allowed because `ttn.loop()` must be called often! In addition, LoraStack
requires specifying what your device offers as a storage medium and what pins are used for sending chip select and
reset and receiving interrupts from the Semtech radio.

> Note: Frankly, `delay()` calls are
a waste of either time or power and you should learn to structure your code such that they are unnecessary.
For periodic actions you can use something like the [Timer library](https://github.com/JChristensen/Timer)
used above. To manage more complex state and periodic actions, and still breathe easy, consider using the [Respire library](https://github.com/frankleonrose/Respire-Arduino).

## Tests

When you submit pull requests, please include tests.

Copyright (c) 2018 Frank Leon Rose