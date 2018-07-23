#include <LoraStack.h>

#include <ParameterStore.h>
#include <RamStore.h>

#include <Timer.h>

// Lorawan Device ID, App ID, and App Key
const char *devEui = "0001020304050607";
const char *appEui = "0001020304050607";
const char *appKey = "000102030405060708090A0B0C0D0E0F";

#define MY_FREQUENCY_PLAN TTN_FP_US915 // One of TTN_FP_EU868, TTN_FP_US915, TTN_FP_AS920_923, TTN_FP_AS923_925, TTN_FP_KR920_923

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
  .rst = 4, // Reset
  .dio = {3, 6, LMIC_UNUSED_PIN}, // SX1276 pins DIO0, DIO1, DIO2
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
