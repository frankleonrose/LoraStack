#include <Arduino.h>
#include <Stream.h>
// #include <avr/pgmspace.h>
#include "lmic.h"
#include <Arduino_LoRaWAN_ttn.h>

class ParameterStore;

#if defined(LS_LOGGING_OVERRIDE)
  // Developer has #defined their own LS_LOG_* elsewhere
  #if !defined(LS_LOG_INFO) || !defined(LS_LOG_ERROR) || !defined(LS_LOG_DEBUG)
    #error LS_LOGGING_OVERRIDE defined but missing definition of LS_LOG_INFO, LS_LOG_ERROR, or LS_LOG_DEBUG
  #endif
#elif defined(LOGGING_ARDUINO)
  #include "Logging.h"
  #define LS_LOG_INFO(...)   Log.Info(__VA_ARGS__)
  #define LS_LOG_ERROR(...)  Log.Error(__VA_ARGS__)
  #define LS_LOG_DEBUG(...)  Log.Debug(__VA_ARGS__)
#elif defined(LOGGING_SERIAL)
  #define LS_LOG_INFO(...)   Serial.printf(__VA_ARGS__)
  #define LS_LOG_ERROR(...)  Serial.printf(__VA_ARGS__)
  #define LS_LOG_DEBUG(...)  Serial.printf(__VA_ARGS__)
#elif defined(LOGGING_PRINTF)
  #define LS_LOG_INFO(...)   printf(__VA_ARGS__)
  #define LS_LOG_ERROR(...)  printf(__VA_ARGS__)
  #define LS_LOG_DEBUG(...)  printf(__VA_ARGS__)
#elif defined(LOGGING_DISABLED)
  // Silently compile logging to no-ops
  #define LS_LOG_INFO(...)
  #define LS_LOG_ERROR(...)
  #define LS_LOG_DEBUG(...)
#else
  #warning No logging option specified: LOGGING_ARDUINO, LOGGING_SERIAL, LOGGING_PRINTF, LOGGING_DISABLED, or LS_LOGGING_OVERRIDE
  #define LS_LOG_INFO(...)
  #define LS_LOG_ERROR(...)
  #define LS_LOG_DEBUG(...)
#endif

#define TTN_DEFAULT_SF 7
#define TTN_DEFAULT_FSB 2
#define TTN_RETX "7"

#define TTN_PWRIDX_EU868 "1"
#define TTN_PWRIDX_US915 "5"
#define TTN_PWRIDX_AS920_923 "1" // TODO: should be 0, but the current RN2903AS firmware doesn't accept that value (probably still using EU868: 1=14dBm)
#define TTN_PWRIDX_AS923_925 "1" // TODO: should be 0
#define TTN_PWRIDX_KR920_923 "1" // TODO: should be 0

#define TTN_BUFFER_SIZE 300

typedef uint8_t port_t;

enum ttn_response_t
{
  TTN_ERROR_SEND_COMMAND_FAILED = (-1),
  TTN_ERROR_UNEXPECTED_RESPONSE = (-10),
  TTN_SUCCESSFUL_TRANSMISSION = 1,
  TTN_SUCCESSFUL_RECEIVE = 2
};

enum ttn_fp_t
{
  TTN_FP_EU868,
  TTN_FP_US915,
  TTN_FP_AS920_923,
  TTN_FP_AS923_925,
  TTN_FP_KR920_923
};

class LoraStack_LoRaWAN : public Arduino_LoRaWAN_ttn {
  ParameterStore &_store;

  public:
  LoraStack_LoRaWAN();
  LoraStack_LoRaWAN(
    const lmic_pinmap &pinmap,
    ParameterStore &store);
  virtual bool GetOtaaProvisioningInfo(
			OtaaProvisioningInfo *pProvisioningInfo
	);
  virtual bool GetAbpProvisioningInfo(
			AbpProvisioningInfo *pProvisioningInfo
	);
  virtual bool GetSavedSessionInfo(
			SessionInfo *pSessionInfo,
			uint8_t *pExtraSessionInfo,
			size_t nExtraSessionInfo,
			size_t *pnExtraSessionActual
			);
  virtual void NetSaveSessionInfo(
			const SessionInfo &SessionInfo,
			const uint8_t *pExtraSessionInfo,
			size_t nExtraSessionInfo
			);
  virtual void NetSaveFCntUp(
			uint32_t uFcntUp
			);
	virtual void NetSaveFCntDown(
			uint32_t uFcntDown
			);
};

class LoraStack {
  // https://github.com/TheThingsNetwork/arduino-device-lib/blob/master/src/TheThingsNetwork.h
  LoraStack_LoRaWAN &_lorawan;
  ParameterStore &_store;
  bool _begun;

  public:

  LoraStack(
    LoraStack_LoRaWAN &lorawan,
    ParameterStore &store,
    ttn_fp_t fp,
    uint8_t sf = TTN_DEFAULT_SF,
    uint8_t fsb = TTN_DEFAULT_FSB);

  bool begin();
  // void reset(bool adr = true);
  // void showStatus();
  // size_t getHardwareEui(char *buffer, size_t size);
  // size_t getAppEui(char *buffer, size_t size);
  void onMessage(void (*cb)(const uint8_t *payload, size_t size, port_t port));
  bool provision(const char *appEui, const char *devEui, const char *appKey);
  bool join(const char *appEui, const char *devEui, const char *appKey, int8_t retries = -1, uint32_t retryDelay = 10000);
  bool join(int8_t retries = -1, uint32_t retryDelay = 10000);
  bool personalize(const char *devAddr, const char *nwkSKey, const char *appSKey);
  // bool personalize();
  ttn_response_t sendBytes(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false, uint8_t sf = 0);
  // ttn_response_t poll(port_t port = 1, bool confirm = false);
  // void sleep(uint32_t mseconds);
  // void wake();
  // void saveState();
  // void linkCheck(uint16_t seconds);
  // uint8_t getLinkCheckGateways();
  // uint8_t getLinkCheckMargin();
};
