/*
 * LoraStack.cpp
 * Wraps LMiC library and ties it to non-volatile storage.
 * Currently supports LoraWAN 1.0
 * Copyright (c) 2017 Frank Leon Rose
 */
#include <Arduino.h>
#include <SPI.h>
#include "LoraStack.h"
#include "lmic.h"
#include "hal/hal.h"
#include "lmic/oslmic.h"
#include <Arduino_LoRaWAN_ttn.h>
#include <ParameterStore.h>

const lmic_pinmap lmic_pins = {
  // Not sure why we need to define these here if we're overriding in initializer
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = { 3, 6, LMIC_UNUSED_PIN },
};

#define TTN_NETWORK_ID 0x13

#define CR "\r\n"

typedef void (*JoinResultCallbackFn) (u1_t *appskey, u1_t *nwkskey, u1_t *devaddr);
typedef void (*TransmitResultCallbackFn) (uint16_t error, uint32_t seq_no, u1_t *received, u1_t length);

bool setupLora(TransmitResultCallbackFn txcb);
void loopLora(void);
void loraJoin(uint32_t seq_no, u1_t *appkey, u1_t *appeui, u1_t *deveui, JoinResultCallbackFn joincb);
void loraSetSessionKeys(uint32_t seq_no, u1_t *appskey, u1_t *nwkskey, u1_t *devaddr);
bool loraSendBytes(uint8_t *data, uint16_t len);
void loraSetSF(uint sf);

#if defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be NOT set. Update \
       config.h in the lmic library to set it.
#endif

typedef enum LoraModeEnum {
  NeedsConfiguration,   // Not enough information to do anything
  ReadyToJoin,          // Has AppKey, AppEUI, and DevEUI
  Ready,                // We have session vars via ABP or OTAA - communicate at will
} LoraMode;

static LoraMode mode = NeedsConfiguration;

static JoinResultCallbackFn onJoinCb = NULL;
static TransmitResultCallbackFn onTransmitCb = NULL;

static u1_t join_appkey[16];
static u1_t join_appeui[8];
static u1_t join_deveui[8];

// void os_getArtEui (u1_t* buf) {
//   LS_LOG_DEBUG(F("Asking for AppEUI: %*m"), (int)sizeof(join_appeui), join_appeui);
//   memcpy(buf, join_appeui, sizeof(join_appeui));
// }
// void os_getDevEui (u1_t* buf) {
//   LS_LOG_DEBUG(F("Asking for DevEUI: %*m"), (int)sizeof(join_deveui), join_deveui);
//   memcpy(buf, join_deveui, sizeof(join_deveui));
// }
// void os_getDevKey (u1_t* buf) {
//   LS_LOG_DEBUG(F("Asking for AppKey: %*m"), (int)sizeof(join_appkey), join_appkey);
//   memcpy(buf, join_appkey, sizeof(join_appkey));
// }

static osjob_t timeoutjob;
static void txtimeout_func(osjob_t *job) {
  if (LMIC.opmode & OP_JOINING) {
     // keep waiting.. and don't time out.
     return;
  }
  digitalWrite(LED_BUILTIN, LOW); // off
  LS_LOG_DEBUG(F("Transmit Timeout" CR));
  //txActive = false;
  LMIC_clrTxData ();
}

bool loraSendBytes(uint8_t *data, uint16_t len) {
  if (mode!=Ready) {
    LS_LOG_DEBUG(F("mode not ready, not sending" CR));
    return false; // Did not enqueue
  }
  ostime_t t = os_getTime();
  //os_setTimedCallback(&txjob, t + ms2osticks(100), tx_func);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    LS_LOG_DEBUG(F("OP_TXRXPEND, not sending" CR));
    return false; // Did not enqueue
  }
  else {
    // Prepare upstream data transmission at the next possible time.
    LS_LOG_DEBUG(F("Packet queued" CR));
    digitalWrite(LED_BUILTIN, HIGH); // off
    LMIC_setTxData2(1, data, len, 0);
    if (! (LMIC.opmode & OP_JOINING)) {
      // connection is up, message is queued:
      // Timeout TX after 20 seconds
      os_setTimedCallback(&timeoutjob, t + ms2osticks(20000), txtimeout_func);
    }
    return true;
  }
}

void xonEvent (ev_t ev) {
    LS_LOG_DEBUG("%d: ", os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            LS_LOG_DEBUG(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            LS_LOG_DEBUG(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            LS_LOG_DEBUG(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            LS_LOG_DEBUG(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            LS_LOG_DEBUG(F("EV_JOINING"));
            break;
        case EV_JOINED:
            LS_LOG_DEBUG(F("EV_JOINED"));
            mode = Ready;
            if (onJoinCb) {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkSKey[16];
              u1_t appSKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkSKey, appSKey);
              devaddr = __builtin_bswap32(devaddr); // Settings and Bluetooth deal with devAddr as big endian byte array
              onJoinCb(appSKey, nwkSKey, (u1_t *)&devaddr);
            }
            break;
        case EV_RFU1:
            LS_LOG_DEBUG(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            LS_LOG_DEBUG(F("EV_JOIN_FAILED"));
            if (onJoinCb) {
              onJoinCb(NULL, NULL, NULL);
            }
            break;
        case EV_REJOIN_FAILED:
            LS_LOG_DEBUG(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            os_clearCallback(&timeoutjob);
            LS_LOG_DEBUG(F("EV_TXCOMPLETE (includes waiting for RX windows)" CR));
            digitalWrite(LED_BUILTIN, LOW); // off
            if (onTransmitCb) {
              LS_LOG_DEBUG(F("Calling transmit callback..." CR));
              u1_t *received = NULL;
              u1_t len = 0;
              if (LMIC.dataLen>0) {
                received = LMIC.frame+LMIC.dataBeg;
                len = LMIC.dataLen;
                LS_LOG_DEBUG(F("%*m" CR), len, received);
              }
              uint32_t tx_seq_no = LMIC_getSeqnoUp()-1; // LMIC_getSeqnoUp returns the NEXT one. We want to return the one used.
              onTransmitCb(0 /* success */, tx_seq_no, received, len);
            }

            break;
        case EV_LOST_TSYNC:
            LS_LOG_DEBUG(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            LS_LOG_DEBUG(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            LS_LOG_DEBUG(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            LS_LOG_DEBUG(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            LS_LOG_DEBUG(F("EV_LINK_ALIVE"));
            break;
        case EV_SCAN_FOUND:
            LS_LOG_DEBUG(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            LS_LOG_DEBUG(F("EV_TXSTART"));
            break;
        default:
            LS_LOG_DEBUG(F("Unknown event: %d"), (int)ev);
            break;
    }
    LS_LOG_DEBUG(CR);
}

static void configureLora(uint32_t seq_no) {
  #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #endif

    LMIC_selectSubBand(1);

    LS_LOG_DEBUG(F("%d 125khz and %d 500khz LoRa channels active" CR), LMIC.activeChannels125khz, LMIC.activeChannels500khz);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10,20);

    LS_LOG_DEBUG(F("Set LoRa seq no: %u"), seq_no);
    LMIC_setSeqnoUp(seq_no);
}

void resetLora() {
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
}

bool setupLora(TransmitResultCallbackFn txcb) {
    LS_LOG_INFO(F("Initializing LoRa radio module" CR));

    onTransmitCb = txcb;

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // resetLora() gets called later when we have session or OTAA keys.
    // But call it here to force initialization failure before proceeding further.
    resetLora();

    return true;
}

void loopLora() {
  if (mode!=NeedsConfiguration) {
    os_runloop_once();
  }
}

void loraJoin(uint32_t seq_no, u1_t *appkey, u1_t *appeui, u1_t *deveui, JoinResultCallbackFn joincb) {
  onJoinCb = joincb;

  memcpy(join_appkey, appkey, sizeof(join_appkey));
  memcpy(join_appeui, appeui, sizeof(join_appeui));
  memcpy(join_deveui, deveui, sizeof(join_deveui));

  resetLora();

  configureLora(seq_no);

  if (LMIC_startJoining()) {
    LS_LOG_DEBUG(F("Started joining."));
  }
  else {
    LS_LOG_DEBUG(F("Error: Expected to start joining, but did not!"));
  }

  mode = ReadyToJoin;
}

void loraSetSessionKeys(uint32_t seq_no, u1_t *appskey, u1_t *nwkskey, u1_t *devaddr) {
  LS_LOG_DEBUG("Devaddr: %*m", 4, devaddr);
  LS_LOG_DEBUG("AppSkey: %*m", 16, appskey);
  LS_LOG_DEBUG("NwkSkey: %*m", 16, nwkskey);

  resetLora();

  devaddr_t da = *(devaddr_t *)devaddr;
  da = __builtin_bswap32(da); // Settings and Bluetooth deal with devAddr as big endian byte array
  LMIC_setSession(0x13 /* TTN_NETWORK_ID */, da, nwkskey, appskey);

  configureLora(seq_no);

  mode = Ready;
}

void loraSetSF(uint sf) {
  dr_t dr;
  switch (sf) {
    case 7: dr = DR_SF7; break;
    case 8: dr = DR_SF8; break;
    case 9: dr = DR_SF9; break;
    case 10: dr = DR_SF10; break;
    default:
      dr = DR_SF10;
      LS_LOG_DEBUG(F("Invalid SF value: %d" CR), sf);
      break;
  }
  LMIC_setDrTxpow(dr,20);
}

LoraStack_LoRaWAN::LoraStack_LoRaWAN(
  const lmic_pinmap &pinmap,
  ParameterStore &store)
  : Arduino_LoRaWAN_ttn(pinmap), _store(store) {
}

bool LoraStack_LoRaWAN::GetOtaaProvisioningInfo(
    OtaaProvisioningInfo *pProvisioningInfo
  ) {
  LS_LOG_DEBUG(F("GetOtaaProvisioningInfo %x" CR), pProvisioningInfo);
  static_assert(32==sizeof(OtaaProvisioningInfo), "Unexpected OTAA provisioning requirement");
  if (pProvisioningInfo) {
    int ret1 = _store.get("APPEUI", pProvisioningInfo->AppEUI, sizeof(pProvisioningInfo->AppEUI));
    int ret2 = _store.get("APPKEY", pProvisioningInfo->AppKey, sizeof(pProvisioningInfo->AppKey));
    int ret3 = _store.get("DEVEUI", pProvisioningInfo->DevEUI, sizeof(pProvisioningInfo->DevEUI));
    return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS;
  }
  else {
    LS_LOG_DEBUG(F("Need to figure out whether there is OTAA configuration in parameter store." CR));
    return false;
  }
}

bool LoraStack_LoRaWAN::GetAbpProvisioningInfo(
		AbpProvisioningInfo *pProvisioningInfo
  ) {
  LS_LOG_DEBUG(F("GetAbpProvisioningInfo %x" CR), pProvisioningInfo);
  static_assert(48==sizeof(AbpProvisioningInfo), "Unexpected ABP provisioning requirement");
  int ret1 = _store.get("NWKSKEY", pProvisioningInfo->NwkSKey, sizeof(pProvisioningInfo->NwkSKey));
  int ret2 = _store.get("APPSKEY", pProvisioningInfo->AppSKey, sizeof(pProvisioningInfo->AppSKey));
  int ret3 = _store.get("DEVADDR", &pProvisioningInfo->DevAddr);
  int ret4 = _store.get("NETID", &pProvisioningInfo->NetID);

  int ret5 = _store.get("FCNTUP", &pProvisioningInfo->FCntUp);
  if (ret5!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load Up frame counter (%d)" CR), ret5);
  }
  int ret6 = _store.get("FCNTDN", &pProvisioningInfo->FCntDown);
  if (ret6!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load Down frame counter (%d)" CR), ret6);
  }
  // Don't include ret5 & ret6 in return value. They aren't absolutely required for ABP.
  return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS && ret4==PS_SUCCESS;
}


bool LoraStack_LoRaWAN::GetSavedSessionInfo(
    SessionInfo *pSessionInfo,
    uint8_t *pExtraSessionInfo,
    size_t nExtraSessionInfo,
    size_t *pnExtraSessionActual
    ) {
  LS_LOG_DEBUG(F("GetSavedSessionInfo %x" CR), pSessionInfo);
  pSessionInfo->V1.Tag = kSessionInfoTag_V1;
  pSessionInfo->V1.Size = sizeof(SessionInfoV1);
  pSessionInfo->V1.Rsv2 = 0;
  pSessionInfo->V1.Rsv3 = 0;
  if (PS_SUCCESS!=_store.get("NETID", &pSessionInfo->V1.NetID)) {
    return false;
  }
  if (PS_SUCCESS!=_store.get("DEVADDR", &pSessionInfo->V1.DevAddr)) {
    return false;
  }
  if (PS_SUCCESS!=_store.get("NWKSKEY", pSessionInfo->V1.NwkSKey, sizeof(pSessionInfo->V1.NwkSKey))) {
    return false;
  }
  if (PS_SUCCESS!=_store.get("APPSKEY", pSessionInfo->V1.AppSKey, sizeof(pSessionInfo->V1.AppSKey))) {
    return false;
  }
  if (PS_SUCCESS!=_store.get("FCNTUP", &pSessionInfo->V1.FCntUp)) {
    return false;
  }
  if (PS_SUCCESS!=_store.get("FCNTDN", &pSessionInfo->V1.FCntDown)) {
    return false;
  }
  return true;
}

void LoraStack_LoRaWAN::NetSaveSessionInfo(
    const SessionInfo &SessionInfo,
    const uint8_t *pExtraSessionInfo,
    size_t nExtraSessionInfo
    ) {
  LS_LOG_DEBUG(F("NetSaveSessionInfo %x" CR), &SessionInfo);
  if (SessionInfo.V1.Tag != kSessionInfoTag_V1) {
    LS_LOG_ERROR(F("Unknown session info tag %u expecting %u"), SessionInfo.V1.Tag, kSessionInfoTag_V1);
    return;
  }
  if (SessionInfo.V1.Size != sizeof(SessionInfoV1)) {
    LS_LOG_ERROR(F("Unknown session info size %u expectingn %u"), SessionInfo.V1.Tag, sizeof(SessionInfoV1));
    return;
  }
  if (PS_SUCCESS!=_store.set("NETID", SessionInfo.V1.NetID)) {
    LS_LOG_ERROR(F("Failed to save NETID"));
    return;
  }
  if (PS_SUCCESS!=_store.set("DEVADDR", SessionInfo.V1.DevAddr)) {
    LS_LOG_ERROR(F("Failed to save DEVADDR"));
    return;
  }
  if (PS_SUCCESS!=_store.set("NWKSKEY", SessionInfo.V1.NwkSKey, sizeof(SessionInfo.V1.NwkSKey))) {
    LS_LOG_ERROR(F("Failed to save NWKSKEY"));
    return;
  }
  if (PS_SUCCESS!=_store.set("APPSKEY", SessionInfo.V1.AppSKey, sizeof(SessionInfo.V1.AppSKey))) {
    LS_LOG_ERROR(F("Failed to save APPSKEY"));
    return;
  }
  if (PS_SUCCESS!=_store.set("FCNTUP", SessionInfo.V1.FCntUp)) {
    LS_LOG_ERROR(F("Failed to save FCNTUP"));
    return;
  }
  if (PS_SUCCESS!=_store.set("FCNTDN", SessionInfo.V1.FCntDown)) {
    LS_LOG_ERROR(F("Failed to save FCNTDN"));
    return;
  }
}
void LoraStack_LoRaWAN::NetSaveFCntUp(
    uint32_t uFcntUp
    ) {
  LS_LOG_DEBUG(F("NetSaveFCntUp %d" CR), uFcntUp);
  if (PS_SUCCESS!=_store.set("FCNTUP", uFcntUp)) {
    LS_LOG_ERROR(F("Failed to save FCNTUP"));
  }
}
void LoraStack_LoRaWAN::NetSaveFCntDown(
    uint32_t uFcntDown
    ) {
  LS_LOG_DEBUG(F("NetSaveFCntDown %d" CR), uFcntDown);
  if (PS_SUCCESS!=_store.set("FCNTDN", uFcntDown)) {
    LS_LOG_ERROR(F("Failed to save FCNTDN"));
  }
}


LoraStack::LoraStack(
    LoraStack_LoRaWAN &lorawan,
    ParameterStore &store,
    ttn_fp_t fp,
    uint8_t sf,
    uint8_t fsb)
  : _lorawan(lorawan), _store(store), _begun(false) {
}

bool LoraStack::begin() {
  if (!_begun) {
    _begun = _lorawan.begin();
  }
  return _begun;
}

bool LoraStack::join(
  const char *appEui,
  const char *devEui,
  const char *appKey,
  int8_t retries,
  uint32_t retryDelay) {
  return provision(appEui, devEui, appKey) && join(retries, retryDelay);
}

bool LoraStack::join(
  int8_t retries,
  uint32_t retryDelay) {
  // Initiate join with provisioned appEui, devEui, and appKey
  return begin();
}

uint8_t hexDigit(const char hex) {
  if ('0'<=hex && hex<='9') {
    return ((uint8_t)hex) - '0';
  }
  else if ('A'<=hex && hex<='F') {
    return ((uint8_t)hex) - 'A' + 0xA;
  }
  else if ('a'<=hex && hex<='f') {
    return ((uint8_t)hex) - 'a' + 0xA;
  }
  return 16;
}

bool hexToBytes(uint8_t *buffer, uint16_t size, const char *hex) {
  // LS_LOG_DEBUG("hexToBytes %s into len %d %x" CR, hex, (int)size, buffer);
  uint16_t i = 0;
  while (i<size && hex[0]!=0 && hex[1]!=0) {
    uint8_t msn = hexDigit(*hex++);
    uint8_t lsn = hexDigit(*hex++);
    if (msn==16 || lsn==16) {
      return false;
    }
    buffer[i++] = (msn<<4) + lsn;
  }
  return (i==size) && *hex==0; // Used up input and filled buffer
}

bool LoraStack::personalize(
  const char *devAddr,
  const char *nwkSKey,
  const char *appSKey) {
  uint32_t devAddrLong = 0;
  uint8_t nwkSKeyBytes[16];
  uint8_t appSKeyBytes[16];

  if (!hexToBytes((uint8_t *)&devAddrLong, sizeof(devAddrLong), devAddr)) {
    LS_LOG_DEBUG(F("Failed to parse DevAddr: %s" CR), devAddr);
    return false;
  }
  devAddrLong = ntohl(devAddrLong); // Correct for network byte ordering.
  if (!hexToBytes(nwkSKeyBytes, sizeof(nwkSKeyBytes), nwkSKey)) {
    LS_LOG_DEBUG(F("Failed to parse NwkSKey: %s"), nwkSKey);
    return false;
  }
  if (!hexToBytes(appSKeyBytes, sizeof(appSKeyBytes), appSKey)) {
    LS_LOG_DEBUG(F("Failed to parse AppSKey: %s"), appSKey);
    return false;
  }
  // LS_LOG_DEBUG(F("Writing DEVADDR: %x" CR), devAddrLong);
  int ret1 = _store.set("DEVADDR", devAddrLong);
  // LS_LOG_DEBUG(F("Writing NWKSKEY: %*m" CR), sizeof(nwkSKeyBytes), nwkSKeyBytes);
  int ret2 = _store.set("NWKSKEY", nwkSKeyBytes, sizeof(nwkSKeyBytes));
  // LS_LOG_DEBUG(F("Writing APPSKEY: %*m" CR), sizeof(appSKeyBytes), appSKeyBytes);
  int ret3 = _store.set("APPSKEY", appSKeyBytes, sizeof(appSKeyBytes));
  // LS_LOG_DEBUG(F("Returning: %d %d %d" CR), ret1, ret2, ret3);
  return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS;
}

bool LoraStack::provision(
  const char *appEui,
  const char *devEui,
  const char *appKey) {
  // Configure node with appEui, devEui, and appKey. Delete existing OTAA & ABP values if different.
  uint8_t appEuiBytes[8];
  uint8_t devEuiBytes[8];
  uint8_t appKeyBytes[16];
  if (!hexToBytes(appEuiBytes, sizeof(appEuiBytes), appEui)) {
    LS_LOG_DEBUG(F("Failed to parse app EUI: %s"), appEui);
    return false;
  }
  if (!hexToBytes(devEuiBytes, sizeof(devEuiBytes), devEui)) {
    LS_LOG_DEBUG(F("Failed to parse app EUI: %s"), devEui);
    return false;
  }
  if (!hexToBytes(appKeyBytes, sizeof(appKeyBytes), appKey)) {
    LS_LOG_DEBUG(F("Failed to parse app key: %s"), appKey);
    return false;
  }

  // ParameterStore.Transaction(_store);
  int ret1 = _store.set("APPEUI", appEuiBytes, sizeof(appEuiBytes));
  int ret2 = _store.set("DEVEUI", devEuiBytes, sizeof(devEuiBytes));
  int ret3 = _store.set("APPKEY", appKeyBytes, sizeof(appKeyBytes));
  return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS;
}

ttn_response_t LoraStack::sendBytes(
  const uint8_t *payload,
  size_t length,
  port_t port,
  bool confirm,
  uint8_t sf) {
  // TODO: port & sf
  LS_LOG_DEBUG(F("Sending bytes: %*m" CR), length, payload);
  bool ok = _lorawan.SendBuffer(payload, length, NULL, NULL, confirm);

  if (ok) {
    return TTN_SUCCESSFUL_TRANSMISSION;
  }
  else {
    LS_LOG_ERROR(F("Error sending bytes." CR));
    return TTN_ERROR_SEND_COMMAND_FAILED;
  }
}

typedef void (*ReceiveBufferCbFn)(const uint8_t *payload, size_t size, port_t port);

static void ReceiveBufferCb(
  void *ctx,
  const uint8_t *pBuffer,
  size_t nBuffer) {
  LS_LOG_DEBUG("Received message: %*m", nBuffer, pBuffer);
  if (ctx) {
    void (*cb)(const uint8_t *payload, size_t size, port_t port) = (ReceiveBufferCbFn)ctx;
    cb(pBuffer, nBuffer, 1); // TODO: Port
  }
}

void LoraStack::onMessage(void (*cb)(const uint8_t *payload, size_t size, port_t port)) {
  LS_LOG_DEBUG("Register receive callback." CR);
	_lorawan.SetReceiveBufferBufferCb(ReceiveBufferCb, (void *)cb);
}

