/*
 * LoraStack.cpp
 * Wraps LMiC library and ties it to non-volatile storage.
 * Currently supports LoraWAN 1.0
 * Copyright (c) 2017 Frank Leon Rose
 */

#include <functional> // Include standard libs before someone else #defines min/max etc.

#include <Arduino.h>
#include <SPI.h>
#include "LoraStack.h"
#include "lmic.h"
#include "hal/hal.h"
#include "lmic/oslmic.h"
#include <Arduino_LoRaWAN_ttn.h>
#include <ParameterStore.h>

#define TTN_NETWORK_ID 0x13

#define CR "\r\n"

#if defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be NOT set. Update \
       config.h in the lmic library to set it.
#endif

LoraStack_LoRaWAN::LoraStack_LoRaWAN(
  const lmic_pinmap &pinmap,
  ParameterStore &store)
  : Arduino_LoRaWAN_ttn(pinmap), _store(store) {
}

static void reverseMem(uint8_t *ptr, size_t size) {
  uint8_t *b = ptr;
  uint8_t *e = ptr + size - 1;
  while (b<e) {
    uint8_t temp = *b;
    *b = *e;
    *e = temp;
    b++;
    e--;
  }
}

bool LoraStack_LoRaWAN::GetOtaaProvisioningInfo(
    OtaaProvisioningInfo *pProvisioningInfo
  ) {
  LS_LOG_DEBUG(F("GetOtaaProvisioningInfo %x" CR), pProvisioningInfo);
  static_assert(32==sizeof(OtaaProvisioningInfo), "Unexpected OTAA provisioning requirement");
  if (pProvisioningInfo) {
    int ret1 = _store.get("APPEUI", pProvisioningInfo->AppEUI, sizeof(pProvisioningInfo->AppEUI));
    // In code hex strings and storage, store these MSB.
    // For some reason, LoRaWAN code expects AppEUI & DevEUI reversed.
    reverseMem(pProvisioningInfo->AppEUI, sizeof(pProvisioningInfo->AppEUI));
    int ret2 = _store.get("APPKEY", pProvisioningInfo->AppKey, sizeof(pProvisioningInfo->AppKey));
    int ret3 = _store.get("DEVEUI", pProvisioningInfo->DevEUI, sizeof(pProvisioningInfo->DevEUI));
    reverseMem(pProvisioningInfo->DevEUI, sizeof(pProvisioningInfo->DevEUI));
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
  if (ret1!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load NSKSKEY (%d)" CR), ret1);
  }
  int ret2 = _store.get("APPSKEY", pProvisioningInfo->AppSKey, sizeof(pProvisioningInfo->AppSKey));
  if (ret2!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load APPSKEY (%d)" CR), ret2);
  }
  int ret3 = _store.get("DEVADDR", &pProvisioningInfo->DevAddr);
  if (ret3!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load DEVADDR (%d)" CR), ret3);
  }
  int ret4 = _store.get("NETID", &pProvisioningInfo->NetID);
  if (ret4!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load net ID (%d)" CR), ret4);
  }

  int ret5 = _store.get("FCNTUP", &pProvisioningInfo->FCntUp);
  if (ret5!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load Up frame counter (%d)" CR), ret5);
    pProvisioningInfo->FCntUp = 0;
  }
  int ret6 = _store.get("FCNTDN", &pProvisioningInfo->FCntDown);
  if (ret6!=PS_SUCCESS) {
    LS_LOG_INFO(F("Failed to load Down frame counter (%d)" CR), ret6);
    pProvisioningInfo->FCntDown = 0;
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
  return begin() && LMIC_startJoining();
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
  int ret4 = _store.set("NETID", TTN_NETWORK_ID);
  return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS && ret4==PS_SUCCESS;
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
  int ret4 = _store.set("NETID", TTN_NETWORK_ID);
  return ret1==PS_SUCCESS && ret2==PS_SUCCESS && ret3==PS_SUCCESS && ret4==PS_SUCCESS;
}

/* static */ void LoraStack::txCallback(void *context_this, bool success) {
  LoraStack *self = reinterpret_cast<LoraStack*>(context_this);
  auto cb = self->_txCallback;
  if (cb) {
    self->_txCallback = nullptr;
    cb(success);
  }
}

ttn_response_t LoraStack::sendBytes(
  const uint8_t *payload,
  size_t length,
  port_t port,
  bool confirm,
  std::function<void(bool)> cb,
  uint8_t sf) {
  // TODO: port & sf
  LS_LOG_DEBUG(F("Sending bytes: %*m with %s" CR), length, payload, confirm ? "ack" : "no ack");
  bool ok = _lorawan.SendBuffer(payload, length, LoraStack::txCallback, this, confirm);

  if (ok) {
    return TTN_SUCCESSFUL_TRANSMISSION; // Successfully enqueued - NOT an indication that the message was received
  }
  else {
    LS_LOG_ERROR(F("Error sending bytes." CR));
    return TTN_ERROR_SEND_COMMAND_FAILED;
  }
}

typedef void (*ReceiveBufferCbFn)(const uint8_t *payload, size_t size, port_t port);

static void ReceiveBufferCb(
  void *ctx,
  uint8_t uPort,
  const uint8_t *pBuffer,
  size_t nBuffer) {
  LS_LOG_DEBUG("Received message: %*m", nBuffer, pBuffer);
  if (ctx) {
    void (*cb)(const uint8_t *payload, size_t size, port_t port) = (ReceiveBufferCbFn)ctx;
    cb(pBuffer, nBuffer, uPort);
  }
}

void LoraStack::onMessage(void (*cb)(const uint8_t *payload, size_t size, port_t port)) {
  LS_LOG_DEBUG("Register receive callback." CR);
	_lorawan.SetReceiveBufferBufferCb(ReceiveBufferCb, (void *)cb);
}

