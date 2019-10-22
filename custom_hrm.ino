/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <ChaChaPoly.h>
#include "crc32.h"

#define SERVICE_UUID 0xDA026848F2D511E9B26A76BC64B1963E
#define CHARACTERISTIC_UUID 0xF974F2F4F2D511E9BB23A6C064B1963E
#define AES_KEY "w9z$C&E)H@McQfTjWnZr4u7x!A%D*G-J"
#define AES_KEY_LEN 256
#define RSSI_THRESHOLD -70
#define MAX_PACKET_SIZE 64

#define CMD_AUTHENTICATE 7
#define PKT_HEAD_PLAIN_SIZE 20

typedef enum {
  stateAdvertising, stateConnected, stateConnAuth, stateConnAuthNearby
} State_t;

typedef enum {
  eventNone, eventConnected, eventDisconnected, eventAuthPass, eventAuthFail,
  eventRegionEntered, eventRegionExited, eventUartRx
} Event_t;

typedef struct {
  uint32_t crc;
  uint32_t pktLen;
  uint32_t nonce1;
  uint64_t nonce2;
} UartPktHdrPlain_t;

typedef struct {
  uint32_t cmd;
  uint32_t dataLen;
} UartPktHdrEnc_t;

static const char *stateNames[] = {
  "Advertising",
  "Connected",
  "ConnAuth",
  "ConnAuthNearby"
};

BLEService dummySvc = BLEService(SERVICE_UUID);
BLECharacteristic dummyChar = BLECharacteristic(CHARACTERISTIC_UUID);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance
BLEUart bleuart;

uint8_t  bps = 0;

static State_t state = stateAdvertising;
static Event_t event = eventNone;

static uint16_t g_conn_handle = 0;
static crc32_t crc;
static uint64_t lastNonce = 0;

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 HRM Example");
  Serial.println("-----------------------\n");

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'BluebaruKey'");
  Bluefruit.setName("BluebaruKey");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Set up Rssi changed callback
  Bluefruit.setRssiCallback(rssi_changed_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(uart_rx_cb);

  Serial.println("Configuring the dummy Service");
  setupDummy();

  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.Advertising.addService(dummySvc);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  

  Serial.println("Advertising"); 
}

void setupDummy(void) {
  dummySvc.begin();

  dummyChar.setProperties(CHR_PROPS_READ);
  dummyChar.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  dummyChar.setFixedLen(1);
  dummyChar.begin();

  uint8_t data = 7;
  dummyChar.write(&data, 1);
}

void uart_rx_cb(uint16_t conn_hdl) {
  pub_event(eventUartRx);
}

void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  g_conn_handle = conn_handle;

  pub_event(eventConnected);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  pub_event(eventDisconnected);
}

static void set_state(State_t st) {
  Serial.print("Switching from state ");
  Serial.print(stateNames[state]);
  Serial.print(" to ");
  Serial.println(stateNames[st]);
  
  state = st;
}

static void pub_event(Event_t ev) {
  event = ev;
}

void rssi_changed_callback(uint16_t conn_hdl, int8_t rssi) {
  (void) conn_hdl;
  Serial.printf("Rssi = %d", rssi);
  Serial.println();
  
  if (state == stateConnAuth && rssi > RSSI_THRESHOLD) {
    pub_event(eventRegionEntered);
  } else if (state == stateConnAuthNearby && rssi < RSSI_THRESHOLD) {
    pub_event(eventRegionExited);
  }
}

static void rssi_monitoring_start() {
  BLEConnection* connection = Bluefruit.Connection(g_conn_handle);
  
  // Start monitoring rssi of this connection
  // This function should be called in connect callback
  // Input argument is value difference (to current rssi) that triggers callback
  connection->monitorRssi(10);
}

static void rssi_monitoring_stop() {
  BLEConnection* connection = Bluefruit.Connection(g_conn_handle);

  connection->stopRssi();
}

static void set_door_state(bool locked) {
  // TODO
}

static void state_advertising(Event_t ev) {
  switch (ev) {
    case eventConnected:
      Serial.println("Connected!");
      set_state(stateConnected);
      break;
    default:
      Serial.println("Unexpected event");
      goto cleanup;
      break;
  }

cleanup:
  return;
}

static bool verify_auth() {
  bool authPassed = false;

  const size_t nonceSize = 12;
  const size_t tagLen = 16;
  
  ChaChaPoly chacha;
  uint8_t buf[MAX_PACKET_SIZE];
  uint8_t decryptedBuf[MAX_PACKET_SIZE];
  
  size_t avail = bleuart.available();
  size_t readBytes = avail > MAX_PACKET_SIZE ? MAX_PACKET_SIZE : avail;

  uint32_t calcCrc = 0;
  uint32_t pktCrc = 0;
  uint64_t *nonceFull = (uint64_t *)((uint8_t *)buf + 12);
  uint32_t truncNonce = 0;
  uint8_t *encryptedData = (uint8_t *)buf + PKT_HEAD_PLAIN_SIZE;
  
  bleuart.read(buf, readBytes);
  Serial.println((char *)buf);
//  for (int ndx = 0; ndx < readBytes; ndx++) {
//    Serial.println(buf[ndx]);
//  }

  UartPktHdrPlain_t *pkt = (UartPktHdrPlain_t *)buf;
  UartPktHdrEnc_t *hdrEnc = (UartPktHdrEnc_t *)decryptedBuf;

  uint8_t *nonce = (uint8_t *)&pkt->nonce1;
  size_t encryptedDataLen = pkt->pktLen - PKT_HEAD_PLAIN_SIZE - tagLen;
  uint8_t *tag = (uint8_t *)buf + PKT_HEAD_PLAIN_SIZE + encryptedDataLen;

  // verify CRC
  pktCrc = pkt->crc;
  memset(&pkt->crc, 0, 4); // clear CRC in packet
  
  crc32_t c;
  crc32_init(&c);
  crc32_start(&c);
  crc32_update(&c, buf, pkt->pktLen);
  calcCrc = crc32_finalize(&c);

  if (pktCrc != calcCrc) {
    Serial.println("CRC fail!");
    goto cleanup;
  }

  Serial.println("CRC pass");

  // decrypt packet
  for (int ndx = 0; ndx < 12; ndx++) {
    Serial.print(*(nonce + ndx));
  }
  Serial.println("");
  Serial.println("Decrypting...");
  chacha.setKey((const uint8_t *)AES_KEY, AES_KEY_LEN);
  chacha.setIV(nonce, nonceSize);

  chacha.decrypt(decryptedBuf, encryptedData, encryptedDataLen);

  Serial.println("Checking tag...");
  if (!chacha.checkTag(tag, tagLen)) {
    Serial.println("Bad tag");
    goto cleanup;
  }

  Serial.println("Tag passed. Nonce: ");
  truncNonce = *nonceFull;
  Serial.println(truncNonce);

  if (*nonceFull <= lastNonce) {
    Serial.println("Bad nonce");
    goto cleanup;
  }

  Serial.print("Command: ");
  Serial.println(hdrEnc->cmd);
  if (hdrEnc->cmd != CMD_AUTHENTICATE) {
    Serial.println("Bad cmd type");
    goto cleanup;
  }

  lastNonce = *nonceFull;
  authPassed = true;

cleanup:
  return authPassed;
}

static void state_connected(Event_t ev) {
    switch (ev) {
    case eventUartRx:
      Serial.println("Uart Rx");
      if (verify_auth()) {
        pub_event(eventAuthPass);
      } else {
        pub_event(eventAuthFail);
      }
      break;
    case eventAuthFail:
      Serial.println("Authentication failed");
      startAdv();
      set_state(stateAdvertising);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      startAdv();
      set_state(stateAdvertising);
      break;
    case eventAuthPass:
      Serial.println("Authentication passed!");
      rssi_monitoring_start();
      set_state(stateConnAuth);
      break;
    default:
      Serial.println("Unexpected event");
      break;
  }

cleanup:
  return;
}

static void state_conn_auth(Event_t ev) {
  switch (ev) {
    case eventRegionEntered:
      Serial.println("Region entered! Unlocking door...");
      set_door_state(false);
      set_state(stateConnAuthNearby);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      rssi_monitoring_stop();
      startAdv();
      set_state(stateAdvertising);
      break;
    default:
      Serial.println("Unexpected event");
      break;
  }
  
cleanup:
  return;
}

static void state_conn_auth_nearby(Event_t ev) {
  switch (ev) {
    case eventRegionExited:
      Serial.println("Region exited! Locking door...");
      set_door_state(true);
      set_state(stateConnAuth);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      set_door_state(true);
      rssi_monitoring_stop();
      startAdv();
      set_state(stateAdvertising);
      break;
    default:
      Serial.println("Unexpected event");
      break;
  }
cleanup:
  return;
}

void loop() {
  Event_t newEvent = eventNone;
  
  digitalToggle(LED_RED);

  if (event != eventNone) {
    newEvent = event;
    event = eventNone;

    switch (state) {
      case stateAdvertising:
        state_advertising(newEvent);
        break;
      case stateConnected:
        state_connected(newEvent);
        break;
      case stateConnAuth:
        state_conn_auth(newEvent);
        break;
      case stateConnAuthNearby:
        state_conn_auth_nearby(newEvent);
        break;
    }
  }

  // Only send update once per second
  delay(100);
}
