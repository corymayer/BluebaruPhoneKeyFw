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
#define RSSI_THRESHOLD -75

#define BUTTON_PRESS_TIME_MS 500
#define WILL_LOCK_TIMEOUT_MS 5000
#define AUTH_CHALLENGE_TIMER_PERIOD 2000

#define BATTERY_UPDATE_PERIOD_MS 5000
#define IDLE_SLEEP_PERIOD_MS 5000
#define CONNECTED_SLEEP_PERIOD_MS 200

#define CMD_AUTHENTICATE 7
#define MAX_PACKET_SIZE 64
#define PKT_HEAD_PLAIN_SIZE 20

#define VBAT_PIN          A7
#define VBAT_MV_PER_LSB   (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F) // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)

typedef enum {
  stateAdvertising, stateDisconnected, stateConnected, 
  stateConnAuth, stateConnAuthUnlocked, stateConnAuthWillLock
} State_t;

typedef enum {
  eventNone, eventConnected, eventDisconnected, eventAuthPass, eventAuthFail,
  eventRegionEntered, eventRegionExited, eventUartRx, eventWillLockTimeout
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
  "Disconnected",
  "Connected",
  "ConnAuth",
  "ConnAuthUnlocked",
  "ConnAuthWillLock"
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

static int pinLock = 25;
static int pinUnlock = 26;
static int pinTrunk = 27;
static int pinBattery = 31;

static bool willLockTimerEnable = false;
static int willLockTimerStart = 0;

static bool authChallengeTimerEnable = true;
static bool authChallengeTimerLast = 0;

static int lastBatteryUpdateTime = 0;

void setup() {
  // init pins
  pinMode(pinLock, OUTPUT);
  pinMode(pinUnlock, OUTPUT);
  pinMode(pinTrunk, OUTPUT);
  
  digitalWrite(pinLock, LOW);
  digitalWrite(pinUnlock, LOW);
  digitalWrite(pinTrunk, LOW);

  // Get a single ADC sample and throw it away
  readVBAT();
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 HRM Example");
  Serial.println("-----------------------\n");

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // turn off blue LED
  Bluefruit.autoConnLed(false);

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

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(uart_rx_cb);

  Serial.println("Configuring the dummy Service");
  setupDummy();

  updateBatSvc();

  startAdv();
}

int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V) 
  analogReference(AR_INTERNAL_3_0);
  
  // Set the resolution to 12-bit (0..4095) 
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  
  // Let the ADC settle
  delay(1);
  
  // Get the raw 12-bit, 0..3000mV ADC value 
  raw = analogRead(VBAT_PIN);
  
  // Set the ADC back to the default settings 
  analogReference(AR_DEFAULT); 
  analogReadResolution(10);
  
  return raw;
}

uint8_t mvToPercent(float mvolts) { 
  uint8_t battery_level;
  
  if (mvolts >= 3000) {
    battery_level = 100;
  } else if (mvolts > 2900) {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  } else if (mvolts > 2740) {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  } else if (mvolts > 2440) {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  } else if (mvolts > 2100) {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  } else {
    battery_level = 0;
  }
  
  return battery_level;
}

void updateBatSvc() {
  // Get a raw ADC reading 
  int vbat_raw = readVBAT();
  
  // Convert from raw mv to percentage (based on LIPO chemistry) 
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

  float vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  Serial.print("Bat percent: ");
  Serial.println(vbat_per);
  Serial.print("Bat mV: ");
  Serial.println(vbat_mv);

  blebas.write(vbat_per);
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
  
  if (rssi > RSSI_THRESHOLD && (state == stateConnAuth || state == stateConnAuthWillLock)) {
    pub_event(eventRegionEntered);
  } else if (rssi < RSSI_THRESHOLD && (state == stateConnAuthUnlocked)) {
    pub_event(eventRegionExited);
  }
}

static void rssi_monitoring_start() {
  BLEConnection* connection = Bluefruit.Connection(g_conn_handle);
  
  // Start monitoring rssi of this connection
  // This function should be called in connect callback
  // Input argument is value difference (to current rssi) that triggers callback
  connection->monitorRssi(5);
}

static void rssi_monitoring_stop() {
  BLEConnection* connection = Bluefruit.Connection(g_conn_handle);

  connection->stopRssi();
}

static void set_door_state(bool locked) {
  int pin = 0;
  if (locked) {
    pin = pinLock;
    Serial.println("Locking door...");
    
    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);
  } else {
    // double unlock for all doors
    Serial.println("Unlocking doors...");
    pin = pinUnlock;

    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);

    delay(BUTTON_PRESS_TIME_MS);

    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);
  }
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

static void send_auth_rdy() {
  char *msg = "authChallenge";

  Serial.println("Sending auth challenge");
  bleuart.write(msg, strlen(msg));
}

static void state_advertising(Event_t ev) {
  switch (ev) {
    case eventConnected:
      Serial.println("Connected!");
      Bluefruit.Advertising.stop(); 
      set_state(stateConnected);
      authChallengeTimerEnable = true;
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }

cleanup:
  return;
}

static void state_disconnected(Event_t ev) {
  switch (ev) {
    case eventConnected:
      Serial.println("Connected!");
      set_state(stateConnected);
      authChallengeTimerEnable = true;
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }

cleanup:
  return;
}

static void state_connected(Event_t ev) {
  BLEConnection* connection;
  switch (ev) {
    case eventUartRx:
      Serial.println("Uart Rx");
      if (verify_auth()) {
        authChallengeTimerEnable = false;
        pub_event(eventAuthPass);
      } else {
        pub_event(eventAuthFail);
      }
      break;
    case eventAuthFail:
      Serial.println("Authentication failed");
      connection = Bluefruit.Connection(g_conn_handle);
      connection->disconnect();
      startAdv();
      set_state(stateAdvertising);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      set_state(stateDisconnected);
      break;
    case eventAuthPass:
      Serial.println("Authentication passed!");
      rssi_monitoring_start();
      set_state(stateConnAuth);
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }

cleanup:
  return;
}

static void state_conn_auth(Event_t ev) {
  switch (ev) {
    case eventRegionEntered:
      Serial.println("Region entered");
      set_door_state(false);
      set_state(stateConnAuthUnlocked);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      rssi_monitoring_stop();
      set_state(stateDisconnected);
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }
  
cleanup:
  return;
}

static void state_conn_auth_unlocked(Event_t ev) {
  switch (ev) {
    case eventRegionExited:
      Serial.println("Region exited");
      willLockTimerEnable = true;
      willLockTimerStart = millis();
      set_state(stateConnAuthWillLock);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      set_door_state(true);
      rssi_monitoring_stop();
      set_state(stateDisconnected);
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }
cleanup:
  return;
}

static void state_conn_auth_will_lock(Event_t ev) {
  switch (ev) {
    case eventRegionEntered:
      Serial.println("Region entered");
      set_state(stateConnAuthUnlocked);
      break;
    case eventWillLockTimeout:
      Serial.println("Will lock timeout!");
      set_door_state(true);
      set_state(stateConnAuth);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      set_door_state(true);
      rssi_monitoring_stop();
      set_state(stateDisconnected);
      break;
    default:
      Serial.print("Unexpected event: ");
      Serial.println(ev);
      break;
  }
}

void loop() {
  Event_t newEvent = eventNone;

  if (event != eventNone) {
    newEvent = event;
    event = eventNone;

    switch (state) {
      case stateAdvertising:
        state_advertising(newEvent);
        break;
      case stateDisconnected:
        state_disconnected(newEvent);
        break;
      case stateConnected:
        state_connected(newEvent);
        break;
      case stateConnAuth:
        state_conn_auth(newEvent);
        break;
      case stateConnAuthUnlocked:
        state_conn_auth_unlocked(newEvent);
        break;
      case stateConnAuthWillLock:
        state_conn_auth_will_lock(newEvent);
        break;
    }
  }

  // janky timers
  int curTimeMs = millis();
  if (willLockTimerEnable) {
    if (curTimeMs - willLockTimerStart > WILL_LOCK_TIMEOUT_MS) {
      pub_event(eventWillLockTimeout);
      willLockTimerEnable = false;
    }
  }

  if (authChallengeTimerEnable) {
    if (curTimeMs - authChallengeTimerLast > AUTH_CHALLENGE_TIMER_PERIOD) {
      authChallengeTimerLast = curTimeMs;

      send_auth_rdy();
    }
  }

  // update battery level
  if (state != stateDisconnected && curTimeMs - lastBatteryUpdateTime > BATTERY_UPDATE_PERIOD_MS) {
    lastBatteryUpdateTime = curTimeMs;

    updateBatSvc();
  }

  // delay calls vTaskDelay and puts MCU to sleep
  if (stateDisconnected == state) {
    Serial.println("Sleeping for 5s");
    delay(IDLE_SLEEP_PERIOD_MS);
  } else {
    delay(CONNECTED_SLEEP_PERIOD_MS);
  }
}
