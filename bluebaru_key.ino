/**
 * Bluebaru Phone Key Firmware
 * https://github.com/corymayer/BluebaruPhoneKeyFw
 */
#include <bluefruit.h>
#include <ChaChaPoly.h>
#include "crc32.h"
#include "adafruit_utils.h"
#include "ble.h"

// set to zero to disable GPIO output, useful for debugging
#define ACTUATE_DOOR 1

#define AES_KEY "w9z$C&E)H@McQfTjWnZr4u7x!A%D*G-J" // change this to your key
#define AES_KEY_LEN 256
#define RSSI_THRESHOLD -75 // adjust this to change how close you need to get to unlock the door

#define BUTTON_PRESS_TIME_MS 500
#define WILL_LOCK_TIMEOUT_MS 5000
#define AUTH_CHALLENGE_TIMER_PERIOD 2000

#define BATTERY_UPDATE_PERIOD_MS 5000
#define IDLE_SLEEP_PERIOD_MS 5000
#define CONNECTED_SLEEP_PERIOD_MS 200

#define CMD_AUTHENTICATE 7
#define MAX_PACKET_SIZE 64
#define PKT_HEAD_PLAIN_SIZE 20

#define SERIAL_INIT_DELAY_MS 10
#define SERIAL_BAUD 115200

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

static uint8_t curConnHandle = 0;

static State_t state = stateAdvertising;
static Event_t event = eventNone;

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

/**
 * Sets the new state and echos to UART.
 */
static void set_state(State_t st) {
  Serial.print("Switching from state ");
  Serial.print(stateNames[state]);
  Serial.print(" to ");
  Serial.println(stateNames[st]);
  
  state = st;
}

/**
 * Publishes an event by setting the event var.
 * TODO: this is not thread safe, should find a way to tap into FreeRTOS queues in the future.
 */
static void pub_event(Event_t ev) {
  event = ev;
}

/**
 * Callback for data recvd on UART.
 */
void ble_uart_rx_cb(uint16_t conn_hdl) {
  pub_event(eventUartRx);
}

void ble_connect_cb(uint16_t conn_hdl) {
  curConnHandle = conn_hdl;
  pub_event(eventConnected);
}

void ble_disconnect_cb(uint16_t conn_hdl, uint8_t reason) {
  pub_event(eventDisconnected);
}

void ble_rssi_cb(uint16_t handle, int8_t rssi) {
  Serial.printf("Rssi = %d", rssi);
  Serial.println();
  
  if (rssi > RSSI_THRESHOLD && (state == stateConnAuth || state == stateConnAuthWillLock)) {
    pub_event(eventRegionEntered);
  } else if (rssi < RSSI_THRESHOLD && (state == stateConnAuthUnlocked)) {
    pub_event(eventRegionExited);
  }
}

/**
 * Initializes the hardware peripherals.
 */
void initPeripherals(void) {
  pinMode(pinLock, OUTPUT);
  pinMode(pinUnlock, OUTPUT);
  pinMode(pinTrunk, OUTPUT);
  
  digitalWrite(pinLock, LOW);
  digitalWrite(pinUnlock, LOW);
  digitalWrite(pinTrunk, LOW);

  // start up ADC
  readVBAT();

  // init serial
  while (!Serial) delay(SERIAL_INIT_DELAY_MS);
  Serial.begin(SERIAL_BAUD);
}

void setup() {  
  initPeripherals();
  
  Serial.println("Bluebaru Phone Key\n");
  
  BLEStart(ble_connect_cb, ble_disconnect_cb, ble_rssi_cb, ble_uart_rx_cb);
}

/**
 * Locks or unlocks the door. If ACTUATE_DOOR is set, the fob GPIO will be output accordingly.
 */
static void set_door_state(bool locked) {
  int pin = 0;
  if (locked) {
    pin = pinLock;
    Serial.println("Locking door...");

#if ACTUATE_DOOR
    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);
#endif
  } else {
    // double unlock for all doors
    Serial.println("Unlocking doors...");
    pin = pinUnlock;

#if ACTUATE_DOOR
    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);

    delay(BUTTON_PRESS_TIME_MS);

    digitalWrite(pin, HIGH);
    delay(BUTTON_PRESS_TIME_MS);
    digitalWrite(pin, LOW);
#endif
  }
}

/**
 * Reads auth response, decrypts it using the ChaChaPoly cipher,
 * and verifies the connection is secure.
 * TODO: break down into smaller funcs
 */
static bool verify_auth() {
  bool authPassed = false;

  const size_t nonceSize = 12;
  const size_t tagLen = 16;
  
  ChaChaPoly chacha;
  uint8_t buf[MAX_PACKET_SIZE];
  uint8_t decryptedBuf[MAX_PACKET_SIZE];
  
  size_t avail = BLEUartGetBytesAvail();
  size_t readBytes = avail > MAX_PACKET_SIZE ? MAX_PACKET_SIZE : avail;

  uint32_t calcCrc = 0;
  uint32_t pktCrc = 0;
  uint64_t *nonceFull = (uint64_t *)((uint8_t *)buf + 12);
  uint32_t truncNonce = 0;
  uint8_t *encryptedData = (uint8_t *)buf + PKT_HEAD_PLAIN_SIZE;
  
  BLEUartRead(buf, readBytes);
  Serial.println((char *)buf);

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

/**
 * Sends the authentication challenge message to the iOS app over BLE UART.
 */
static void send_auth_rdy() {
  char *msg = "authChallenge";

  Serial.println("Sending auth challenge");
  BLEUartWrite((uint8_t *)msg, strlen(msg));
}

static void state_advertising(Event_t ev) {
  switch (ev) {
    case eventConnected:
      Serial.println("Connected!");
      BLEStopAdvertising();
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
      BLEDisconnect(curConnHandle);
      
      set_state(stateAdvertising);
      BLEStartAdvertising();
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      set_state(stateDisconnected);
      break;
    case eventAuthPass:
      Serial.println("Authentication passed!");
      BLERssiMonitorStart(curConnHandle);
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
      BLERssiMonitorStop(curConnHandle);
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
      BLERssiMonitorStop(curConnHandle);
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
      willLockTimerEnable = false;
      set_state(stateConnAuthUnlocked);
      break;
    case eventWillLockTimeout:
      Serial.println("Will lock timeout!");
      set_door_state(true);
      set_state(stateConnAuth);
      break;
    case eventDisconnected:
      Serial.println("Disconnected");
      willLockTimerEnable = false;
      set_door_state(true);
      BLERssiMonitorStop(curConnHandle);
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

    BLEBatSvcUpdate();
  }

  // delay calls vTaskDelay and puts MCU to sleep to save power
  if (stateDisconnected == state) {
    Serial.println("Sleeping for 5s");
    delay(IDLE_SLEEP_PERIOD_MS);
  } else {
    delay(CONNECTED_SLEEP_PERIOD_MS);
  }
}
