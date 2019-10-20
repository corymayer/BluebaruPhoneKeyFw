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
#include <ChaCha.h>

#define AES_KEY 0x78214125442A472D4B6150645267556B58703273357638792F423F4528482B4D
#define RSSI_THRESHOLD -70

typedef enum {
  stateAdvertising, stateConnected, stateConnAuth, stateConnAuthNearby
} State_t;

typedef enum {
  eventNone, eventConnected, eventDisconnected, eventAuthPass, eventAuthFail,
  eventRegionEntered, eventRegionExited
} Event_t;

BLEService testSvc = BLEService(0xDA026848F2D511E9B26A76BC64B1963E);
BLECharacteristic testChar = BLECharacteristic(0xF974F2F4F2D511E9BB23A6C064B1963E);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

uint8_t  bps = 0;

static State_t state = stateAdvertising;
static Event_t event = eventNone;

static uint16_t g_conn_handle = 0;

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

  Serial.println("Configuring the test Service");
  setupTest();

  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(testSvc);

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

void setupTest(void) {
  testSvc.begin();

  testChar.setProperties(CHR_PROPS_READ);
  testChar.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  testChar.setFixedLen(1);
  testChar.begin();

  uint8_t data = 7;
  testChar.write(&data, 1);
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
  Serial.print(state);
  Serial.print(" to ");
  Serial.println(st);
  
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
      // TODO temp, assume auth passes
      pub_event(eventAuthPass);
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

static void state_connected(Event_t ev) {
    switch (ev) {
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
