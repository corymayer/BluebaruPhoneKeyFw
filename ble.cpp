#include "ble.h"
#include <bluefruit.h>
#include "adafruit_utils.h"

#define SERVICE_UUID 0xDA026848F2D511E9B26A76BC64B1963E
#define CHARACTERISTIC_UUID 0xF974F2F4F2D511E9BB23A6C064B1963E

BLEService dummySvc = BLEService(SERVICE_UUID);
BLECharacteristic dummyChar = BLECharacteristic(CHARACTERISTIC_UUID);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance
BLEUart bleuart;

static uint16_t g_conn_handle = 0;

void (*connectCb)(uint16_t handle) = NULL;
void (*disconnectCb)(uint16_t handle, uint8_t reason) = NULL;
void (*rssiCb)(uint16_t handle, int8_t rssi) = NULL;
void (*uartRxCb)(uint16_t handle) = NULL;

/**
 * BLE connection callback.
 */
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  g_conn_handle = conn_handle;

  if (connectCb) {
    connectCb(conn_handle);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  if (disconnectCb) {
    disconnectCb(conn_handle, reason);
  }
}

void rssi_changed_callback(uint16_t conn_hdl, int8_t rssi) {
  if (rssiCb) {
    rssiCb(conn_hdl, rssi);
  }
}

void uart_rx_cb(uint16_t conn_hdl) {
  if (uartRxCb) {
    uartRxCb(conn_hdl);
  }
}

/**
 * Dummy service setup for testing.
 */
void setupDummy(void) {
  dummySvc.begin();

  dummyChar.setProperties(CHR_PROPS_READ);
  dummyChar.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  dummyChar.setFixedLen(1);
  dummyChar.begin();

  uint8_t data = 7;
  dummyChar.write(&data, 1);
}

/**
 * Creates BLE services and registers them to the BLE manager.
 */
void bleCreateSvcs(void) {
  // Configure and start Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start battery service
  Serial.println("Configuring the Battery Service");
  blebas.begin();

  // Configure and start UART service
  bleuart.begin();
  bleuart.setRxCallback(uart_rx_cb);

  Serial.println("Configuring the dummy Service");
  setupDummy();

  BLEBatSvcUpdate();
}

void BLEStart(void (*conn_cb)(uint16_t), 
              void (*disconn_cb)(uint16_t, uint8_t), 
              void (*rssi_cb)(uint16_t, int8_t),
              void (*uart_rx_cb)(uint16_t)) {
  connectCb = conn_cb;
  disconnectCb = disconn_cb;
  rssiCb = rssi_cb;
  uartRxCb = uart_rx_cb;
  
  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // turn off blue LED
  Bluefruit.autoConnLed(false);

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'BluebaruKey'");
  Bluefruit.setName("BluebaruKey");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(conn_cb);
  Bluefruit.Periph.setDisconnectCallback(disconn_cb);

  // Set up Rssi changed callback
  Bluefruit.setRssiCallback(rssi_cb);

  bleCreateSvcs();
  BLEStartAdvertising();
}


void BLEDisconnect(uint16_t conn_handle) {
  BLEConnection *connection;

  connection = Bluefruit.Connection(conn_handle);
  connection->disconnect();
}

size_t BLEUartGetBytesAvail() {
  return bleuart.available();
}

int BLEUartRead(uint8_t *buf, size_t len) {
  return bleuart.read(buf, len);
}

size_t BLEUartWrite(uint8_t *data, size_t len) {
  return bleuart.write(data, len);
}

void BLEBatSvcUpdate(void) {
  // Get a raw ADC reading 
  int vbat_mv = readVBAT();
  
  // Convert from raw mv to percentage (based on LIPO chemistry) 
  uint8_t vbat_per = mvToPercent(vbat_mv);

  Serial.print("Bat percent: ");
  Serial.println(vbat_per);
  Serial.print("Bat mV: ");
  Serial.println(vbat_mv);

  blebas.write(vbat_per);
}

void BLEStartAdvertising(void) {
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

void BLEStopAdvertising(void) {
  Bluefruit.Advertising.stop(); 
}

void BLERssiMonitorStart(uint16_t conn_handle) {
  BLEConnection *connection = Bluefruit.Connection(g_conn_handle);
  
  // Start monitoring rssi of this connection
  // This function should be called in connect callback
  // Input argument is value difference (to current rssi) that triggers callback
  connection->monitorRssi(5);
}

void BLERssiMonitorStop(uint16_t conn_handle) {
  BLEConnection *connection = Bluefruit.Connection(g_conn_handle);

  connection->stopRssi();
}
