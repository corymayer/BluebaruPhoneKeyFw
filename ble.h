#include <stdio.h>
#include <stdint.h>

/**
 * Configures BLE, creates the services, and starts advertising.
 */
void BLEStart(void (*conn_cb)(uint16_t), 
              void (*disconn_cb)(uint16_t, uint8_t), 
              void (*rssi_cb)(uint16_t, int8_t),
              void (*uart_rx_cb)(uint16_t));

/**
 * Disconnects a BLE connection.
 * @param conn_handle connection handle
 */
void BLEDisconnect(uint16_t conn_handle);

/**
 * Gets the current bytes available to read. Must be actively connected.
 * @return number of bytes available
 */
size_t BLEUartGetBytesAvail();

/**
 * BLE UART read
 * @param buffer to read into
 * @param len number of bytes to read
 * @return n bytes read
 */
int BLEUartRead(uint8_t *buf, size_t len);

/**
 * BLE UART write
 * @param data data to write
 * @param len number of byte to write
 * @return n bytes written
 */
size_t BLEUartWrite(uint8_t *data, size_t len);

/**
 * Gets the current battery level and updates the battery service with the new value.
 */
void BLEBatSvcUpdate(void);

/**
 * Starts advertising.
 */
void BLEStartAdvertising(void);

/**
 * Stops advertising.
 */
void BLEStopAdvertising(void);

/**
 * Starts RSSI monitoring on the active connection.
 */
void BLERssiMonitorStart(uint16_t conn_handle);

/**
 * Stops RSSI monitoring on the active connection
 */
void BLERssiMonitorStop(uint16_t conn_handle);
