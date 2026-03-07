# ESP32 YYS sensor component for ESP-IDF providing CO, O2, H2S and CH4

Component for Espressif ESP32 ESP-IDF framework.

This component uses the subcomponent esp32_rmt_uart_v5 for receiving the data from the YYS sensor. Because of this dependency this component only works for ESP-IDF v5.x.

The communication protocol is as follows:

![protocol](doc/image.png)

## How to Use

Clone this repository to your project components directory.

## Configuration

```c
typedef struct yys_sensor_s {
    char *name;                  /*!< Optional name of this sensor */
    uint8_t *buffer;             /*!< Buffer for received sensor data */
    int baudrate;                /*!< baud rate, normally 9600 */
    uint8_t rx_pin;              /*!< Receive I/O pin */
    uint8_t cnt;                 /*!< Internal bit receive counter */
    uint16_t co;                 /*!< Latest raw CO value */
    uint16_t o2;                 /*!< Latest raw O2 value */
    uint16_t h2s;                /*!< Latest raw H2S value */
    uint16_t ch4;                /*!< Latest raw CH4 value */
    uint16_t error_cnt;          /*!< Receive error counter */
    uint8_t data_cnt;            /*!< Receive data counter */
    bool data_ready;             /*!< New data available flag */
    uint8_t debug;               /*!< Bitmask for debugging */
} yys_sensor_t;
```

Every second 1 update is sent by the sensor.
