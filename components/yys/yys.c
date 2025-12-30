#include <string.h>
#include "esp_err.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rmt_uart.h"
#include "yys.h"

static const char *TAG = "YYS";

// UART
#define UART_BUFFER_SIZE 256

static esp_timer_handle_t yys_message_end_timer;

void yys_check_and_save_data(yys_sensor_t *sensor)
{
    uint8_t *buf = sensor->buffer;
    uint8_t cksum = 0x3c + 0x04 + buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8];

    if (sensor->debug & 2) {
        ESP_LOGI(sensor->name, "#CNT %d", sensor->cnt);
    }
    if (cksum == buf[9]) {
        sensor->o2 = (uint16_t)buf[0] << 8 | (uint16_t)buf[1];
        sensor->co = (uint16_t)buf[2] << 8 | (uint16_t)buf[3];
        sensor->h2s = (uint16_t)buf[4] << 8 | (uint16_t)buf[5];
        sensor->ch4 = (uint16_t)buf[6] << 8 | (uint16_t)buf[7];
        sensor->data_cnt++;
        sensor->data_ready = true;
    } else if (sensor->debug & 4) {
        ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, 9, ESP_LOG_ERROR);
        ESP_LOGE(sensor->name, "#CK %02X %02X", cksum, buf[9]);
    }
    sensor->cnt = 0xff;
}

void yys_message_end_timeout(void *args)
{
    yys_check_and_save_data((yys_sensor_t *)args);
}

static void rx_task_yys_sensor(void *args)
{
    yys_sensor_t *sensor = (yys_sensor_t *)args;

    uint8_t rx_pin = sensor->rx_pin;
    uint8_t *buf = sensor->buffer;
    static uint8_t last_byte = 0;

    rmt_uart_config_t uart_config = {
        .baud_rate = 9600,                  // Your baud rate
        .mode = RMT_UART_MODE_RX_ONLY,      // Receiver only
        .data_bits = RMT_UART_DATA_8_BITS,
        .parity = RMT_UART_PARITY_DISABLE,
        .stop_bits = RMT_UART_STOP_BITS_1,
        .rx_io_num = rx_pin,                // Your RX GPIO pin
        .buffer_size = 64                   // RX buffer size (bytes)
    };

    ESP_ERROR_CHECK(rmt_uart_init(0, &uart_config));

    uint8_t rx_buf[32];
    size_t length = 0;

    // Start timeout timer if message end is not detected
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &yys_message_end_timeout,
        .arg = args,
        .name = "yys_message_end_timeout",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    // Fire timeout after 100ms
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &yys_message_end_timer));

    while (true) {
        length = rmt_uart_read_bytes(0, rx_buf, sizeof(rx_buf), portMAX_DELAY);
        uint8_t i = 0;
        while (length-- > 0) {
            uint8_t rx_byte = rx_buf[i++];

            if (sensor->debug & 2) {
                ESP_LOGI(sensor->name, "#%d RXB=%02X", sensor->cnt, rx_byte);
            }
            if (sensor->cnt < 16) {
                buf[sensor->cnt++] = rx_byte;
            } else if (last_byte == 0x3c && rx_byte == 0x04) {
                // START sequence received
                sensor->cnt = 0;
                ESP_ERROR_CHECK(esp_timer_start_once(yys_message_end_timer, 100000));
            }
            last_byte = rx_byte;
        }
        if (sensor->cnt == 10) {
            esp_timer_stop(yys_message_end_timer);
            //esp_timer_delete(yys_message_end_timer);
            yys_check_and_save_data(sensor);
            last_byte = 0xff;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

esp_err_t yys_init(yys_sensor_t **sensor, uint8_t rx_pin, uint8_t tx_pin)
{
    yys_sensor_t *yys_sensor = calloc(1, sizeof(yys_sensor_t));

    ESP_LOGI("YYS", "Initialize YYS");
    // YYS Multi Sensor
    yys_sensor->name = "YYS";
    yys_sensor->baudrate = 9600;
    yys_sensor->rx_pin = rx_pin;
    yys_sensor->buffer = malloc(16);
    yys_sensor->cnt = 0xff;
    *sensor = yys_sensor;
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor", 4096, (void *)yys_sensor, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI("YYS", "YYS initialized");
    return ESP_OK;
}

bool yys_data_ready(yys_sensor_t *sensor)
{
    if (sensor->data_ready) {
        sensor->data_ready = false;
        return true;
    }
    return false;
}

uint16_t yys_get_co_raw(yys_sensor_t *sensor)
{
    return sensor->co;
}

uint16_t yys_get_o2_raw(yys_sensor_t *sensor)
{
    return sensor->o2;
}

uint16_t yys_get_h2s_raw(yys_sensor_t *sensor)
{
    return sensor->h2s;
}

uint16_t yys_get_ch4_raw(yys_sensor_t *sensor)
{
    return sensor->ch4;
}

float yys_get_co(yys_sensor_t *sensor)
{
    return (float)sensor->co;  // e.g. 6 = 6ppm
}

float yys_get_o2(yys_sensor_t *sensor)
{
    return (float)sensor->o2 * 0.1;  // e.g. 209 = 20.9%
}

float yys_get_h2s(yys_sensor_t *sensor)
{
    return (float)sensor->h2s * 0.1;  // e.g. 672 = 67.2ppm
}

float yys_get_ch4(yys_sensor_t *sensor)
{
    return (float)sensor->ch4;  // e.g. 6 = 6ppm
}

void yys_dump(yys_sensor_t *sensor)
{
    if (sensor->debug & 1) {
        ESP_LOGI(TAG, "O2=%f %%  CO=%f ppm  H2S=%f ppm  CH4=%f ppm  DCNT=%d  ERR=%d",
                 yys_get_o2(sensor), yys_get_co(sensor),
                 yys_get_h2s(sensor), yys_get_ch4(sensor),
                 sensor->data_cnt, sensor->error_cnt);
    }
}
