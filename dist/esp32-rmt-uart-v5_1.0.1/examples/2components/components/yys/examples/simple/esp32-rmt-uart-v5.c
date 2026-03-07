#include "driver/gpio.h"

#include "yys.h"

#define YYS_PIN_NUM_NC      GPIO_NUM_11
#define YYS_PIN_NUM_TX      GPIO_NUM_12
#define YYS_PIN_NUM_RX      GPIO_NUM_13

yys_sensor_t *yys_sensor = NULL;

void app_main(void)
{
    ESP_ERROR_CHECK(yys_init(&yys_sensor, YYS_PIN_NUM_RX, YYS_PIN_NUM_TX));
    yys_sensor->debug = 1;

    while (true) {
        if (yys_data_ready(yys_sensor)) {
            yys_dump(yys_sensor);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
