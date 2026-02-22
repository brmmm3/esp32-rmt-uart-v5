#include <driver/gpio.h>

#include "yys.h"
#include "ze08.h"

#define YYS_CHANNEL         0
#define YYS_PIN_NUM_TX      0  /* 0=DISABLED  GPIO_NUM_12 */
#define YYS_PIN_NUM_RX      GPIO_NUM_13
// ZE08-CH2O (SW-UART1) (Formaldehyde)
#define ZE08_CHANNEL        1
#define ZE08_PIN_NUM_TX     0  /* 0=DISABLED  GPIO_NUM_3 */
#define ZE08_PIN_NUM_RX     GPIO_NUM_11

yys_t *yys_sensor = NULL;
ze08_t *ze08_sensor = NULL;

void app_main(void)
{
    ESP_ERROR_CHECK(yys_init(&yys_sensor, YYS_CHANNEL, YYS_PIN_NUM_RX, YYS_PIN_NUM_TX));
    ESP_ERROR_CHECK(ze08_init(&ze08_sensor, ZE08_CHANNEL, ZE08_PIN_NUM_RX, ZE08_PIN_NUM_TX));
    //yys_sensor->debug = 3;
    //ze08_sensor->debug = 7;
    while (true) {
        if (yys_data_ready(yys_sensor)) {
            yys_dump_values(yys_sensor, true);
        }
        if (ze08_data_ready(ze08_sensor)) {
            ze08_dump_values(ze08_sensor, true);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
