/* ULP I2C bit bang BMP-180 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <math.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


const gpio_num_t gpio_led = GPIO_NUM_2;
const gpio_num_t gpio_scl = GPIO_NUM_32;
const gpio_num_t gpio_sda = GPIO_NUM_33;
//const gpio_num_t gpio_builtin = GPIO_NUM_22;


static void init_ulp_program()
{
    rtc_gpio_init(gpio_led);
    rtc_gpio_set_direction(gpio_led, RTC_GPIO_MODE_OUTPUT_ONLY);

    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Set ULP wake up period to T = 1000ms
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 150000);

    /* Start the program */
    err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

}


#define ac1 ((short)ulp_ac1)
#define ac2 ((short)ulp_ac2)
#define ac3 ((short)ulp_ac3)
#define ac4 ((uint16_t)ulp_ac4)
#define ac5 ((uint16_t)ulp_ac5)
#define ac6 ((uint16_t)ulp_ac6)
#define b1 ((short)ulp_b1)
#define b2 ((short)ulp_b2)
#define mb ((short)ulp_mb)
#define mc ((short)ulp_mc)
#define md ((short)ulp_md)

int computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

static void print_status()
{
//    printf("%d ", ac1);
//    printf("%d ", ac2);
//    printf("%d ", ac3);
//    printf("%d ", ac4);
//    printf("%d ", ac5);
//    printf("%d\n", ac6);
//    printf("%d ", b1);
//    printf("%d ", b2);
//    printf("%d ", mb);
//    printf("%d ", mc);
//    printf("%d ", md);
//    printf("\n");

    printf("rawT: %d Address: 0x%04x counter: %d\n", ulp_temp & 0xFFFF, ulp_temp >> 19, ulp_counter & 0xFFFF);
    printf("rawP1/2: %d %d\n", ulp_pressure & 0xFFFF, ulp_pressure2 & 0xFFFF);

    int oversampling= 1;

    int32_t UP= ((((ulp_pressure & 0xFFFF) << 8) | (ulp_pressure2 & 0xFFFF)) >> (8-oversampling));
    int32_t B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    float temp;
    B5 = computeB5(ulp_temp & 0xFFFF);
    temp = (B5+8) >> 4;
    temp /= 10;
    printf("\nTemperature: %.2f\n", temp);

    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + (int32_t)3791)>>4);
    float pressure = p, altitude_meters= 3;
    printf("Pressure: %.2f\n\n", (pressure / pow(1.0-altitude_meters/44330, 5.255))/100.0);
}

void app_main()
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
    } else {

    	printf("ULP wakeup, printing status\n");
        print_status();
    }

    printf("Entering deep sleep\n\n");

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );

    esp_deep_sleep_start();
}
