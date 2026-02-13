#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits

//Set the PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 50 Hz for a 20ms period.

//Calculate the values for the minimum (0.75ms) and maximum (2.25) servo pulse widths
#define LEDC_DUTY_MIN           (220) // Set duty to 2.6% (0 deg angle position)
#define LEDC_DUTY_MAX           (590) // Set duty to 7.56% to achieve an angle of 90% (max)

#define STEP_HIGH_SPEED      (6.1) //or 6 -- speed fast -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED       (2.46) //or 3 -- speed slow -- 90 deg in 1.5 sec

static void example_ledc_init(void);

void app_main(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 3.75% (0 degrees)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

 while(1) {

    //go from 0 to 90 in 0.6s
    for (int i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_HIGH_SPEED) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(10 /portTICK_PERIOD_MS);    
    }
    
    // vTaskDelay(1000 /portTICK_PERIOD_MS);    

    // go from 90 to 0 in 0.6s
    for (int i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_HIGH_SPEED) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(10 /portTICK_PERIOD_MS);
    }

    // vTaskDelay(1000 /portTICK_PERIOD_MS);    
 }
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set initial duty to 0%
        .hpoint         = 0
    };
ledc_channel_config(&ledc_channel);
}