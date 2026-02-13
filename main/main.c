#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "esp_adc/adc_oneshot.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (9)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits

//PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 50 Hz for a 20ms period.
//minimum and maximum servo pulse widths
#define LEDC_DUTY_MIN           (220) // Set duty to 2.7% (0 deg angle position)
#define LEDC_DUTY_MAX           (590) // Set duty to 7.2% to achieve an angle of 90% (max)
//step sized to change how fast the servo motor rotates
#define STEP_HIGH_SPEED      (6.1) //speed fast -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED       (2.46) //speed slow -- 90 deg in 1.5 sec

#define MODE_SELECTOR     ADC_CHANNEL_4 //MUST BE ADC CHANNEL
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12

static void example_ledc_init(void);

void app_main(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 3.75% (0 degrees)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    /* ---Initalize ADC--- */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_oneshot_config_channel(adc1_handle, MODE_SELECTOR, &chan_config);     // Configure the chan

    int modeSel_adc_bits;                                   // ADC reading (bits)

    while(1) {

        adc_oneshot_read(adc1_handle, MODE_SELECTOR, &modeSel_adc_bits);    // Read ADC bits

        // read from Mode Selector potentiometer & determine the selected mode
        if (modeSel_adc_bits<1024) {
            // MODE SELECTED: OFF
            // printf("OFF\n");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        } else if (modeSel_adc_bits<2048) {
            // MODE SELECTED: INT
            // printf("INT\n");
            //go from 0 to 90 in LOW SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in LOW SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(1000 /portTICK_PERIOD_MS); //user selected value
        } else if (modeSel_adc_bits<3072) {
            // MODE SELECTED: LOW
            //go from 0 to 90 in LOW SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in LOW SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        } else {
            // MODE SELECTED: HIGH
            // printf("HIGH\n");
            //go from 0 to 90 in HIGH SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_HIGH_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in HIGH SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_HIGH_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        }
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