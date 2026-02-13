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
#define STEP_HIGH_SPEED      (12.2) //speed fast -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED       (4.92) //speed slow -- 90 deg in 1.5 sec

#define MODE_SELECTOR     ADC_CHANNEL_4 //MUST BE ADC CHANNEL
#define DELAY_TIME_SELECTOR     ADC_CHANNEL_3 //MUST BE ADC CHANNEL
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
    adc_oneshot_config_channel(adc1_handle, DELAY_TIME_SELECTOR, &chan_config);     // Configure the chan

    int modeSel_adc_bits;                                   // ADC reading (bits)
    int delayTimeSel_adc_bits;                                   // ADC reading (bits)
    int INTtimeDelay;
    int timeInterval;                                       //helper variable for time delay  
    int state = 0;      // helper var to indicate state: 0 - STOP, 1- WAIT_DELAY_TIME, 2 - MOVE0TO90, 3- MOVE90TO0, 4 -FINISH current cycle to 0
    float duty = LEDC_DUTY_MIN;     //helper var
    float current_step = STEP_LOW_SPEED;     // helper var - speed used for this cycle
    float requested_step = STEP_LOW_SPEED;   // helper var - speed knob position

    while(1) {

        adc_oneshot_read(adc1_handle, MODE_SELECTOR, &modeSel_adc_bits);    // Read ADC bits
        adc_oneshot_read(adc1_handle, DELAY_TIME_SELECTOR, &delayTimeSel_adc_bits);    // Read ADC bits

        bool off_selected = (modeSel_adc_bits < 1024);     // OFF or engine off
        bool int_selected = (modeSel_adc_bits >= 1024 && modeSel_adc_bits < 2048);
        bool low_selected = (modeSel_adc_bits >= 2048 && modeSel_adc_bits < 3072);
        bool high_selected = (modeSel_adc_bits >= 3072);

        //determine the delay time selected by driver
        //0-1364 - 1sec
        if (delayTimeSel_adc_bits<1365) { INTtimeDelay=1000;} 
        //1365-2729 - 3 sec
        else if (delayTimeSel_adc_bits<2730) {INTtimeDelay=3000;} 
        //2730-4095 -- 5sec
        else { INTtimeDelay=5000;}

        // read from Mode Selector potentiometer & determine the selected mode

        // OFF or engine off behavior
        if (off_selected) {
            // if (state == 2 || state == 3) {
            //     state = 4;   // finish cycle and return to 0°
            // }
            // else if (state == 1) {
            //     // remain stationary in hesitation
            //     state = 1;
            // }
            // else {
            //     state = 0;     // already parked
            // }
            if (state == 0) {
                state = 0;   // already parked
            } else {
                state = 4;   // ALWAYS finish cycle and park
            }
        }

        // INT mode
        else if (int_selected) {
            if (state == 0) {
                state = 1;
                timeInterval = 0;
            }
        }

        // LOW continuous
        else if (low_selected) {
            if (state == 0) {
                state = 2;
            }
        }

        // HIGH continuous
        else if (high_selected) {
            if (state == 0) {
                state = 2;
            }
        }

        //determine step that you'll use to move wiper up/down
        // float step;
        // if (modeSel_adc_bits>3072){step=STEP_HIGH_SPEED;}
        // else{step=STEP_LOW_SPEED;}
        if (high_selected) {
            requested_step = STEP_HIGH_SPEED;
        } else {
            requested_step = STEP_LOW_SPEED;
        }

        switch (state) {

            case 1:     //wait
                timeInterval += 20;

                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

                if (timeInterval >= INTtimeDelay && !off_selected) {
                    state = 2;
                }
                break;

            case 2://MOVE FROM 0 to 90
                duty += current_step;
                if (duty >= LEDC_DUTY_MAX) {
                    duty = LEDC_DUTY_MAX;
                    state = 3;
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 3: //MOVE FROM 90 to 0
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;

                    //switch speeds only after the previous cycle is complete
                    current_step = requested_step;

                    if (off_selected) {
                    // if (off_selected || state == 4) {
                        state = 0;       // park and stop
                    }
                    else if (int_selected) {
                        state = 1;
                        timeInterval = 0;
                    }
                    else {
                        state = 2;    // continuous LOW/HIGH
                    }
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 4: //FINISH CYCLE
                // always move toward 0°
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;

                    //switch speeds only after the previous cycle is complete
                    current_step = requested_step;

                    state = 0;   // parked
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 0: //STOP
            default:
                // parked and stationary
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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