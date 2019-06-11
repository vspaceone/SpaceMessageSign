/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define GPIO_DATA    18
#define GPIO_CLOCK    19
#define GPIO_LATCH    21
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_DATA) | (1ULL<<GPIO_CLOCK) | (1ULL<<GPIO_LATCH) )


#define DATA_H    gpio_set_level(GPIO_DATA, 1);
#define DATA_L    gpio_set_level(GPIO_DATA, 0);

#define CLOCK_H    gpio_set_level(GPIO_CLOCK, 1);
#define CLOCK_L    gpio_set_level(GPIO_CLOCK, 0);

#define LATCH_H    gpio_set_level(GPIO_LATCH, 1);
#define LATCH_L    gpio_set_level(GPIO_LATCH, 0);


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define REFRESH_RATE   (0.0005)   // sample test interval for the second timer
#define TIMER_INTERVAL0_SEC (3.4179) // sample test interval for the first timer

#define LED_MATRIX_COLOR_NONE 0x0
#define LED_MATRIX_COLOR_RED 0x1
#define LED_MATRIX_COLOR_GREEN 0x2
#define LED_MATRIX_COLOR_ORANGE 0x3

extern const uint8_t flash_font_5x7[];
extern const uint8_t flash_font_hd44780[];

volatile uint32_t displayBuffer[200];

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR refresh_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;


    // UNSER SHIT


    uint_fast8_t mask = 0;

    static uint_fast8_t row = 0;

    row++;

    // Latch last line from shift register
    LATCH_H;
    LATCH_L;

    mask = 1<<(row-1);
    for (uint8_t i = 0; i < 200; i++){

        DATA_L;

        // Set high with a rising edge, don't setting low may be faster
        if (displayBuffer[i] & mask){
        DATA_H;
        }
        // Shift the register
        CLOCK_L;
        CLOCK_H;


    }

    DATA_L;

    if (row >= 7){
        row = 0;
    }

    // END UNSER SHIT


    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        TIMERG0.int_clr_timers.t1 = 1;
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void refresh_timer_init(int timer_idx, 
    double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, refresh_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

void glcd_draw_string(const char *s, uint16_t x, uint16_t y, uint8_t color)
{
    uint8_t c=0;
    uint8_t i=0;
    uint8_t temp=0;
    uint8_t width=0;

    uint8_t h = 0;
    uint8_t w = 0;
    uint8_t d = 0;
        

    c=*s;
    i=x;

    while (c != '\0')
    {
        for (w = 0; w < 6; w++)
        {
        temp = flash_font_5x7[(c*6)+w];
        if(color == 1)
        {
        displayBuffer[(i+w)*2] = 0;
        displayBuffer[(i+w)*2+1] = temp;
        }
        else if(color == 2)
        {
        displayBuffer[(i+w)*2] = temp;
        displayBuffer[(i+w)*2+1] = temp;
        }
        else
        {
        displayBuffer[(i+w)*2] = temp;
        displayBuffer[(i+w)*2+1] = 0;

        }
        }

        i+=6;
        c=*++s;
    }
}

void gpio_init(){
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void display_init(){

    CLOCK_H;
    DATA_L;
    LATCH_L;

    for(uint_fast8_t i = 0; i < 200; i++){ //200
        displayBuffer[i] = 0;
        DATA_L;
        CLOCK_L;
        //_NOP();
        //_NOP();
        CLOCK_H;
    }

    for(uint_fast8_t i = 0; i < 7; i++){
        displayBuffer[199-i] = (1<<i);
        //displayBuffer[i] = (1<<i);
    }
}

static void timer_example_evt_task(void *arg)
{
    // Block for 500ms.  
    const TickType_t xDelay = 480 / portTICK_PERIOD_MS;
    uint32_t animationPosition = 0;
    const uint32_t animationLength = 4;

    char animation[4] = "-/|\\";
    char animText[2];

    animText[1] = "\0";

    while (1) {

        animText[0] = animation[animationPosition];

        animationPosition++;
        if (animationPosition >= animationLength){
            animationPosition = 0;
        }

        glcd_draw_string(animText, 95 - 15, 0, LED_MATRIX_COLOR_GREEN);

        vTaskDelay(xDelay);
    }
}

/*
 * In this example, we will test hardware timer0 and timer1 of timer group0.
 */
void app_main()
{
    gpio_init();
    display_init();
    glcd_draw_string("vspace rocks!",0,0,LED_MATRIX_COLOR_ORANGE);
    refresh_timer_init(TIMER_1, REFRESH_RATE);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
}

