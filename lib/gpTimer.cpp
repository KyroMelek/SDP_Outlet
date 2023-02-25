#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

static const char *TAG = "OUTLET";

#include <iostream>
using std::cout;
using std::endl;

float zeroVoltageTransformer = 0;
float zeroCurrentTransformer = 0;

struct data
{
  float apparentPower;
  float timeShift;
  time_t now;
};

uint32_t transformerVoltageToRealVoltage(uint32_t voltage)
{
  return voltage;
}

uint32_t transformerCurrentToRealCurrent(uint32_t current)
{
  return current;
}

#define TIMER_DIVIDER (50000)                        //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds

struct ISR_DATA
{
  uint64_t timer_counter_value;
};

static xQueueHandle s_timer_queue;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
  printf("Counter: 0x%08x%08x\r\n", (uint32_t)(counter_value >> 32),
         (uint32_t)(counter_value));
  printf("Time   : %.8f s\r\n", (double)counter_value / TIMER_SCALE);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
  BaseType_t high_task_awoken = pdFALSE;

  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);

  ISR_DATA isr_data = {timer_counter_value};

  /* Now just send the event data back to the main program task */
  xQueueSendFromISR(s_timer_queue, &isr_data, &high_task_awoken);

  return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 */
static void timer_setup_init(int group, int timer)
{
  // clock = 80Mhz; sampling at 120Hz would result in non integer number of ticks (80/120 = 2/3), so we will sample at 160Hz instead
  // 80Mhz/160Hz = 500,000 for divider, however divider max is about 65,000 so we will divide by 50,000 instead which yields 1.6Khz, therefore 1.6kHz/160Hz = 10 ticks
  // ISR will execute every 10 ticks to sample at 160Hz
  timer_config_t config = {TIMER_ALARM_EN, TIMER_START, TIMER_INTR_MAX, TIMER_COUNT_UP, TIMER_AUTORELOAD_EN, 50000};

  timer_init(TIMER_GROUP_0, TIMER_0, &config);

  /* Timer's counter will initially start from value below.
     Also, if auto_reload is set, this value will be automatically reload on alarm */
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);

  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

extern "C"
{
  void app_main(void)
  {
    esp_adc_cal_characteristics_t *adc_chars = new (esp_adc_cal_characteristics_t);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, adc_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11));

    s_timer_queue = xQueueCreate(10, sizeof(ISR_DATA));
    timer_setup_init(TIMER_GROUP_0, TIMER_0);

    float runningSumApparentPower = 0;
    while (1)
    {
      ISR_DATA isr_data;
      xQueueReceive(s_timer_queue, &isr_data, portMAX_DELAY);

      printf("Group[%d], timer[%d] alarm event\n", TIMER_GROUP_0, TIMER_0);

      /* Print the timer values passed by event */
      printf("------- EVENT TIME --------\n");
      print_timer_counter(isr_data.timer_counter_value);

      /* Print the timer values as visible by this task */
      printf("-------- TASK TIME --------\n");
      uint64_t task_counter_value;
      timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
      print_timer_counter(task_counter_value);

      // Wait for the next cycle.
      // xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
      // Perform action here. xWasDelayed value can be used to determine
      // whether a deadline was missed if the code here took too long.

      uint32_t ADCReadingV = adc1_get_raw(ADC1_CHANNEL_5);
      uint32_t voltage = esp_adc_cal_raw_to_voltage(ADCReadingV, adc_chars);
      cout << "Voltage reading is: " << voltage << "mV" << endl;

      uint32_t trueVoltage = transformerVoltageToRealVoltage(voltage);

      uint32_t ADCreadingI = adc1_get_raw(ADC1_CHANNEL_4);
      uint32_t current = esp_adc_cal_raw_to_voltage(ADCreadingI, adc_chars);
      cout << "Current reading is: " << current << "mV" << endl;

      uint32_t trueCurrent = transformerCurrentToRealCurrent(current);

      float apparentPower = trueVoltage * trueCurrent;
      runningSumApparentPower += apparentPower;
    }
  }
}