#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <driver/timer.h>

#include <queue>

#include <iostream>
using std::cout;
using std::endl;

static const char *TAG = "OUTLET";

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

extern "C"
{
  void app_main()
  {
    esp_adc_cal_characteristics_t *adc_chars = new (esp_adc_cal_characteristics_t);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, adc_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11));

    // clock = 80Mhz; sampling at 120Hz would result in non integer number of ticks (80/120 = 2/3), so we will sample at 160Hz instead
    // 80Mhz/160Hz = 500,000 for divider, however divider max is about 65,000 so we will divide by 50,000 instead which yields 1.6Khz, therefore 1.6kHz/160Hz = 10 ticks
    // ISR will execute every 10 ticks to sample at 160Hz
    timer_config_t config = {TIMER_ALARM_EN, TIMER_START, TIMER_INTR_MAX, TIMER_COUNT_UP, TIMER_AUTORELOAD_EN, 50000};

    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
   Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

    timer_start(TIMER_GROUP_0, TIMER_0);

    TickType_t xLastWakeTime;

    // 100Hz seems to be max frequency
    const TickType_t xFrequency = pdMS_TO_TICKS((1 / 60) * 1000);

    BaseType_t xWasDelayed;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    float runningSumApparentPower = 0;

    for (;;)
    {
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