#ifdef esp32mcu

#include <Arduino.h>
#include <driver/ledc.h>
#include "mcu.hpp"

const int PWM_WHITE_GPIO = 18;
const int PWM_WARM_GPIO = 19;
const ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
const ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
const int PWM_FREQ = 4000;
const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_16_BIT;

int brightness = 50;
float temperature = 50.0f;

static volatile int mode = 1;
static volatile bool touch_detected = false;
static SemaphoreHandle_t pwm_mutex = NULL;
static SemaphoreHandle_t mode_input_mutex = NULL; 

void touch_sensor_isr(void *arg) {
    touch_detected = true;
}

void setup_pwm() {
  // Configure timer
  ledc_timer_config_t timer_cfg = {
      .speed_mode = PWM_MODE,
      .duty_resolution = PWM_RESOLUTION, // Use the correct constant
      .timer_num = PWM_TIMER,
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

  // Configure channels
  ledc_channel_config_t white_ch_cfg = {
      .gpio_num = PWM_WHITE_GPIO,
      .speed_mode = PWM_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = PWM_TIMER,
      .duty = 0,
      .hpoint = 0
  };
  
  ledc_channel_config_t warm_ch_cfg = {
      .gpio_num = PWM_WARM_GPIO,
      .speed_mode = PWM_MODE,
      .channel = LEDC_CHANNEL_1,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = PWM_TIMER,
      .duty = 0,
      .hpoint = 0
  };
  
  ESP_ERROR_CHECK(ledc_channel_config(&white_ch_cfg));
  ESP_ERROR_CHECK(ledc_channel_config(&warm_ch_cfg));
}

void pwm_control_task(void *pvParameters) {
  while (1) {
    // Get current values (atomic read)
    int current_brightness, current_temperature;
    xSemaphoreTake(pwm_mutex, portMAX_DELAY);
    current_brightness = brightness;
    current_temperature = temperature;
    xSemaphoreGive(pwm_mutex);

    // Calculate duty cycles (16-bit values)
    float temp_factor = current_temperature / 100.0;
    float bright_factor = current_brightness / 100.0;
    
    uint32_t white_duty = (uint32_t)((1.0 - temp_factor) * bright_factor * 65535);
    uint32_t warm_duty = (uint32_t)(temp_factor * bright_factor * 65535);

    // Set PWM duties
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, white_duty);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, warm_duty);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);

    vTaskDelay(pdMS_TO_TICKS(10)); // Update every 10ms
  }
}

void input_task(void *pvParameters) {
    while (1) {
        if (touch_detected) {
            xSemaphoreTake(pwm_mutex, portMAX_DELAY);
            mode = (mode % 5) + 1;
            touch_detected = false;
            xSemaphoreGive(pwm_mutex);
        }

        int current_mode;
        xSemaphoreTake(pwm_mutex, portMAX_DELAY);
        current_mode = mode;
        xSemaphoreGive(pwm_mutex);

        int new_brightness, new_temperature;
        handle_mode_input(current_mode, &new_brightness, &new_temperature);

        xSemaphoreTake(pwm_mutex, portMAX_DELAY);
        brightness = new_brightness;
        temperature = new_temperature;
        xSemaphoreGive(pwm_mutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int read_brightness() {
    // TODO: Implement actual brightness reading logic
    // For now, returning a default value
    return 50;  // Default 50% brightness
}

int read_temperature() {
    // TODO: Implement actual temperature reading logic
    // For now, returning a default value
    return 50;  // Default 50% temperature
}

void handle_mode_input(int current_mode, int *new_brightness, int *new_temperature) {
    switch (current_mode) {
        case 1: // e.g., Touch mode
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
        case 2: // e.g., Bluetooth mode
            // Placeholder: Implement Bluetooth input logic
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
        case 3: // e.g., WiFi mode
            // Placeholder: Implement WiFi input logic
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
        case 4: // e.g., Mode 4
            // Placeholder: Implement other input logic
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
        case 5: // e.g., Mode 5
            // Placeholder: Implement other input logic
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
        default:
            *new_brightness = read_brightness();
            *new_temperature = read_temperature();
            break;
    }
}
#endif