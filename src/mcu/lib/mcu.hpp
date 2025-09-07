#ifdef esp32mcu

#include <Arduino.h>
#include <driver/ledc.h>

extern int brightness;
extern float temperature;
extern SemaphoreHandle_t pwm_mutex;

int read_brightness();
int read_temperature();
// Function declarations
void touch_sensor_isr(void *arg);
void setup_pwm();
void pwm_control_task(void *pvParameters);
void input_task(void *pvParameters);
void handle_mode_input(int current_mode, int *new_brightness, int *new_temperature);
#endif