#ifdef esp32mcu

#include "lib/mcu.hpp"

void mcu_setup() {
  setup_pwm();
  pwm_mutex = xSemaphoreCreateMutex();
  xTaskCreate(pwm_control_task, "PWM Control", 2048, NULL, 2, NULL);
  xTaskCreate(input_task, "Input Handler", 2048, NULL, 1, NULL);
}

void mcu_loop() {
  vTaskDelete(NULL);
}


#endif