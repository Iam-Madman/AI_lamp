#if defined(esp32capture)
#include "secrets.h"
#include "capture/esp32capture.hpp"
#elif defined(esp32cam)
#include "cam/esp32cam.hpp"
#elif defined(esp32mcu)
#include "mcu/esp32mcu.hpp"
#else
#error "No valid configuration"
#endif

void setup() {
#if defined(esp32capture)
    capture_setup();
#elif defined(esp32cam)
    cam_setup();
#elif defined(esp32mcu)
    mcu_setup();
#else
#error "No setup function defined"
#endif
}

void loop() {
#if defined(esp32capture)
    capture_loop();
#elif defined(esp32cam)
    cam_loop();
#elif defined(esp32mcu)
    mcu_loop();
#else
#error "No loop function defined"
#endif
}