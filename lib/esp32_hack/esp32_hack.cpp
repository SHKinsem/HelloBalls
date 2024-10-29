#include <Arduino.h>

void aPinMode(int pinNum, int pinDir) {
  // Enable GPIO32 or 33 as output. 
  if (pinNum == 32 || pinNum == 33) {
    uint64_t gpioBitMask = (pinNum == 32) ? 1ULL<<GPIO_NUM_32 : 1ULL<<GPIO_NUM_33;
    gpio_mode_t gpioMode = (pinDir == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = gpioMode;
    io_conf.pin_bit_mask = gpioBitMask;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
  } else pinMode(pinNum, pinDir);
}