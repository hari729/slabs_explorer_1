#include "board_config.h"

#include <Arduino.h>

void set_motor_pwm(int pwm_channel, int dir_pin, int duty) {
  duty = constrain(duty, -PWM_MAX_DUTY, PWM_MAX_DUTY);

  if (duty >= 0) {
    digitalWrite(dir_pin, LOW);
    ledcWrite(pwm_channel, duty);
  } else {
    digitalWrite(dir_pin, HIGH);
    ledcWrite(pwm_channel, PWM_MAX_DUTY + duty);
  }
}
