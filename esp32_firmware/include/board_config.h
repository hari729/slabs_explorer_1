#ifndef ESP32_FIRMWARE_BOARD_CONFIG_H
#define ESP32_FIRMWARE_BOARD_CONFIG_H

/**
 * @file board_config.h
 * @brief Shared board-level constants for the ESP32 firmware.
 *
 * Keep these in sync with the actual wiring. They are used by both the
 * normal control loop and the boot-time autotuner.
 */

// Motor driver pins
static const int LEFT_PWM_PIN = 19;
static const int LEFT_DIR_PIN = 18;
static const int RIGHT_PWM_PIN = 26;
static const int RIGHT_DIR_PIN = 25;

// Encoder pins
static const int RIGHT_ENC_A_PIN = 32;
static const int RIGHT_ENC_B_PIN = 33;
static const int LEFT_ENC_A_PIN = 34;
static const int LEFT_ENC_B_PIN = 35;

// Safety / mode pins
static const int ESTOP_PIN = 4;
static const int TUNE_MODE_PIN = 27;  // pull LOW at boot to enter tuning mode

// Encoder direction signs
static const int LEFT_ENCODER_SIGN = -1;
static const int RIGHT_ENCODER_SIGN = 1;

// PWM setup
static const int PWM_FREQUENCY_HZ = 20000;
static const int PWM_RESOLUTION_BITS = 10;
static const int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
static const int LEFT_PWM_CHANNEL = 0;
static const int RIGHT_PWM_CHANNEL = 1;

/**
 * @brief Drive a single motor with signed duty.
 * @param pwm_channel LEDC channel for the PWM pin.
 * @param dir_pin Direction pin.
 * @param duty Signed duty in [-PWM_MAX_DUTY, PWM_MAX_DUTY].
 */
void set_motor_pwm(int pwm_channel, int dir_pin, int duty);

#endif  // ESP32_FIRMWARE_BOARD_CONFIG_H
