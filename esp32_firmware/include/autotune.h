#ifndef ESP32_FIRMWARE_AUTOTUNE_H
#define ESP32_FIRMWARE_AUTOTUNE_H

/**
 * @file autotune.h
 * @brief Boot-time motor identification / PI auto-tuner.
 *
 * Called from setup() when the tune-mode pin is pulled LOW at boot.
 * Lifts are mandatory: motors spin up to full duty. After printing results
 * the firmware halts and waits for a power cycle.
 */

void run_autotune();

#endif  // ESP32_FIRMWARE_AUTOTUNE_H
