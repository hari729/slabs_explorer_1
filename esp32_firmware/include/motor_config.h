#ifndef ESP32_FIRMWARE_MOTOR_CONFIG_H
#define ESP32_FIRMWARE_MOTOR_CONFIG_H

/**
 * @file motor_config.h
 * @brief Runtime motor configuration for the ESP32 firmware.
 *
 * Values are stored in NVS so they persist across power cycles. A host can
 * update them at runtime via the ConfigCmd / ConfigReport protocol.
 */

#include <cstdint>

#include "protocol.h"

namespace motor_config
{

static constexpr uint16_t kVersion = 1;
static constexpr uint32_t kMagic = 0x534C4243U;  // "SLBC"

static constexpr int kLeft = 0;
static constexpr int kRight = 1;
static constexpr int kFwd = 0;
static constexpr int kRev = 1;

// In-memory representation mirrors the wire ConfigCmd.
using RobotConfig = esp32_protocol::ConfigCmd;

/**
 * @brief Compiled defaults matching the original hardcoded constants.
 */
const RobotConfig & compiled_defaults();

/**
 * @brief Validate a config and clamp all fields to sane ranges.
 * @return true if the config was valid (possibly after clamping),
 *         false if it was non-finite or structurally unusable.
 */
bool validate_and_clamp(RobotConfig & cfg);

/**
 * @brief Load config from NVS. Falls back to compiled_defaults on any error.
 */
bool load(RobotConfig & cfg);

/**
 * @brief Save config to NVS.
 */
bool save(const RobotConfig & cfg);

/**
 * @brief Print the current config to Serial (human-readable).
 */
void print(const RobotConfig & cfg);

}  // namespace motor_config

#endif  // ESP32_FIRMWARE_MOTOR_CONFIG_H
