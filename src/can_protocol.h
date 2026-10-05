#pragma once
#include <stdint.h>

// CAN id layout: (node_id << 5) | cmd_id, standard 11-bit identifiers.
static constexpr uint8_t FOC_CMD_SET_TARGET = 0x00;  // host -> mcu
static constexpr uint8_t FOC_CMD_STATE      = 0x01;  // mcu  -> host
static constexpr uint8_t FOC_CMD_STATUS     = 0x02;  // mcu  -> host

static constexpr uint8_t FOC_MODE_IDLE     = 0;
static constexpr uint8_t FOC_MODE_VELOCITY = 1;
static constexpr uint8_t FOC_MODE_POSITION = 2;

static constexpr uint8_t FOC_FLAG_ENABLE       = 1u << 0;
static constexpr uint8_t FOC_FLAG_CLEAR_FAULTS = 1u << 1;
static constexpr uint8_t FOC_FLAG_SET_ZERO     = 1u << 2;

static constexpr uint8_t FOC_STATUS_ENABLED = 1u << 0;
static constexpr uint8_t FOC_STATUS_FAULT_A = 1u << 1;
static constexpr uint8_t FOC_STATUS_FAULT_B = 1u << 2;

struct __attribute__((packed)) foc_set_target_t {
  float   target;   // rad (position) or rad/s (velocity)
  int16_t limit;    // position mode: velocity limit in 0.01 rad/s; 0 = firmware default
  uint8_t mode;
  uint8_t flags;
};

struct __attribute__((packed)) foc_state_t {
  float pos;        // continuous multi-turn shaft angle [rad]
  float vel;        // [rad/s]
};
struct __attribute__((packed)) foc_status_t {
  uint8_t  mode;
  uint8_t  flags;
  uint16_t loop_us;
  float    voltage_q;
};
static_assert(sizeof(foc_set_target_t) == 8, "wire size");
static_assert(sizeof(foc_state_t) == 8, "wire size");
static_assert(sizeof(foc_status_t) == 8, "wire size");