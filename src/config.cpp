#include "config.h"


struct node_calib_t {
    float*    lut;
    float     zero_electric_angle;
    Direction direction;
};

// Cal values
static const node_calib_t NODE_CALIB[8] = {
    /* 0 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 1 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 2 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 3 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 4 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 5 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 6 */ { nullptr, NOT_SET, Direction::UNKNOWN },
    /* 7 */ { nullptr, NOT_SET, Direction::UNKNOWN },
};

bool get_config(foc_node_cfg* cfg) {
    cfg->motor_id = read_node_id();

    const node_calib_t& c = NODE_CALIB[cfg->motor_id & 0x07];

    cfg->calibrationLut      = c.lut;                // stays nullptr when absent
    cfg->zero_electric_angle = c.zero_electric_angle; // NOT_SET when absent
    cfg->sensor_direction    = c.direction;           // UNKNOWN when absent

    return cfg->calibrationLut != nullptr;
}

uint8_t read_node_id() {
    pinMode(SEL_1_PIN, INPUT_PULLUP);
    pinMode(SEL_2_PIN, INPUT_PULLUP);
    pinMode(SEL_3_PIN, INPUT_PULLUP);
    delayMicroseconds(50);

    const uint8_t b2 = !digitalRead(SEL_3_PIN);
    const uint8_t b1 = !digitalRead(SEL_2_PIN);
    const uint8_t b0 = !digitalRead(SEL_1_PIN);

    return (uint8_t)((b2 << 2) | (b1 << 1) | b0);
}