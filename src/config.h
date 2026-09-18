#pragma once
#include <Arduino.h>
#include "pins.h"
#include <SimpleFOC.h>

struct foc_node_cfg {
    uint8_t motor_id;
    float calibrationLut[200];
    float zero_electric_angle;
    Direction sensor_direction;
};

bool get_config(foc_node_cfg* cfg);