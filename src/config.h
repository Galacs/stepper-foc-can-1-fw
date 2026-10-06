#pragma once
#include <Arduino.h>
#include "pins.h"
#include <SimpleFOC.h>

#define CAL_LUT_LEN 200

struct foc_node_cfg {
    uint8_t motor_id;
    float *calibrationLut;
    float zero_electric_angle;
    Direction sensor_direction;
};

uint8_t read_node_id();
bool get_config(foc_node_cfg* cfg);