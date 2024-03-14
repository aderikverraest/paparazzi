//
// Created by maximecapelle on 7-3-24.
//

#ifndef MAVLABCOURSE2024_G1_CASCADE_FILTER_H
#define MAVLABCOURSE2024_G1_CASCADE_FILTER_H

// Standard libs
#include "std.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"
#include <stdint.h>

#include "modules/computer_vision/lib/vision/image.h"

        // Global Variables //
extern uint8_t y_min;
extern uint8_t y_max;
extern uint8_t u_min;
extern uint8_t u_max;
extern uint8_t v_min;
extern uint8_t v_max;

extern bool cf_draw;

// Main functions
extern void cascade_filter_init(void);
extern void cascade_filter_periodic(void);
uint32_t image_ground_detector(struct image_t *img, struct image_t *img_out, uint32_t *count_left, uint32_t *count_right, uint16_t *xmin, uint16_t *xmax);

#endif //MAVLABCOURSE2024_G1_CASCADE_FILTER_H
