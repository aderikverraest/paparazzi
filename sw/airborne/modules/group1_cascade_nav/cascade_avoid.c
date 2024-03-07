//
// Created by maximecapelle on 7-3-24.
//

#include "modules/group1_cascade_nav/cascade_avoid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define CASCADE_AVOID_VERBOSE TRUE

// Print functions
#define PRINT(string,...) fprintf(stderr, "[CASCADE_AVOID->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


// Define Navigation States
enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS,
    REENTER_ARENA
};

// DECLARE FUNCTIONS
uint8_t chooseRandomIncrementAvoidance(void);

// AVOID SETTINGS
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_max_speed = 0.0f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free


// INITIALIZE GLOBAL VARIABLES
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t nav_command = 0;
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.


// CHANNEL TO RECEIVE INFO FROM CASCADE FILTER
#ifndef CASCADE_FILTER_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event cascade_filter_ev;
static void cascade_filter_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t __attribute__((unused)) quality, int16_t __attribute__((unused)) extra)
{
    nav_command = pixel_x;
    color_count = pixel_y;
}

// INIT FUNCTION
void cascade_avoid_init(void)
{
    // Initialise random values
    srand(time(NULL));
    chooseRandomIncrementAvoidance();

    // bind our cascade filter callbacks to receive the cascade filter outputs
    AbiBindMsgVISUAL_DETECTION(CASCADE_FILTER_ID, &cascade_filter_ev, cascade_filter_cb);
}


// PERIODIC FUNCTION
void cascade_avoid_periodic(void)
{
    // Only run the module if we are in the correct flight mode
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        navigation_state = SEARCH_FOR_SAFE_HEADING;
        obstacle_free_confidence = 0;
        return;
    }

    // compute current color thresholds
    int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
    VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

    // update our safe confidence using color threshold
    if(color_count < color_count_threshold){
        obstacle_free_confidence++;
    } else {
        obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    }

    // bound obstacle_free_confidence
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
    float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

    switch (navigation_state){
        case SAFE:
//            if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
//                navigation_state = OUT_OF_BOUNDS;
//            }
            if (obstacle_free_confidence == 0){
                navigation_state = OBSTACLE_FOUND;
            } else {
                // Fly in circles
                guidance_h_set_heading_rate(0.2);
                guidance_h_set_body_vel(speed_sp, 0);
            }

            break;
        case OBSTACLE_FOUND:
            // stop
            guidance_h_set_body_vel(0, 0);

            // randomly select new search direction
            chooseRandomIncrementAvoidance();

            navigation_state = SEARCH_FOR_SAFE_HEADING;

            break;
        case SEARCH_FOR_SAFE_HEADING:
            guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
                navigation_state = SAFE;
            }
            break;
        case OUT_OF_BOUNDS:
            // stop
            guidance_h_set_body_vel(0, 0);

            // start turn back into arena
            guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

            navigation_state = REENTER_ARENA;

            break;
        case REENTER_ARENA:
            // force floor center to opposite side of turn to head back into arena
//            if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
//                // return to heading mode
//                guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
//
//                // reset safe counter
//                obstacle_free_confidence = 0;
//
//                // ensure direction is safe before continuing
//                navigation_state = SAFE;
//            }
            navigation_state = SAFE;
            break;
        default:
            break;
    }
    return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
    // Randomly choose CW or CCW avoiding direction
    if (rand() % 2 == 0) {
        avoidance_heading_direction = 1.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
    } else {
        avoidance_heading_direction = -1.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
    }
    return false;
}






