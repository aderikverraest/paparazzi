//
// Created by maximecapelle on 7-3-24.
//
//#define ORANGE_AVOIDER_VERBOSE 1
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
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS,
    REENTER_ARENA
};

float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
float oag_max_speed;         // max flight speed [m/s]
float oag_heading_rate;

// DECLARE FUNCTIONS
uint8_t chooseRandomIncrementAvoidance(void);

// AVOID SETTINGS
float floor_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float cf_max_speed = 1.0;               // max flight speed [m/s]
float speed_sp;
float cf_heading_rate = RadOfDeg(15);
float cf_max_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free


// INITIALIZE GLOBAL VARIABLES
enum navigation_state_t navigation_state = SAFE;   // current state in state machine
int32_t floor_count = 0;                // orange color count from color filter for obstacle detection
float nav_command = 0;
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
uint32_t pixel_width = 0;
uint32_t pixel_y = 0;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.


// CHANNEL TO RECEIVE INFO FROM CASCADE FILTER
#ifndef CASCADE_FILTER_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event cascade_filter_ev;
static void cascade_filter_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    nav_command = pixel_x;
    floor_count = quality;
}

// INIT FUNCTION
void cascade_avoid_init(void)
{
    VERBOSE_PRINT("IN CASCADE AVOID INIT");
    // Initialise random values
    srand(time(NULL));
    guidance_h_set_pos(0, 0);
    chooseRandomIncrementAvoidance();

    // bind our cascade filter callbacks to receive the cascade filter outputs
    AbiBindMsgVISUAL_DETECTION(CASCADE_FILTER_ID, &cascade_filter_ev, cascade_filter_cb);
}


// PERIODIC FUNCTION
void cascade_avoid_periodic(void)
{
    VERBOSE_PRINT("IN CASCADE AVOID PERIODIC");
    // Only run the module if we are in the correct flight mode
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        navigation_state = SAFE;
        obstacle_free_confidence = 0;
        return;
    }

    // compute current color thresholds
//    int32_t floor_count_threshold = floor_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
    int32_t floor_count_threshold = 1500;
    int32_t floor_count_threshold_reenter = 3000;
    fprintf(stderr, "Floor Count: %d  Threshold: %d State: %d \n", floor_count, floor_count_threshold, navigation_state);

    // Setting the flight speed
    if (nav_command <= 20) {
        speed_sp = cf_max_speed;
    } else {
        speed_sp = 0;
    }


    switch (navigation_state){
        case SAFE:
            if (floor_count < floor_count_threshold){
                navigation_state = OUT_OF_BOUNDS;
            }
            else {
                // Steering plus forward speed
                guidance_h_set_heading_rate((nav_command/100)*cf_max_heading_rate);
                fprintf(stderr, "Speed: %f Heading Rate: %f \n", speed_sp, (nav_command/100)*cf_max_heading_rate);
                guidance_h_set_body_vel(speed_sp, 0);
            }

            break;
//        case OBSTACLE_FOUND:
//            // stop
//            guidance_h_set_body_vel(0, 0);
//
//            // randomly select new search direction
//            chooseRandomIncrementAvoidance();
//
//            navigation_state = SEARCH_FOR_SAFE_HEADING;
//
//            break;
        case SEARCH_FOR_SAFE_HEADING:
            guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
                navigation_state = SAFE;
            }
            break;
        case OUT_OF_BOUNDS:
            chooseRandomIncrementAvoidance();
            // stop
            guidance_h_set_body_vel(0, 0);

            // start turn back into arena
            guidance_h_set_heading_rate(avoidance_heading_direction * cf_heading_rate);

            navigation_state = REENTER_ARENA;

            break;
        case REENTER_ARENA:
//             force floor center to opposite side of turn to head back into arena
            if (floor_count >= floor_count_threshold_reenter) {
                // return to heading mode
                guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

                // ensure direction is safe before continuing
                navigation_state = SAFE;
            }
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
    if (pixel_width > pixel_y) {
        avoidance_heading_direction = 1.f;
    } else {
        avoidance_heading_direction = -1.f;
    }
//    if (rand() % 2 == 0) {
//        avoidance_heading_direction = 1.f;
//
//        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
//    } else {
//        avoidance_heading_direction = -1.f;
//        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
//    }
    return false;
}






