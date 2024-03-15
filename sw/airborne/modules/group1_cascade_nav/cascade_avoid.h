//
// Created by maximecapelle on 7-3-24.
//

#ifndef MAVLABCOURSE2024_CASCADE_AVOID_H
#define MAVLABCOURSE2024_CASCADE_AVOID_H



// AVOID SETTINGS
extern float floor_color_count_frac;
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]

extern void cascade_avoid_init(void);
extern void cascade_avoid_periodic(void);


#endif //MAVLABCOURSE2024_CASCADE_AVOID_H
