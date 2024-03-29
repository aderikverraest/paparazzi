//
// Created by daan on 12-3-24.
//

#ifndef GROUP_1_CV_H
#define GROUP_1_CV_H

#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

extern uint8_t y_min;
extern uint8_t y_max;
extern uint8_t u_min;
extern uint8_t u_max;
extern uint8_t v_min;
extern uint8_t v_max;
extern int edge_threshold;

void assign_kernel_values(int_fast8_t const *kernel_r, int_fast8_t *kernel);
void generate_kernel (struct FloatEulers const *angles, int_fast8_t *kernel);
void unaligned_sum(struct image_t *input, int *output, struct FloatEulers *angles, int axis);
void heading_command(const int* input, uint16_t start, uint16_t end, int *xMin, int *xMax);
void downsample_yuv422(struct image_t* input, struct image_t* output, uint8_t downsample_factor);
u_int32_t find_max_y(struct image_t* input, uint16_t* output);
void heading_command_v2(const int* edge_input, const u_int16_t* y_input, uint16_t start, uint16_t end, int *xMin, int *xMax, int*maxGreen);
void fill_green_below_max(struct image_t* image, const u_int16_t* y_input);
uint32_t image_ground_detector(struct image_t *img, struct image_t *img_out, uint32_t *count_left, uint32_t *count_right, uint16_t *xmin, uint16_t *xmax);
void threshold_img(struct image_t *img_in, struct image_t *img_out, uint8_t threshold);
void avg_pool(struct image_t *img, struct image_t *img_out, uint8_t threshold, uint8_t n_iterations);
struct image_t *image_ground_filler(struct image_t *img);


#endif //GROUP_1_CV_H
