//
// Created by daan on 12-3-24.
//

#ifndef GROUP_1_CV_H
#define GROUP_1_CV_H

#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

extern int edge_threshold;

uint8_t ker_mul(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width, int YUV);
void image_convolution(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total);
void assign_kernel_values(int_fast8_t const *kernel_r, int_fast8_t *kernel);
void generate_kernel (struct FloatEulers const *angles, int_fast8_t *kernel);
void unaligned_sum(struct image_t *input, int *output, struct FloatEulers *angles, int axis);
void heading_command(const int* input, uint16_t start, uint16_t end, int *xMin, int *xMax);
void downsample_yuv422(struct image_t* input, struct image_t* output, uint8_t downsample_factor);
u_int32_t find_max_y(struct image_t* input, uint16_t* output);
void heading_command_v2(const int* edge_input, const u_int16_t* y_input, uint16_t start, uint16_t end, int *xMin, int *xMax, int*maxGreen);
void fill_green_below_max(struct image_t* image, const u_int16_t* y_input);


#endif //GROUP_1_CV_H
