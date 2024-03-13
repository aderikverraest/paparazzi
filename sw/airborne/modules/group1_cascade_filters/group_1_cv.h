//
// Created by daan on 12-3-24.
//

#ifndef GROUP_1_CV_H
#define GROUP_1_CV_H

#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

uint8_t ker_mul(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width);
void image_convolution(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total);
void assign_kernel_values(int_fast8_t const *kernel_r, int_fast8_t *kernel);
void generate_kernel (struct FloatEulers const *angles, int_fast8_t *kernel);
void unaligned_sum(struct image_t *input, int *output, struct FloatEulers *angles, int axis);
void heading_command(const int* input, int width, int *xMin, int *xMax);


#endif //GROUP_1_CV_H
