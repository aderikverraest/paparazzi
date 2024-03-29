//
// Created by daan on 29-3-24.
//

#ifndef GITCLONE_CONVOLUTION_H
#define GITCLONE_CONVOLUTION_H

#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

uint8_t ker_mul_3x3(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width, int YUV);
void image_convolution_3x3(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total);

#endif //GITCLONE_CONVOLUTION_H
