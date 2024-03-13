#define EDGE_THRESHOLD 15000

#include <stdlib.h>
#include "modules/group1_cascade_filters/group_1_cv.h"

uint8_t ker_mul(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width) {
    int gradient;

    switch (setting) {
        default: {
            gradient = *(source - 1 - width) * kernel[0] + *(source - width) * kernel[1] + *(source + 1 - width) * kernel[2] +
                       *(source - 1) * kernel[3]         + *source * kernel[4]           + *(source + 1) * kernel[5] +
                       *(source - 1 + width) * kernel[6] + *(source + width) * kernel[7] + *(source + 1 + width) * kernel[8];
            break; }
        case 1: { // Upper Edge
            gradient = *(source - 1) * kernel[3]         + *source * kernel[4]           + *(source + 1) * kernel[5] +
                       *(source - 1 + width) * kernel[6] + *(source + width) * kernel[7] + *(source + 1 + width) * kernel[8];
            break; }
        case 2: { // Lower Edge
            gradient = *(source - 1 - width) * kernel[0] + *(source - width) * kernel[1] + *(source + 1 - width) * kernel[2] +
                       *(source - 1) * kernel[3]         + *source * kernel[4]           + *(source + 1) * kernel[5];
            break; }
        case 3: { // Left Edge
            gradient = *(source - width) * kernel[1] + *(source + 1 - width) * kernel[2] +
                       *source * kernel[4]           + *(source + 1) * kernel[5] +
                       *(source + width) * kernel[7] + *(source + 1 + width) * kernel[8];
            break; }
        case 4: { // Right Edge
            gradient = *(source - 1 - width) * kernel[0] + *(source - width) * kernel[1] +
                       *(source - 1) * kernel[3]         + *source * kernel[4] +
                       *(source - 1 + width) * kernel[6] + *(source + width) * kernel[7];
            break; }
        case 5: {// Top Left Corner
            gradient = *source * kernel[4]           + *(source + 1) * kernel[5] +
                       *(source + width) * kernel[7] + *(source + 1 + width) * kernel[8];
            break; }
        case 6: {// Top Right Corner
            gradient = *(source - 1) * kernel[3]         + *source * kernel[4] +
                       *(source - 1 + width) * kernel[6] + *(source + width) * kernel[7];
            break; }
        case 7: {// Bottom Left Corner
            gradient = *(source - width) * kernel[1] + *(source + 1 - width) * kernel[2] +
                       *source * kernel[4]           + *(source + 1) * kernel[5];
            break; }
        case 8: {// Bottom Right Corner
            gradient = *(source - 1 - width) * kernel[0] + *(source - width) * kernel[1] +
                       *(source - 1) * kernel[3]         + *source * kernel[4];
            break; }
    }
    return (uint8_t) ((int) abs(gradient) / total);
}

void image_convolution(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total) {
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;
    source++;

    // Copy the creation timestamp (stays the same)
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;

    int height = output->h;
    int width = output->w;

    if (output->type == IMAGE_YUV422) {
        // Top Left Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 5, width);
        source += 2;
        *dest++ = 0;

        // Upper Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul(source, kernel, kernel_total, 1, width);
            source += 2;
            *dest++ = 0;
        }

        // Top Right Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 6, width);
        source += 2;
        *dest++ = 0;

        for (int y = 1; y < height - 1; y++) {
            // Left Edge
            *dest++ = ker_mul(source, kernel, kernel_total, 3, width);
            source += 2;
            *dest++ = 0;

            // Middle
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 0, width);
                source += 2;
                *dest++ = 0;
            }

            // Right Edge
            *dest++ = ker_mul(source, kernel, kernel_total, 4, width);
            source += 2;
            *dest++ = 0;
        }

        // Bottom Left Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 7, width);
        source += 2;
        *dest++ = 0;

        // Bottom Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul(source, kernel, kernel_total, 2, width);
            source += 2;
            *dest++ = 0;
        }

        // Bottom Right Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 8, width);
        source += 2;
        *dest++ = 0;
    } else {
        if (output->type == IMAGE_GRAYSCALE) {
            // Top Left Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 5, width);
            source++;

            // Upper Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 1, width);
                source++;
            }

            // Top Right Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 6, width);
            source++;

            for (int y = 1; y < height - 1; y++) {
                // Left Edge
                *dest++ = ker_mul(source, kernel, kernel_total, 3, width);
                source++;

                // Middle
                for (int x = 1; x < width - 1; x++) {
                    *dest++ = ker_mul(source, kernel, kernel_total, 0, width);
                    source++;
                }

                // Right Edge
                *dest++ = ker_mul(source, kernel, kernel_total, 4, width);
                source++;
            }

            // Bottom Left Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 7, width);
            source++;

            // Bottom Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 2, width);
                source++;
            }

            // Bottom Right Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 8, width);
            source++;
        }
    }
}

void assign_kernel_values(int_fast8_t const *kernel_r, int_fast8_t *kernel) {
    for (int i = 0; i < 9; ++i) {
        *kernel++ = *kernel_r++;
    }
}

void generate_kernel (struct FloatEulers const *angles, int_fast8_t *kernel) {
    switch ((int) ((angles->phi+45)/0.0872664626)) { //Every 5 degrees
        case 0: { // Angle = -45
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 1: { // Angle = -40
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 2: { // Angle = -35
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 3: { // Angle = -30
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 4: { // Angle = -25
            int_fast8_t kernel_r[9] = {-1, 0, 1, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 5: { // Angle = -20
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 6: { // Angle = -15
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 7: { // Angle = -10
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 8: { // Angle = -5
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        default: case 9: { // Angle = 0
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 10: { // Angle = 5
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 11: { // Angle = 10
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 12: { // Angle = 15
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 13: { // Angle = 20
            int_fast8_t kernel_r[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 14: { // Angle = 25
            int_fast8_t kernel_r[9] = {0, 1, 2, -1, 0, 1, -1, 0, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 15: { // Angle = 30
            int_fast8_t kernel_r[9] = {0, 1, 2, -1, 0, 1, -2, -1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 16: { // Angle = 35
            int_fast8_t kernel_r[9] = {0, 1, 2, -1, 0, 1, -2, -1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 17: { // Angle = 40
            int_fast8_t kernel_r[9] = {0, 1, 2, -1, 0, 1, -2, -1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 18: { // Angle = 45
            int_fast8_t kernel_r[9] = {0, 1, 2, -1, 0, 1, -2, -1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
    }
}

void unaligned_sum(struct image_t *input, int *output, struct FloatEulers *angles, int axis) {
    int width = input->w;
    int height = input->h;

    uint8_t* source = input->buf;
    source++;

    if (fabsf(angles->phi) < 0.0872664626f) {
        if (!axis) {
            if (input->type == IMAGE_GRAYSCALE) {
                // Sum per column for a grayscale with low (-5<phi<5) roll angle
                for (int i = 0; i < width; ++i) {
                    for (int j = 0; j < height; ++j) {
                        *output += *(source + j * width);
                    }
                    source++;
                    output++;
                }
            } else {
                // Sum per column for a YUC with low (-5<phi<5) roll angle
                for (int i = 0; i < width; ++i) {
                    for (int j = 0; j < height; ++j) {
                        *output += *(source + 2 * j * width);
                    }
                    source += 2;
                    output++;
                }
            }
        } else {
            if (input->type == IMAGE_GRAYSCALE) {
                // Sum per row` values for a greyscale with low (-5<phi<5) roll angle
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        *output += *source++;
                    }
                    output++;
                }
            } else {
                // Sum per row` values for a YUC with low (-5<phi<5) roll angle
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        *output += *source;
                        source += 2;
                    }
                    output++;
                }
            }
        }
    } else {
        int current_offset = 0;
        int counter = 0;
        int increment = (int) roundf(1 / tanf(angles->phi));

        if (!axis) {
            if (input->type == IMAGE_GRAYSCALE) {
                // Sum per column for a greyscale with high (5<abs(phi)<45) roll angle
                for (int i = 0; i < width; ++i) {
                    for (int j = 0; j < height; ++j) {
                        if (i + current_offset < 0 || i + current_offset > width) break;

                        *output += *(source + j * width + current_offset);
                        if (++counter < increment) current_offset += -(increment < 0) + (increment > 0);
                    }
                    source++;
                    output++;
                }
            } else {
                // Sum per column for a YUC with high (5<abs(phi)<45) roll angle
                for (int i = 0; i < width; ++i) {
                    for (int j = 0; j < height; ++j) {
                        if (i + current_offset < 0 || i + current_offset > width) break;

                        *output += *(source + 2 * j * width + 2 * current_offset);
                        if (++counter < increment) current_offset += -(increment < 0) + (increment > 0);
                    }
                    source += 2;
                    output++;
                }
            }
        } else {
            if (input->type == IMAGE_GRAYSCALE) {
                // Sum per row for a greyscale with high (5<abs(phi)<45) roll angle
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        if (i + current_offset < 0 || i + current_offset > height) break;

                        *output += *(source + width * current_offset);
                        if (++counter < increment) current_offset += -(increment < 0) + (increment > 0);
                        source++;
                    }
                    output++;
                }
            } else {
                // Sum per row for a YUC with high (5<abs(phi)<45) roll angle
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        if (i + current_offset < 0 || i + current_offset > height) break;

                        *output += *(source + width * current_offset);
                        if (++counter < increment) current_offset += -(increment < 0) + (increment > 0);
                        source += 2;
                    }
                    output++;
                }
            }
        }
    }
}

void heading_command(const int* input, int width, int *xMin, int *xMax) {
    int x_min, x_max, current_start;
    int current_len = 0;
    int max_len = 0;

    int i;
    for (i = 0; i < width; ++i) {
        if (*input++ < EDGE_THRESHOLD) {
            if (!current_len) current_start = i;
            current_len += 1;
        } else {
            if (current_len > 0) {
                if (current_len > max_len) {
                    max_len = current_len;
                    x_min = current_start;
                    x_max = i;
                    current_len = 0;
                } else {
                    current_len = 0;
                }
            }
        }
    }

    if (current_len > 0) {
        if (current_len > max_len) {
            x_min = current_start;
            x_max = i;
        }
    }
    *xMin = x_min;
    *xMax = x_max;
}