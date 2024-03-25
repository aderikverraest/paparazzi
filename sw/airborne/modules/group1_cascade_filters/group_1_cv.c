#include <stdlib.h>
#include <stdio.h>
#include "modules/group1_cascade_filters/group_1_cv.h"

// Defines for the Nav Command Functions that determine the threshold
int edge_threshold = 300;
int epsilon = 7;

/**
 * Kernel multiplication for a 3 x 3 kernel.
 * Pixels outside the edges are taken to be 0.
 * @param[in] source The source values
 * @param[in] kernel The kernel values
 * @param total The kernel total
 * @param setting Variable is used to indicate which pixels the multiplication uses at a specific edges.
 * @param YUV A bool that adds an extra pixel offset if the YUV format is used.
 */
uint8_t ker_mul(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width, int YUV) {
    int value;
    int offset = 1 + YUV;

    switch (setting) {
        default: {  // No edge
            value = *(source - offset - width) * kernel[0] + *(source - width) * kernel[1] + *(source + offset - width) * kernel[2] +
                       *(source - offset) * kernel[3]         + *source * kernel[4]           + *(source + offset) * kernel[5] +
                       *(source - offset + width) * kernel[6] + *(source + width) * kernel[7] + *(source + offset + width) * kernel[8];
            break; }
        case 1: { // Upper Edge
            value = *(source - offset) * kernel[3]         + *source * kernel[4]           + *(source + offset) * kernel[5] +
                       *(source - offset + width) * kernel[6] + *(source + width) * kernel[7] + *(source + offset + width) * kernel[8];
            break; }
        case 2: { // Lower Edge
            value = *(source - offset - width) * kernel[0] + *(source - width) * kernel[1] + *(source + offset - width) * kernel[2] +
                       *(source - offset) * kernel[3]         + *source * kernel[4]           + *(source + offset) * kernel[5];
            break; }
        case 3: { // Left Edge
            value = *(source - width) * kernel[1] + *(source + offset - width) * kernel[2] +
                       *source * kernel[4]           + *(source + offset) * kernel[5] +
                       *(source + width) * kernel[7] + *(source + offset + width) * kernel[8];
            break; }
        case 4: { // Right Edge
            value = *(source - offset - width) * kernel[0] + *(source - width) * kernel[1] +
                       *(source - offset) * kernel[3]         + *source * kernel[4] +
                       *(source - offset + width) * kernel[6] + *(source + width) * kernel[7];
            break; }
        case 5: {// Top Left Corner
            value = *source * kernel[4]           + *(source + offset) * kernel[5] +
                       *(source + width) * kernel[7] + *(source + offset + width) * kernel[8];
            break; }
        case 6: {// Top Right Corner
            value = *(source - offset) * kernel[3]         + *source * kernel[4] +
                       *(source - offset + width) * kernel[6] + *(source + width) * kernel[7];
            break; }
        case 7: {// Bottom Left Corner
            value = *(source - width) * kernel[1] + *(source + offset - width) * kernel[2] +
                       *source * kernel[4]           + *(source + offset) * kernel[5];
            break; }
        case 8: {// Bottom Right Corner
            value = *(source - offset - width) * kernel[0] + *(source - width) * kernel[1] +
                       *(source - offset) * kernel[3]         + *source * kernel[4];
            break; }
    }

    return (uint8_t) ((int) abs(value) / total);
}

/**
 * Image convolution for a 3x3 kernel
 * @param[in] input Input image
 * @param[in,out] output Output image
 * @param[in] kernel The kernel values
*/
void image_convolution(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total) {
    // Copy buffer pointers
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;

    // Copy the creation timestamp (stays the same)
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;

    int height = output->h;
    int width = output->w;

    if (output->type == IMAGE_YUV422) {
        // Skip The first U/V pixel
        *dest++ = 127;
        source++;

        // Top Left Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 5, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        // Upper Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul(source, kernel, kernel_total, 1, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Top Right Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 6, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        for (int y = 1; y < height - 1; y++) {
            // Left Edge
            *dest++ = ker_mul(source, kernel, kernel_total, 3, 2 * width, 1);
            source += 2;
            *dest++ = 127;

            // Middle
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 0, 2 * width, 1);
                source += 2;
                *dest++ = 127;
            }

            // Right Edge
            *dest++ = ker_mul(source, kernel, kernel_total, 4, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Bottom Left Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 7, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        // Bottom Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul(source, kernel, kernel_total, 2, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Bottom Right Corner
        *dest++ = ker_mul(source, kernel, kernel_total, 8, 2 * width, 1);
    } else {
        if (output->type == IMAGE_GRAYSCALE) {
            // Top Left Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 5, width, 0);
            source++;

            // Upper Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 1, width, 0);
                source++;
            }

            // Top Right Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 6, width, 0);
            source++;

            for (int y = 1; y < height - 1; y++) {
                // Left Edge
                *dest++ = ker_mul(source, kernel, kernel_total, 3, width, 0);
                source++;

                // Middle
                for (int x = 1; x < width - 1; x++) {
                    *dest++ = ker_mul(source, kernel, kernel_total, 0, width, 0);
                    source++;
                }

                // Right Edge
                *dest++ = ker_mul(source, kernel, kernel_total, 4, width, 0);
                source++;
            }

            // Bottom Left Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 7, width, 0);
            source++;

            // Bottom Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul(source, kernel, kernel_total, 2, width, 0);
                source++;
            }

            // Bottom Right Corner
            *dest++ = ker_mul(source, kernel, kernel_total, 8, width, 0);
            source++;
        }
    }
}

// Copies the value of a reference kernel to another kernel
void assign_kernel_values(int_fast8_t const *kernel_r, int_fast8_t *kernel) {
    for (int i = 0; i < 9; ++i) {
        *kernel++ = *kernel_r++;
    }
}


// Generates a rotated (WARNING: DEPRECATED, no longer used due to unreliable angle information)
// Kernel values are generated by rotating an image with representative values for the 0deg case. (See rotated_kernels.py)
void generate_kernel (struct FloatEulers const *angles, int_fast8_t *kernel) {
    switch ((int) ((angles->phi+45)/0.0872664626)) { //Every 5 degrees
        case 0: { // Angle = -45
            int_fast8_t kernel_r[9] = {0, -1, -2, 1, 0, -1, 2, 1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 1: { // Angle = -40
            int_fast8_t kernel_r[9] = {0, -1, -2, 1, 0, -1, 2, 1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 2: { // Angle = -35
            int_fast8_t kernel_r[9] = {0, -1, -2, 1, 0, -1, 2, 1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 3: { // Angle = -30
            int_fast8_t kernel_r[9] = {0, -1, -2, 1, 0, -1, 2, 1, 0};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 4: { // Angle = -25
            int_fast8_t kernel_r[9] = {0, -1, -1, 1, 0, 0, 2, 1, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 5: { // Angle = -20
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 6: { // Angle = -15
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 7: { // Angle = -10
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 8: { // Angle = -5
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 9: { // Angle = 0
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 10: { // Angle = 5
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 11: { // Angle = 10
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 12: { // Angle = 15
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 13: { // Angle = 20
            int_fast8_t kernel_r[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 14: { // Angle = 25
            int_fast8_t kernel_r[9] = {-1, -1, 0, 0, 0, 1, 1, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 15: { // Angle = 30
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 16: { // Angle = 35
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 17: { // Angle = 40
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
        case 18: { // Angle = 45
            int_fast8_t kernel_r[9] = {-2, -1, 0, -1, 0, 1, 0, 1, 2};
            assign_kernel_values(kernel_r, kernel);
            break;
        }
    }
}

/**
 * Sums pixel intensity for a certain direction
 * (WARNING: Partly Deprecated, angles summing no longer supported due to unreliable angle information)
 * @param input[in] Input Image
 * @param output[in,out] Output array
 * @param angles[in] (Deprecated) Image angles
 * @param axis Axis over which is summed. Axis = 0 returns sum per column. Axis = 1 returns sum per row.
 */
void unaligned_sum(struct image_t *input, int *output, struct FloatEulers *angles, int axis) {
    int width = input->w;
    int height = input->h;

    uint8_t* source = input->buf;
    // hard code angle < 5 deg
//    if (fabsf(angles->phi) < 0.0872664626f) {
    if (true) {
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
                // skip the first pixel
                source++;

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
                // Sum per row values for a greyscale with low (-5<phi<5) roll angle
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        *output += *source++;
                    }
                    output++;
                }
            } else {
                // skip the first pixel
                source++;

                // Sum per row values for a YUC with low (-5<phi<5) roll angle
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
                // skip the first pixel
                source++;

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
                // skip the first pixel
                source++;

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

/**
 * Determines the maximal and minimal pixel index for the largest gap in values
 * @param input[in] Sum per row of edge intensity
 * @param start Starting index of the floor
 * @param end Ending index of the floor
 * @param iMin pointer to the int value of the lower pixel index for the largest gap
 * @param iMax pointer to the int value of the upper pixel index for the largest gap
 */
void heading_command(const int* input, uint16_t start, uint16_t end, int *iMin, int *iMax) {
    int i_min = 0, i_max = 0, current_start = 0, current_len = 0, max_len = 0;

    int i;
    for (i = start; i < end; ++i) {
        if (*input++ < edge_threshold) {
            if (!current_len) current_start = i;
            current_len += 1;
        } else {
            if (current_len > 0) {
                if (current_len > max_len) {
                    max_len = current_len;
                    i_min = current_start;
                    i_max = i;
                    current_len = 0;
                } else {
                    current_len = 0;
                }
            }
        }
    }

    // Check if the final value is part of the largest gap
    if (current_len > 0) {
        if (current_len > max_len) {
            i_min = current_start;
            i_max = i;
        }
    }
    *iMin = i_min;
    *iMax = i_max;
}


// Downsampling Function wrapper to ensure desired functionality
void downsample_yuv422(struct image_t* input, struct image_t* output, uint8_t downsample_factor) {
    image_create(output,
                     input->w / downsample_factor,
                     input->h / downsample_factor,
                     IMAGE_YUV422);
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;
    if (downsample_factor > 1) {
        image_yuv422_downsample(input, output,downsample_factor);
    }
}

/**
 * Finds the maximal pixel index per row for which the value is not zero
 * @param input[in] Boolean image
 * @param output[in,out] Array of row-wise maximal pixel indices
 * @return The count of floor pixels filled to the highest index meeting the threshold
 */
uint32_t find_max_y(struct image_t* input, uint16_t* output) {
    u_int8_t* source = input->buf;
    source ++;
    uint32_t count = 0;

    for (int i = 0; i < input->h; ++i) {
        for (int j = 0; j < input->w; j++) {
            if (*source > 0) *output = j;
            source += 2;
        }
        count += *output;
        output++;
    }

    return count;
}

/**
 * Determines the maximal and minimal pixel index for the largest gap in values weighted for the amount of green that is
   within the bounds
 * @param edge_input[in] Sum per row of edge intensity
 * @param y_input[in] Array of row-wise maximal pixel indices
 * @param start Starting index of the ground
 * @param end Ending index of the ground
 * @param iMin Lower index of the most desirable gap
 * @param iMax Maximal index of the most desirable gap
 * @param maxGreen Value with the most green
 */
void heading_command_v2(const int* edge_input, const u_int16_t* y_input, uint16_t start, uint16_t end, int *iMin, int *iMax, int *maxGreen) {
    int i_min = 0, i_max = 0, current_start = 0, current_len = 0,  avg_green = 0, max_green = 0;
    uint8_t min_len = 5;

    int i;
    for (i = start; i < end; ++i) {
        if (*edge_input++ < edge_threshold) {
            if (current_len == 0) current_start = i;
            current_len += 1;
        } else {
            if (current_len > min_len) {
                avg_green = 0;
                for (int j = current_start; j < i; ++j) {
                    avg_green += y_input[j];
                }
                avg_green = (int) avg_green / (current_len + epsilon);

                if (avg_green > max_green) {
                    max_green = avg_green;
                    i_min = current_start;
                    i_max = i;
                }
            }
            current_len = 0;
        }
    }

    if (current_len > min_len) {
        avg_green = 0;
        for (int j = current_start; j < i; ++j) {
            avg_green += y_input[j];
        }
        avg_green = (int) avg_green / (current_len + epsilon);

        if (avg_green > max_green) {
            max_green = avg_green;
            i_min = current_start;
            i_max = i;
        }
    }

    *iMin = i_min;
    *iMax = i_max;
    *maxGreen = max_green;
}

/**
 * Fill in the pixels green below the maximal y value
 * @param image[in, out] The image to be filled
 * @param y_input[in] Array of row-wise maximal pixel indices
 */
void fill_green_below_max(struct image_t* image, const u_int16_t* y_input) {
    //  Set color detected image equal to 255 below the maximum y-value, this effectively filters out the carpet for
    //  the edge detection following hereafter

    u_int8_t *source = image->buf;
    source++;

    for (int i = 0; i < image->h; ++i) {
        for (int j = 0; j < y_input[i]; ++j) {
            *source = 255;
            source += 2;
        }
        source += (2 * (image->w - y_input[i]));
    }
}


// Filter Thresholds (Default values if not defined in conf file)

// Luma
uint8_t y_min = 0;
uint8_t y_max = 169;

// Chroma Blue
uint8_t u_min = 0;
uint8_t u_max = 107;

// Chroma Red
uint8_t v_min = 0;
uint8_t v_max = 146;

// Masks
uint8_t y_ground_mask = 255;
uint8_t u_ground_mask = 128;
uint8_t v_ground_mask = 128;

/**
 * Counts and colours the ground pixel in an image.
 * A pixel is considered ground if it is between the thresholds defined above
 * @param img[in] Input image
 * @param img_out[in, out] Output image
 * @param count_left Count to the left of center
 * @param count_right Count to the right of center
 * @param iMin Minimal index of the floor pixels
 * @param iMax Maximal index of the floor pixels
 * @return floor pixel count
 */
uint32_t image_ground_detector(struct image_t *img, struct image_t *img_out, uint32_t *count_left, uint32_t *count_right, uint16_t *iMin, uint16_t *iMax) {
    // Copy the thresholds to local scope
    uint8_t y_m = y_min;
    uint8_t y_M = y_max;
    uint8_t u_m = u_min;
    uint8_t u_M = u_max;
    uint8_t v_m = v_min;
    uint8_t v_M = v_max;

    uint16_t cnt = 0;
    uint8_t *source = (uint8_t *)img->buf;
    uint8_t *dest = (uint8_t *)img_out->buf;

    // Initialise the maximal and minimal edges in reverse
    uint16_t i_min = 255;
    uint16_t i_max = 0;

    // Pointers for the pixel values
    uint8_t *yp, *up, *vp;
    uint8_t *yp_dest, *up_dest, *vp_dest;

    for (uint16_t y = 0; y < img->h; y++) {
        for (uint16_t x = 0; x < img->w; x++) {
            if (x % 2 == 0) {
                yp = source + 1;
                up = source;
                vp = source + 2;

                yp_dest = dest + 1;
                up_dest = dest;
                vp_dest = dest + 2;
            } else {
                yp = source + 1;
                up = source - 2;
                vp = source;

                yp_dest = dest + 1;
                up_dest = dest - 2;
                vp_dest = dest;
            }

            if (*yp >= y_m && *yp <= y_M
            && *up >= u_m && *up <= u_M
            && *vp >= v_m &&*vp <= v_M) {
                cnt++;

                // Set destination to the drawing colour
                *yp_dest = y_ground_mask;
                *up_dest = u_ground_mask;
                *vp_dest = v_ground_mask;

                // Check if the edges are in the right location
                i_min = i_min > y ? y: i_min;
                i_max = i_max < y ? y: i_max;

            } else {
                *yp_dest = 0;
                *up_dest = 128;
                *vp_dest = 128;
            }
            if (x < img->w / 2) {
                *count_left += 1;
            } else {
                *count_right += 1;
            }

            source += 2;
            dest += 2;
        }
    }

    // Set the value of the outer scope pointers to the right value
    *iMin = i_min;
    *iMax = i_max;

    return cnt;
}

/**
 * Create a binary image
 * @param img_in[in] Input image
 * @param img_out[in, out] Output image
 * @param threshold Threshold value, for the Y value
 */
void threshold_img(struct image_t *img_in, struct image_t *img_out, uint8_t threshold) {
    uint8_t *source = (uint8_t *)img_in->buf;
    uint8_t *dest = (uint8_t *)img_out->buf;

    uint8_t *yp;
    uint8_t *yp_dest, *up_dest, *vp_dest;
    for (uint16_t y = 0; y < img_in->h; y++) {
        for (uint16_t x = 0; x < img_in->w; x++) {
            if (x % 2 == 0) {
                yp = source + 1;

                yp_dest = dest + 1;
                up_dest = dest;
                vp_dest = dest + 2;
            } else {
                yp = source + 1;

                yp_dest = dest + 1;
                up_dest = dest - 2;
                vp_dest = dest;
            }

            if (*yp < threshold) {
                *yp_dest = 0;
                *up_dest = 128;
                *vp_dest = 128;
            } else {
                *yp_dest = 255;
                *up_dest = 128;
                *vp_dest = 128;
            }

            source += 2;
            dest += 2;
        }
    }
}

/**
 * Average pool and then threshold an image for a certain number of iterations
 * @param img[in,out] Input image (WARNING Is altered during this function)
 * @param img_out[in,out] Average Pooled Image
 * @param threshold threshold value for intensity Y
 * @param n_iterations number of iterations of pooling
 */
void avg_pool(struct image_t *img, struct image_t *img_out, uint8_t threshold, uint8_t n_iterations) {
    int_fast8_t avg_kernel[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (int i = 0; i < n_iterations; i++) {
        image_convolution(img, img_out, avg_kernel, 9);
        threshold_img(img_out, img, threshold);
    }
    image_copy(img, img_out);
}

// DEPRECATED replaced with fill_green_below_max and image_ground_detector
// Fill the ground pixel with the drawing colour
struct image_t *image_ground_filler(struct image_t *img) {
    uint8_t *buffer = img->buf;
    //  Only check up until horizon
    uint16_t horizon = img->w / 2;
    uint8_t *yp, *up, *vp;
    for (uint16_t y = 0; y < img->h; y++) {
        for (uint16_t x = 0; x < horizon; x += 1) {
            if (x % 2 == 0) {
                yp = buffer + 1;
                up = buffer;
                vp = buffer + 2;
            } else {
                yp = buffer + 1;
                up = buffer - 2;
                vp = buffer;
            }

            if (*yp == y_ground_mask && *up == u_ground_mask &&
                *vp == v_ground_mask) {

                *yp = y_ground_mask;
                *up = y_ground_mask;
                *vp = y_ground_mask;
            }
            buffer += 2;
        }
    }
    return img;
}