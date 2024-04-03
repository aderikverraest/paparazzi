//
// Created by daan on 29-3-24.
//

#include "convolution.h"

/**
 * Kernel multiplication for a 3 x 3 kernel.
 * Pixels outside the edges are taken to be 0.
 * @param[in] source The source values
 * @param[in] kernel The kernel values
 * @param total The kernel total
 * @param setting Variable is used to indicate which pixels the multiplication uses at a specific edges.
 * @param YUV A bool that adds an extra pixel offset if the YUV format is used.
 */
uint8_t ker_mul_3x3(uint8_t const *source, int_fast8_t const *kernel, uint8_t total, uint8_t setting, int width, int YUV) {
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
void image_convolution_3x3(struct image_t *input, struct image_t *output, int_fast8_t const *kernel, uint8_t kernel_total) {
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
        // Skip The first U/V value
        *dest++ = 127;
        source++;

        // Top Left Corner
        *dest++ = ker_mul_3x3(source, kernel, kernel_total, 5, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        // Upper Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 1, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Top Right Corner
        *dest++ = ker_mul_3x3(source, kernel, kernel_total, 6, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        for (int y = 1; y < height - 1; y++) {
            // Left Edge
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 3, 2 * width, 1);
            source += 2;
            *dest++ = 127;

            // Middle
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul_3x3(source, kernel, kernel_total, 0, 2 * width, 1);
                source += 2;
                *dest++ = 127;
            }

            // Right Edge
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 4, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Bottom Left Corner
        *dest++ = ker_mul_3x3(source, kernel, kernel_total, 7, 2 * width, 1);
        source += 2;
        *dest++ = 127;

        // Bottom Edge
        for (int x = 1; x < width - 1; x++) {
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 2, 2 * width, 1);
            source += 2;
            *dest++ = 127;
        }

        // Bottom Right Corner
        *dest = ker_mul_3x3(source, kernel, kernel_total, 8, 2 * width, 1);
    } else {
        if (output->type == IMAGE_GRAYSCALE) {
            // Top Left Corner
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 5, width, 0);
            source++;

            // Upper Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul_3x3(source, kernel, kernel_total, 1, width, 0);
                source++;
            }

            // Top Right Corner
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 6, width, 0);
            source++;

            for (int y = 1; y < height - 1; y++) {
                // Left Edge
                *dest++ = ker_mul_3x3(source, kernel, kernel_total, 3, width, 0);
                source++;

                // Middle
                for (int x = 1; x < width - 1; x++) {
                    *dest++ = ker_mul_3x3(source, kernel, kernel_total, 0, width, 0);
                    source++;
                }

                // Right Edge
                *dest++ = ker_mul_3x3(source, kernel, kernel_total, 4, width, 0);
                source++;
            }

            // Bottom Left Corner
            *dest++ = ker_mul_3x3(source, kernel, kernel_total, 7, width, 0);
            source++;

            // Bottom Edge
            for (int x = 1; x < width - 1; x++) {
                *dest++ = ker_mul_3x3(source, kernel, kernel_total, 2, width, 0);
                source++;
            }

            // Bottom Right Corner
            *dest = ker_mul_3x3(source, kernel, kernel_total, 8, width, 0);
        }
    }
}