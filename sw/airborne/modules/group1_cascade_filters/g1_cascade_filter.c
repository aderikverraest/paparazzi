//
// Created by maximecapelle on 7-3-24.
//

// Other files
//#define OBJECT_DETECTOR_VERBOSE 1
#include "modules/group1_cascade_filters/g1_cascade_filter.h"
#include "modules/group1_cascade_filters/group_1_cv.h"
#include "modules/group1_cascade_filters/convolution.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/lib/vision/image.h"

// Setting up printing syntax
#define PRINT(string,...) fprintf(stderr, "[CASCADE_FILTER->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef CASCADE_FILTER_CAMERA_FPS
#define CASCADE_FILTER_CAMERA_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

    // Structures //

// Output variables that will be transmitted to navigation
struct output_variables_object
{
    // Add important variables here
    int32_t nav_command;
    uint32_t pixel_count;
    bool updated;
    int16_t img_w;
    int16_t img_h;
    uint32_t count_left;
    uint32_t count_right;
};


                                        // Global Variables //
// Define Mutex
static pthread_mutex_t mutex;

// Define Global Memory
struct output_variables_object global_memory[1];

// Drawing on image
bool cf_draw = true;

// Downsampling
uint8_t downsample_factor = 4;


    // Function Definitions
static struct image_t* cascade_filter(struct image_t* img, uint8_t __attribute__((unused)) filter)
{
    // Down sampling
    struct image_t img_ds;
    downsample_yuv422(img, &img_ds, downsample_factor);
    image_copy(&img_ds, img);

    // Seperated floor count for turning information
    uint32_t count_left = 0;
    uint32_t count_right = 0;

    // Minimal and maximal index between which the floor is found
    uint16_t iMinG;
    uint16_t iMaxG;

    // Reserve images to copy output into
    struct image_t img2;
    struct image_t img3;
    image_create(&img2, img->w, img->h, img->type);
    image_create(&img3, img->w, img->h, img->type);
    image_copy(img, &img2);
    image_copy(img, &img3);

    // Floor counting
    uint32_t floor_count = image_ground_detector(img, &img2, &count_left, &count_right, &iMinG, &iMaxG);
    image_copy(&img2, img);

    // Average Pooling
    avg_pool(img, &img3, 220, 1);

    // Array allocation for the maximal index at which the ground occurs
    u_int16_t* max_ground_height_array = (u_int16_t*) calloc(img->h, sizeof(u_int16_t));

    // Floor counting and colouring
    floor_count = find_max_y(&img3, max_ground_height_array);
    fill_green_below_max(img, max_ground_height_array);
    image_copy(img, &img3);

    // 4 times 3x3 gaussion blur to approximate 9x9 gaussian blur
    int_fast8_t kernel_gaussian[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};
    for (int i = 0; i < 2; ++i) {
        image_convolution_3x3(img, &img2, kernel_gaussian, 16);
        image_convolution_3x3(&img2, img, kernel_gaussian, 16);
    }

    // Sobel kernel for horizontal edges in the image (Representing vertical edges in real life)
    int_fast8_t kernel_edge[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

    // Edge Detection
    image_convolution_3x3(img, &img2, kernel_edge, 8);
    image_copy(&img2, img);

    fprintf(stderr, "iMin: %d iMax: %d", iMinG, iMaxG);

    // Array allocation for the edge summation per row
    int* edges_sum_array = (int*) calloc(img->h, sizeof(int));

    // Sum over the rows the get the vertical edge sum
    unaligned_sum(&img2, edges_sum_array, &img->eulers, 1);

    // Create index values
    int iMinE = 0;
    int iMaxE = 0;
    //    heading_command(output, xMinG, xMaxG, &xMinE, &xMaxE); DEPRECATED HEADING GENERATION
    // Calculate the indices for the bounds between which the drone should fly
    heading_command_v2(edges_sum_array, max_ground_height_array, iMinG, iMaxG, &iMinE, &iMaxE);

    // Free allocated memory for arrays
    free(edges_sum_array);
    free(max_ground_height_array);

    // Minimal point
    struct point_t dir_min;
    dir_min.x = (int) round(img->w / 2);
    dir_min.y = iMinE;

    // Maximal point
    struct point_t dir_max;
    dir_max.x = dir_min.x;
    dir_max.y = iMaxE;

    // Centre point between the two
    struct point_t dir_mid;
    dir_mid.x = dir_min.x;
    dir_mid.y = (int) ((dir_min.y + dir_max.y) / 2);

    // Draw output on the appropriate image
    image_copy(&img3, img);
    image_draw_line(img, &dir_min, &dir_max);
    uint8_t color[4] = {127, 255, 127, 255};
    image_draw_crosshair(img, &dir_mid, color, 10);


    // Recenter the nav command around the middle of the image
    int32_t nav_command = dir_mid.y - (img->h / 2); // Pixel direction

    // Normalization:
    // nav_command between -100 and 100 (-100 -> 60% of half)
    nav_command = nav_command * 100 / (img->h / 2) / 0.8;
    if (nav_command > 100)
    {
        nav_command = 100;
    } else if (nav_command < -100) {
        nav_command = -100;
    }

    // SEND GLOBAL MEMORY
    pthread_mutex_lock(&mutex);
    global_memory[0].nav_command = nav_command;
    global_memory[0].pixel_count = floor_count;
    global_memory[0].updated = true;
    global_memory[0].count_left = count_left;
    global_memory[0].count_right = count_right;
    global_memory[0].img_w = img->w;
    global_memory[0].img_h = img->h;
    pthread_mutex_unlock(&mutex);

    return img;
}

void cascade_filter_init(void)
{
    VERBOSE_PRINT("IN CASCADE FILTER INIT");
    memset(global_memory, 0, sizeof(struct output_variables_object));
    pthread_mutex_init(&mutex, NULL);
#ifdef CASCADE_FILTER_CAMERA
    #ifdef CASCADE_FILTER_LUM_THRESH_MIN
    y_min = CASCADE_FILTER_Y_THRESH_MIN
    y_max = CASCADE_FILTER_Y_THRESH_MAX
    u_min = CASCADE_FILTER_U_THRESH_MIN
    u_max = CASCADE_FILTER_U_THRESH_MAX
    v_min = CASCADE_FILTER_V_THRESH_MIN
    v_max = CASCADE_FILTER_V_THRESH_MAX
    #endif

#ifdef CASCADE_FILTER_DRAW
  cf_draw = CASCADE_FILTER_DRAW;
#endif

  cv_add_to_device(&CASCADE_FILTER_CAMERA, cascade_filter, CASCADE_FILTER_CAMERA_FPS, 0);
#endif
}

// PERIODIC FUNCTION
void cascade_filter_periodic(void)
{


    VERBOSE_PRINT("IN CASCADE FILTER PERIODIC");
    static struct output_variables_object local_memory[1]; // Defines an array of structures of type "output_variables_object"
    pthread_mutex_lock(&mutex);                    // It ensures that no other thread can modify the shared data while this function is executing.
    memcpy(local_memory, global_memory, sizeof(struct output_variables_object));; // Copies the global filters into the local filters.
    pthread_mutex_unlock(&mutex);                  // Unlock previous lock

    if(local_memory[0].updated){
        // Might have to make custom message
        AbiSendMsgVISUAL_DETECTION(CASCADE_FILTER_MSG_ID,
                                   local_memory[0].nav_command, // called int16_t pixel_x
                                   local_memory[0].count_left, // called int16_t pixel_y
                                   local_memory[0].img_w,   // called int16_t pixel_width,
                                   local_memory[0].img_h,   // called int16_t pixel_height,
                                   local_memory[0].pixel_count,       // called int32_t quality,
                                   local_memory[0].count_right);      // called int16_t extra
        local_memory[0].updated = false;
    }
}