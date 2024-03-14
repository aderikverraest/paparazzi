//
// Created by maximecapelle on 7-3-24.
//

// Other files
//#define OBJECT_DETECTOR_VERBOSE 1
#include "modules/group1_cascade_filters/g1_cascade_filter.h"
#include "modules/group1_cascade_filters/cv.h"
#include "modules/group1_cascade_filters/group_1_cv.h"
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
};


    // Global Variables //

// Define Mutex
static pthread_mutex_t mutex;

// Define Global Memory
struct output_variables_object global_memory[1];

// Filter Thresholds (Default values if not defined in conf file)

// Luma
uint8_t y_min = 41;
uint8_t y_max = 183;

// Chroma Blue
uint8_t u_min = 53;
uint8_t u_max = 121;

// Chroma Red
uint8_t v_min = 134;
uint8_t v_max = 249;

uint8_t downsample_factor = 1;

// Drawing on image
bool cf_draw = true;


      // Functions //
    // Declarations

    // Function Definitions
static struct image_t* cascade_filter(struct image_t* img, uint8_t filter)
{

    // DOWN SAMPLING
    struct image_t img_ds;
    downsample_yuv422(img, &img_ds, downsample_factor);
    image_copy(&img_ds, img);

    // CONVOLUTIONS
    struct image_t img2;
    struct image_t img3;
    image_create(&img2, img->w, img->h, img->type);
    image_create(&img3, img->w, img->h, img->type);
    image_copy(img, &img2);
    image_copy(img, &img3);
    image_to_grayscale(&img2, img);

    int_fast8_t kernel_gaussian[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};
    image_convolution(img, &img2, kernel_gaussian, 16);
    image_copy(&img2, img);


    for (int i = 0; i < 2; ++i) {
        image_convolution(img, &img2, kernel_gaussian, 16);
        image_convolution(&img2, img, kernel_gaussian, 16);
    }
    image_copy(img, &img3);

    int_fast8_t* kernel_edge = (int_fast8_t*) malloc(8*sizeof(int_fast8_t));
    generate_kernel(&img->eulers, kernel_edge);
    image_convolution(img, &img2, kernel_edge, 8);
    free(kernel_edge);
    image_copy(&img2, img);

    int* output = (int*) calloc(img->h, sizeof(int));
    unaligned_sum(&img2, output, &img->eulers, 1);

    int xMin = 0;
    int xMax = 0;
    heading_command(output, img->h, &xMin, &xMax);
    free(output);

    struct point_t Xmin;
    Xmin.x = (int) round(img->w / 2);
    Xmin.y = xMin;

    struct point_t Xmax;
    Xmax.x = Xmin.x;
    Xmax.y = xMax;

    struct point_t Xmid;
    Xmid.x = Xmin.x;
    Xmid.y = (int) ((xMin + xMax) / 2);

    image_copy(&img3, img);
    image_draw_line(img, &Xmin, &Xmax);
    uint8_t color[4] = {127, 255, 127, 255};
    image_draw_crosshair(img, &Xmid, color, 10);

    uint32_t nav_command = (xMin + xMax / 2); // Pixel direction
    uint32_t count = 32;

    fprintf(stderr, "Color_count: %d  nav_command: %d \n", count, nav_command);



    // SEND GLOBAL MEMORY
    pthread_mutex_lock(&mutex);
    global_memory[0].nav_command = nav_command;
    global_memory[0].pixel_count = count;
    global_memory[0].updated = true;
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
                                   0, // called int16_t pixel_y
                                   0,   // called int16_t pixel_width,
                                   0,   // called int16_t pixel_height,
                                   local_memory[0].pixel_count,       // called int32_t quality,
                                   0);      // called int16_t extra
        local_memory[0].updated = false;
    }
}