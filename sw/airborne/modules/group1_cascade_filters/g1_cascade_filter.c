//
// Created by maximecapelle on 7-3-24.
//

// Other files
#include "modules/group1_cascade_filters/g1_cascade_filter.h"
#include "modules/group1_cascade_filters/cv.h"
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
uint8_t y_min = 0;
uint8_t y_max = 0;

// Chroma Blue
uint8_t u_min = 0;
uint8_t u_max = 0;

// Chroma Red
uint8_t v_min = 0;
uint8_t v_max = 0;

// Drawing on image
bool cf_draw = false;


      // Functions //
    // Declarations

    // Function Definitions
static struct image_t* cascade_filter(struct image_t* img, uint8_t filter)
{
    // Dummy img
    struct image_t* process_img;

    // Run Cascading filters
    uint32_t count = image_yuv422_colorfilt(img, process_img, y_min, y_max, u_min, u_max, v_min, v_max);
    image_to_grayscale(img, img);

    uint32_t nav_command = 0;
    VERBOSE_PRINT("Color count: %u  Nav Command: %u\n", count, nav_command);


    // Update Global memory
    pthread_mutex_lock(&mutex);
    global_memory[0].nav_command = nav_command;
    global_memory[0].pixel_count = count;
    global_memory[0].updated = true;
    pthread_mutex_unlock(&mutex);


    return img;
}

void cascade_filter_init(void)
{
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
    static struct output_variables_object local_memory[1]; // Defines an array of structures of type "output_variables_object"
    pthread_mutex_lock(&mutex);                    // It ensures that no other thread can modify the shared data while this function is executing.
    memcpy(local_memory, global_memory, sizeof(struct output_variables_object)); // Copies the global filters into the local filters.
    pthread_mutex_unlock(&mutex);                  // Unlock previous lock

    if(local_memory[0].updated){
        // Might have to make custom message
        AbiSendMsgVISUAL_DETECTION(CASCADE_FILTER_MSG_ID,
                                   local_memory[0].nav_command, // called int16_t pixel_x
                                   local_memory[0].pixel_count, // called int16_t pixel_y
                                   0,   // called int16_t pixel_width,
                                   0,   // called int16_t pixel_height,
                                   0,       // called int32_t quality,
                                   0);      // called int16_t extra
        local_memory[0].updated = false;
    }
}