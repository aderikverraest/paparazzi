//
// Created by maximecapelle on 7-3-24.
//

// Other files
//#define OBJECT_DETECTOR_VERBOSE 1
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
uint8_t y_min = 41;
uint8_t y_max = 183;

// Chroma Blue
uint8_t u_min = 53;
uint8_t u_max = 121;

// Chroma Red
uint8_t v_min = 134;
uint8_t v_max = 249;

// Drawing on image
bool cf_draw = true;
uint8_t y_ground_mask = 255;
uint8_t u_ground_mask = 123;
uint8_t v_ground_mask = 123;

uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;
bool cod_draw1 = false;
bool cod_draw2 = false;



      // Functions //
    // Declarations
struct image_t *image_ground_detector(struct image_t *img) {
  //  return img;
  uint8_t y_m = 0;
  uint8_t y_M = 255;
  uint8_t u_m = 0;
  uint8_t u_M = 110;
  uint8_t v_m = 0;
  uint8_t v_M = 130;

  uint16_t cnt = 0;
  uint8_t *dest = (uint8_t *)img->buf;
  uint8_t *yp, *up, *vp;
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x += 2) {
      obtainYUV(img, dest, x, y, &yp, &up, &vp);
      if (*yp >= y_m && *yp <= y_M && *up >= u_m && *up <= u_M && *vp >= v_m &&
          *vp <= v_M) {
        *yp = y_ground_mask;
        *up = u_ground_mask;
        *vp = v_ground_mask;
      }
    }
  }
  // Go trough all the pixels
//  for (uint16_t y = 0; y < img->h; y++) {
//    for (uint16_t x = 0; x < img->w; x += 2) {
//      // Check if the color is inside the specified values
//      if ((dest[1] >= y_m) && (dest[1] <= y_M) && (dest[0] >= u_m) &&
//          (dest[0] <= u_M) && (dest[2] >= v_m) && (dest[2] <= v_M)) {
//        cnt++;
//        // UYVY
//        dest[0] = u_ground_mask; // U
//        dest[1] = y_ground_mask; // Y
//        dest[2] = v_ground_mask; // V
//        dest[3] = y_ground_mask; // Y
//      }
//
//      // Go to the next 2 pixels
//      dest += 4;
//    }
//  }
  return img;
}

void obtainYUV(struct image_t *img, uint8_t *buffer, int x, int y, uint8_t **yp,
               uint8_t **up, uint8_t **vp) {
  if (x % 2 == 0) {
    // Even x
    *up = &buffer[y * 2 * img->w + 2 * x];     // U
    *yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y1
    *vp = &buffer[y * 2 * img->w + 2 * x + 2]; // V
  } else {
    // Uneven x
    *up = &buffer[y * 2 * img->w + 2 * x - 2]; // U
    *vp = &buffer[y * 2 * img->w + 2 * x];     // V
    *yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y2
  }
}

struct image_t *image_ground_filler(struct image_t *img) {
  uint8_t *buffer = img->buf;

  //  Only check up until horizon
  uint16_t horizon = img->w / 2;
  uint16_t top_x;
  uint8_t *yp, *up, *vp;
  for (uint16_t y = 0; y < img->h; y++) {
    top_x = 0;
    for (uint16_t x = 0; x < horizon; x += 1) {
      obtainYUV(img, buffer, x, y, &yp, &up, &vp);
      if (*yp == y_ground_mask && *up == u_ground_mask &&
          *vp == v_ground_mask) {
        top_x = x;
      }
    }

    for (uint16_t x = 0; x < top_x; x += 1) {
      obtainYUV(img, buffer, x, y, &yp, &up, &vp);
      *yp = y_ground_mask;
      *up = y_ground_mask;
      *vp = y_ground_mask;
    }
  }
  return img;
}

    // Function Definitions
static struct image_t* cascade_filter(struct image_t* img, uint8_t filter)
{
    VERBOSE_PRINT("IN CASCADE FUNCTION");
    // Run Cascading filters
//    uint32_t count = image_yuv422_colorfilt(img, img, y_min, y_max, u_min, u_max, v_min, v_max);
//    image_to_grayscale(img, img);
    image_ground_detector(img);
    image_ground_filler(img);
    uint32_t nav_command = 0;
//    uint32_t count = 32;
    VERBOSE_PRINT("Color count: %u  Nav Command: %u\n", count, nav_command);


    // Update Global memory
    pthread_mutex_lock(&mutex);
    global_memory[0].nav_command = nav_command;
//    global_memory[0].pixel_count = count;
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