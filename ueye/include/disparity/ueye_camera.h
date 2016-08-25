#ifndef _ueye_camera_H
#define _ueye_camera_H

#include <uEye.h>
#include "ueye_exceptions.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

using namespace std;

/*! \struct TUeye_config
 *  \brief It contains the main camera configuration parameters
 */
typedef struct
{
    int cameraid;    // Camera ID
    int img_width;   // Image width
    int img_height;  // Image height
    int img_left; // Left space to the image center
    int img_top;  // Top space to the image center
    int img_bpp;  // Image codification bitmap
    int fps;         // Frame rate
    int param_mode;  // Camera load parameters mode
    string file_str; // Path to the parameters file
    int pixel_clock; // Pixel clock frequency
    double exposure; // Exposure time

    bool mirror_updown; // Mirror UpDown
    bool mirror_leftright; // Mirror LeftRigth
}TUeye_config;

/*!

  \class CUeye_Camera
  \brief Driver for Ueye cameras (UI122xLE-M,UI122xLE-C)

  - connection using usb
  - get sensor hardware information
  - change sensor parameters and options
  - allows diferent parameter loadings (default, user defined, eeprom, file)

*/
class CUeye_Camera
{
  private:

    int pnCol_;       //!< Bit depth of the color setting
    int pnColMode_;   //!< Ueye color mode that corresponds to pnCol
    int memId_;       //!< Memory Id
    int error_;       //!< Camera error
    char* error_msg_; //!< Cmera error message

    /*! \brief Gets sensor information
     *
     * Gets the sensor information and some main configuration parameters
     *
     */
    void get_sensor_info();

    /*! \brief Get the main parameters
     *
     * Get the camera main parameters and fill the class struct parameters
     */
    void get_params();

    /*! \brief Get camera color mode
     *
     * Get camera color mode and set the bit per pixel (bpp)
     */
    void get_color_mode();

    /*! \brief Print camera parameters on screen
     *
     * Prinet camera parameters on screen
     */
    void print_params();

    /*! \brief Allocate memory
     *
     * Allocate memory for the next image
     */
    void alloc_memory();

    /*! \brief Set color mode
     *
     * Set color mode depending on the bits per pixel (bpp)
     */
    void set_color_mode();

    /*! \brief Set the default pixel clock
     *
     * Set the default pixel clock.
     *
     * NOTE: A pixel clock change implies frame rate and exposure changes
     */
    void set_default_pixel_clock();

    /*! \brief Set display mode
     *
     * Set display mode: actually only DIB (Image stored in RAM memory)
     */
    void set_display_mode();

    /*! \brief Set the main parameters
     *
     * Set the camera main parameters depending on the camera mode:
     *    0: Default parameters
     *    1: User defined parameters
     *    2: EEPROM parameters loading
     *    3: Load parameters from FILE
     */
    void set_params();

    /*! \brief Gets Area of Interest (AOI)
     *
     * Gets an area of interest.
     *
     * NOTE: AOI changes implies changes on the frame rate and memory allocation
     */
    void get_AOI();

    /*! \brief Generate a camera exception
     *
     * Creates the message and generates the camera exception
     */
    void is_cam_exception(const string& text);

    /*! \brief Generate a feature exception
     *
     * Creates the message and generates the feature exception
     */
    void is_feature_exception(const string& text,const string& feature);

  public:

    TUeye_config params_;              //!< Camera parameters
  	vector<unsigned char> image_data_; //!< Image data
  	HIDS hCam_;                        //!< Camera handler.
    SENSORINFO sensor_info_;           //!< Sensor information
    int img_data_size_;                //!< Image serialized data size
  	int img_step_;                     //!< Line increment.
  	bool camera_open_;                 //!< Check if the camera is open
    char* imgMem_;                     //!< Image memory
    double input_exposure; //read-in exposure time
    /*! \brief Object constructor
     *
     * Assigns names and initializes variables
     */
    CUeye_Camera();

    /*! \brief Object destructor
     */
    ~CUeye_Camera();

    /*! \brief List cameras
     *
     * Detect, list and print the ueye cameras connected to the PC
     */
    void list_cameras();
    void Enable_Event();


    int Wait_next_image();
    /*! \brief Initialize camera to take images
     *
     * Initialize camera, get sensor information, configure parameters, allocate memory and print parameters set
     */
    void init_camera();

    /*! \brief Get images
     *
     * Get images in the allocated memory
     */
    bool get_image();

    /*! \brief Set image format
     *
     * Set the requested image format
     */
    void set_img_format();

    /*! \brief Set pixel clock
     *
     * Set pixel clock.
     *
     * NOTE: A pixel clock change implies frame rate and exposure changes
     */
    void set_pixel_clock(const uint& pixel_clock);

    /*! \brief Set frame rate
     *
     * Set frame rate allowed in the interval of the sensor
     *
     * \param fps Desired frame rate.
     *
     * If 'fps' is 0 means an internal frame rate change due to other functions
     */
    void set_frame_rate(const double& fps);

    /*! \brief Set exposure
     *
     * Set exposure allowed in the interval of the sensor
     *
     * \param exp Desired exposure.
     *
     * If 'exp' is 0 means an internal frame rate change due to other functions
     */
    int set_hardware_gain(int gain);
    int get_hardware_gain();
    int set_HW_gain_factor(int gain_factor);
    //int query_gain_factor();
    void set_exposure(const double& exp);
    void getExposure();
    /*! \brief Set mirror
     *
     * Set camera mirroring, Updown and Leftright
     */
    void set_mirror();

    /*! \brief Set Area of Interest (AOI)
     *
     * Sets an area of interest.
     *
     * NOTE: AOI changes implies changes on the frame rate and memory allocation
     */
    void set_AOI(const int& x,const int & y,const int& width,const int& height);

    /*! \brief Close camera
     *
     * Close camera
     */
    void close_camera();
};

#endif
