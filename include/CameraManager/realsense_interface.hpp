/**
 * @file realsense_interface.hpp
 * @author William Streck
 * @brief Interface functions for the realsense package.
 * @version 0.1
 * @date 2025-02-22
 * 
 */

#ifndef REALSENSE_INTERFACE_HPP
#define REALSENSE_INTERFACE_HPP

#include "camera_thread.hpp" // for externs

using namespace std;

#define FPS_SLEEP_BUFF 5 ///< Amount to increase perceived fps by to ensure sleeping does not cause frame loss.

/**
 * @brief Intel RealSense camera thread function.
 * Directly interfaces with realsense-ros package.
 * 
 * @param parsed Camera information map.
 * @param tmap_index Numeric index in thread map. Of minimal significance.
 */
void realsense_cam_thread(map<string, string> parsed, int tmap_index);

#endif
