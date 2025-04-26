/**
 * @file camera_thread.hpp
 * @author William Streck
 * @brief Shares the camera thread function.
 * @version 0.1
 * @date 2025-02-05
 * 
 */

#ifndef CAMERA_THREAD_HPP
#define CAMERA_THREAD_HPP

#include "camera_object.hpp"
#include "command_generation.hpp"
#include "streaming.hpp"

// Compatibility with certain development configurations (e.g. Wanderer2)
#ifndef SIGKILL
    #include <signal.h>
#endif

using namespace std;

#define CAMERA_ID_FAIL -400 ///< Failure identifier for attaining a camera ID
#define CAMERA_FAILURE_RETRY 400 ///< Sleep time for the handler loop in milliseconds when a camera fails to start.

extern list<int> threads_end; ///< List of threads to end.
extern map<int, Camera> cameras; ///< Map of camera objects. Used for seeing if a camera is on.
extern bool running; ///< Running flag for the handler loop. Doesn't need to be atomic.
extern map<int, string> local_cams; ///< Map of cameras that are being used locally.
extern map<int, string> streaming_cams; ///< Map of cameras that are streaming. Used for ffmpeg process management.
extern map<int, map<string, int>> cam_command_map; ///< Map of end flags for each thread, regardless of type.

/**
 * @brief Logitech C920 camera thread function.
 * 
 * @param parsed Information map for camera operation.
 * @param tmap_index Numeric index in thread map. Of minimal significance.
 */
void logi_cam_thread(map<string, int> parsed, int tmap_index);

/**
 * @brief This is only here to work numeric safety around clarity.
 * Just returns the same number so long as it's valid.
 * 
 * @param quality The quality to verify.
 * @return clarity The resulting clarity, ideally the exact same but casted.
 */
clarity preset_from_quality(int quality);

#endif
