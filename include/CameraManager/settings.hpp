/**
 * @file settings.hpp
 * @author William Streck
 * @brief Contains the settings class to configure the camera.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <string>

/**
 * @brief Allows use of clarity presets. Advised.
 * 
 */
enum clarity {
    lowest, ///< 160x90, 5fps
    low, ///< 320x180, 5fps
    lowish, ///< 320x180, 10fps
    okayish, ///< 640x360, 5fps
    okay, ///< 640x360, 10fps
    okayer, ///< 640x360, 15fps
    medium, ///< 1280x720, 10fps
    mediumish, ///< 1280x720, 15fps
    highish, ///< 1920x1080, 5fps
    high, ///< 1920x1080, 10fps
    higher, ///< 1920x1080, 15fps
    highest ///< 1920x1080, 20fps
};

#define CLARITY_PRESETS highest

#define GENERIC_ATTRBUTE_LIMIT_LOW 0 ///< Low limit for generic attributes on the logitech cams
#define GENERIC_ATTRBUTE_LIMIT_HIGH 255 ///< High limit for generic attributes on the logitech cams

// Basic attributes
#define ATTR_BRIGHTNESS 0 ///< 0-255 - no recommended changes - use as needed
#define ATTR_CONTRAST 1 ///< 0-255 - increasing to around 150 can be helpful with consistent lighting
#define ATTR_SATURATION 2 ///< 0-255 - increasing in general makes things very clear
#define ATTR_SHARPNESS 3 ///< 0-255 - recommended to increase significantly at higher resolutions
#define ATTR_GAIN 4 ///< 0-255 - some cameras automatically increase this - manual changes may not work
#define ATTR_AUTO_WHITE_BALANCE 5 ///< 0 (disabled) or 1 (enabled) - not recommended to change

// Resolution and fps - requires special handling
#define ATTR_LOCAL_RESOLUTION 6 ///< Local resolution - requires special handling
#define ATTR_STREAM_RESOLUTION 7 ///< Streaming resolution - requires special handling
#define ATTR_LOCAL_FPS 8 ///< Local fps - requires special handling
#define ATTR_STREAM_FPS 9 ///< Streaming fps - requires special handling
#define ATTR_INTERNAL_FPS 10 ///< Internal fps - Special case called when we need change fps on both. (RARE)
#define ATTR_INTERNAL_RES 11 ///< Internal resolution - Special case called when both are low. (May occur, uncommon)

/**
 * @brief settings class to configure the camera.
 * Requires device index.
 * Supports several optional parameters,
 * @note Default values are from the logitech cameras used on the rover.
 * Other cameras need to manually set their settings or pull the default value from the V4L2 driver.
 */
class settings {
    public:
        // mandatory
        int device_index; // device index for the camera - corresponds either to wrist (-1) or logitech naming convention
        double width = 0; // width of the camera in pixels - 160-1920
        double height = 0; // height of the camera in pixels - 90-1080
        double fps = 0; // frames per second - 5-20 - 30 is possible but not recommended

        // optional
        double brightness = 128; // 0-255 - no recommended changes - use as needed
        double contrast = 128; // 0-255 - increasing to around 150 can be helpful with consistent lighting
        double saturation = 128; // 0-255 - increasing in general makes things very clear
        double sharpness = 128; // 0-255 - recommended to increase significantly at higher resolutions
        double gain = 0; // 0-255 - some cameras automatically increase this - manual changes may not work
        double enable_auto_white_balance = 1; // auto white balance enable - 0 (disabled) or 1 (enabled) - recommended to leave enabled

        /**
         * @brief Allows use of a clarity preset to configure the settings class automatically.
         * 
         * @param c clarity preset to use.
         */
        void use_preset(clarity c);

        /**
         * @brief Get the resolution for ffmpeg.
         * 
         * @return std::string resolution string for ffmpeg.
         */
        std::string get_resolution_for_ffmpeg();

        /**
         * @brief Return a deep copy of the settings.
         * Used when making separate streaming and local settings.
         * 
         * @return settings 
         */
        settings deep_copy();
};

#endif
