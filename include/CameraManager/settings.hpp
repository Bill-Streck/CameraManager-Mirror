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

// TODO should we define this as light ordinal of highest or something?
#define CLARITY_PRESETS 9

/**
 * @brief Allows use of clarity presets. Advised.
 * 
 */
enum clarity {
    lowest, ///< 160x90, 5fps
    low, ///< 320x180, 5fps
    lowish, ///< 320x180, 10fps
    okay, ///< 640x360, 10fps
    okayish, ///< 640x360, 15fps
    medium, ///< 1280x720, 10fps
    mediumish, ///< 1280x720, 15fps
    high, ///< 1920x1080, 10fps
    higher, ///< 1920x1080, 15fps
    highest ///< 1920x1080, 20fps
};

#define GENERIC_ATTRBUTE_LIMIT_LOW 0 ///< Low limit for generic attributes on the logitech cams
#define GENERIC_ATTRBUTE_LIMIT_HIGH 255 ///< High limit for generic attributes on the logitech cams
#define ATTR_BRIGHTNESS 0 ///< 0-255 - no recommended changes - use as needed
#define ATTR_CONTRAST 1 ///< 0-255 - increasing to around 150 can be helpful with consistent lighting
#define ATTR_SATURATION 2 ///< 0-255 - increasing in general makes things very clear
#define ATTR_SHARPNESS 3 ///< 0-255 - recommended to increase significantly at higher resolutions
#define ATTR_GAIN 4 ///< 0-255 - some cameras automatically increase this - manual changes may not work
#define ATTR_AUTO_WHITE_BALANCE 5 ///< 0 (disabled) or 1 (enabled) - not recommended to change

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
        int device_index;
        double width = 0;
        double height = 0;
        double fps = 0;

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
};

#endif
