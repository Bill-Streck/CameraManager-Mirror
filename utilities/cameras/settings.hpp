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
    // TODO assign these properly and comment them
    lowest,
    low,
    okay,
    medium,
    high,
    highest
};

/**
 * @brief settings class to configure the camera.
 * Requires device index.
 * Supports several optional parameters,
 * @note default values are set to 50 for all ranged optional parameters.
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
