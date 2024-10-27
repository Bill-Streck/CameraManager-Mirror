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

/**
 * @brief Allows use of clarity presets. Advised.
 * 
 */
enum clarity {
    lowest,
    low,
    okay,
    medium,
    high,
    highest,
    custom
};

/**
 * @brief settings class to configure the camera.
 * Requires device index.
 * Supports several optional parameters,
 * but if using custom clarity mandatory parameters are width, height, and fps.
 * @note default values are set to 50 for all ranged optional parameters.
 */
class settings {
    public:
        // mandatory
        int device_index;
        int width = 0;
        int height = 0;
        int fps = 0;

        // optional
        // TODO test which ones work on the Logitech cams
        int brightness = 50; // out of 100
        int contrast = 50; // out of 100
        int saturation = 50; // out of 100
        int hue = 50; // out of 100
        int gain = 50; // out of 100
        int exposure = 50; // out of 100
        int gamma = 50; // out of 100
        int enable_auto_white_balance = 0; // auto white balance enable - 0 (disabled) or 1 (enabled)

        /**
         * @brief Allows use of a clarity preset to configure the settings class automatically.
         * 
         * @param c clarity preset to use.
         */
        void use_preset(clarity c);
};

#endif
