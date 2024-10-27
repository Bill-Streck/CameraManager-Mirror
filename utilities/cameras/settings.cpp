/**
 * @file settings.cpp
 * @author William Streck
 * @brief helper functions for settings class
 * @version 0.1
 * @date 2024-10-25
 * 
 */

#ifndef SETTINGS_CPP
#define SETTINGS_CPP

#include "settings.hpp"

void settings::use_preset(clarity c) {
    if (c == lowest) {
        width = 160;
        height = 90;
        fps = 5;
    } else if (c == low) {
        width = 320;
        height = 180;
        fps = 10;
    } else if (c == okay) {
        width = 640;
        height = 360;
        fps = 15;
        // TODO there was something else in here
    } else if (c == medium) {
        width = 1280;
        height = 720;
        fps = 20;
    } else if (c == high) {
        width = 1920;
        height = 1080;
        fps = 25;
    } else if (c == highest) {
        // FIXME absolutely not
        width = 3840;
        height = 2160;
        fps = 30;
    }
    // if custom, do nothing - put in for vanity
}

#endif
