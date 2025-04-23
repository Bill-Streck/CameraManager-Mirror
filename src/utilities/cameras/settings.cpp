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
        fps = 5;
    } else if (c == lowish) {
        width = 320;
        height = 180;
        fps = 10;
    } else if (c == okayish) {
        width = 640;
        height = 360;
        fps = 5;
    } else if (c == okay) {
        width = 640;
        height = 360;
        fps = 10;
    } else if (c == okayer) {
        width = 640;
        height = 360;
        fps = 15;
    } else if (c == medium) {
        width = 1280;
        height = 720;
        fps = 10;
    } else if (c == mediumish) {
        width = 1280;
        height = 720;
        fps = 15;
    } else if (c == highish) {
        width = 1920;
        height = 1080;
        fps = 5;
    } else if (c == high) {
        width = 1920;
        height = 1080;
        fps = 10;
    } else if (c == higher) {
        width = 1920;
        height = 1080;
        fps = 15;
    } else if (c == highest) {
        width = 1920;
        height = 1080;
        fps = 20;
    }
}

std::string settings::get_resolution_for_ffmpeg() {
    return std::to_string(width) + "x" + std::to_string(height);
}

settings settings::deep_copy() {
    settings copy;
    copy.device_index = device_index;
    copy.width = width;
    copy.height = height;
    copy.fps = fps;
    copy.brightness = brightness;
    copy.contrast = contrast;
    copy.saturation = saturation;
    copy.sharpness = sharpness;
    copy.gain = gain;
    copy.enable_auto_white_balance = enable_auto_white_balance;
    return copy;
}

#endif
