/**
 * @file Decoder.hpp
 * @author William Streck
 * @brief Decoder header, abstracted.
 * @note Inactive on rover. Adapted or adaptable to base station utility.
 * @version 0.1
 * @date 2025-03-25
 * 
 */

#ifndef DECODER_HPP
#define DECODER_HPP

#include <opencv2/opencv.hpp>
#include "settings.hpp"
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/avutil.h>
    #include <libavutil/opt.h>
}

 #endif
 