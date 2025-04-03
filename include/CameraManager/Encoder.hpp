/**
 * @file Encoder.hpp
 * @author William Streck
 * @brief Abstracted Encoder header.
 * @version 0.1
 * @date 2025-03-24
 * 
 */


#ifndef ENCODER_HPP
#define ENCODER_HPP 

#include <opencv2/opencv.hpp>
#include "settings.hpp"
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/avutil.h>
    #include <libavutil/opt.h>
}

#define ENCODER_NAME "libx264rgb" ///< Encoder name for OpenCV

using namespace std;

class Encoder {
    public:
        /**
         * @brief Create a fresh encoder for a thread.
         * 
         */
        Encoder();

        /**
         * @brief Destroy the encoder and any tied resources.
         * 
         */
        ~Encoder();

        /**
         * @brief Sets the context parameters for the encoder.
         * If pixel count does not match the input, the frame will be downsized if possible.
         * If the frame cannot be downsized, the context will be changed when the frame is submitted.
         * @param set settings Parameters for encoder.
         */
        // FIXME frame downsizing
        void set_parameters(settings set);

        /**
         * @brief Set the bitrate limit. Experimental.
         * 
         * @param bitrate Bitrate limit in BITS.
         */
        void set_bitrate(int bitrate);

        /**
         * @brief Sets the GOP proportion relative to fps
         * 
         * @param gop_proportion
         */
        void set_GOP_proportion(int gop_proportion);

        /**
         * @brief Encodes a frame into a vector of bytes.
         * 
         * @param frame frame to compress
         * @return uchar* byte array to transmit
         */
        pair<uchar*, int> encode(cv::Mat frame);

        /**
         * @brief Simple encoder flush
         * 
         */
        void flush_encoder();

    private:
        AVCodec* codec;
        AVCodecContext* context;
        AVFrame* frame;
        AVPacket* packet;
        int width;
        int height;

        /**
         * @brief Regenerates the encoder
         * 
         */
        void fresh_encoder();
};

#endif
