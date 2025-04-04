/**
 * @file Encoder.cpp
 * @author your name (you@domain.com)
 * @brief Encoder implementation, hidden. Must (or really really should) match decoder internally.
 * @version 0.1
 * @date 2025-03-24
 * 
 */

#include "Encoder.hpp"
#include <libavutil/imgutils.h>
// #include "ffmpeg_encoder_decoder/encoder.hpp"

#define TEMP

#ifdef TEMP
Encoder::Encoder() {}
Encoder::~Encoder() {}

// temporarily just avif encode in image
vector<uchar> Encoder::encode(cv::Mat image) {
    std::vector<uchar> buffer;
    std::vector<int> params = {
        cv::IMWRITE_AVIF_QUALITY, 40,
        cv::IMWRITE_AVIF_SPEED, 4,
    };
    bool success = cv::imencode(".avif", image, buffer, params);

    if (!success) {
        // TODO don't do that
        throw "Could not encode image.";
    }

    return buffer;
}
#endif

#ifdef PERMANENT
Encoder::Encoder() {
    // Set up the codec
    codec = avcodec_find_encoder_by_name(ENCODER_NAME);
    if (!codec) {
        // TODO don't do that
        throw "Could not find encoder.";
    }

    context = avcodec_alloc_context3(codec);
    if (!context) {
        // TODO don't do that
        throw "Could not allocate context.";
    }

    av_opt_set(context->priv_data, "tune", "zerolatency", 0);
    av_opt_set(context->priv_data, "preset", "ultrafast", 0);
    
}

Encoder::~Encoder() {
    // Free the frame
    av_frame_free(&frame);

    // Free the context
    avcodec_free_context(&context);

    // Free the packet
    av_packet_free(&packet);
}

void Encoder::set_parameters(settings set) {
    // Set the context parameters
    context->width = set.width;
    context->height = set.height;
    context->time_base = (AVRational){1, set.fps};
    context->framerate = (AVRational){set.fps, 1};
    context->gop_size = set.fps; // Default to fps for GOP size
    context->max_b_frames = 0; // No B frames - they cause latency
    context->pix_fmt = AV_PIX_FMT_BGR24;

    // FIXME try and use VAAPI for hardware encoding

    // Experimental
    // context->bit_rate = set.bitrate;
    // TODO array of default bit rates by quality

    // TODO see what happens if you double set the parameters
    auto ret = avcodec_open2(context, codec, NULL);
    if (ret < 0) {
        // TODO don't do that
        throw "Could not open codec.";
    }

    // Allocate the frame
    frame = av_frame_alloc();
    if (!frame) {
        // TODO don't do that
        throw "Could not allocate frame.";
    }

    frame->format = context->pix_fmt;
    frame->width = context->width;
    frame->height = context->height;
    frame->linesize[0] = context->width * 3;

    // TODO test without this
    ret = av_image_alloc(
        frame->data, frame->linesize, width, height, static_cast<AVPixelFormat>(context->pix_fmt), 64);

    packet = av_packet_alloc();
    if (!packet) {
        // TODO don't do that
        throw "Could not allocate packet.";
    }
    packet->data = nullptr;
    packet->size = 0;

    // TODO wrapper frame?

    ret = av_frame_get_buffer(frame, 0);
}

void Encoder::set_bitrate(int bitrate) {
    // Experimental
    // TODO test
    context->bit_rate = bitrate;
}

void Encoder::set_GOP_proportion(int gop_proportion) {
    // Set the GOP proportion
    // TODO test
    // TODO check for math and divide by zero I didn't even read this.
    context->gop_size = context->framerate.num / gop_proportion;
}

pair<uchar*, int> Encoder::encode(cv::Mat image) {
    frame->data[0] = image.data;
    auto ret = avcodec_send_frame(context, frame);
    if (ret < 0) {
        // TODO don't do that
        throw "Could not send frame.";
    }

    ret = avcodec_receive_packet(context, packet);
    if (ret < 0) {
        // TODO don't do that
        throw "Could not receive packet.";
    }

    auto size = packet->size;

    return make_pair(packet->data, size);
}

void Encoder::flush_encoder() {
    // Flush the encoder
    avcodec_send_frame(context, nullptr);
    while (true) {
        auto ret = avcodec_receive_packet(context, packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            // TODO don't do that
            throw "Could not receive packet.";
        }
        av_packet_unref(packet);
    }
}
#endif