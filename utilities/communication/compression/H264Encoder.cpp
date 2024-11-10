// FIXME don't leave the file in this state
/**
 * @file H264Encoder.cpp
 * @author lsvng
 * @brief H264 VEncoder
 * 
 */

#include "H264Encoder.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>

namespace Codec
{
  H264Encoder::H264Encoder()
    : mContext(nullptr)
    , mPacket(nullptr)
    , mFrame(nullptr)
  {
    av_register_all();

    AVCodec* wCodec;

    // Configure FFmpeg
    wCodec = avcodec_find_encoder_by_name("libx264rgb");
    if (!wCodec)
    {
      printf("Error: Cannot find encoder.\n");
      exit(1);
    }

    mContext = avcodec_alloc_context3(wCodec);
    if (!mContext)
    {
      printf("ERROR: Cannot start codec context.\n");
      exit(1);
    }

    av_opt_set(mContext->priv_data, "tune", "zerolatency", 0);
    av_opt_set(mContext->priv_data, "preset", "veryfast", 0);
    // av_opt_set(mContext->priv_data, "profile", "high444", 0);
    av_opt_set(mContext->priv_data, "aspect", "16:9", 0);
    // av_opt_set(mContext->priv_data, "vf", "yadif=0:1:0,scale=640:360,pad=640:360:24:0:black", 0);

    mContext->bit_rate     = 4000000;
    mContext->width        = 640; // resolution must be a multiple of two..
    mContext->height       = 360;
    mContext->time_base    = {1, 15};
    mContext->gop_size     = 15;
    // mContext->max_b_frames = 1;
    mContext->framerate    = {15, 1};

    // streamRate = 100?
    // TODO check pix fmt
    mContext->pix_fmt      = AV_PIX_FMT_BGR24;
    int wRet;
    wRet = avcodec_open2(mContext, wCodec, nullptr);
    if (wRet < 0)
    {
      printf("ERROR: Cannot opening codec.\n");
      exit(1);
    }

    mPacket = av_packet_alloc();
    mPacket->data = nullptr;
    mPacket->size = 0;

    mFrame = av_frame_alloc();
    if (!mFrame) 
    {
      printf("ERROR: Could not allocate video frame\n");
      exit(1);
    }

    mFrame->format = mContext->pix_fmt;
    mFrame->width = mContext->width;
    mFrame->height = mContext->height;

    wRet = av_frame_get_buffer(mFrame, 0);
    if (wRet < 0)
    {
      printf("ERROR: Cannot get frame buffer.\n");
      exit(1);
    }

    printf("FFmpeg encoder is ready.\n");
  }

  H264Encoder::~H264Encoder()
  {
    av_frame_free(&mFrame);
    avcodec_free_context(&mContext);
    av_packet_free(&mPacket);

    printf("End of H264Encoder program.\n");
  }

  uint8_t* H264Encoder::encode(uint8_t* iRawData)
  {
    if (!iRawData)
    {
      return nullptr;
    }

    mFrame->data[0] = iRawData;
    // R: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 0]
    // G: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 1]
    // B: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 2]

    send(mContext, mFrame);

    // Receive mPacket from FFmpeg codec
    receive(mContext, mPacket);

    // Return encoded video data
    return mPacket->data;
  }

  int H264Encoder::getPacketSize()
  {
    if (!mPacket->data)
    {
      return 0;
    }

    return mPacket->size;
  }

  void H264Encoder::send(AVCodecContext* iContext, AVFrame* iFrame)
  {
    int wRet = avcodec_send_frame(mContext, mFrame);
    if (wRet < 0)
    {
      printf("Cannot send packets. Error: %d\n", wRet);
      iFrame = nullptr;
    }
  }

  void H264Encoder::receive(AVCodecContext* iContext, AVPacket* iPacket)
  {
   int wRet = avcodec_receive_packet(iContext, iPacket);
    if (wRet < 0)
    {
      printf("Cannot encode frame. Error: %d\n", wRet);
      iPacket = nullptr;
    }
  }
}