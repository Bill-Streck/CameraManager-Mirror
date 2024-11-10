// FIXME don't leave file in this state
/**
 * @file H264Decoder.cpp
 * @author lsvng
 * @brief H264 Decoder
 * 
 */

#include "H264Decoder.hpp"

namespace Codec
{
  H264Decoder::H264Decoder()
    : mContext(nullptr)
    , mPacket(nullptr)
    , mFrame(nullptr)
  {
    AVCodec* wCodec;

    // Configure FFmpeg
    wCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!wCodec < 0)
    {
      printf("Error: Cannot find decoder.\n");
      exit(1);
    }

    mContext = avcodec_alloc_context3(wCodec); // decoder plug in
    if (!mContext)
    {
      printf("ERROR: Cannot allocate codec context.\n");
      exit(1);
    }

    av_opt_set(mContext->priv_data, "preset", "veryfast", 0);
    av_opt_set(mContext->priv_data, "tune", "zerolatency", 0);

    mPacket = av_packet_alloc();
    mPacket->data = nullptr;
    mPacket->size = 0;
    
    mFrame = av_frame_alloc();
    if (!mFrame) 
    {
      printf("Could not allocate video frame\n");
      exit(1);
    }

    mContext->pix_fmt = AV_PIX_FMT_BGR24;
    mFrame->format = mContext->pix_fmt;
    mFrame->width = mContext->width;
    mFrame->height = mContext->height;

    // avcodec_open2(mContext, wCodec, nullptr);
    if (avcodec_open2(mContext, wCodec, nullptr) < 0)
    {
      printf("ERROR: Cannot opening codec.\n");
      exit(1);
    }

    printf("FFmpeg Decoder is ready.\n");
  }

  H264Decoder::~H264Decoder()
  {
    av_frame_free(&mFrame);
    av_packet_free(&mPacket);
    avcodec_free_context(&mContext);

    printf("End of Decoder program.\n");
  }

  uint8_t* H264Decoder::decode(uint8_t* iEncodedData, ssize_t iSize)
  {
    if (!iEncodedData)
    {
      return nullptr;
    }

    mPacket->data = iEncodedData;
    mPacket->size = iSize; // packet length
    // R: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 0]
    // G: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 1]
    // B: mFrame->data[0][(3 * i * mImageWidth) + (j * 3) + 2]
    
    // Send mPacket to FFmpeg codec
    send(mContext, mPacket);

    // Receive mFrame from FFmpeg codec
    receive(mContext, mFrame);
    
    // Return decoded video data
    return &mFrame->data[0][0];
  }

  void H264Decoder::send(AVCodecContext* iContext, AVPacket* iPacket)
  {
    int wRet = avcodec_send_packet(iContext, iPacket);

    if (wRet < 0)
    {
      printf("Cannot send packets. Error: %d\n", wRet);
      iPacket = nullptr;
    }
  }

  void H264Decoder::receive(AVCodecContext* iContext, AVFrame* iFrame)
  {
    int wRet = avcodec_receive_frame(iContext, iFrame);
    
    if (wRet < 0)
    {
      printf("Cannot receive frames. Error: %d\n", wRet);
      iFrame = nullptr;
    }
  }
}