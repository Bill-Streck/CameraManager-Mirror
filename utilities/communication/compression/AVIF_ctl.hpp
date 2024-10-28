#ifndef AVIF_CTL_H
#define AVIF_CTL_H

// #include <avif/avif.h>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Starts the AVIF encoder.
 * 
 */
void avif_init(void);

/**
 * @brief Cleans up the AVIF encoder.
 * Not all too necessary, but good practice.
 * 
 */
void avif_cleanup(void);

/**
 * @brief Allows for setting the global AVIF quality.
 * 
 * @param quality 
 */
void avif_set_quality(int quality);

/**
 * @brief Compresses a frame to AVIF.
 * 
 * @param frame the frame to compress.
 * // TODO doc check
 * @return std::vector<uint8_t> the compressed frame as a vector of bytes.
 */
bool compress_to_avif(const cv::Mat frame, std::vector<uchar> &buf);

// TODO decoder may ultimately be removed
/**
 * @brief Decompresses an AVIF frame.
 * 
 * @param data compressed input data.
 * @return cv::Mat the decompressed frame.
 */
cv::Mat decompress_from_avif(const std::vector<uchar> data);

#endif