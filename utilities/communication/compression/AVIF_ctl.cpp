#include "AVIF_ctl.hpp"

std::vector<int> params; ///< Parameters for the AVIF encoder.

void avif_init(void) {
    // encoder params
    params.push_back(cv::IMWRITE_AVIF_QUALITY);
    params.push_back(AVIF_QUALITY);
    params.push_back(cv::IMWRITE_AVIF_SPEED);
    params.push_back(AVIF_SPEED);
}

void avif_cleanup(void) {
    // encoder params
    params.clear();
    params.shrink_to_fit();
}

void avif_set_quality(int quality) {
    params[1] = quality;
}

bool compress_to_avif(const cv::Mat frame, std::vector<uchar> &buf) {
    if (frame.empty()) {
        std::cerr << "Empty frame" << std::endl;
        return false;
    }

    return cv::imencode(".avif", frame, buf, params);
}

cv::Mat decompress_from_avif(const std::vector<uchar> data) {
    return cv::imdecode(data, cv::IMREAD_COLOR);
}
