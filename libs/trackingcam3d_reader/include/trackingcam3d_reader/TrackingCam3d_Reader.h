#ifndef TRACKINGCAM_READER_H
#define TRACKINGCAM_READER_H

#include <chrono>
#include <map>
#include <string>
#include <stdexcept>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>
#include "MyDepthIllustrator.h"
#include "MyFpsMeter.h"
#include "TrackingCam3d_Client.h"

class TrackingCamReader
{
public:
    // Constructor takes:
    // - addresses: camera server addresses
    // - stream_requests: commands like "left=1000"
    // - stream_names: short stream names like "left", "depth"
    // Possible stream names: "left", "left_rect", "left_gray_rect", "right"
    // "right_rect", "right_gray_rect", "depth", "disparity", "test"
    TrackingCamReader(const std::vector<std::string>& stream_requests);
    ~TrackingCamReader();

	bool connect();
    void disconnect();

    std::map<std::string, cv::Mat> getLatestFrames();
    void showImage(const std::string& img_type, const cv::Mat& image);

private:
	std::string server_addr;
    TrackingCam3d_Client cam_client;
    std::vector<std::string> stream_requests_;  // for example, {"left=1000", "depth=1000"}
    std::vector<std::string> stream_names;
};

#endif
