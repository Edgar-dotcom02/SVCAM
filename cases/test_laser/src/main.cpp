#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <signal.h>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <trackingcam3d_reader/TrackingCam3d_Reader.h>
#include "dxlSlave.h"

std::atomic<bool> stop{false};

void signalHandler(int signum) 
{
    stop.store(true, std::memory_order_relaxed);
    std::cout << std::endl << "SHUTDOWN: Received exit signal. Stopping program..." << std::endl;
}

int main(int argc, char** argv) 
{
    signal(SIGINT, signalHandler);
	
	DxlSlave slave;
	if (!slave.connect()) {
		std::cerr << "DxlSlave connection error" << std::endl;
		return 1;
	}
	
	YAML::Node config;
	std::string config_base_path = "../config/";
	std::string config_path = config_base_path + "config.yaml";
	try {
		config = YAML::LoadFile(config_path);
	}
	catch (YAML::BadFile& e) 
    {
		slave.disconnect();
		std::cerr << "Failed to load config file " << config_path << std::endl;
		return 1;
	}

    // Camera intrinsics
    const double fx = config["camera"]["intrinsics"]["fx"].as<double>();
    const double fy = config["camera"]["intrinsics"]["fy"].as<double>();
    const double cx = config["camera"]["intrinsics"]["cx"].as<double>();
    const double cy = config["camera"]["intrinsics"]["cy"].as<double>();
    const double inv_fx = 1.0 / fx;
    const double inv_fy = 1.0 / fy;

    // Laserscan params
    const int num_readings = config["laserscan"]["num_readings"].as<int>();
    const float angle_min = config["laserscan"]["angle_min"].as<float>();
    const float angle_max = config["laserscan"]["angle_max"].as<float>();
    const float range_min = config["laserscan"]["range_min"].as<float>();
    const float range_max = config["laserscan"]["range_max"].as<float>();
    const float k_pixel = config["laserscan"]["k_pixel"].as<float>();
    const float min_valid_percent = config["laserscan"]["min_valid_percent"].as<float>();
    const int read_delay_ms = config["laserscan"]["read_delay_ms"].as<int>();
    const int max_no_frames = config["laserscan"]["max_no_frames"].as<int>();

    const std::vector<std::string> stream_req = {"depth=1000000"};
    TrackingCamReader cam_reader(stream_req);
 	if (!cam_reader.connect()) 
    {
 		slave.disconnect();
 		std::cerr  << "Failed to connect camera client" << std::endl;
 		return 1;
 	}

    int no_frame_count = 0;
    for (int i = 1; i < num_readings+1; ++i) 
    {
        slave.callMethod(i, 0);
    }

    std::cout << "Starting laserscan." << std::endl;

    while (!stop.load(std::memory_order_relaxed)) 
    {
        auto frames = cam_reader.getLatestFrames();
        if (frames.empty()) 
        {
            ++no_frame_count;
            if (no_frame_count > max_no_frames) 
			{
				std::cerr << "ERROR: No frames received for a long time. Stopping.\n";
                stop.store(true, std::memory_order_relaxed);
				continue;
			}
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        no_frame_count = 0;

        // Extract RGB and depth
        cv::Mat depth_raw;
        if (frames.find("depth") != frames.end() && !frames["depth"].empty()) {
            depth_raw = frames["depth"].clone();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        // Resize depth to RGB size (640x480) for alignment
        cv::Mat depth_resized;
        cv::resize(depth_raw, depth_resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        depth_resized.convertTo(depth_resized, CV_32FC1, 0.001); // mm to m

        // Validity check
        const int total_px = depth_raw.total();
        const int valid_px = cv::countNonZero(depth_raw);
        const float valid_pct = 100.0f * valid_px / total_px;
        if (valid_pct < min_valid_percent) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        // Initialize scan ranges
        std::vector<uint8_t> ranges(num_readings, 
                            static_cast<uint8_t>(round(range_max*100)));
        std::vector<float> dist_min(num_readings, range_max);

        // ROI depth is the resized float depth (m)
        const cv::Mat& roi_depth = depth_resized;

        // Scan rows and columns
        for (int y = 0; y < roi_depth.rows - 25; ++y) 
        {
            for (int x_bin = 0; x_bin < num_readings; ++x_bin) 
            {
                // Sample pixel
                int x = static_cast<int>(roi_depth.cols - x_bin * k_pixel - 1);
                if (x < 0 || x >= roi_depth.cols) continue;

                float point_z = roi_depth.at<float>(y, x);
                if (point_z <= range_min) continue; // Filter

                float point_x = ((x + roi_depth.cols / 2.0f) - cx) * point_z * inv_fx;
                float point_y = ((y + roi_depth.rows / 2.0f) - cy) * point_z * inv_fy;
                float dist = std::sqrt(point_x * point_x + point_z * point_z + point_y * point_y);

                if (dist < dist_min[x_bin]) {
                    dist = std::sqrt(point_x * point_x + point_z * point_z);
                    dist_min[x_bin] = dist;
                    ranges[x_bin] = static_cast<uint8_t>(round(dist*100));
                }
            }
        }

        for (int i = 1; i < ranges.size() + 1; ++i) 
        {
            slave.callMethod(i, (int)ranges[i-1]);
        }

        // float min_range = range_max;
        // if (!ranges.empty()) {
        //     min_range = *std::min_element(ranges.begin(), ranges.end());
        // }
        // std::cout << "min range = " << min_range << " m)\n";

        // Save .csv file of ranges
        // std::ofstream csv("../out/scan_sample.csv");
        // csv << "range_m\n";
        // for (float r : ranges) csv << r << "\n";
        // csv.close();

        std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms));
    }

    for (int i = 1; i < num_readings+1; ++i) 
    {
        slave.callMethod(i, 0);
    }

    cam_reader.disconnect();
    slave.disconnect();
    std::cout << "Shutdown complete.\n";
    return 0;
}
