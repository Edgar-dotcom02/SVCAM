#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <thread>
#include <vector>

// #include "dxlSlave.h"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <trackingcam3d_reader/TrackingCam3d_Reader.h>

std::atomic<bool> stop{false};

void signalHandler(int signum) 
{
    stop.store(true, std::memory_order_relaxed);
    std::cout << std::endl << "SHUTDOWN: Received exit signal. Stopping program..." << std::endl;
}

void serializeCloud(const std::vector<float>& cloud_data, const std::string& filename) 
{
    std::ofstream out(filename, std::ios::binary);
    if (!out) 
    {
        std::cerr << "Failed to write " << filename << std::endl;
        return;
    }
    out.write(reinterpret_cast<const char*>(cloud_data.data()), cloud_data.size() * sizeof(float));
    out.close();
}

int main(int argc, char** argv)
{
    std::cout << std::fixed << std::setprecision(2);

    signal(SIGINT, signalHandler);
	
	// DxlSlave slave;
	// if (!slave.connect()) {
	// 	std::cerr << "DxlSlave connection error" << std::endl;
	// 	return 1;
	// }
	
	YAML::Node config;
	std::string config_base_path = "../config/";
	std::string config_path = config_base_path + "config.yaml";
	try {
		config = YAML::LoadFile(config_path);
	}
	catch (YAML::BadFile& e) 
    {
		// slave.disconnect();
		std::cerr << "Failed to load config file " << config_path << std::endl;
		return 1;
	}

    const double fx = config["camera"]["intrinsics"]["fx"].as<double>();
    const double fy = config["camera"]["intrinsics"]["fy"].as<double>();
    const double cx = config["camera"]["intrinsics"]["cx"].as<double>();
    const double cy = config["camera"]["intrinsics"]["cy"].as<double>();

    const float min_valid = config["pointcloud"]["min_valid_percent"].as<float>(5.0f);
    const int max_depth = config["pointcloud"]["max_depth_mm"].as<int>(5000);
    const int read_delay_ms = config["pointcloud"]["read_delay_ms"].as<int>(20);
    const int max_no_frames = config["pointcloud"]["max_no_frames_mm"].as<int>(100);

    const std::vector<std::string> stream_req = {"depth=100000"};
    TrackingCamReader cam_reader(stream_req);
 	if (!cam_reader.connect()) 
    {
 		// slave.disconnect();
 		std::cerr  << "Failed to connect camera client" << std::endl;
 		return 1;
 	}

    int no_frame_count = 0;
    std::cout << "Starting main program." << std::endl;

    while (!stop.load(std::memory_order_relaxed)) 
    {
        auto frames = cam_reader.getLatestFrames();
        if (frames.empty()) 
        {
            ++no_frame_count;
            if (no_frame_count > max_no_frames) 
			{
				std::cerr << "ERROR: No frames received for a long time. Stopping.\n";
				// slave.disconnect();
                stop.store(true, std::memory_order_relaxed);
				continue;
			}
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        no_frame_count = 0;

        // Extracting depth frames
        cv::Mat depth_raw;
        if (frames.find("depth") != frames.end() && !frames["depth"].empty()) {
            depth_raw = frames["depth"].clone();
        }
        else
        {
            // std::cout << "WARNING: No valid frames yet...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        // Check pixels validity
        const int total_px = depth_raw.total();
        const int valid_px = cv::countNonZero(depth_raw);
        const float valid_pct = 100.0f * valid_px / total_px;

        if (valid_pct < min_valid) 
        {
            // std::cout << "WARNING: Low validity, only " << std::fixed << std::setprecision(2) 
			//			 << valid_pixel_percent << "% valid depth. Skipping frame.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        // Build raw point data (X Y Z floats)
        std::vector<float> cloud_data;
        cloud_data.reserve(valid_px * 3);

        const uint16_t* row_ptr = depth_raw.ptr<uint16_t>(0);
        const int width = depth_raw.cols;
        const int height = depth_raw.rows;

        for (int v = 0; v < height; ++v, row_ptr += width) 
        {
            for (int u = 0; u < width; ++u) 
            {
                const uint16_t d_mm = row_ptr[u];
                if (d_mm == 0 || d_mm > max_depth) continue;

                const float z = d_mm / 1000.0f;
                const float x = z * (u - cx) / fx;
                const float y = z * (v - cy) / fy;

                cloud_data.push_back(x);
                cloud_data.push_back(y);
                cloud_data.push_back(z);
            }
        }

        // std::cout << "Frame " << frame_id << ": " << (cloud_data.size() / 3) << " points (" 
        //           << valid_pct << "% valid)" << std::endl;

        uint32_t num_points = cloud_data.size() / 3;
        if (num_points > 0) {
            // std::cout << "Sample: X=" << cloud_data[0] << " Y=" << cloud_data[1] << " Z=" 
            //           << cloud_data[2] << " m" << std::endl;
        }

        // Serialize and save the data as a binary
        const std::string bin_name = "../out/cloud_sample.bin";
        serializeCloud(cloud_data, bin_name);

        // Saving as a .csv file
        // std::ofstream csv("../out/cloud_sample.csv");
        // csv << "X,Y,Z\n";
        // for (size_t i = 0; i < cloud_data.size(); i += 3) {
        //     csv << cloud_data[i] << "," << cloud_data[i+1] << "," << cloud_data[i+2] << "\n";
        // }
        // csv.close();
        // std::cout << "Saved readable CSV: cloud_sample.csv\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms));
    }

    cam_reader.disconnect();
    // slave.disconnect();
    std::cout << "Program shutdown." << std::endl;
    return 0;
}