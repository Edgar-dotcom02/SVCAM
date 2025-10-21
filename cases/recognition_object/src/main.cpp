#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <trackingcam3d_reader/TrackingCam3d_Reader.h>
#include "dxlSlave.h"

#define STATUS_ADDR 24
#define DIST_ADDR   25

std::atomic<bool> stop{false};
void signalHandler(int signum) 
{
	stop.store(true, std::memory_order_relaxed);
	std::cout << std::endl << "SHUTDOWN: Received exit signal. Stopping program..." << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << std::fixed << std::setprecision(2);

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
	catch (YAML::BadFile& e) {
		slave.disconnect();
		std::cerr << "Failed to load config file " << config_path << std::endl;
		return 1;
	}
	
	// Camera intrinsics
    const double inv_fx = 1 / config["camera"]["intrinsics"]["fx"].as<double>();
    const double inv_fy = 1 / config["camera"]["intrinsics"]["fy"].as<double>();
    const double m_cx = config["camera"]["intrinsics"]["cx"].as<double>();
    const double m_cy = config["camera"]["intrinsics"]["cy"].as<double>();
    
	const std::vector<std::string> stream_requests = 
		config["camera"]["stream_requests"].as<std::vector<std::string>>();

    const bool display_flag = config["display"].as<bool>();
    const int max_no_frames = config["max_no_frames"].as<int>();
    const int read_delay_ms = config["read_delay_ms"].as<int>();
	const float min_valid_pixel_percent = config["min_valid_pixels_percent"].as<float>();
	const double max_depth = config["max_depth_mm"].as<double>();
    
 	TrackingCamReader cam_reader(stream_requests);
 	if (!cam_reader.connect()) 
	{
 		slave.disconnect();
 		std::cerr  << "Failed to connect camera client" << std::endl;
 		return 1;
 	}
 	
	cv::Mat last_left, last_depth;
    int no_frame_count = 0;
	int status_code = 1; // Ready or working
	
    std::cout << "Starting Circle detection." << std::endl;
	slave.callMethod(STATUS_ADDR, status_code); 
    
    while (!stop.load(std::memory_order_relaxed))
    {
		int dist_mm = 0;

		auto frames = cam_reader.getLatestFrames();
		if (frames.empty()) 
		{
			++no_frame_count;
			if (no_frame_count > max_no_frames) 
			{
				status_code = 3; // Error
				slave.callMethod(STATUS_ADDR, status_code); 
                slave.callMethod(DIST_ADDR, dist_mm);
				std::cerr << "ERROR: No frames received for a long time. Stopping.\n";
				stop.store(true, std::memory_order_relaxed);
				continue;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
			continue;
		}

		no_frame_count = 0;

		// Extract RGB and depth frames
		if (frames.find("left") != frames.end() && !frames["left"].empty()) {
			last_left = frames["left"].clone();
		}
		if (frames.find("depth") != frames.end() && !frames["depth"].empty()) {
			last_depth = frames["depth"].clone();
		}

		if (last_left.empty() || last_depth.empty()) 
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
			continue;
		}

		// Convert RGB (no resize)
		cv::Mat rgb;
		cv::cvtColor(last_left, rgb, cv::COLOR_YUV2BGR_YUYV);

		// Resize depth to match RGB
		cv::Mat depth;
		cv::resize(last_depth, depth, cv::Size(rgb.cols, rgb.rows), 0, 0, cv::INTER_LINEAR);

		// Check depth values
		double min_val = 0, max_val = 0;
		cv::minMaxLoc(last_depth, &min_val, &max_val);
		if (min_val == 0 && max_val == 0) 
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
			continue;
		}

		bool valid_frame = true;
		int valid_pixels = cv::countNonZero(last_depth);
		double valid_pixel_percent = 0.0;

		if (max_val > max_depth) 
		{
			valid_frame = false;
		}
		if (valid_pixels) 
		{
			valid_pixel_percent = 100.0 * valid_pixels / (last_depth.cols * last_depth.rows);
			// std::cout << "Valid depth pixels: " << valid_pixels << " / " << (last_depth.cols * last_depth.rows) 
			        //   << " (" << std::fixed << std::setprecision(2) << valid_pixel_percent << "%)\n";
		}
		if (valid_pixel_percent < min_valid_pixel_percent) 
		{
			valid_frame = false;
		}

		if (!valid_frame)
		{
			// std::cout << "WARNING: Invalid frame. Skipping" << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
			continue;
		}

		// Convert to grayscale and preprocess
		cv::Mat gray;
		cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
		cv::medianBlur(gray, gray, 5);

		// Detect circles
		std::vector<cv::Vec3f> circles;
		int min_rad = 20; int max_rad = 60;
		cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 2,
							100, 30, min_rad, max_rad); // param1=100, param2=30, minRadius=20, maxRadius=60

		float best_dist = 5.0f; // Reset for the current frame
		float best_x = 0.0f;
		float best_y = 0.0f;
		float best_r = 0.0f;
		bool circle_found_in_frame = false;

		if (!circles.empty()) 
		{
			for (const auto& circle : circles) 
			{
				float x = circle[0], y = circle[1], r = circle[2];
				if (r <= 0 || r > max_rad) 
				{
					continue; // Invalid circle radius
				}

				// Circle ROI
				int roi_x = std::max(0, static_cast<int>(x - r));
				int roi_y = std::max(0, static_cast<int>(y - r));
				int roi_w = std::min(static_cast<int>(2 * r), depth.cols - roi_x);
				int roi_h = std::min(static_cast<int>(2 * r), depth.rows - roi_y);

				if (roi_w <= 0 || roi_h <= 0) 
				{
					continue; // Invalid circle ROI
				}
				
				cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
				cv::Mat roi_depth = depth(roi);

				// Compute mean depth (16UC1, mm)
				double sum = 0.0;
				int count = 0;
				double roi_min, roi_max;
				cv::minMaxLoc(roi_depth, &roi_min, &roi_max);

				for (int i = 0; i < roi_depth.rows; ++i)
				{
					for (int j = 0; j < roi_depth.cols; ++j) 
					{
						uint16_t d = roi_depth.at<uint16_t>(i, j);
						if (d > 0 && d < 5000) {
							sum += d;
							++count;
						}
					}
				}
				
				if (count > 0) 
				{
					float dist = 0;
					float point_z = (sum / count) * 0.001f; // mm to meters
					float point_x = (x - m_cx) * point_z * inv_fx;
					float point_y = (y - m_cy) * point_z * inv_fy;
					dist = std::sqrt(point_x * point_x + point_y * point_y + point_z * point_z);
					
					if (dist < best_dist) {
						best_x = x;
						best_y = y;
						best_dist = dist;
						best_r = r;
						circle_found_in_frame = true;
					}
				}
			}

			if (circle_found_in_frame && best_dist < 2.0) 
			{
				dist_mm = (int)(best_dist * 1000);
				cv::Point2f center(best_x, best_y);

				std::cout << "Distance to closest circle: " << best_dist << " m" << std::endl;
				
				if (display_flag) 
				{
					// Draw circle using the best variables
					cv::circle(rgb, center, (int)best_r, cv::Scalar(0, 255, 0), 2);
					std::string dist_str = "Dist: " + std::to_string(best_dist).substr(0, 4) + "m";
					cv::putText(rgb, dist_str,
								cv::Point(center.x, center.y - 5),
								cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
				}
			}
			else 
			{
				dist_mm = 0; // No valid circle found in this frame
			}
		}
		else 
		{
			dist_mm = 0;
		}

		slave.callMethod(DIST_ADDR, dist_mm);

		// Display RGB image
		if (display_flag) 
		{
			cv::Mat rgb_resized;
			cv::resize(rgb, rgb_resized, cv::Size(320, 240));
			cv::imshow("Circle Detection", rgb_resized);
			if (cv::waitKey(1) == 27) 
			{
				stop.store(true, std::memory_order_relaxed);
				continue;
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms));
    }

	slave.callMethod(DIST_ADDR, 0);
    if (status_code != 3) status_code = 2; // shutdown without error
	slave.callMethod(STATUS_ADDR, status_code); 
 	
 	if (display_flag) cv::destroyAllWindows();
    cam_reader.disconnect();
    slave.disconnect();
    std::cout << "Circle detection program shutdown." << std::endl;
    return 0;
}
