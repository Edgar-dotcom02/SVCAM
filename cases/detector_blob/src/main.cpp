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
	
    TrackingCamReader cam_reader(stream_requests);
 	if (!cam_reader.connect()) 
	{
 		slave.disconnect();
 		std::cerr  << "Failed to connect camera client" << std::endl;
 		return 1;
 	}
 	
	cv::Mat img;
    int no_frame_count = 0;
    int status_code = 1;  // Ready or working

    std::cout << "Starting blob detection." << std::endl;
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
                status_code = 3;  // Error
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

        if (frames.find("left") != frames.end() && !frames["left"].empty()) 
        {
            img = frames["left"].clone();
        }
        else
        {
            slave.callMethod(DIST_ADDR, dist_mm);
            // std::cout << "WARNING: Invalid frame. Skipping" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms / 2));
            continue;
        }

        cv::Mat frame;
        cv::cvtColor(img, frame, cv::COLOR_YUV2BGR_YUYV);

        cv::resize(frame, frame, cv::Size(640, 480));
        float x_size = (float)frame.cols, y_size = (float)frame.rows;
        // float x_offset = x_size / 2.0, y_size = 2.0;

        // Processing image and detect red circle
        cv::Mat blur, hsv, mask;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0);
        cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::sort(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) > cv::contourArea(b);
            });

        // If the object is found
        if (!contours.empty()) 
        {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[0], center, radius);

            // Clip position
            center.x = std::max(0.0f, std::min(center.x, x_size));
            center.y = std::max(0.0f, std::min(center.y, y_size));

            // Estimate distance from area
            double area = CV_PI * radius * radius;
            double k;
            if (area > 16500) k = 165000;
            else if (area > 6500) k = 42462;
            else if (area > 4900) k = 24450;
            else if (area > 3100) k = 12462;
            else k = 5800;

            double dist = area / k; // meters
            dist_mm = (int)(dist * 1000);
            std::cout << "Distance: " << dist << " m" << std::endl;

            // Convert to 3D coordinates
            double point_z = dist;
            double point_x = (center.x - m_cx) * point_z * inv_fx;
            double point_y = (center.y - m_cy) * point_z * inv_fy;

            // std::cout << "Ball position: (" << point_x << ", " << point_y << ", " << point_z << ")\n";

            if (display_flag)
            {
                // Draw detection
                cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                            "Dist: " + std::to_string(dist).substr(0, 4) + "m",
                            cv::Point(center.x + 10, center.y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(255, 255, 255), 2);
            }
        }

        slave.callMethod(DIST_ADDR, dist_mm);

        if (display_flag)
        {
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(320, 240));
            cv::imshow("Blob Detector", resized);
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
    std::cout << "Blob detection program shutdown." << std::endl;
    return 0;
}