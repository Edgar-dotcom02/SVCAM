#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <trackingcam3d_reader/TrackingCam3d_Reader.h>
#include "dxlSlave.h"

#define STATUS_ADDR     24
#define FACES_NUM_ADDR  25
#define FACE_DIST_ADDR  26
#define FACE_X_ADDR     28
#define FACE_Y_ADDR     28

// Struct to hold face detection face_datas
struct FaceData {
    uint8_t num_faces;
    float closest_dist;
    int16_t x;
    int16_t y;
};

// Global stop flag
std::atomic<bool> stop{false};

// Signal handler for Ctrl+C
void signalHandler(int signum) 
{
    stop.store(true, std::memory_order_relaxed);
    std::cout << std::endl << "SHUTDOWN: Received exit signal. Stopping program..." << std::endl;
}

void setFaceData(DxlSlave& dxl_slave, FaceData& data);

int main(int argc, char** argv) 
{
    std::cout << std::fixed << std::setprecision(2);

    signal(SIGINT, signalHandler);

    DxlSlave slave;
	if (!slave.connect()) {
		std::cerr << "DxlSlave connection error" << std::endl;
		return 1;
	}

    // Load config file
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

    // Camera and face recognition algorithm parameters
    cv::Mat K = (cv::Mat_<double>(3,3) << 
        config["camera"]["intrinsics"]["fx"].as<double>(), 0, config["camera"]["intrinsics"]["cx"].as<double>(),
        0, config["camera"]["intrinsics"]["fy"].as<double>(), config["camera"]["intrinsics"]["cy"].as<double>(),
        0, 0, 1);
    std::string haar_cascade = config_base_path + config["camera"]["haar_cascade"].as<std::string>();
    std::vector<std::string> stream_requests = config["camera"]["stream_requests"].as<std::vector<std::string>>();
    
    bool display_flag = config["display"].as<bool>();
    int max_no_frames = config["max_no_frames"].as<int>();
    int read_delay_ms = config["read_delay_ms"].as<int>();

    // Camera intrinsics
    double inv_fx = 1.0 / K.at<double>(0,0);
    double inv_fy = 1.0 / K.at<double>(1,1);
    double m_cx = K.at<double>(0,2);
    double m_cy = K.at<double>(1,2);

    // Loading haar cascade
    cv::CascadeClassifier face_cascade;
    if (!face_cascade.load(haar_cascade)) 
    {
        slave.disconnect();
        throw std::runtime_error("ERROR: Failed to load face cascade: " + haar_cascade);
        return 1;
    }

    // Initializing cam_reader object
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

    std::cout << "Starting Face detection." << std::endl;
    slave.callMethod(STATUS_ADDR, status_code); 
    FaceData face_data = {0, 0.0f, 0, 0};

    while (!stop.load(std::memory_order_relaxed)) 
    {
        face_data = {0, 0.0f, 0, 0};

        auto frames = cam_reader.getLatestFrames();
		if (frames.empty()) 
		{
			++no_frame_count;
			if (no_frame_count > max_no_frames) 
			{
                status_code = 3; // Error
				slave.callMethod(STATUS_ADDR, status_code); 
                setFaceData(slave, face_data);
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

        cv::Mat rgb;
        cv::cvtColor(last_left, rgb, cv::COLOR_YUV2BGR_YUYV);

        // Convert to grayscale for face detection
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, 1.3, 5);

        if (!faces.empty()) 
        {
            face_data.num_faces = static_cast<uint8_t>(std::min(faces.size(), size_t(255)));
            double min_dist_z = std::numeric_limits<double>::max();
            cv::Rect closest_face;

            for (const auto& face : faces) 
            {
                cv::rectangle(rgb, face, cv::Scalar(255, 0, 0), 2);

                cv::Rect inner(
                    face.x + 30,
                    face.y + 30,
                    face.width - 60,
                    face.height - 60
                );
                inner &= cv::Rect(0, 0, last_depth.cols, last_depth.rows);

                if (inner.width <= 0 || inner.height <= 0) {
                    continue;
                }

                cv::Mat roi_depth = last_depth(inner);
                double sum = 0;
                int count = 0;
                for (int i = 0; i < roi_depth.rows; i++) {
                    for (int j = 0; j < roi_depth.cols; j++) {
                        uint16_t d = roi_depth.at<uint16_t>(i, j);
                        if (d > 0) {
                            sum += d;
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    double point_z = (sum / count) * 0.001;
                    if (point_z < min_dist_z) {
                        min_dist_z = point_z;
                        
                        // Recalculate full distance
                        double point_x = ((face.x + face.width / 2) - m_cx) * min_dist_z * inv_fx;
                        double point_y = ((face.y + face.height / 2) - m_cy) * min_dist_z * inv_fy;
                        
                        face_data.closest_dist = static_cast<float>(
                            std::sqrt(point_x * point_x + point_y * point_y + min_dist_z * min_dist_z));
                        face_data.x = static_cast<int16_t>(point_x * 1000.0);
                        face_data.y = static_cast<int16_t>(point_y * 1000.0);
                        closest_face = face;
                    }
                }
            }

            if (face_data.closest_dist > 0) 
            {
                std::cout << "Distance to face: " << face_data.closest_dist << " m" << std::endl;
                if (display_flag) 
                {
                    cv::putText(rgb, "Distance: " + std::to_string(face_data.closest_dist) + "m",
                                cv::Point(closest_face.x, closest_face.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                }
            }
            else
            {
                std::cout << "Face detected, but no valid distance computed." << std::endl;
            }
        }

        setFaceData(slave, face_data);

        if (display_flag)
        {
            cv::Mat resized;
            cv::resize(rgb, resized, cv::Size(320, 240));
            cv::imshow("Face detection", resized);
            if (cv::waitKey(1) == 27) 
            {
                stop.store(true, std::memory_order_relaxed);
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(read_delay_ms));
    }

    setFaceData(slave, face_data);
    if (status_code != 3) status_code = 2; // shutdown without error
	slave.callMethod(STATUS_ADDR, status_code); 

    if (display_flag) cv::destroyAllWindows();
    cam_reader.disconnect();
    slave.disconnect();
    std::cout << "Face detection program shutdown." << std::endl;
    return 0;
}

void setFaceData(DxlSlave& dxl_slave, FaceData& data)
{
    int n_faces = static_cast<int>(data.num_faces);
    int face_dist = static_cast<int>(round(data.num_faces * 1000.0));
    int face_x = static_cast<int>(data.x);
    int face_y = static_cast<int>(data.y);

    dxl_slave.callMethod(FACES_NUM_ADDR, n_faces);
    dxl_slave.callMethod(FACE_DIST_ADDR, face_dist);
    dxl_slave.callMethod(FACE_X_ADDR, face_x);
    dxl_slave.callMethod(FACE_Y_ADDR, face_y);
}