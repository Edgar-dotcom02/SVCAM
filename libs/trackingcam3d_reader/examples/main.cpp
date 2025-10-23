#include <trackingcam3d_reader/TrackingCam3d_Reader.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>

#define MAX_NO_FRAMES   10000
#define READ_DEL        40 // in ms

// Global stop flag
std::atomic<bool> stop{false};

// Signal handler for Ctrl+C
void signalHandler(int signum) 
{
    stop.store(true, std::memory_order_relaxed);
    std::cout << "\nReceived exit signal, stopping program...\n";
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);

    std::vector<std::string> stream_requests = {"left=10000"}; // default request

    if (argc >= 2) {
        stream_requests.clear(); // reset the default stream

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i] + std::string("=10000");
            stream_requests.emplace_back(arg);
        }
    }

    TrackingCamReader cam_reader(stream_requests);
    if (!cam_reader.connect())
    {
    	std::cerr << "Failed to connect camera client";
    	return 1;
	}
    // MyFpsMeter fps("Camera FPS", 0.1);

    while (!stop.load(std::memory_order_relaxed)) 
    {
        auto frames = cam_reader.getLatestFrames();

        static int no_frame_count = 0;
        if (frames.empty())
        {
            ++no_frame_count;
            if (no_frame_count > MAX_NO_FRAMES) {
                std::cerr << "No frames received for a long time. Exiting.\n";
                stop.store(true, std::memory_order_relaxed);
                continue;
            }
        }
        else 
        {
            no_frame_count = 0;
            // fps.update();
            // fps.print_fps(false);

            for (const auto& frame : frames)
            {
                cam_reader.showImage(frame.first, frame.second);
            }
            if (cv::waitKey(1) == 27) {
                stop.store(true, std::memory_order_relaxed);
                continue;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(READ_DEL));
    }

    cv::destroyAllWindows();

    cam_reader.disconnect();
    std::cout << "Stopping TrackingCamReader...\n";
    return 0;
}
