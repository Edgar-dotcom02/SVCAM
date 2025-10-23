#include <trackingcam3d_reader/TrackingCam3d_Reader.h>

TrackingCamReader::TrackingCamReader(const std::vector<std::string>& stream_requests)
    : cam_client(), server_addr("tcp://localhost"), stream_requests_(stream_requests),
     is_opened(0)
{
    // Getting the stream names from the stream requests
    for (const auto& stream : stream_requests_)
    {
    	size_t eq = stream.find('=');
        stream_names.emplace_back(eq != std::string::npos ? stream.substr(0, eq) : stream);
    }
    
    // Ensure all requested streams are supported
    for (const auto& name : stream_names) 
    {
        cam_client.support(name.c_str());
    }
    
    // Open camera connection
    cam_client.open(server_addr);
    is_opened = 1;
}

TrackingCamReader::~TrackingCamReader() 
{
    disconnect();
    std::cout << "Camera stream Stopped." << std::endl;
}

bool TrackingCamReader::connect()
{
    // Request streams
    char reply_buf[2000];
    for (const auto& req : stream_requests_) 
    {
        int len = req.size() + 1; // +1 for \0
        int sent = cam_client.ctrl_send(req.c_str(), len);
        if (sent >= 0) {
            int rcv_len = cam_client.ctrl_recv(reply_buf, sizeof(reply_buf));
            if (rcv_len > 0) {
                std::cout << "Stream request '" << req << "' reply: " << reply_buf << std::endl;
            } else {
                std::cerr << "WARNING: No reply for stream request: " << req << std::endl;
                cam_client.close();
                return false;
            }
        } 
        else 
        {
            std::cerr << "WARNING: Failed to send stream request: " << req << " for camera." << std::endl;
            cam_client.close();
            return false;
        }
    }
    
    std::cout << "Camera stream Connected." << std::endl;
    return true;
}

void TrackingCamReader::disconnect() 
{
    if (is_opened) {
        cam_client.ctrl_send("nowant=1", 9);
        cam_client.close();
    }
    is_opened = 0;
}

std::map<std::string, cv::Mat> TrackingCamReader::getLatestFrames() 
{
    std::map<std::string, cv::Mat> output;

    auto images = cam_client.receive(); // map<string, CvMat*>
    for (auto &pair : images) {
        cv::Mat img = cv::cvarrToMat(pair.second, true); // true = copy data
        output[pair.first] = img;
    }
    
    return output;
}

void TrackingCamReader::showImage(const std::string& img_type, const cv::Mat& image)
{
    cv::Mat img;

    if ((image.type() == CV_8UC1) || (image.type() == CV_8UC3))
        img = image.clone();
    else if (image.type() == CV_16UC1)
        img = myGetRgbDepthMap(image.clone(), 100, 1000, 2000, 10000);
    else if (image.type() == CV_16SC1)
        img = myGetDispMap(image.clone());
    else if (image.type() == CV_8UC2)
        cv::cvtColor(image.clone(), img, cv::COLOR_YUV2BGR_YUYV);

    cv::resize(img, img, cv::Size(320, 240));
    cv::imshow(img_type, img);
}