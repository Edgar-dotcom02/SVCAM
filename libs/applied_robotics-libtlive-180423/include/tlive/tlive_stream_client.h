#ifndef TLIVE_STREAM_CLIENT_H
#define TLIVE_STREAM_CLIENT_H

//#include <zmq.h>
#include <opencv2/opencv.hpp>
#include <tbb/tbb.h>

namespace tlive {

class StreamClient
{
	//Replace it with your network routines ////////////////////////////////////////////////////////////
	virtual int video_recv(void *buf, size_t len) {return 0;}
	////////////////////////////////////////////////////////////////////////////////////////////////////

	struct SupportedImage
	{
		//Likely some additional description could appear here
		int nothing;
	};

	std::map<std::string, SupportedImage> supported_images_;
	CvMat* decode_image(char* buf, std::string encoding, int w, int h, int max_size);
	static const int buf_size_ = 512 + 640*480*3*5;
	char* buf_;
	std::vector<CvMat*> release_list_;
	std::map<std::string, CvMat*> images_;

protected:
	std::string addr_;
	std::string video_addr_;
	std::string ctrl_addr_;
	int is_opened_;

public:
	StreamClient(): is_opened_(0) {buf_ = new char[buf_size_];};
	~StreamClient();
	void support(const char* name);
	void support(const char** list, int n);

	std::map<std::string, CvMat*> receive();
	std::string get_addr();
};

};


#endif