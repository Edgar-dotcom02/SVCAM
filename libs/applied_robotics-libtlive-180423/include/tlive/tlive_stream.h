#ifndef TLIVE_STREAM
#define TLIVE_STREAM

#include <opencv/cv.h>
#include <tbb/tbb.h>
//#include <my3dproc/MyPerformanceMeter.h>

namespace tlive {

class StreamJob
{
public:
	CvMat* img;
	std::string img_name;
	std::vector<uchar> vec; //for cv encoders
	std::string encoding;
	char* pt;
	bool is_allocated;
	int size;
	int width;
	int height;
	StreamJob() : img(NULL), is_allocated(false){};
	~StreamJob(){if(is_allocated) delete[] pt;};
	StreamJob(CvMat* _img, std::string _img_name, std::string _encoding): img_name(_img_name), encoding(_encoding), img(_img), is_allocated(false) {};
	void init(CvMat* _img, std::string _img_name, std::string _encoding) {img_name = _img_name; encoding = _encoding; img = _img; is_allocated = false;};
	int encode();
	void operator()(){encode();};
};

class CvMatCloneJob
{
	CvMat* tmp;
public:
	CvMat* src;
	CvMat* dst;
	CvMatCloneJob();
	CvMatCloneJob(CvMat* _src, CvMat* _dst): src(_src), dst(_dst){};
	void operator()() {tmp = cvCloneMat(src); dst = tmp;};
};

class CopyJob
{
public:
	void* src;
	void* dst;
	int size;
	CopyJob();
	CopyJob(void* _src, void* _dst, int _size) : src(_src), dst(_dst), size(_size) {};
	void operator()() {
		cv::Mat s(1, size, CV_8UC1, src); 
		cv::Mat d(1, size, CV_8UC1, dst);
		s.copyTo(d);}
};

class Stream
{
	bool is_task;
	tbb::task* root_task;
	unsigned long long int timestamp_ms_;
	unsigned long long int settings_timestamp_ms_;

	//Replace it with your network routines ////////////////////////////////////////////////////////////
	virtual int send_data(void *data, size_t size, void (*free_cb)(void* data, void* hint), void *hint) {return 0;};
	////////////////////////////////////////////////////////////////////////////////////////////////////

public:
	//MyPerformanceMeter perf_;
	int last_size;
	Stream(): is_task(false), timestamp_ms_(0), /*perf_(MyPerformanceMeter("Stream", false, false)),*/ last_size(0){};
	~Stream(){};
	std::vector<StreamJob> stream_jobs;
	void add_job(CvMat* _img, std::string _img_name, std::string _encoding);
	void stream();
	void stream_async();
	void join();
	void set_timestamp_ms(unsigned long long int timestamp_ms);
	void set_settings_timestamp_ms(unsigned long long int timestamp_ms);
};

class StreamTask : public tbb::task 
{
public:
	Stream* stream;
	tbb::task* execute() {stream->stream(); return NULL;}
};

};

#endif