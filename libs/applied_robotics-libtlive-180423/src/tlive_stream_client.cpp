#include "tlive_stream_client.h"
#include <lz4.h>
#include <lz4hc.h>
#include <zstd.h>
#include <tconfig/tconfig.h>

namespace tlive {

using namespace tconfig;

StreamClient::~StreamClient()
{
}

class DecodeJob
{
	int width;
	int height;
	int size;
	std::string encoding;
	char* buf;
public:
	CvMat* dst;
	bool is_allocated;
	std::string img_name;
	DecodeJob(){};
	DecodeJob(std::string _img_name, char* _buf, std::string _encoding, int _width, int _height, int _size): img_name(_img_name), buf(_buf), encoding(_encoding), width(_width), height(_height), size(_size), is_allocated(false), dst(NULL){};
	CvMat* decode();
	void operator()(){decode();};
};

CvMat* DecodeJob::decode()
{
	dst = NULL;
	int w = width;
	int h = height;
	if(encoding == "gray8")
	{
		dst = cvCreateMatHeader(h, w, CV_8UC1);
		dst->data.ptr = (uchar*)buf;
	}
	else if(encoding == "gray16")
	{
		dst = cvCreateMatHeader(h, w, CV_16UC1);
		dst->data.ptr = (uchar*)buf;
	}
	else if(encoding == "gray16s")
	{
		dst = cvCreateMatHeader(h, w, CV_16SC1);
		dst->data.ptr = (uchar*)buf;
	}
	else if(encoding == "yuv422")
	{
		dst = cvCreateMatHeader(h, w, CV_8UC2);
		dst->data.ptr = (uchar*)buf;
	}
	else if(encoding == "jpg" || encoding == "jp2" || encoding == "png" || encoding == "bmp")
	{
		CvMat* s = cvCreateMatHeader(1, size, CV_8UC1);
		s->data.ptr = (uchar*)buf;
		dst = cvDecodeImageM(s, 1);//CV_LOAD_IMAGE_ANYDEPTH);
		is_allocated = true; // mark that we allocated new memory instead of setting of pointer
	}
	else if(encoding == "gray8_lz4" || encoding == "gray8_zstd")
	{
		dst = cvCreateMat(h, w, CV_8UC1);
		is_allocated = true;
		
		if(encoding == "gray8_lz4")
			LZ4_decompress_safe(buf, (char*)dst->data.ptr, size, h * w * 1);
		else
			ZSTD_decompress((char*)dst->data.ptr, h * w * 1, buf, size);
	}
	else if(encoding == "gray16_lz4" || encoding == "gray16_zstd" || encoding == "gray16s_lz4" || encoding == "gray16s_zstd")
	{
		bool is_signed = encoding == "gray16s_lz4" || encoding == "gray16s_zstd";
		dst = cvCreateMat(h, w, is_signed? CV_16SC1: CV_16UC1);
		is_allocated = true;
		if(encoding == "gray16_lz4" || encoding == "gray16s_lz4")
			LZ4_decompress_safe(buf, (char*)dst->data.ptr, size, h * w * 2);
		else
			ZSTD_decompress((char*)dst->data.ptr, h * w * 2, buf, size);
	}
	else if(encoding == "yuv422_lz4" || encoding == "yuv422_zstd")
	{
		dst = cvCreateMat(h, w, CV_8UC2);
		cv::Mat dst_1ch(h, w*2, CV_8UC1, dst->data.ptr);
		is_allocated = true;
		CvMat* tr = cvCreateMat(w*2, h, CV_8UC1);
		if(encoding == "yuv422_lz4")
			LZ4_decompress_safe(buf, (char*)tr->data.ptr, size, h * w * 2);
		else
			ZSTD_decompress((char*)tr->data.ptr, h * w * 2, buf, size);
		cv::transpose(cv::cvarrToMat(tr), dst_1ch);
		cvReleaseMat(&tr);
	}

	return dst;
}

template <typename T> struct TbbStlContainerInvoker {
  void operator()(T& it) const {it();}
};

std::map<std::string, CvMat*> StreamClient::receive()
{
	for(std::vector<CvMat*>::iterator it = release_list_.begin(); it != release_list_.end(); it++)
		cvReleaseMat(&(*it));
	images_.clear();

	int size = video_recv((void*)buf_, buf_size_);
	if(size <= 0)
		return images_;
		
	std::map<std::string, Param> attrs;
	parse_params(attrs, buf_);
	
	std::vector<DecodeJob> decode_jobs;
	for(std::map<std::string, SupportedImage>::iterator it = supported_images_.begin(); it != supported_images_.end(); it++)
	{
		std::string name = it->first;
		Param param_addr, param_encoding, param_width, param_height, param_size;
		if(!get_param(attrs, name, param_addr)						||
		   !get_param(attrs, name + "_encoding", param_encoding)	||
		   !get_param(attrs, name + "_width", param_width)			||
		   !get_param(attrs, name + "_height", param_height))
		{
			continue;
		}
		//if(param_encoding.str() == "lz4" || param_encoding == "

		decode_jobs.push_back(DecodeJob(name, buf_ + param_addr.i(), param_encoding.str(), param_width.i(), param_height.i(), buf_size_ - param_addr.i()));
	}

	tbb::parallel_for_each(decode_jobs.begin(), decode_jobs.end(), TbbStlContainerInvoker<DecodeJob>());

	for(std::vector<DecodeJob>::iterator it = decode_jobs.begin(); it != decode_jobs.end(); it++)
	{
		if(it->dst == NULL)
			continue;
		images_[it->img_name] = it->dst;
		if(it->is_allocated)
			release_list_.push_back(it->dst);
	}

	return images_;
}

void StreamClient::support(const char* name)
{
	supported_images_[name];
}

void StreamClient::support(const char** list, int n)
{
	for(int i = 0; i < n; i++)
		support(list[i]);
}

std::string StreamClient::get_addr() {return addr_;}

};