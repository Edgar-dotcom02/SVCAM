#include "tlive_client.h"

namespace tlive {

Client::~Client()
{
	if(is_opened_)
		close();
}

int Client::open(std::string address)
{
	addr_ = address;
	video_addr_ = addr_ + ":5563";
	ctrl_addr_ = addr_ + ":5564";

    video_soc_context_ = zmq_ctx_new();
    video_soc_ = zmq_socket(video_soc_context_, ZMQ_SUB);
    zmq_setsockopt(video_soc_, ZMQ_SUBSCRIBE, "", 0);
	int is_get_only_last_msg = 1;
	zmq_setsockopt(video_soc_, ZMQ_CONFLATE, &is_get_only_last_msg, sizeof(is_get_only_last_msg));
	zmq_connect(video_soc_, video_addr_.c_str());
	
	ctrl_soc_context_ = zmq_ctx_new();
    ctrl_soc_ = zmq_socket(ctrl_soc_context_, ZMQ_REQ);
	//zmq_connect(ctrl_soc, "tcp://localhost:5564");
	zmq_connect(ctrl_soc_, ctrl_addr_.c_str());

	std::string ctrl_req;
	const int ctrl_resp_size = 512;
	char ctrl_resp[ctrl_resp_size];

	//stream.get_video_format_data();
	is_opened_ = 1;
	return 0;
}

int Client::close()
{
	zmq_close(video_soc_);
    zmq_ctx_destroy(video_soc_context_);
    zmq_close(ctrl_soc_);
    zmq_ctx_destroy(ctrl_soc_context_);
	is_opened_ = 0;
	return 0;
}

int Client::video_recv(void *buf, size_t len)
{
	return zmq_recv(video_soc_, (void*)buf, len, ZMQ_NOBLOCK);
}

int Client::ctrl_send(const void *buf, size_t len)
{
	return zmq_send(ctrl_soc_, buf, len, ZMQ_DONTWAIT);
}

int Client::ctrl_recv(void *buf, size_t len)
{
	return zmq_recv(ctrl_soc_, buf, len, 0);
}

};