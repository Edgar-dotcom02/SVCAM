#ifndef TLIVE_CLIENT_H
#define TLIVE_CLIENT_H

#include <zmq.h>
#include <tlive/tlive_stream_client.h>

namespace tlive {

class Client : public StreamClient
{
	void *video_soc_context_;
	void *video_soc_;
	void *ctrl_soc_context_;
	void *ctrl_soc_;
	virtual int video_recv(void *buf, size_t len);
public:
	Client(){};
	~Client();
	int open(std::string address = "tcp://localhost");
	int close();
	
	int ctrl_send(const void *buf, size_t len);
	int ctrl_recv(void *buf, size_t len);
};

};

#endif