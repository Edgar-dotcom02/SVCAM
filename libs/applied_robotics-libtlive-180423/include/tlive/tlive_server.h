#ifndef TLIVE_SERVER_H
#define TLIVE_SERVER_H

#include <zmq.h>
#include <tlive/tlive_stream.h>

namespace tlive {

class Server : public Stream
{
	virtual int send_data(void *data, size_t size, void (*free_cb)(void* data, void* hint), void *hint);
	void *video_soc_context;
    void *video_soc;
	void *ctrl_soc_context;
    void *ctrl_soc;
public:
	Server();
	void open();

	int ctrl_send(const void *buf, size_t len);
	int ctrl_recv(void *buf, size_t len);
};


};

#endif