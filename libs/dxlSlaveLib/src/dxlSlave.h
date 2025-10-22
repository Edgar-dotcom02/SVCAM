#include <iostream>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class DxlSlave {
    public:
        DxlSlave();
        ~DxlSlave();

        bool connect();
        bool callMethod(const std::int32_t& reg, const std::int32_t& val);
        void disconnect();

    private:
        int sockfd;
        std::string host_;
        int port_;
        struct sockaddr_in dest_addr;
};
