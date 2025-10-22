#include "dxlSlave.h"

DxlSlave::DxlSlave()
    : host_("127.0.0.1"), port_(17832)
{}

DxlSlave::~DxlSlave()
{
    disconnect();
    std::cout << "DxlSlave Disconnected." << std::endl;
}

bool DxlSlave::connect()
{
    FILE* pipe = popen("systemctl status dxlSlave.service", "r");
    if (!pipe) {
        std::cerr << "Error running systemctl status" << std::endl;
        return false;
    }

    std::string output;
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    int status = pclose(pipe);

    // Parse output for Active: active (running)
    bool is_running = output.find("Active: active (running)") != std::string::npos;
    if (!is_running) {
        std::cerr << "ERROR: dxlSlave.service is not running." << std::endl;
        return false;
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Ошибка создания сокета" << std::endl;
        return false;
    }
    // std::cout<<"sockfd created";

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(17832);
    if (inet_pton(AF_INET, host_.c_str(), &dest_addr.sin_addr) <= 0) {
        disconnect();
        std::cerr << "Ошибка в адресе" << std::endl;
        return false;
    }

    std::cout << "DxlSlave Connected." << std::endl;
    return true;
}

bool DxlSlave::callMethod(const std::int32_t& reg, const std::int32_t& val)
{
    std::string message = "change|" +  std::to_string(reg) + "|" +  std::to_string(val);
    size_t sent_bytes = sendto(sockfd, message.c_str(), strlen(message.c_str()), 0,
                                (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (sent_bytes < 0) {
        std::cerr << "Ошибка отправки" << std::endl;
        disconnect();
        return false;
    }

    return true;
}

void DxlSlave::disconnect()
{
    if (sockfd >= 0) 
    {
        close(sockfd);
        sockfd = -1;
    }
}
