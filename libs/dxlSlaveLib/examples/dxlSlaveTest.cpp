#include "dxlSlave.h"
#include <chrono> 
#include <thread> 

#define DELAY    500 // ms
#define REG_NUM  24

int main()
{
    DxlSlave slave;
    int data = 0;

    if (slave.connect())
    {
        slave.callMethod(REG_NUM, data);
        for (size_t i = 0; i < 20; ++i) 
        {
            data += 10;
            slave.callMethod(REG_NUM, data);
            std::cout << "Register " << REG_NUM << ": " << data << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(DELAY));
        }
    }
    else
    {
        std::cerr << "Connection error." << std::endl;
        return 1;
    }

    slave.callMethod(REG_NUM, 0);
    slave.disconnect();
    return 0;
}