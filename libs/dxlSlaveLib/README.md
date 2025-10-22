Библиотека для обмена данными между SVCAM и внешними контроллерами по DXL. Библиотека написана в языке С++. 
Для установки, выполняйте следующие комманды:
cd dxlSlaveLib/
mkdir -p build && cd build
cmake ..
make
sudo make install

Для подключения библиотеки в свою программу используется заголовок: #include “dxlSlave.h”
Для работы с библиотекой, необходимо создать объект DxlSlave (например DxlSlave slave), который необходимо инициализировать следующим образом:
bool DxlSlave::connect() - возвращает 1 если подключение к сервису dxlSlave.service прошлло успешно. Например, slave.connect();
	
Запись значений в любой регистр (с адреса от 0 до 31) делается через метод:
bool DxlSlave::callMethod(const std::int32_t& reg, const std::int32_t& val)
где reg - номер регистра, куда хотим записать значение val. Например, slave.callMethod(24, 0);
	
При выходе или завершении программы нужно вызывать метод:
bool DxlSlave::disconnect() 
Например, slave.disconnect();

При компиляции через g++ добавьте параметь -ldxlSlave чтобы подключить библиотеку. Например: g++ -ldxlSlave ...
Если используйте cmake, то в CMakeLists.txt своего проекта можно подключить библиотеку следующим образом:
find_package(DxlSlave REQUIRED)
include_directories(${dxlSlave_INCLUDE_DIRS})
target_link_libraries(<имя_проекта> DxlSlave::dxlSlave)