Клиентская библиотека, позволяющая получить изображения с телекамеры и их использовать для дальнейших отработок или для визуализации. 
Для установки, выполняйте следующие комманды:
mkdir -p build && cd build
cmake ..
make
sudo make install

После установки, в любую программу можно использовать данную библиотеку используя заголовки:
#include <trackingcam3d_reader/TrackingCam3d_Reader.h>

При добавлении заголовочного файла становится доступен объект TrackingCamReader, который отвечает за подключение камеры и получение кадров. Его необходимо инициализировать следующим образом:

TrackingCamReader(const std::vector<std::string>& stream_requests);

stream_requests - {"изображение1=[кол-во кадров1]", "изображение2=[кол-во кадров2]", ...}, т.е. список типов изображений с количеством кадров для каждого изображения (например, {"left=1000", "depth=1000"}). 
Список поддерживаемых изображений:
left, left_rect, left_gray_rect, right, right_rect, right_gray_rect, depth, disparity

Основные методы класса TrackingCamReader:
bool connect() - возвращает 1 если подключение успешно прошлло.

std::map<std::string, cv::Mat> getLatestFrames() - Извлекает самые последние полученные кадры для всех типов запрошенных изображений. Этот вызов не является блокирующим. Он возвращает std::map<std::string, cv::Mat>, где:  (std::string) - имя потока (указанное в списке stream_names при инициализации), а значение (cv::Mat) - фактическое изображение/кадр данных из потока. Если новых данных нет, std::map будет пустым.


void TrackingCamReader::showImage(const std::string& img_type, const cv::Mat& image) - позволяет визуализации изображения image типом img_type.
 
void disconnect() - вызывается при выходе или завершении программы. 

При компиляции c g++ добавьте параметь -ltrackingcam3d_reader чтобы подключить библиотеку. Например g++ -ltrackingcam3d_reader ...

Если используйте cmake, то в CMakeLists.txt своего проекта можно подключить библиотеку:
find_package(trackingcam3d_reader REQUIRED)
target_link_libraries(<project_name> trackingcam3d_reader)