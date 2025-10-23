#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: ./read_cloud <cloud_bin_file>\n";
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        std::cerr << "Failed to open " << filename << "\n";
        return 1;
    }

    // Read all bytes
    in.seekg(0, std::ios::end);
    size_t byte_size = in.tellg();
    in.seekg(0, std::ios::beg);

    if (byte_size % 12 != 0) {
        std::cerr << "Invalid file size: " << byte_size << " bytes (not multiple of 12)\n";
        return 1;
    }

    size_t num_points = byte_size / 12;
    std::vector<float> cloud_data(num_points * 3);
    in.read(reinterpret_cast<char*>(cloud_data.data()), byte_size);
    in.close();

    std::cout << "Read " << num_points << " points from " << filename << "\n";
    for (size_t i = 0; i < std::min<size_t>(10, num_points); ++i) {  // Print first 10
        float x = cloud_data[i * 3];
        float y = cloud_data[i * 3 + 1];
        float z = cloud_data[i * 3 + 2];
        std::cout << std::fixed << std::setprecision(3)
                  << "Point " << i << ": X=" << x << " Y=" << y << " Z=" << z << " m\n";
    }
    if (num_points > 10) std::cout << "... (truncated)\n";

    return 0;
}