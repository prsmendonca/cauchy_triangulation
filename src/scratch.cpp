#include <fstream>
#include <iostream>
#include <string>
#include <gtsam/geometry/triangulation.h>


int main(int argc, char* argv[]) {
    std::cout << "argc = " << argc << "\n";
    std::cout << "Hello, GTSAM!\n";

    int num_3D_points;
    int num_cameras;
    int idx_3D;
    int idx_cam;
    double x;
    double y;
    std::ifstream point_stream(argv[1], std::ios::in | std::ios::binary);
    std::cout << "2D point coordinates\n";
    while (point_stream.read(reinterpret_cast<char *>(&idx_3D), sizeof(idx_3D)))
    {
        point_stream.read(reinterpret_cast<char *>(&idx_cam), sizeof(idx_cam));
        point_stream.read(reinterpret_cast<char *>(&x), sizeof(x));
        point_stream.read(reinterpret_cast<char *>(&y), sizeof(y));

        std::cout << "pt: " << idx_3D << "; cam: " << idx_cam << "; (x, y): (" << x << ", " << y << ")\n";
    }
    point_stream.close();

    const int CAM_PARAMETERS = 12;
    double cam[CAM_PARAMETERS];
    std::ifstream camera_stream(argv[2], std::ios::in | std::ios::binary);
    std::cout << "Cameras \n";
    while (camera_stream.read(reinterpret_cast<char *>(cam), sizeof(double) * CAM_PARAMETERS))
    {
        std::cout << "cam:";
        for (int i = 0; i < 12; ++i)
        {
            std::cout << " " << cam[i];
        }
        std::cout << "\n";
    }
    camera_stream.close();

    return EXIT_SUCCESS;
}