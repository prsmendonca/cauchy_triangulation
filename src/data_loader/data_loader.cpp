#include <fstream>
#include <sstream>

#include "data_loader.h"

DataLoader::DataLoader(std::string cameras_file_name,
                       std::string measurements_file_name,
                       std::string points_file_name)
    : m_current_point_index(0)
{
    // Open file with cameras.
    std::ifstream cameras_stream(cameras_file_name.c_str(), std::ios::binary);
    if (!cameras_stream)
    {
        std::stringstream error_message;
        error_message << "Failed to open " << cameras_file_name;
        throw std::runtime_error(error_message.str());
    }

    // Read camera data.
    const int NUM_CAM_PARAMETERS = 9 + 3 + 5;
    double data[NUM_CAM_PARAMETERS];
    while (cameras_stream.read(reinterpret_cast<char *>(data), sizeof(double) * NUM_CAM_PARAMETERS))
    {
        gtsam::Rot3 rot(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
        gtsam::Point3 trans;
        trans << data[9], data[10], data[11];
        gtsam::Pose3 pose(rot, trans);
        gtsam::Cal3_S2 intrinsics(data[12], data[13], data[14], data[15], data[16]);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, intrinsics);
        this->m_cameras.push_back(camera);
    }
    cameras_stream.close();

    // Open file with measurements.
    std::ifstream measurements_stream(measurements_file_name.c_str(), std::ios::binary);
    if (!measurements_stream)
    {
        std::stringstream error_message;
        error_message << "Failed to open " << cameras_file_name;
        throw std::runtime_error(error_message.str());
    }

    // Read measurement data.
    int idx_3D;
    while (measurements_stream.read(reinterpret_cast<char *>(&idx_3D), sizeof(idx_3D)))
    {
        int idx_cam;
        measurements_stream.read(reinterpret_cast<char *>(&idx_cam), sizeof(idx_cam));
        // The point idx_3D is seen by camera idx_cam.
        this->m_pt_idx_to_camera_idx[idx_3D].push_back(idx_cam);

        double x, y;
        measurements_stream.read(reinterpret_cast<char *>(&x), sizeof(x));
        measurements_stream.read(reinterpret_cast<char *>(&y), sizeof(y));
        gtsam::Point2 measurement;
        measurement << x, y;
        // The measurement of point idx_3D as seen by camera idx_cam is measurement.
        this->m_pt_idx_to_measurements[idx_3D].push_back(measurement);
    }
    measurements_stream.close();

    if (points_file_name.empty())
    {
        // No 3D points for ground truth.
        return;
    }
    else
    {
        // Open file with 3D points.
        std::ifstream points_stream(points_file_name.c_str(), std::ios::binary);
        if (!points_stream)
        {
            std::stringstream error_message;
            error_message << "No ground truth available." << points_file_name;
            throw std::runtime_error(error_message.str());
        }

        double x, y, z;
        while (points_stream.read(reinterpret_cast<char *>(&x), sizeof(x)))
        {
            points_stream.read(reinterpret_cast<char *>(&y), sizeof(y));
            points_stream.read(reinterpret_cast<char *>(&z), sizeof(z));
            gtsam::Point3 point;
            point << x, y, z;
            this->m_points.push_back(point);
        }
    }
}

bool DataLoader::SetRequestedPointIndex(int requested_3D_point_idx)
{
    if (requested_3D_point_idx < 0 || requested_3D_point_idx >= this->m_pt_idx_to_camera_idx.size())
    {
        return false;
    }
    else
    {
        this->m_current_point_index = requested_3D_point_idx;
        return true;
    }
}

gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>
DataLoader::GetCameras() const
{
    std::vector<int> camera_idxs = this->m_pt_idx_to_camera_idx.at(this->m_current_point_index);
    gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> observing_cameras(camera_idxs.size());
    for (int i = 0; i < camera_idxs.size(); ++i)
    {
        observing_cameras[i] = this->m_cameras[camera_idxs[i]];
    }

    return observing_cameras;
}

gtsam::Point2Vector
DataLoader::GetMeasurements() const
{
    return this->m_pt_idx_to_measurements.at(this->m_current_point_index);
}

std::optional<gtsam::Point3>
DataLoader::GetPoint() const
{
    if (this->m_points.empty())
    {
        return std::nullopt;
    }
    else
    {
        return this->m_points[this->m_current_point_index];
    }
}

int DataLoader::GetNumberOfPoints() const
{
    return this->m_pt_idx_to_camera_idx.size();
}