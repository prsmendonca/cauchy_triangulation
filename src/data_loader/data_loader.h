#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <optional>
#include <string>
#include <vector>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/linear/NoiseModel.h>

class DataLoader
{
public:
    DataLoader(std::string cameras_file_name,
               std::string measurements_file_name,
               std::string point_file_name = "");
    bool SetRequestedPointIndex(int requested_3D_point_idx);
    gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> GetCameras() const;
    gtsam::Point2Vector GetMeasurements() const;
    std::optional<gtsam::Point3> GetPoint() const;
    int GetNumberOfPoints() const;

protected:
    int m_current_point_index;

    gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> m_cameras;
    gtsam::Point3Vector m_points;
    std::unordered_map<int, std::vector<int>> m_pt_idx_to_camera_idx;
    std::unordered_map<int, gtsam::Point2Vector> m_pt_idx_to_measurements;
};

#endif // DATA_LOADER_H