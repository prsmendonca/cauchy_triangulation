#pragma once

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/linear/NoiseModel.h>

template <class CALIBRATION>
GTSAM_EXPORT gtsam::Point3 triangulate_cauchy(
    const gtsam::CameraSet<gtsam::PinholeCamera<CALIBRATION>> &poses,
    const typename gtsam::PinholeCamera<CALIBRATION>::MeasurementVector &measurements,
    bool optimize = false,
    const gtsam::SharedNoiseModel &model = nullptr);

#include "cauchy_triangulation.cpp"