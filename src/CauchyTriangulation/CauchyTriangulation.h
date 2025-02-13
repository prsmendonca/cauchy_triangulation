#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/linear/NoiseModel.h>


GTSAM_EXPORT gtsam::Point3 triangulateCauchy(
    const gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> &poses,
    const gtsam::Point2Vector &calibratedMeasurements,
    const gtsam::SharedNoiseModel &measurementNoise);