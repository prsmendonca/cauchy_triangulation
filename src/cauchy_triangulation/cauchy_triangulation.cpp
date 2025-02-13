template <class CALIBRATION>
gtsam::Point3 triangulate_cauchy(
    const gtsam::CameraSet<gtsam::PinholeCamera<CALIBRATION>> &cameras,
    const typename gtsam::PinholeCamera<CALIBRATION>::MeasurementVector &measurements,
    bool optimize,
    const gtsam::SharedNoiseModel &model)
{
    size_t m = measurements.size();
    assert(m == cameras.size());

    // Get the variance of the noise.
    double sigma = model ? model->sigmas().mean() : 1e-4;
    double variance = sigma * sigma;
    double precision = 1 / variance;
    gtsam::Matrix33 Id = gtsam::Matrix::Identity(3, 3);

    // Construct poses from cameras, and remove effect of intrinsics.
    gtsam::Point2Vector calibrated_measurements;
    calibrated_measurements.reserve(cameras.size());
    std::vector<gtsam::Pose3> poses;
    poses.reserve(cameras.size());
    for (int i = 0; i < m; ++i)
    {
        poses.push_back(cameras[i].pose());
        calibrated_measurements.push_back(cameras[i].calibration().calibrate(measurements[i]));
    }

    // Construct the system matrices:
    gtsam::Matrix33 M = gtsam::Matrix::Zero(3, 3);
    gtsam::Matrix31 Mc = gtsam::Matrix::Zero(3, 1);

    for (size_t i = 0; i < m; ++i)
    {
        // Create homogeneous parameter of Cauchy noise model,
        // Direct
        // \tilde S = [S + m@m.T m]
        //            [m.T       1]
        // Inverse
        // \tilde S^{-1} = [S^{-1}              -S^{-1}@m]
        //                  -m.T@S^{-1}  1 + m.T@S^{-1}@m]
        auto measurement = calibrated_measurements[i];
        auto neg_normalized_measurement = -measurement * precision;
        double norm_squared_measurement = 0;

        gtsam::Matrix33 hS = gtsam::Matrix::Zero(3, 3);
        hS.block<2, 2>(0, 0).diagonal() << variance, variance;
        hS.block<2, 2>(0, 0) += measurement * measurement.transpose();
        hS.block<2, 1>(0, 2) = measurement;
        hS.block<1, 2>(2, 0) = measurement.transpose();
        hS(2, 2) = 1;

        gtsam::Matrix33 ihS = gtsam::Matrix::Zero(3, 3);
        ihS.block<2, 2>(0, 0).diagonal() << precision, precision;
        ihS.block<2, 1>(0, 2) = neg_normalized_measurement;
        ihS.block<1, 2>(2, 0) = neg_normalized_measurement.transpose();
        ihS(2, 2) = 1 - neg_normalized_measurement.transpose() * measurement;

        // Create the matrix Mi and the vector Mici.
        const gtsam::Matrix33 AiT = poses[i].rotation().matrix();
        const gtsam::Matrix33 &Ai = AiT.transpose();
        const gtsam::Matrix33 invA_S_invAT = AiT * hS * Ai;
        const gtsam::Matrix33 AT_invS_A = AiT * ihS * Ai;
        const gtsam::Matrix33 Mi = invA_S_invAT.trace() * AT_invS_A - Id;

        M += Mi;
        Mc += (Mi * poses[i].translation().matrix());
    }

    return M.llt().solve(Mc);
}