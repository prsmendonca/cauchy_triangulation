#include <chrono>
#include <iostream>
#include <random>
#include <optional>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/triangulation.h>

#include "cauchy_triangulation.hpp"
#include "data_loader.h"

void PrintCovarianceStats(const gtsam::Matrix &mat, const std::string &method)
{
    std::cout << "RMS: " << sqrt(mat.rowwise().squaredNorm().mean()) << std::endl;

    gtsam::Matrix centered = mat;                                             //.rowwise()  -mat.colwise().mean();
    gtsam::Matrix cov = (centered.adjoint() * centered) / double(mat.rows()); // - 1);
    std::cout << method << " covariance: " << std::endl;
    std::cout << cov << std::endl;
    std::cout << "Trace sqrt: " << sqrt(cov.trace()) << std::endl
              << std::endl;
}

void PrintDuration(const std::chrono::nanoseconds dur, double num_samples,
                   const std::string &method)
{
    double nanoseconds = dur.count() / num_samples;
    std::cout << "Time taken by " << method << ": " << nanoseconds * 1e-3
              << " microseconds per point" << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <cameras_file> <measurements_file> [3D_points_file]\n ";
        return EXIT_FAILURE;
    }

    std::string points_file_name("");
    if (argc == 4)
    {
        points_file_name = argv[3];
    }

    DataLoader data_loader(argv[1], argv[2], points_file_name);

    int num_points = data_loader.GetNumberOfPoints();

    gtsam::Matrix errorsDLT = gtsam::Matrix::Zero(num_points, 3);
    gtsam::Matrix errorsLOST = gtsam::Matrix::Zero(num_points, 3);
    gtsam::Matrix errorsDLTOpt = gtsam::Matrix::Zero(num_points, 3);
    gtsam::Matrix errorsLOSTOpt = gtsam::Matrix::Zero(num_points, 3);
    gtsam::Matrix errorsCauchy = gtsam::Matrix::Zero(num_points, 3);

    std::chrono::nanoseconds durationDLT(0);
    std::chrono::nanoseconds durationDLTOpt(0);
    std::chrono::nanoseconds durationLOST(0);
    std::chrono::nanoseconds durationLOSTOpt(0);
    std::chrono::nanoseconds durationCauchy(0);

    std::chrono::nanoseconds deltaDLT(0);
    std::chrono::nanoseconds deltaDLTOpt(0);
    std::chrono::nanoseconds deltaLOST(0);
    std::chrono::nanoseconds deltaLOSTOpt(0);
    std::chrono::nanoseconds deltaCauchy(0);

    const double measurementSigma = 1e-2;
    const double rank_tol = 1e-9;
    gtsam::SharedNoiseModel measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, measurementSigma);

    std::vector<int> valid_indices;
    valid_indices.reserve(num_points);

    for (int i = 0; i < num_points; ++i)
    {
        if (!data_loader.SetRequestedPointIndex(i))
        {
            continue;
        }

        auto cameras = data_loader.GetCameras();
        auto measurements = data_loader.GetMeasurements();
        auto landmarks = data_loader.GetPoint();

        try
        {
            // Use DLT + optimization to establish ground truth
            auto dltOptStart = std::chrono::high_resolution_clock::now();
            auto estimateDLTOptim = gtsam::triangulatePoint3<gtsam::Cal3_S2>(
                cameras, measurements, rank_tol, true, measurementNoise, false);
            auto dltOptEnd = std::chrono::high_resolution_clock::now();

            auto landmark = landmarks.value_or(estimateDLTOptim);
            errorsDLTOpt.row(i) = estimateDLTOptim - landmark;

            // DLT
            auto dltStart = std::chrono::high_resolution_clock::now();
            auto estimateDLT = gtsam::triangulatePoint3<gtsam::Cal3_S2>(
                cameras, measurements, rank_tol, false, measurementNoise, false);
            auto dltEnd = std::chrono::high_resolution_clock::now();
            errorsDLT.row(i) = estimateDLT - landmark;

            // LOST
            auto lostStart = std::chrono::high_resolution_clock::now();
            auto estimateLOST = gtsam::triangulatePoint3<gtsam::Cal3_S2>(
                cameras, measurements, rank_tol, false, measurementNoise, true);
            auto lostEnd = std::chrono::high_resolution_clock::now();
            errorsLOST.row(i) = estimateLOST - landmark;

            // LOST + optimization; should be the same as ground truth
            auto lostOptStart = std::chrono::high_resolution_clock::now();
            auto estimateLOSTOpt = gtsam::triangulatePoint3<gtsam::Cal3_S2>(
                cameras, measurements, rank_tol, true, measurementNoise, true);
            auto lostOptEnd = std::chrono::high_resolution_clock::now();
            durationLOSTOpt += std::chrono::high_resolution_clock::now() - lostOptStart;
            errorsLOSTOpt.row(i) = estimateLOSTOpt - landmark;

            // Cauchy
            auto cauchyStart = std::chrono::high_resolution_clock::now();
            auto estimateCauchy = triangulate_cauchy<gtsam::Cal3_S2>(
                cameras, measurements, false, measurementNoise);
            auto cauchyEnd = std::chrono::high_resolution_clock::now();
            durationCauchy += std::chrono::high_resolution_clock::now() - cauchyStart;
            errorsCauchy.row(i) = estimateCauchy - landmark;

            valid_indices.push_back(i);

            durationDLT += dltEnd - dltStart;
            durationLOST += lostEnd - lostStart;
            durationDLTOpt += dltOptEnd - dltOptStart;
            durationLOSTOpt += lostOptEnd - lostOptStart;
            durationCauchy += cauchyEnd - cauchyStart;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    int num_valid_points = valid_indices.size();
    gtsam::Matrix validErrorsDLT = gtsam::Matrix::Zero(num_valid_points, 3);
    gtsam::Matrix validErrorsDLTOpt = gtsam::Matrix::Zero(num_valid_points, 3);
    gtsam::Matrix validErrorsLOST = gtsam::Matrix::Zero(num_valid_points, 3);
    gtsam::Matrix validErrorsLOSTOpt = gtsam::Matrix::Zero(num_valid_points, 3);
    gtsam::Matrix validErrorsCauchy = gtsam::Matrix::Zero(num_valid_points, 3);

    for (int idx = 0; idx < num_valid_points; ++idx)
    {
        validErrorsDLT.row(idx) = errorsDLT.row(valid_indices[idx]);
        validErrorsLOST.row(idx) = errorsLOST.row(valid_indices[idx]);
        validErrorsDLTOpt.row(idx) = errorsDLTOpt.row(valid_indices[idx]);
        validErrorsLOSTOpt.row(idx) = errorsLOSTOpt.row(valid_indices[idx]);
        validErrorsCauchy.row(idx) = errorsCauchy.row(valid_indices[idx]);
    }

    PrintCovarianceStats(validErrorsDLT, "DLT");
    PrintCovarianceStats(validErrorsLOST, "LOST");
    PrintCovarianceStats(validErrorsDLTOpt, "DLT_OPT");
    PrintCovarianceStats(validErrorsLOSTOpt, "LOST_OPT");
    PrintCovarianceStats(validErrorsCauchy, "Cauchy");

    PrintDuration(durationDLT, num_points, "DLT");
    PrintDuration(durationLOST, num_points, "LOST");
    PrintDuration(durationDLTOpt, num_points, "DLT_OPT");
    PrintDuration(durationLOSTOpt, num_points, "LOST_OPT");
    PrintDuration(durationCauchy, num_points, "Cauchy");

    return EXIT_SUCCESS;
}