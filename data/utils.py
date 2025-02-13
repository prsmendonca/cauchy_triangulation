import matplotlib.pyplot as plt
import numpy
from pathlib import Path
import scipy
import struct
import scipy.linalg


def car(hX):
    return hX[..., :-1] / hX[..., -1]

def hom(X):
    shape = list(X.shape)
    shape[-1] = 1
    return numpy.concatenate((X, numpy.ones(shape)), axis=-1)


def decompose_camera(cam):
    t = -scipy.linalg.inv(cam[:, :3]) @ cam[:, 3]
    
    K, R = scipy.linalg.rq(cam[:, :3])
    if scipy.linalg.det(R) < 0:
        K[:, 0] *= -1
        R[0, :] *= -1
    
    K /= K[2, 2]
    if K[0, 0] < 0:
        for i in range(2):
            K[:, i] *= -1
            R[i, :] *= -1

    return K, R, t

def DLT_triangulation(cameras, measurements):
    # Number of cameras
    n_cameras = len(cameras)
    # Number of 3D points
    n_points = len(measurements)

    assert(n_cameras == n_points), "Number of cameras and 3D points must be equal."

    # Construct the matrix A
    A = numpy.zeros((3 * n_cameras, 4 + n_cameras), dtype=numpy.float64)
    for i in range(n_cameras):
        camera = cameras[i]
        measurement = measurements[i]

        # Construct the matrix A
        A[3*i:, :] = camera[0, :] - measurement[0] * camera[2, :]
        A[2 * i + 1, :] = camera[1, :] - measurement[1] * camera[2, :]

    # Perform SVD
    U, S, V = numpy.linalg.svd(A)

    # Extract the solution
    X = V[-1, :]

    return X


def show_cam(cam, ax, scale=3.0):
    # Add the axes of the pinhole camera to the plot.
    K, R, t = decompose_camera(cam)
    start = t.flatten()
    ax.plot(*start, "o", color="k")
    for i, color in enumerate(("r", "g", "b")):
        end_in_cam = numpy.zeros(3)
        end_in_cam[i] = scale
        end = (R.T @ end_in_cam + t).flatten()
        data = numpy.stack((start, end), axis=0)
        ax.plot(*data.T, color=color)


def save_cameras(file_name, cameras):
    with open(file_name, "wb") as cameras_file:
        for cam in cameras:
            K, R, t = decompose_camera(cam)

            rotation = R.T
            translation = t
            intrinsics = numpy.array([K[0, 0], K[1, 1], K[0, 1], K[0, 2], K[1, 2]])
            data = numpy.hstack(
                (
                    rotation.flatten(),
                    translation.flatten(),
                    intrinsics.flatten()
                )
            )
            
            for i in range(len(data)):
                cameras_file.write(struct.pack("d", data[i]))
            
    cameras_file.close()

def save_measurements(file_name, measurements):
    # Iterate over 3D points
    with open(file_name, "wb") as measurements_file:
        for idx_3D, measurements_in_cam in enumerate(measurements):
            for idx_cam, measurement in enumerate(measurements_in_cam):
                if measurement[0] == -1:
                    continue
            
                measurements_file.write(
                    struct.pack(
                        "iidd", idx_3D, idx_cam, measurement[0], measurement[1]
                    )
                )

    measurements_file.close()

def save_points(file_name, points):
    # Iterate over 3D points
    with open(file_name, "wb") as points_file:
        for pt in points:
            for i in range(3):
                points_file.write(struct.pack("d", pt[i]))

    points_file.close()


def load_corridor_or_house_models(corridor=True):
    main = "corridor" if corridor else "model_house"
    secondary = "bt" if corridor else "house"

    path_3D = Path(main).joinpath("3D")
    points_3D = numpy.loadtxt(path_3D.joinpath(secondary + ".p3d"))

    cam_file_names = [i for i in path_3D.glob(secondary + ".*.P")]
    cam_file_names.sort()
    cameras = []
    for file_name in cam_file_names:
        cam = numpy.loadtxt(file_name)
        cameras.append(cam)
    cameras = numpy.stack(cameras)


    path_2D = Path(main).joinpath("2D")
    meas_file_names = [i for i in path_2D.glob(secondary + ".*.corners")]
    meas_file_names.sort()
    corners = []
    for idx_cam, file_name in enumerate(meas_file_names):
        corners.append(numpy.loadtxt(file_name))

    measurements = []
    with open(path_2D.joinpath(secondary + ".nview-corners"), "rt") as match_file:
        for line in match_file:
            measurement = []
            for idx_cam, chars in enumerate(line.rstrip().split()):
                if chars == "*":
                    corner = [-1, -1]
                else:
                    corner = corners[idx_cam][int(chars)]
                measurement.append(corner)
            measurements.append(numpy.array(measurement))


    plt.close("all")
    fig = plt.figure(figsize=(6, 6))
    ax = plt.axes(projection="3d")
    ax.scatter(*(points_3D.T), marker=".")
    for cam in cameras:
        show_cam(cam, ax)
    ax.view_init(elev=-166, azim=44)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(main)

    plt.show()

    return cameras, measurements, points_3D