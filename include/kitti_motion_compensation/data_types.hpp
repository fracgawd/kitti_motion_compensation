#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <optional>

namespace kmc {

//** typdefs **//

// Note that this is homogeneous set of points (the last column is just 1)
using Pointcloud = Eigen::MatrixX4d;

// Yea I know I know, I would have liked to use some fancy official time
// tracking structure, but assuming there are no recordings that cross midnight,
// and that microsecond preciscion is enough, than we can just use double -_-
// I am pretty sure I will regret this decision later :()
using Time = double;

using Path = std::filesystem::path;

// This represents the 6 component rotation and pose vector as used in the lie algebra
using Twist = Eigen::Matrix<double, 6, 1>;

using Affine3d = Eigen::Affine3d;
using MatrixX4d = Eigen::MatrixX4d;
using VectorXd = Eigen::VectorXd;
using Vector4d = Eigen::Vector4d;
using Index = Eigen::Index;

//** structs **//

struct Oxts {
  Time stamp;

  // See the "raw data development kit" README for an explanation of the
  // oxts files
  double lat;
  double lon;
  double alt;
  double roll;
  double pitch;
  double yaw;
  double vf;
  double vl;
  double vu;
};

struct LidarScan {
  Time stamp_start;
  Time stamp_middle;  // camera trigger time
  Time stamp_end;

  Pointcloud cloud;
  VectorXd intensities;  // intrinsic part of the measurement (at least for KITTI)
  VectorXd timestamps;   // calculated value (KITTI -should- have included per-point timestamps -_-)
};

struct Image {
  Time stamp;

  cv::Mat image;
};

struct Images {
  Image image_00;
  Image image_01;
  Image image_02;
  Image image_03;
};

//** data structures with some logic **//

struct Frame {
  Frame(Affine3d const &start_pose, Affine3d const &end_pose, kmc::LidarScan const &lidar_scan,
        std::optional<kmc::Images> const camera_images = std::nullopt)
      : T_start{start_pose}, T_end{end_pose}, scan{lidar_scan}, images{camera_images} {}
  Affine3d T_start;  // the SE3 pose at the start of the scan (at timestamp scan.scan_start)
  Affine3d T_end;    // the SE3 pose at the end of the scan (at timestamp scan.scan_end)

  kmc::LidarScan scan;

  // for people who just want to process the pointclouds, they don't need to
  // load the images, but there are probably enough times where someone will
  // want to project a pointcloud onto one of the images to visualize the effect
  // of the motion compensation that we will build the repo with the images as
  // an optional part of the frame.
  std::optional<kmc::Images> images;
};

}  // namespace kmc

namespace kmc::viz {

// pinhole camera intrinsic + projection matrix
typedef Eigen::Matrix<double, 3, 4> P;

struct CameraCalibration {
  Eigen::Vector2d S;
  Eigen::Matrix3d K;
  Eigen::Matrix<double, 5, 1> D;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  Eigen::Vector2d S_rect;
  Eigen::Matrix3d R_rect;
  P P_rect;
};

struct CameraCalibrations {
  CameraCalibration camera_00;
  CameraCalibration camera_01;
  CameraCalibration camera_02;
  CameraCalibration camera_03;
};

}  // namespace kmc::viz