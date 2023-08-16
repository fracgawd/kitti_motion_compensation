#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <optional>

namespace kmc {

//** typdefs **//

// Note that this is a pointcloud *WITH* itensity values, not just a set of 3D
// points
typedef Eigen::MatrixX4d Pointcloud;

// Yea I know I know, I would have liked to use some fancy official time
// tracking structure, but assuming there are no recordings that cross midnight,
// and that microsecond preciscion is enough, than we can just use double -_-
// I am pretty sure I will regret this decision later :()
typedef double Time;

typedef std::filesystem::path Path;

//** structs **//

struct Oxts {
  Time stamp;

  // See the "raw data development kit" README for an explanation of the
  // oxts files
  double vf;
  double vl;
  double vu;
};

struct LidarScan {
  Time stamp_start;
  Time stamp_middle;  // camera trigger time
  Time stamp_end;

  Pointcloud cloud;
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
  Frame(Oxts odometry, LidarScan scan, std::optional<Images> images = std::nullopt)
      : odometry_{odometry}, scan_{scan}, images_{images} {}

  Oxts odometry_;
  LidarScan scan_;

  // for people who just want to process the pointclouds, they don't need to
  // load the images, but there are probably enough times where someone will
  // want to project a pointcloud onto one of the images to visualize the effect
  // of the motion compensation that we will build the repo with the images as
  // an optional part of the frame.
  std::optional<Images> images_;
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