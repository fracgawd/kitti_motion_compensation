#pragma once

#include <chrono>
#include <filesystem>
#include <optional>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

namespace kmc {

//** typdefs **//

// Note that this is a pointcloud *WITH* itensity values, not just a set of 3D
// points
typedef Eigen::Matrix<double, -1, 4> PointCloud;

typedef std::chrono::system_clock::time_point Time;

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

struct LidarCloud {
  Time stamp_start;
  Time stamp_middle; // camera trigger time
  Time stamp_end;

  PointCloud cloud;
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
  Frame(Oxts odometry, LidarCloud cloud,
        std::optional<Images> images = std::nullopt)
      : odometry_{odometry}, cloud_{cloud}, images_{images} {}

  Oxts odometry_;
  LidarCloud cloud_;

  // for people who just want to process the pointclouds, they don't need to
  // load the images, but there are probably enough times where someone will
  // want to project a pointcloud onto one of the images to visualize the effect
  // of the motion compensation that we will build the repo with the images as
  // an optional part of the frame.
  std::optional<Images> images_;
};

} // namespace kmc