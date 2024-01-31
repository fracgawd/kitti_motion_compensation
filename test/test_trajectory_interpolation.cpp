#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/trajectory_interpolation.hpp"

using namespace kmc;

class TrajectoryInterpolationFixture : public ::testing::Test {
 protected:
  void SetUp() {
    Path const data_folder{"../assets/2011_09_26/2011_09_26_drive_0005_sync"};

    odometry_0_ = LoadOxts(data_folder, 0).value();
    odometry_1_ = LoadOxts(data_folder, 1).value();
    odometry_2_ = LoadOxts(data_folder, 2).value();
  }

  Oxts odometry_0_;
  Oxts odometry_1_;
  Oxts odometry_2_;
};

TEST_F(TrajectoryInterpolationFixture, TestOutOfRangeTime) {
  auto const trajectory_interpolator{trajectory_interpolation::TrajectoryInterpolator(odometry_0_, odometry_2_)};

  EXPECT_DEATH(trajectory_interpolator.GetPoseAtTime(0), "c");
}

TEST_F(TrajectoryInterpolationFixture, TestMiddleInterpolation) {
  auto const trajectory_interpolator{trajectory_interpolation::TrajectoryInterpolator(odometry_0_, odometry_2_)};

  Affine3d const interpolated_pose_1{trajectory_interpolator.GetPoseAtTime(odometry_1_.stamp)};
  Affine3d const groundtruth_pose_1{OxtsToPose(odometry_1_)};

  // unfortunately we cannot use the -- interpolated_pose_1 * groundtruth_pose_1.inv() = I -- idea here because small
  // errors in the rotation cause big errors in the translation because the numbers there are so big
  ASSERT_FLOAT_EQ((interpolated_pose_1.matrix() - groundtruth_pose_1.matrix()).sum(), -0.01504419);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}