#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/trajectory_interpolation.hpp"
#include "kitti_motion_compensation/utilities_for_testing.hpp"

using namespace kmc;

class TrajectoryInterpolationFixtureRealOdometry : public ::testing::Test {
 protected:
  void SetUp() {
    Path const data_folder{"../testing_assets/2011_09_26/2011_09_26_drive_0005_sync"};

    odometry_0_ = LoadOxts(data_folder, 0).value();
    odometry_1_ = LoadOxts(data_folder, 1).value();
    odometry_2_ = LoadOxts(data_folder, 2).value();
  }

  Oxts odometry_0_;
  Oxts odometry_1_;
  Oxts odometry_2_;
};

TEST_F(TrajectoryInterpolationFixtureRealOdometry, TestOutOfRangeTime) {
  auto const trajectory_interpolator{trajectory_interpolation::TrajectoryInterpolator(odometry_0_, odometry_2_)};

  EXPECT_DEATH(trajectory_interpolator.GetPoseAtTime(0), "c");
}

TEST_F(TrajectoryInterpolationFixtureRealOdometry, TestInterpolationClass) {
  auto const trajectory_interpolator{trajectory_interpolation::TrajectoryInterpolator(odometry_0_, odometry_2_)};

  Affine3d const interpolated_pose_1{trajectory_interpolator.GetPoseAtTime(odometry_1_.stamp)};
  Affine3d const groundtruth_pose_1{OxtsToPose(odometry_1_)};

  // unfortunately we cannot use the -- interpolated_pose_1 * groundtruth_pose_1.inv() = I -- idea here because small
  // errors in the rotation cause big errors in the translation because the numbers there are so large
  ASSERT_FLOAT_EQ((interpolated_pose_1.matrix() - groundtruth_pose_1.matrix()).sum(), -0.01504419);
}

TEST_F(TrajectoryInterpolationFixtureRealOdometry, TestInterpolationFunction) {
  Affine3d const interpolated_pose_1{
      trajectory_interpolation::InterpolateTrajectory(odometry_0_, odometry_2_, odometry_1_.stamp)};
  Affine3d const groundtruth_pose_1{OxtsToPose(odometry_1_)};

  ASSERT_FLOAT_EQ((interpolated_pose_1.matrix() - groundtruth_pose_1.matrix()).sum(), -0.01504419);
}

class TrajectoryInterpolationFixtureArtificialPoses : public ::testing::Test {
 protected:
  void SetUp() {
    time_0_ = 0;
    pose_0_ = ArtificialPose(0, 0);

    time_1_ = 50;
    pose_1_ = ArtificialPose(0.5, 0.5);

    time_2_ = 100;
    pose_2_ = ArtificialPose(1.0, 1.0);
  }

 private:
  Affine3d ArtificialPose(double const x_rotation, double const x_translation) {
    Eigen::Affine3d pose{Affine3d::Identity()};
    pose.rotate(Eigen::AngleAxisd{x_rotation, Eigen::Vector3d::UnitX()});
    pose.translation() = Eigen::Vector3d{x_translation, 0, 0};

    return pose;
  }

 protected:
  Time time_0_;
  Affine3d pose_0_{Affine3d::Identity()};

  Time time_1_;
  Affine3d pose_1_{Affine3d::Identity()};

  Time time_2_;
  Affine3d pose_2_{Affine3d::Identity()};
};

TEST_F(TrajectoryInterpolationFixtureArtificialPoses, TestInterpolationClassPoseConstructor) {
  auto const trajectory_interpolator{
      trajectory_interpolation::TrajectoryInterpolator(time_0_, pose_0_, time_2_, pose_2_)};

  Affine3d const interpolated_pose_1{trajectory_interpolator.GetPoseAtTime(time_1_)};

  ASSERT_TRUE(utilities_for_testing::TransformationMatricesAreTheSame(interpolated_pose_1, pose_1_));
}

TEST_F(TrajectoryInterpolationFixtureArtificialPoses, TestRelativePoseBetweenTimes) {
  auto const trajectory_interpolator{
      trajectory_interpolation::TrajectoryInterpolator(time_0_, pose_0_, time_2_, pose_2_)};

  auto const tf_0_1{trajectory_interpolator.RelativePoseBetweenTimes(time_0_, time_1_)};
  auto const tf_1_2{trajectory_interpolator.RelativePoseBetweenTimes(time_1_, time_2_)};

  ASSERT_TRUE(utilities_for_testing::TransformationMatricesAreTheSame(tf_0_1, tf_1_2));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}