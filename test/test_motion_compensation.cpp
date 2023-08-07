#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"

// TODO(jack): make test fixture
kmc::Frame MakeMotionCompensationTestFrame() {
  // moving directly forward at 1m/s
  kmc::Oxts const odometry{kmc::Time(0.0), 1.0, 0.0, 0.0};

  // This represents a very simple kitti scan. Counter clockwise starting to the
  // right of the car 5 meters, then right in front of the car at five meters,
  // and then directly to the left of the car 5 meters.
  //
  // The real question is though, where were these points when the camera was
  // triggered (the <requested_time> parameter)...? That is what we are about to
  // solve :)
  //
  kmc::Pointcloud cloud = Eigen::MatrixX4d(3, 4);
  cloud.row(0) = Eigen::Vector4d{0.0, -5.0, 0.0, 0.0};
  cloud.row(1) = Eigen::Vector4d{5.0, 0.0, 0.0, 0.0};
  cloud.row(2) = Eigen::Vector4d{0.0, 5.0, 0.0, 0.0};

  kmc::LidarScan const scan{kmc::Time(0.0), kmc::Time(0.05), kmc::Time(0.1),
                            cloud};

  return kmc::Frame{odometry, scan};
}

TEST(FractionOfScanCompletedTest, XXX) {
  kmc::Frame const test_frame{MakeMotionCompensationTestFrame()};

  kmc::Pointcloud const &test_cloud{test_frame.scan_.cloud};

  ASSERT_FLOAT_EQ(kmc::FractionOfScanCompleted(test_cloud.row(0)), 0.25);
  ASSERT_FLOAT_EQ(kmc::FractionOfScanCompleted(test_cloud.row(1)), 0.5);
  ASSERT_FLOAT_EQ(kmc::FractionOfScanCompleted(test_cloud.row(2)), 0.75);
}

TEST(PsuedoTimeStampTest, XXX) {
  kmc::Frame const test_frame{MakeMotionCompensationTestFrame()};

  kmc::Time const scan_start{test_frame.scan_.stamp_start};
  kmc::Time const scan_end{test_frame.scan_.stamp_end};
  kmc::Pointcloud const &test_cloud{test_frame.scan_.cloud};

  kmc::Time const point_1_stamp{
      kmc::GetPseudoTimeStamp(test_cloud.row(0), scan_start, scan_end)};
  kmc::Time const point_2_stamp{
      kmc::GetPseudoTimeStamp(test_cloud.row(1), scan_start, scan_end)};
  kmc::Time const point_3_stamp{
      kmc::GetPseudoTimeStamp(test_cloud.row(2), scan_start, scan_end)};

  ASSERT_FLOAT_EQ(point_1_stamp, 0.025);
  ASSERT_FLOAT_EQ(point_2_stamp, 0.05);
  ASSERT_FLOAT_EQ(point_3_stamp, 0.075);
}

TEST(MotionCompensationTest, XXX) {
  kmc::Frame const test_frame{MakeMotionCompensationTestFrame()};
  kmc::Time const requested_time{test_frame.scan_.stamp_middle};

  kmc::Pointcloud motion_compensated_pointcloud{
      kmc::MotionCompensate(test_frame, requested_time)};

  // for (Eigen::Index i{0}; i < motion_compensated_pointcloud.rows(); ++i) {
  //   std::cout << motion_compensated_pointcloud.row(i).matrix() << std::endl;
  // }

  // From the point where we are half way into the scan(i.e.
  // requested_time=0.05 for this test case), moving forward at 1m/s, a point
  // measured 1/4 of the way through the scan is now 0.025m behind us.
  Eigen::Vector4d const point_1{motion_compensated_pointcloud.row(0)};
  ASSERT_FLOAT_EQ(point_1(0), -0.025);
  ASSERT_FLOAT_EQ(point_1(1), -5.0);
  ASSERT_FLOAT_EQ(point_1(2), 0.0);

  // The requested time is the same as the actual time for this point (sure I
  // know its technically pseudo time) the point is the same as what the lidar
  // measured.
  Eigen::Vector4d const point_2{motion_compensated_pointcloud.row(1)};
  ASSERT_FLOAT_EQ(point_2(0), test_frame.scan_.cloud.row(1)(0));
  ASSERT_FLOAT_EQ(point_2(1), test_frame.scan_.cloud.row(1)(1));
  ASSERT_FLOAT_EQ(point_2(2), test_frame.scan_.cloud.row(1)(2));

  // From the point where we are half way into the scan(i.e.
  // requested_time=0.05 for this test case), moving forward at 1m/s, a point
  // measured 3/4 of the way through the scan is 0.025m in front of us.
  Eigen::Vector4d const point_3{motion_compensated_pointcloud.row(2)};
  ASSERT_FLOAT_EQ(point_3(0), 0.025);
  ASSERT_FLOAT_EQ(point_3(1), 5.0);
  ASSERT_FLOAT_EQ(point_3(2), 0.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}