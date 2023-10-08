#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/timestamp_mocking.hpp"

using namespace kmc;

// TODO(jack): make test fixture - this is copied in two places - BAD!!!
Frame MakeMotionCompensationTestFrame() {
  // This is how the KITTI dataset is collected with respect to time, three scans shown for example:
  //
  //    Time: ----- 0ms ---- 50ms ---- 100ms ----- 150ms ---- 200ms ---- 250ms ---- 300ms ---- 350ms ----
  // ----------------------------------------------------------------------------------------------------
  //    Oxts:                 O(0)                 O(1)                  O(2)                  O(4)
  // ----------------------------------------------------------------------------------------------------
  // Scan(0):      start     middle     end
  // Scan(1):                           start      middle     end
  // Scan(2):                                                 start      middle     end
  // ----------------------------------------------------------------------------------------------------
  //  Camera:                 C(0)                 C(1)                  C(2)                  C(4)
  //
  // NOTE: because the Oxts data starts after the lidar scan we can't motion compensate Scan(0) (or the
  // last one, Scan(n))

  // represent the first three Oxts measurements - O(0), O(1), O(2) - here we have a roughly two meter forward motion
  // from O(0)to O(2).
  Oxts const odometry_0{Time(0.05), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Oxts const odometry_1{Time(0.15), 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Oxts const odometry_2{Time(0.25), 0.0, 0.00002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // represent Scan(1)
  Pointcloud cloud_1 = MatrixX4d(3, 4);
  cloud_1.row(0) = Vector4d{0.0, -5.0, 0.0, 1.0};
  cloud_1.row(1) = Vector4d{5.0, 0.0, 0.0, 1.0};
  cloud_1.row(2) = Vector4d{0.0, 5.0, 0.0, 1.0};

  Time const stamp_start{Time(0.1)};
  Time const stamp_middle{Time(0.15)};
  Time const stamp_end{Time(0.2)};

  VectorXd const timestamps_1 = GetPseudoTimeStamps(cloud_1, stamp_start, stamp_end);
  VectorXd const intensities_1 = VectorXd(3);

  LidarScan const scan_1{stamp_start, stamp_middle, stamp_end, cloud_1, intensities_1, timestamps_1};

  return MakeFrame(odometry_0, odometry_1, odometry_2, scan_1);
}

TEST(FractionOfScanCompletedTest, XXX) {
  Frame const test_frame{MakeMotionCompensationTestFrame()};

  Pointcloud const &test_cloud{test_frame.scan.cloud};

  ASSERT_FLOAT_EQ(FractionOfScanCompleted(test_cloud.row(0)), 0.25);
  ASSERT_FLOAT_EQ(FractionOfScanCompleted(test_cloud.row(1)), 0.5);
  ASSERT_FLOAT_EQ(FractionOfScanCompleted(test_cloud.row(2)), 0.75);
}

TEST(PsuedoTimeStampTest, XXX) {
  Frame const test_frame{MakeMotionCompensationTestFrame()};

  Pointcloud const &test_cloud{test_frame.scan.cloud};
  Time const scan_start{test_frame.scan.stamp_start};
  Time const scan_end{test_frame.scan.stamp_end};

  Time const point_1_stamp{GetPseudoTimeStamp(test_cloud.row(0), scan_start, scan_end)};
  Time const point_2_stamp{GetPseudoTimeStamp(test_cloud.row(1), scan_start, scan_end)};
  Time const point_3_stamp{GetPseudoTimeStamp(test_cloud.row(2), scan_start, scan_end)};

  ASSERT_FLOAT_EQ(point_1_stamp, 0.125);
  ASSERT_FLOAT_EQ(point_2_stamp, 0.15);
  ASSERT_FLOAT_EQ(point_3_stamp, 0.175);
}

TEST(PsuedoTimeStampFrameInitializationTest, XXX) {
  // Test the `GetPseudoTimeStamps()` called in the test data construction method
  Frame const test_frame{MakeMotionCompensationTestFrame()};

  Time const point_1_stamp{test_frame.scan.timestamps(0)};
  Time const point_2_stamp{test_frame.scan.timestamps(1)};
  Time const point_3_stamp{test_frame.scan.timestamps(2)};

  ASSERT_FLOAT_EQ(point_1_stamp, 0.125);
  ASSERT_FLOAT_EQ(point_2_stamp, 0.15);
  ASSERT_FLOAT_EQ(point_3_stamp, 0.175);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
