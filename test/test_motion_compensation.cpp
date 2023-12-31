#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/lie_algebra.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"
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
  cloud_1.row(0) = Vector4d{0.0, 5.0, 0.0, 1.0};
  cloud_1.row(1) = Vector4d{5.0, 0.0, 0.0, 1.0};
  cloud_1.row(2) = Vector4d{0.0, -5.0, 0.0, 1.0};

  Time const stamp_start{Time(0.1)};
  Time const stamp_middle{Time(0.15)};
  Time const stamp_end{Time(0.2)};

  VectorXd const timestamps_1 = GetPseudoTimeStamps(cloud_1, stamp_start, stamp_end);
  VectorXd const intensities_1 = VectorXd(3);

  LidarScan const scan_1{stamp_start, stamp_middle, stamp_end, cloud_1, intensities_1, timestamps_1};

  return MakeFrame(odometry_0, odometry_1, odometry_2, scan_1);
}

TEST(ScaledTimeDisplacmentTest, XXX) {
  Frame const test_frame{MakeMotionCompensationTestFrame()};

  Time const scan_start{test_frame.scan.stamp_start};
  Time const scan_end{test_frame.scan.stamp_end};
  // The "anchor time" is named here to make it clear what it is, in the context of the `ScaledTimeDisplacment()`
  // function. But what it really is refered to in the rest of the code is usually the `requested_time`, which is the
  // point in time that we want to motion compensate the pointcloud too. Which is usally half way through the scan, when
  // the cameras are triggered.
  Time anchor_time{test_frame.scan.stamp_middle};

  Time point_i_time{0.125};
  double const x0{ScaledTimeDisplacment(point_i_time, anchor_time, scan_start, scan_end)};
  ASSERT_FLOAT_EQ(x0, -0.25);

  point_i_time = 0.175;
  double const x1{ScaledTimeDisplacment(point_i_time, anchor_time, scan_start, scan_end)};
  ASSERT_FLOAT_EQ(x1, 0.25);

  // Change the anchor time now so it isn't centered in the middle
  anchor_time = test_frame.scan.stamp_middle + 0.025;

  point_i_time = 0.125;  // the query time simulate the individual point times
  double const x2{ScaledTimeDisplacment(point_i_time, anchor_time, scan_start, scan_end)};
  ASSERT_FLOAT_EQ(x2, -0.50);

  point_i_time = 0.2;  // the query time simulate the individual point times
  double const x3{ScaledTimeDisplacment(point_i_time, anchor_time, scan_start, scan_end)};
  ASSERT_FLOAT_EQ(x3, 0.25);
}

TEST(MotionCompensationTest, XXX) {
  Frame const test_frame{MakeMotionCompensationTestFrame()};
  Time const requested_time{test_frame.scan.stamp_middle};

  Pointcloud const motion_compensated_cloud{MotionCompensateFrame(test_frame, requested_time)};

  Vector4d const point_1{motion_compensated_cloud.row(0)};
  ASSERT_FLOAT_EQ(point_1(0), -0.27829874);
  ASSERT_FLOAT_EQ(point_1(1), 5.0);
  ASSERT_FLOAT_EQ(point_1(2), 0.0);
  ASSERT_FLOAT_EQ(point_1(3), 1.0);

  Vector4d const point_2{motion_compensated_cloud.row(1)};
  ASSERT_FLOAT_EQ(point_2(0), test_frame.scan.cloud.row(1)(0));
  ASSERT_FLOAT_EQ(point_2(1), test_frame.scan.cloud.row(1)(1));
  ASSERT_FLOAT_EQ(point_2(2), test_frame.scan.cloud.row(1)(2));
  ASSERT_FLOAT_EQ(point_2(3), test_frame.scan.cloud.row(1)(3));

  Vector4d const point_3{motion_compensated_cloud.row(2)};
  ASSERT_FLOAT_EQ(point_3(0), 0.27829874);
  ASSERT_FLOAT_EQ(point_3(1), -5.0);
  ASSERT_FLOAT_EQ(point_3(2), 0.0);
  ASSERT_FLOAT_EQ(point_3(3), 1.0);
}

// Test a edge case frames - the edge case frames are either the first or last frame of a sequence because they are
// mising and odometry need to interpolate their start and end pose respectively.

Frame MakeMotionCompensationTestEdgeCaseFrame() {
  Oxts const odometry_0{Time(0.05), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Oxts const odometry_1{Time(0.15), 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // represent Scan(0)
  Pointcloud cloud_0 = MatrixX4d(3, 4);
  cloud_0.row(0) = Vector4d{0.0, -5.0, 0.0, 1.0};
  cloud_0.row(1) = Vector4d{5.0, 0.0, 0.0, 1.0};
  cloud_0.row(2) = Vector4d{0.0, 5.0, 0.0, 1.0};

  Time const stamp_start{Time(0.0)};
  Time const stamp_middle{Time(0.05)};
  Time const stamp_end{Time(0.1)};

  VectorXd const timestamps_0 = GetPseudoTimeStamps(cloud_0, stamp_start, stamp_end);
  VectorXd const intensities_0 = VectorXd(3);

  LidarScan const scan_0{stamp_start, stamp_middle, stamp_end, cloud_0, intensities_0, timestamps_0};

  // We simulate the `LoadSingleFrame` frame function being applied to `frame_id=0` by passing `odometry_0` twice
  return MakeFrame(odometry_0, odometry_0, odometry_1, scan_0);
}

// TEST(MotionCompensationTestEdgeCase, XXX) {
//   // TODO(jack): add edge case motion compensation logic
//   ASSERT_TRUE(false);
// }

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
