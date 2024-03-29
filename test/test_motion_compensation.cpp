#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"
#include "kitti_motion_compensation/timestamp_mocking.hpp"

using namespace kmc;

class TestFrameFixture : public ::testing::Test {
 protected:
  void SetUp() {
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
    // NOTE: because the Oxts data starts after the lidar scan we can't motion compensate Scan(0) or the
    // last one, Scan(n)

    // Oxts measurements - O(0), O(1), O(2)
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

    VectorXd const intensities_1{VectorXd(3)};
    VectorXd const timestamps_1{GetPseudoTimeStamps(cloud_1, stamp_start, stamp_end)};

    LidarScan const scan_1{stamp_start, stamp_middle, stamp_end, cloud_1, intensities_1, timestamps_1};

    frame_ = std::make_unique<Frame>(MakeFrame(odometry_0, odometry_1, odometry_2, scan_1));
  }

  std::unique_ptr<Frame> frame_;  // has to be a pointer because we can't default construct Frame
};

TEST_F(TestFrameFixture, MotionCompensateFrame) {
  Time const requested_time{frame_->scan.stamp_middle};

  Pointcloud const motion_compensated_cloud{MotionCompensateFrame(*frame_, requested_time)};

  Vector4d const point_1{motion_compensated_cloud.row(0)};
  ASSERT_FLOAT_EQ(point_1(0), -0.27829874);
  ASSERT_FLOAT_EQ(point_1(1), 5.0);
  ASSERT_FLOAT_EQ(point_1(2), 0.0);
  ASSERT_FLOAT_EQ(point_1(3), 1.0);

  Vector4d const point_2{motion_compensated_cloud.row(1)};
  ASSERT_FLOAT_EQ(point_2(0), frame_->scan.cloud.row(1)(0));
  ASSERT_FLOAT_EQ(point_2(1), frame_->scan.cloud.row(1)(1));
  ASSERT_FLOAT_EQ(point_2(2), frame_->scan.cloud.row(1)(2));
  ASSERT_FLOAT_EQ(point_2(3), frame_->scan.cloud.row(1)(3));

  Vector4d const point_3{motion_compensated_cloud.row(2)};
  ASSERT_FLOAT_EQ(point_3(0), 0.27829874);
  ASSERT_FLOAT_EQ(point_3(1), -5.0);
  ASSERT_FLOAT_EQ(point_3(2), 0.0);
  ASSERT_FLOAT_EQ(point_3(3), 1.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
