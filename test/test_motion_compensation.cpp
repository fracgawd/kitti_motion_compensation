#include <gtest/gtest.h>

#include "kitti_motion_compensation/camera_model.hpp"
#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/lie_algebra.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"

using namespace kmc;

// TODO(jack): make test fixture
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

TEST(MotionCompensationTest, XXX) {
  Frame const test_frame{MakeMotionCompensationTestFrame()};
  Time const requested_time{test_frame.scan.stamp_middle};

  Pointcloud const motion_compensated_cloud{MotionCompensateFrame(test_frame, requested_time)};

  Vector4d const point_1{motion_compensated_cloud.row(0)};
  ASSERT_FLOAT_EQ(point_1(0), 0.27829874);
  ASSERT_FLOAT_EQ(point_1(1), -5.0);
  ASSERT_FLOAT_EQ(point_1(2), 0.0);
  ASSERT_FLOAT_EQ(point_1(3), 1.0);

  Vector4d const point_2{motion_compensated_cloud.row(1)};
  ASSERT_FLOAT_EQ(point_2(0), test_frame.scan.cloud.row(1)(0));
  ASSERT_FLOAT_EQ(point_2(1), test_frame.scan.cloud.row(1)(1));
  ASSERT_FLOAT_EQ(point_2(2), test_frame.scan.cloud.row(1)(2));
  ASSERT_FLOAT_EQ(point_2(3), test_frame.scan.cloud.row(1)(3));

  Vector4d const point_3{motion_compensated_cloud.row(2)};
  ASSERT_FLOAT_EQ(point_3(0), -0.27829874);
  ASSERT_FLOAT_EQ(point_3(1), 5.0);
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

TEST(MotionCompensationTestEdgeCase, XXX) {
  // TODO(jack): add edge case motion compensation logic
  ASSERT_TRUE(false);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// namespace XXX {
// using Affine3d = Eigen::Affine3d;

// struct Frame {
//   Frame(Affine3d const &start_pose, Affine3d const &end_pose, kmc::LidarScan const &lidar_scan,
//         std::optional<kmc::Images> const camera_images = std::nullopt)
//       : T_start{start_pose}, T_end{end_pose}, scan{lidar_scan}, images{camera_images} {}
//   Affine3d T_start;  // the SE3 pose at the start of the scan (at scan.scan_start)
//   Affine3d T_end;    // the SE3 pose at the end of the scan (at scan.scan_end)

//   kmc::LidarScan scan;

//   // for people who just want to process the pointclouds, they don't need to
//   // load the images, but there are probably enough times where someone will
//   // want to project a pointcloud onto one of the images to visualize the effect
//   // of the motion compensation that we will build the repo with the images as
//   // an optional part of the frame.
//   std::optional<kmc::Images> images;
// };

// Affine3d InterpolatePose(Affine3d const &pose_0, Affine3d const &pose_1, double const x) {
//   // Given two poses, pose_0 and pose_1, get a pose somewhere in between. The fraction of the distance between the
//   two
//   // poses is specified by "x" (0<x<1)
//   //
//   // If you want to think about is on a very simplified level, but that still capture the essence, this function does
//   // the following (poses don't really work like this, but it helps understanding -_-):
//   //
//   //      pose_interpolated = pose_0 + (pose_1 - pose_0) * x

//   if ((x < 0) or (1 < x)) {
//     std::cerr << "You gave an invalid interpolation fraction -_- you are only allowed to interpolate within two
//     poses, "
//                  "not further than that."
//               << std::endl;
//     exit(0);
//   }

//   kmc::Twist const delta_pose{x * kmc::lie::Log(pose_0.inverse() * pose_1)};

//   return pose_0 * kmc::lie::Exp(delta_pose);
// }

// Affine3d InterpolateFramePose(kmc::Oxts const &odometry_0, kmc::Oxts const &odometry_1, kmc::Time requested_time) {
//   double const x{(requested_time - odometry_0.stamp) / (odometry_1.stamp - odometry_0.stamp)};

//   // TODO(jack): give OxtsToPose a default scale value
//   return InterpolatePose(kmc::OxtsToPose(odometry_0, 1), kmc::OxtsToPose(odometry_1, 1), x);
// }

// Frame MakeFrame(kmc::Oxts const &odometry_n_m_1, kmc::Oxts const &odometry_n, kmc::Oxts const &odometry_n_p_1,
//                 kmc::LidarScan const &lidar_scan, std::optional<kmc::Images> const camera_images = std::nullopt) {
//   // odometry_n_m_1 - oxts packet of the previous frame (n_m_1 = "n minus 1")
//   // odometry_n - oxts of the current frame (n), usually happens at around the middle of the scan
//   // odometry_n_p_1 - oxts packet of the next frame (n_p_1 = "n plus 1")

//   // Because we do not directly get the pose of the sensor at the start and end of the scan we need to interpolate in
//   // between the nearest odometry measurements. For the start of the scan that will be "n-1" and "n" and for the end
//   of
//   // the scan that will be "n" and "n+1".

//   Affine3d const scan_start_pose{InterpolateFramePose(odometry_n_m_1, odometry_n, lidar_scan.stamp_start)};
//   Affine3d const scan_end_pose{InterpolateFramePose(odometry_n//   Eigen::Vector4d const
//   point_1{motion_compensated_cloud.row(0)}; ASSERT_FLOAT_EQ(point_1(0), 0.27829874); ASSERT_FLOAT_EQ(point_1(1),
//   -5.0); ASSERT_FLOAT_EQ(point_1(2), 0.0); ASSERT_FLOAT_EQ(point_1(3), 1.0);

//   Eigen::Vector4d const point_2{motion_compensated_cloud.row(1)};
//   ASSERT_FLOAT_EQ(point_2(0), test_frame.scan.cloud.row(1)(0));
//   ASSERT_FLOAT_EQ(point_2(1), test_frame.scan.cloud.row(1)(1));
//   ASSERT_FLOAT_EQ(point_2(2), test_frame.scan.cloud.row(1)(2));
//   ASSERT_FLOAT_EQ(point_2(3), test_frame.scan.cloud.row(1)(3));

//   Eigen::Vector4d const point_3{motion_compensated_cloud.row(2)};
//   ASSERT_FLOAT_EQ(point_3(0), -0.27829874);
//   ASSERT_FLOAT_EQ(point_3(1), 5.0);
//   ASSERT_FLOAT_EQ(point_3(2), 0.0);
//   ASSERT_FLOAT_EQ(point_3(3), 1.0);, odometry_n_p_1, lidar_scan.stamp_end)};

//   return Frame(scan_start_pose, scan_end_pose, lidar_scan, camera_images);
// }

// // TODO(jack): make test fixture
// Frame MakeMotionCompensationTestFrame2() {
//   // This is how the KITTI dataset is collected with respect to time, three scans shown for example:
//   //
//   //    Time: ----- 0ms ----- 5ms ----- 10ms ----- 15ms ----- 20ms ----- 25ms ----- 30ms ----- 35ms -----
//   // ----------------------------------------------------------------------------------------------------
//   //    Oxts:                 O(0)                 O(1)                  O(2)                  O(4)
//   // ----------------------------------------------------------------------------------------------------
//   // Scan(0):      start     middle     end
//   // Scan(1):                           start      middle     end
//   // Scan(2):                                                 start      middle     end
//   // ----------------------------------------------------------------------------------------------------
//   //  Camera:                 C(0)                 C(1)                  C(2)                  C(4)
//   //
//   // NOTE: because the Oxts data starts after the lidar scan we can't motion compensate Scan(0) (or the
//   // last one, Scan(n))

//   // represent the first three Oxts measurements - O(0), O(1), O(2) - here we have a roughly two meter forward motion
//   // from O(0)to O(2)
//   kmc::Oxts const odometry_0{kmc::Time(0.05), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_1{kmc::Time(0.15), 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_2{kmc::Time(0.25), 0.0, 0.00002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//   // represent Scan(1)
//   kmc::Pointcloud cloud_1 = Eigen::MatrixX4d(3, 4);
//   cloud_1.row(0) = Eigen::Vector4d{0.0, -5.0, 0.0, 1.0};
//   cloud_1.row(1) = Eigen::Vector4d{5.0, 0.0, 0.0, 1.0};
//   cloud_1.row(2) = Eigen::Vector4d{0.0, 5.0, 0.0, 1.0};
//   kmc::LidarScan const scan_1{kmc::Time(0.1), kmc::Time(0.15), kmc::Time(0.2),
//                               cloud_1,        kmc::VectorXd(), kmc::VectorXd()};

//   return MakeFrame(odometry_0, odometry_1, odometry_2, scan_1);
// }

// kmc::Pointcloud MotionCompensateFrame(Frame const &frame, kmc::Time const requested_time) {
//   kmc::Pointcloud const &cloud{frame.scan.cloud};
//   kmc::Time const scan_start{frame.scan.stamp_start};
//   kmc::Time const scan_end{frame.scan.stamp_end};
//   kmc::Twist const total_delta_pose{kmc::lie::Log(frame.T_start.inverse() * frame.T_end)};

//   // TODO(jack): do not copy and paste the time interpolation code
//   // double const x{(requested_time - scan_start) / (scan_end - scan_start)};
//   // Affine3d const pose_at_requested_time{InterpolatePose(frame.T_start, frame.T_end, x)};

//   kmc::Pointcloud motion_compensated_cloud = Eigen::MatrixX4d(cloud.rows(), 4);
//   for (Eigen::Index i{0}; i < cloud.rows(); ++i) {
//     Eigen::Vector4d point_i{cloud.row(i)};
//     point_i(3) = 1;  // TODO(jack): make homogeneous not so sketchily
//     kmc::Time const point_i_stamp{kmc::GetPseudoTimeStamp(point_i, scan_start, scan_end)};

//     double const x{(requested_time - point_i_stamp) / (scan_end - scan_start)};
//     kmc::Twist const delta_pose_i{x * total_delta_pose};

//     Affine3d const delta_tf{kmc::lie::Exp(delta_pose_i)};

//     Eigen::Vector4d const point_i_motion_compensated{delta_tf * point_i};

//     motion_compensated_cloud.row(i) = point_i_motion_compensated;
//     motion_compensated_cloud.row(i)(3) = cloud.row(i)(3);  // transfer intensity
//   }

//   return motion_compensated_cloud;
// }

// TEST(LieTimingTestXXXXXXXXX, XXX) {
//   Frame const test_frame{MakeMotionCompensationTestFrame2()};

//   kmc::Pointcloud const motion_compensated_cloud{MotionCompensateFrame(test_frame, test_frame.scan.stamp_middle)};

//   Eigen::Vector4d const point_1{motion_compensated_cloud.row(0)};
//   ASSERT_FLOAT_EQ(point_1(0), 0.27829874);
//   ASSERT_FLOAT_EQ(point_1(1), -5.0);
//   ASSERT_FLOAT_EQ(point_1(2), 0.0);
//   ASSERT_FLOAT_EQ(point_1(3), 1.0);

//   Eigen::Vector4d const point_2{motion_compensated_cloud.row(1)};
//   ASSERT_FLOAT_EQ(point_2(0), test_frame.scan.cloud.row(1)(0));
//   ASSERT_FLOAT_EQ(point_2(1), test_frame.scan.cloud.row(1)(1));
//   ASSERT_FLOAT_EQ(point_2(2), test_frame.scan.cloud.row(1)(2));
//   ASSERT_FLOAT_EQ(point_2(3), test_frame.scan.cloud.row(1)(3));

//   Eigen::Vector4d const point_3{motion_compensated_cloud.row(2)};
//   ASSERT_FLOAT_EQ(point_3(0), -0.27829874);
//   ASSERT_FLOAT_EQ(point_3(1), 5.0);
//   ASSERT_FLOAT_EQ(point_3(2), 0.0);
//   ASSERT_FLOAT_EQ(point_3(3), 1.0);
// }

// TEST(LieTimingTestXXXXXXXXXXXX, XXXXXX) {
//   kmc::Path const data_folder{"../data/2011_09_26/2011_09_26_drive_0091_sync/"};
//   size_t const frame_id{107};

//   kmc::Oxts const odometry_n_m_1{kmc::LoadOxts(data_folder, frame_id - 1)};
//   kmc::Oxts const odometry_n{kmc::LoadOxts(data_folder, frame_id)};
//   kmc::Oxts const odometry_n_p_1{kmc::LoadOxts(data_folder, frame_id + 1)};

//   kmc::LidarScan const lidar_scan{kmc::LoadLidarScan(data_folder, frame_id)};
//   kmc::Images const images{kmc::LoadImages(data_folder, frame_id)};

//   auto const test_frame{MakeFrame(odometry_n_m_1, odometry_n, odometry_n_p_1, lidar_scan, images)};
//   kmc::Pointcloud const motion_compensated_cloud{MotionCompensateFrame(test_frame, test_frame.scan.stamp_middle)};

//   // project
//   kmc::viz::CameraCalibrations const camera_calibrations{kmc::viz::LoadCameraCalibrations("../data/2011_09_26/")};
//   Eigen::Affine3d const lidar_extrinsics{kmc::LoadLidarExtrinsics("../data/2011_09_26/", true)};

//   kmc::Frame old_frame_format{odometry_n, lidar_scan, images};

//   kmc::Images const projected_imgs_raw{
//       kmc::viz::ProjectPointcloudOnFrame(old_frame_format, camera_calibrations, lidar_extrinsics)};
//   cv::imwrite("output_raw.png", projected_imgs_raw.image_03.image);

//   old_frame_format.scan_.cloud = motion_compensated_cloud;

//   kmc::Images const projected_imgs_mc{
//       kmc::viz::ProjectPointcloudOnFrame(old_frame_format, camera_calibrations, lidar_extrinsics)};
//   cv::imwrite("output_mc.png", projected_imgs_mc.image_03.image);
// }

// namespace XXX
// // TODO(jack): make test fixture
// Frame MakeMotionCompensationTestFrame2() {
//   // This is how the KITTI dataset is collected with respect to time, three scans shown for example:
//   //
//   //    Time: ----- 0ms ----- 5ms ----- 10ms ----- 15ms ----- 20ms ----- 25ms ----- 30ms ----- 35ms -----
//   // ----------------------------------------------------------------------------------------------------
//   //    Oxts:                 O(0)                 O(1)                  O(2)                  O(4)
//   // ----------------------------------------------------------------------------------------------------
//   // Scan(0):      start     middle     end
//   // Scan(1):                           start      middle     end
//   // Scan(2):                                                 start      middle     end
//   // ----------------------------------------------------------------------------------------------------
//   //  Camera:                 C(0)                 C(1)                  C(2)                  C(4)
//   //
//   // NOTE: because the Oxts data starts after the lidar scan we can't motion compensate Scan(0) (or the
//   // last one, Scan(n))

//   // represent the first three Oxts measurements - O(0), O(1), O(2) - here we have a roughly two meter forward motion
//   // from O(0)to O(2)
//   kmc::Oxts const odometry_0{kmc::Time(0.05), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_1{kmc::Time(0.15), 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_2{kmc::Time(0.25), 0.0, 0.00002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//   // represent Scan(1)
//   kmc::Pointcloud cloud_1 = Eigen::MatrixX4d(3, 4);
//   cloud_1.row(0) = Eigen::Vector4d{0.0, -5.0, 0.0, 1.0};
//   cloud_1.row(1) = Eigen::Vector4d{5.0, 0.0, 0.0, 1.0};
//   cloud_1.row(2) = Eigen::Vector4d{0.0, 5.0, 0.0, 1.0};
//   kmc::LidarScan const scan_1{kmc::Time(0.1), kmc::Time(0.15), kmc::Time(0.2),
//                               cloud_1,        kmc::VectorXd(), kmc::VectorXd()};

//   return MakeFrame(odometry_0, odometry_1, odometry_2, scan_1);
// }

// kmc::Pointcloud MotionCompensateFrame(Frame const &frame, kmc::Time const requested_time) {
//   kmc::Pointcloud const &cloud{frame.scan.cloud};
//   kmc::Time const scan_start{frame.scan.stamp_start};
//   kmc::Time const scan_end{frame.scan.stamp_end};
//   kmc::Twist const total_delta_pose{kmc::lie::Log(frame.T_start.inverse() * frame.T_end)};

//   // TODO(jack): do not copy and paste the time interpolation code
//   // double const x{(requested_time - scan_start) / (scan_end - scan_start)};
//   // Affine3d const pose_at_requested_time{InterpolatePose(frame.T_start, frame.T_end, x)};

//   kmc::Pointcloud motion_compensated_cloud = Eigen::MatrixX4d(cloud.rows(), 4);
//   for (Eigen::Index i{0}; i < cloud.rows(); ++i) {
//     Eigen::Vector4d point_i{cloud.row(i)};
//     point_i(3) = 1;  // TODO(jack): make homogeneous not so sketchily
//     kmc::Time const point_i_stamp{kmc::GetPseudoTimeStamp(point_i, scan_start, scan_end)};

// // TODO(jack): make test fixture
// Frame MakeMotionCompensationTestFrame2() {
//   // This is how the KITTI dataset is collected with respect to time, three scans shown for example:
//   //
//   //    Time: ----- 0ms ----- 5ms ----- 10ms ----- 15ms ----- 20ms ----- 25ms ----- 30ms ----- 35ms -----
//   // ----------------------------------------------------------------------------------------------------
//   //    Oxts:                 O(0)                 O(1)                  O(2)                  O(4)
//   // ----------------------------------------------------------------------------------------------------
//   // Scan(0):      start     middle     end
//   // Scan(1):                           start      middle     end
//   // Scan(2):                                                 start      middle     end
//   // ----------------------------------------------------------------------------------------------------
//   //  Camera:                 C(0)                 C(1)                  C(2)                  C(4)
//   //
//   // NOTE: because the Oxts data starts after the lidar scan we can't motion compensate Scan(0) (or the
//   // last one, Scan(n))

//   // represent the first three Oxts measurements - O(0), O(1), O(2) - here we have a roughly two meter forward motion
//   // from O(0)to O(2)
//   kmc::Oxts const odometry_0{kmc::Time(0.05), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_1{kmc::Time(0.15), 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   kmc::Oxts const odometry_2{kmc::Time(0.25), 0.0, 0.00002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//   // represent Scan(1)
//   kmc::Pointcloud cloud_1 = Eigen::MatrixX4d(3, 4);
//   cloud_1.row(0) = Eigen::Vector4d{0.0, -5.0, 0.0, 1.0};
//   cloud_1.row(1) = Eigen::Vector4d{5.0, 0.0, 0.0, 1.0};
//   cloud_1.row(2) = Eigen::Vector4d{0.0, 5.0, 0.0, 1.0};
//   kmc::LidarScan const scan_1{kmc::Time(0.1), kmc::Time(0.15), kmc::Time(0.2),
//                               cloud_1,        kmc::VectorXd(), kmc::VectorXd()};

//   return MakeFrame(odometry_0, odometry_1, odometry_2, scan_1);
// }

// kmc::Pointcloud MotionCompensateFrame(Frame const &frame, kmc::Time const requested_time) {
//   kmc::Pointcloud const &cloud{frame.scan.cloud};
//   kmc::Time const scan_start{frame.scan.stamp_start};
//   kmc::Time const scan_end{frame.scan.stamp_end};
//   kmc::Twist const total_delta_pose{kmc::lie::Log(frame.T_start.inverse() * frame.T_end)};

//   // TODO(jack): do not copy and paste the time interpolation code
//   // double const x{(requested_time - scan_start) / (scan_end - scan_start)};
//   // Affine3d const pose_at_requested_time{InterpolatePose(frame.T_start, frame.T_end, x)};

//   kmc::Pointcloud motion_compensated_cloud = Eigen::MatrixX4d(cloud.rows(), 4);
//   for (Eigen::Index i{0}; i < cloud.rows(); ++i) {
//     Eigen::Vector4d point_i{cloud.row(i)};
//     point_i(3) = 1;  // TODO(jack): make homogeneous not so sketchily
//     kmc::Time const point_i_stamp{kmc::GetPseudoTimeStamp(point_i, scan_start, scan_end)};

//     double const x{(requested_time - point_i_stamp) / (scan_end - scan_start)};
//     kmc::Twist const delta_pose_i{x * total_delta_pose};

//     Affine3d const delta_tf{kmc::lie::Exp(delta_pose_i)};

//     Eigen::Vector4d const point_i_motion_compensated{delta_tf * point_i};

//     motion_compensated_cloud.row(i) = point_i_motion_compensated;
//     motion_compensated_cloud.row(i)(3) = cloud.row(i)(3);  // transfer intensity
//   }

//   return motion_compensated_cloud;
// }
//     double const x{(requested_time - point_i_stamp) / (scan_end - scan_start)};
//     kmc::Twist const delta_pose_i{x * total_delta_pose};

//     Affine3d const delta_tf{kmc::lie::Exp(delta_pose_i)};

//     Eigen::Vector4d const point_i_motion_compensated{delta_tf * point_i};

//     motion_compensated_cloud.row(i) = point_i_motion_compensated;
//     motion_compensated_cloud.row(i)(3) = cloud.row(i)(3);  // transfer intensity
//   }

//   return motion_compensated_cloud;
// }