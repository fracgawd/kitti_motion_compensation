#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"

TEST(DataIoTest, LoadOdometryProperly) {
  kmc::Path const data_folder{
      "../assets/2011_09_26/2011_09_26_drive_0005_sync"};
  size_t const frame_id{0};

  kmc::Frame const frame{kmc::LoadSingleFrame(data_folder, frame_id)};

  kmc::Oxts const odometry{frame.odometry_};
  ASSERT_EQ(odometry.stamp, 47072.349659964);
  ASSERT_EQ(odometry.vf, 3.5147680214713);
  ASSERT_EQ(odometry.vl, 0.037625160413037);
  ASSERT_EQ(odometry.vu, -0.03878884255623);
}

TEST(DataIoTest, LoadPointCloudProperly) {
  kmc::Path const data_folder{
      "../assets/2011_09_26/2011_09_26_drive_0005_sync"};
  size_t const frame_id{0};

  kmc::Frame const frame{kmc::LoadSingleFrame(data_folder, frame_id)};

  // the lidar scans at 10Hz, which is clearly visible here:
  //    end-start = 0.10327s
  // and the camera is triggered half way through the scan when the sweep is
  // facing towards the front:
  //    (middle-start)/0.10327 = 0.5
  kmc::LidarScan const lidar_scan{frame.scan_};
  ASSERT_EQ(lidar_scan.stamp_start, 47072.283701593);
  ASSERT_EQ(lidar_scan.stamp_middle, 47072.335337762);
  ASSERT_EQ(lidar_scan.stamp_end, 47072.386973931);

  // check the length of the scan and the values of the first and last points
  ASSERT_EQ(lidar_scan.cloud.rows(), 123397);
  auto const point_1{lidar_scan.cloud.row(0)};
  ASSERT_FLOAT_EQ(point_1(0), 22.7189998626709);
  ASSERT_FLOAT_EQ(point_1(1), 0.0309999994933605);
  ASSERT_FLOAT_EQ(point_1(2), 0.976999998092651);
  ASSERT_FLOAT_EQ(point_1(3), 0.319999992847443);
  auto const point_n{lidar_scan.cloud.row(123396)};
  ASSERT_FLOAT_EQ(point_n(0), 5.63399982452393);
  ASSERT_FLOAT_EQ(point_n(1), -1.39499998092651);
  ASSERT_FLOAT_EQ(point_n(2), -2.58999991416931);
  ASSERT_FLOAT_EQ(point_n(3), 0.0);
}

TEST(DataIoTest, LoadImagesOptional) {
  kmc::Path const data_folder{
      "../assets/2011_09_26/2011_09_26_drive_0005_sync"};
  size_t const frame_id{0};
  bool load_images{false};

  // load and test a frame without images
  kmc::Frame const frame_without_images{
      kmc::LoadSingleFrame(data_folder, frame_id, load_images)};

  ASSERT_FALSE(frame_without_images.images_.has_value());

  // load and test a frame with images
  load_images = true;
  kmc::Frame const frame_with_images{
      kmc::LoadSingleFrame(data_folder, frame_id, load_images)};

  ASSERT_TRUE(frame_without_images.images_.has_value());
}

TEST(DataIoTest, LoadImagesProperly) { ASSERT_TRUE(false); }

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}