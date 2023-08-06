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

TEST(DataIoTest, LoadPointCloudProperly) { ASSERT_TRUE(false); }

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