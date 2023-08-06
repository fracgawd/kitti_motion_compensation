#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"

TEST(DataIoTest, LoadImagesOptional) {
  kmc::Path const data_folder{"assets/2011_09_26/2011_09_26_drive_0005_sync"};
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}