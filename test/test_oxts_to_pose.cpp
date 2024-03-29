#include <gtest/gtest.h>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"

using namespace kmc;

TEST(OxtsToPoseTest, LoadKnownPoseProperly) {
  Path const data_folder{"../testing_assets/2011_09_26/2011_09_26_drive_0005_sync"};
  size_t const frame_id{0};

  std::optional<Oxts> const oxts{LoadOxts(data_folder, frame_id)};
  ASSERT_TRUE(oxts.has_value());

  auto const pose{OxtsToPose(oxts.value(), 1.0)};

  ASSERT_FLOAT_EQ(pose.rotation().determinant(), 1.0);
  ASSERT_FLOAT_EQ(pose.translation().x(), 937631.25);
  ASSERT_FLOAT_EQ(pose.translation().y(), 6276764);
  ASSERT_FLOAT_EQ(pose.translation().z(), 112.83492);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
