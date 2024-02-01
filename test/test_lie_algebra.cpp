#include <gtest/gtest.h>

#include "kitti_motion_compensation/lie_algebra.hpp"

TEST(LieAlgebraTest, HatAndVeeInverses) {
  Eigen::Vector3d const phi_in{0.1, 0.2, 0.3};
  Eigen::Vector3d const phi_out{kmc::lie::Vee(kmc::lie::Hat(phi_in))};

  ASSERT_FLOAT_EQ(phi_in(0), phi_out(0));
  ASSERT_FLOAT_EQ(phi_in(1), phi_out(1));
  ASSERT_FLOAT_EQ(phi_in(2), phi_out(2));
}

TEST(LieAlgebraTest, So3LogAndExpInverse) {
  Eigen::Vector3d const phi_in{0.1, 0.2, 0.3};
  Eigen::Vector3d const phi_out{kmc::lie::Log(kmc::lie::Exp(phi_in))};

  ASSERT_FLOAT_EQ(phi_in(0), phi_out(0));
  ASSERT_FLOAT_EQ(phi_in(1), phi_out(1));
  ASSERT_FLOAT_EQ(phi_in(2), phi_out(2));
}

TEST(LieAlgebraTest, So3LeftJacobiansInverse) {
  Eigen::Vector3d const phi_in{0.1, 0.2, 0.3};

  Eigen::Matrix3d const left_jacobain{kmc::lie::LeftJacobian(phi_in)};
  Eigen::Matrix3d const inverse_left_jacobain{kmc::lie::InverseLeftJacobian(phi_in)};

  // TODO(jack): add helper function for this for the testing
  // check that (value*value_inv) is an identity matrix
  auto const I{left_jacobain * inverse_left_jacobain};
  ASSERT_FLOAT_EQ(I.trace(), 3.0);
  ASSERT_FLOAT_EQ(I.sum() - I.trace(), 0.0);
}

TEST(LieAlgebraTest, Se3LogAndExpInverse) {
  kmc::Twist xi_in;
  xi_in << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  kmc::Twist const xi_out{kmc::lie::Log(kmc::lie::Exp(xi_in))};

  ASSERT_FLOAT_EQ(xi_in(0), xi_out(0));
  ASSERT_FLOAT_EQ(xi_in(1), xi_out(1));
  ASSERT_FLOAT_EQ(xi_in(2), xi_out(2));
  ASSERT_FLOAT_EQ(xi_in(3), xi_out(3));
  ASSERT_FLOAT_EQ(xi_in(4), xi_out(4));
  ASSERT_FLOAT_EQ(xi_in(5), xi_out(5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
