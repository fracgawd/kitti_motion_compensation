#pragma once
namespace kmc::utilities_for_testing {

bool FloatEqual(float const a, float const b, float const epsilon = 1e-10) { return fabs(a - b) <= epsilon; }

bool TransformationMatricesAreTheSame(Affine3d const& tf1, Affine3d const& tf2) {
  // check that (tf1*tf2_inv) is an identity matrix
  auto const I{tf1 * tf2.inverse()};

  return FloatEqual(I.matrix().trace(), 4.0) and FloatEqual((I.matrix().sum() - I.matrix().trace()), 0.0);
}

}  // namespace kmc::utilities_for_testing