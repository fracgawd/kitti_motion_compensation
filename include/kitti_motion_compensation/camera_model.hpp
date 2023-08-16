#include "kitti_motion_compensation/data_types.hpp"

namespace kmc::viz {

Image ProjectPointcloudOnImage(Eigen::MatrixX4d const &pointcloud_c00_rect, Image const &image,
                               CameraCalibration const camera_calibration, double const max_range = 20.0);

Images ProjectPointcloudOnFrame(Frame const &frame, CameraCalibrations const camera_calibrations,
                                Eigen::Affine3d tf_c00_lo);

}  // namespace kmc::viz