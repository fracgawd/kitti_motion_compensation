#include "kitti_motion_compensation/camera_model.hpp"

namespace kmc::viz {

Image ProjectPointcloudOnImage(Eigen::MatrixX4d const &pointcloud_c00_rect, Image const &image,
                               CameraCalibration const camera_calibration, double const max_range) {
  // In this function we do the "Y = P_rect_xx * X_c00_rect" part. Now we have
  // pixels! Sorry about the transposes again :(
  Eigen::MatrixX3d pixels = (camera_calibration.P_rect * pointcloud_c00_rect.transpose()).transpose();

  // Make pixels homogeneous - this scales them onto the unit image plane
  pixels = pixels.array().colwise() / pixels.col(2).array();

  cv::Mat image_projection{image.image.clone()};
  for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
    double const distance_in_front_of_camera{pointcloud_c00_rect.row(i)(2)};
    // Only points in front of the camera are valid. All points in front of the
    // image but outside the image height-width are automaticall handled by
    // opencv - the z coordinate is the direction perpendicular to the camera
    if (distance_in_front_of_camera < 0.01 or distance_in_front_of_camera > max_range) {
      continue;
    }

    // colore points by their distance in front of the camera (z-axis)
    double const range_scale{distance_in_front_of_camera / (max_range - 0.01)};
    double const color_scale{255.0 * range_scale};

    cv::Point const point{cv::Point(pixels.row(i)(0), pixels.row(i)(1))};
    cv::circle(image_projection, point, 1, cv::Scalar(255 - color_scale, color_scale, 255 - color_scale), 1, 1, 0);
  }

  // TODO(jack): don't forget to copy over the timestamp
  return Image{image.stamp, image_projection};
}

Images ProjectPointcloudOnFrame(Frame const &frame, CameraCalibrations const camera_calibrations,
                                Eigen::Affine3d const tf_c00_lo) {
  // Taken directly from the dev kit docs:
  //
  // """
  // In order to transform a homogeneous point X = [x y z 1]' from the velodyne
  // coordinate system to a homogeneous point Y = [u v 1]' on image plane of
  // camera xx, the following transformation has to be applied:

  // Y = P_rect_xx * R_rect_00 * (R|T)_velo_to_cam * X
  // """
  //
  // In this function, we do the "R_rect_00 * (R|T)_velo_to_cam * X" part

  // lo = "lidar optical" frame - this is the velodyne measurement frame
  Pointcloud const &pointcloud_lo{frame.scan_.cloud};

  Eigen::Index const num_points{pointcloud_lo.rows()};

  // h = "homogeneous" - there is a 1 tacked onto the end of each point which
  // makes doing transforms with them easier :)
  //
  // Note that at this moment the pointcloud no longer has its intensity
  // information, therefore we stop using the Pointcloud typdef because that is
  // explciitly meant to signal a pointcloud with intensity values
  Eigen::MatrixX4d pointcloud_lo_h = Eigen::MatrixX4d::Ones(num_points, 4);
  pointcloud_lo_h.leftCols(3) = pointcloud_lo.leftCols(3);

  // Transform it into the camera_00 frame - all the other cameras are
  // calibrated relative to this camera - it is the anchor sensor for the
  // cameras.
  //
  // Please just ignore the transposes, what we are doing here is just
  // transforming a point in one frame to another.
  //
  //    p_c00 = tf_c00_lo * p_lo
  //
  Eigen::MatrixX4d const pointcloud_c00_h = (tf_c00_lo * pointcloud_lo_h.transpose()).transpose();

  // Now we enter "rectified magical land" by rotating by R_rect_00
  Eigen::Affine3d R_rect_00{Eigen::Affine3d::Identity()};
  R_rect_00 = camera_calibrations.camera_00.R_rect * R_rect_00;

  Eigen::MatrixX4d const pointcloud_c00_rect_h = (R_rect_00 * pointcloud_c00_h.transpose()).transpose();

  // Now project the pointcloud onto all four images using their respective
  // intrinsics
  Image const image_00_projection{
      ProjectPointcloudOnImage(pointcloud_c00_rect_h, frame.images_->image_00, camera_calibrations.camera_00)};
  Image const image_01_projection{
      ProjectPointcloudOnImage(pointcloud_c00_rect_h, frame.images_->image_01, camera_calibrations.camera_01)};
  Image const image_02_projection{
      ProjectPointcloudOnImage(pointcloud_c00_rect_h, frame.images_->image_02, camera_calibrations.camera_02)};
  Image const image_03_projection{
      ProjectPointcloudOnImage(pointcloud_c00_rect_h, frame.images_->image_03, camera_calibrations.camera_03)};

  return Images{image_00_projection, image_01_projection, image_02_projection, image_03_projection};
}

}  // namespace kmc::viz