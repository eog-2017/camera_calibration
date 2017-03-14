#include <vector>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>

class Calibrate {
public:
  Calibrate(float calibSquareDimension, cv::Size chessboardDimension);
  void getIntrinsics();
  void getDistortionCoeffs();
  void getChessboardCorners(cv::Mat image, bool is_hd);
private:
  const float calibSquareDimension;
  const cv::Size chessboardDimension;
  std::vector<std::vector<cv::Point2f> > marker_corners_hd;
  std::vector<std::vector<cv::Point2f> > marker_corners_ir;


  /*std::vector<std::vector<int> > p_hd;
  std::vector<std::vector<int> > p_ir;
  Eigen::Matrix3d intr_hd;
  Eigen::Matrix3d intr_ir;
  std::vector<float> dist_coeff_hd;
  std::vector<float> dist_coeff_ir;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;*/
};
