#include "calibrate.h"

Calibrate::Calibrate(float calibSquareDimension, cv::Size chessboardDimension) {
  this.calibSquareDimension = calibSquareDimension;
  this.chessboardDimension = chessboardDimension;
}

void Calibrate::getIntrinsics() {

}

int Calibrate::getChessboardCorners(cv::Mat image, bool is_hd) {
  std::vector<cv::Point2f> pointBuf;
  bool found = cv::findChessboardCorners(image, this.chessboardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESHOLD | CV_CALIB_CB_NORMALIZE_IMAGE);
  if (found) {
    if (is_hd) this.marker_corners_hd.push_back(pointBuf);
    else this.marker_corners_ir.push_back(pointBuf);
  }
  cv:Mat draw_image;
  image.copyTo(draw_image);

  cv::drawChessboardCorners(draw_image, this.chessboardDimension, pointBuf, found);
  cv::imshow("Checkerboard", draw_image);
  cv::waitKey(1);

  if (is_hd) return this.marker_corners_hd.size();
  else return this.marker_corners_ir.size();
}

int main(int argc, char** argv) {

  /**
   * Take images first from the HD camera and get the parameters
   */

  cv::Size chessboardDimension = Size(argv[2], argv[3]);

  Calibrate calib = new Calibrate(argv(1)), chessboardDimension);

  while(true) {
    cv::Mat image = cv::Zeros(100, 100); //Read from the camera
    calib.getChessboardCorners(image, true); // appends the vector
  }

}
