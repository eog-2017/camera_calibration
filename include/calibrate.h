#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class CalibrateKinect {
public:
    CalibrateKinect(float calib_square_dimension, cv::Size chessboard_dimension, cv::Size ir_img, cv::Size hd_img);
    ~CalibrateKinect();
    void getIntrinsics(bool is_hd);
    void getStereoTx();
    int getChessboardCorners(cv::Mat& image, bool is_hd, bool save_images);
    int getChessboardCorners(cv::Mat& image_1, cv::Mat& image_2, bool save_images);
    void getChessboardCorners();
    double projectPoints();
    void saveIntrinsics();
    void saveStereo();
private:
    float calib_square_dimension;
    cv::Size chessboard_dimension;
    cv::Size ir_img;
    cv::Size hd_img;
    std::vector<std::vector<cv::Point2f> > marker_corners_hd;
    std::vector<std::vector<cv::Point2f> > marker_corners_ir;
    std::vector<std::vector<cv::Point3f> > world_marker_corners;

    std::vector<std::vector<cv::Point2f> > marker_corners_hd_comb;
    std::vector<std::vector<cv::Point2f> > marker_corners_ir_comb;

    cv::Mat camera_matrix_ir;
    cv::Mat dist_coeff_ir;
    std::vector<cv::Mat> rvecs_ir;
    std::vector<cv::Mat> tvecs_ir;

    cv::Mat camera_matrix_hd;
    cv::Mat dist_coeff_hd;
    std::vector<cv::Mat> rvecs_hd;
    std::vector<cv::Mat> tvecs_hd;


    cv::Mat R;
    cv::Mat t;
    cv::Mat E;
    cv::Mat F;
};

class CalibrateFoscam {
public:
    CalibrateFoscam(float calib_square_dimension, cv::Size chessboard_dimension, cv::Size left_img, cv::Size right_img);
    ~CalibrateFoscam();
    void getIntrinsics(bool is_left);
    void getStereoTx();
    int getChessboardCorners(cv::Mat& image, bool is_left, bool save_images);
    int getChessboardCorners(cv::Mat& image_1, cv::Mat& image_2, bool save_images);
    void getChessboardCorners();
    //double projectPoints();
    void saveIntrinsics();
    void saveStereo();
private:
    float calib_square_dimension;
    cv::Size chessboard_dimension;
    cv::Size left_img;
    cv::Size right_img;
    std::vector<std::vector<cv::Point2f> > marker_corners_left;
    std::vector<std::vector<cv::Point2f> > marker_corners_right;
    std::vector<std::vector<cv::Point3f> > world_marker_corners;

    std::vector<std::vector<cv::Point2f> > marker_corners_left_comb;
    std::vector<std::vector<cv::Point2f> > marker_corners_right_comb;

    cv::Mat camera_matrix_left;
    cv::Mat dist_coeff_left;
    std::vector<cv::Mat> rvecs_left;
    std::vector<cv::Mat> tvecs_left;

    cv::Mat camera_matrix_right;
    cv::Mat dist_coeff_right;
    std::vector<cv::Mat> rvecs_right;
    std::vector<cv::Mat> tvecs_right;

    cv::Mat R;
    cv::Mat t;
    cv::Mat E;
    cv::Mat F;
};
