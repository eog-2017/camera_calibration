#include "calibrate.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

/**
  * Change this parameter to acheive better results.
  */

#define NUM_IMAGES 50
#define FPS 5

CalibrateKinect::CalibrateKinect(float calibSquareDimension, cv::Size chessboardDimension, cv::Size ir_img, cv::Size hd_img) {
    this->calib_square_dimension = calibSquareDimension;
    this->chessboard_dimension = chessboardDimension;
    this->ir_img = ir_img;
    this->hd_img = hd_img;

    this->camera_matrix_ir = cv::Mat::eye(3, 3, CV_64F);
    this->dist_coeff_ir = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Point3f> points;

    for (int i = 0; i < chessboardDimension.width; i++) {
        for (int j = 0; j < chessboardDimension.height; j++) {
            cv::Point3f point(i*calibSquareDimension, j*calibSquareDimension, 0.f);
            points.push_back(point);
        }
    }

    world_marker_corners.resize(NUM_IMAGES, points);
    assert(world_marker_corners.size() == NUM_IMAGES);

}
CalibrateKinect::~CalibrateKinect(){
    this->marker_corners_hd.clear();
    this->marker_corners_ir.clear();
    this->world_marker_corners.clear();
}

void CalibrateKinect::getIntrinsics(bool is_hd) {

    std::cout<<"Calculating intrinsics"<<std::endl;

    if(is_hd){
        double rms = cv::calibrateCamera(this->world_marker_corners, this->marker_corners_hd,
                                         this->hd_img, this->camera_matrix_hd, this->dist_coeff_hd,
                                         this->rvecs_ir, this->tvecs_ir);
        std::cout<<"RMS error for HD Camera : " << rms <<std::endl;

    } else {
        double rms = cv::calibrateCamera(this->world_marker_corners, this->marker_corners_ir,
                                         this->ir_img, this->camera_matrix_ir, this->dist_coeff_ir,
                                         this->rvecs_hd, this->tvecs_hd);
        std::cout<<"RMS error for IR Camera : " << rms <<std::endl;
    }

}

void CalibrateKinect::getStereoTx() {
    std::cout<<"Calculating Stero transformation parameters"<<std::endl;
    double rms;
    rms = cv::stereoCalibrate(this->world_marker_corners, this->marker_corners_ir_comb, this->marker_corners_ir_comb,
                              this->camera_matrix_ir, this->dist_coeff_ir, this->camera_matrix_hd, this->dist_coeff_hd, this->ir_img,
                              this->R, this->t, this->E, this->F);
}

void CalibrateKinect::saveIntrinsics() {
    cv::FileStorage fs("calib_intrinsic.yml", cv::FileStorage::WRITE);

    fs << "hd_camera_matrix" << this->camera_matrix_hd << "hd_dist_coeff" << this->dist_coeff_hd;
    fs << "hd_rvecs" << this->rvecs_hd << "hd_tvecs" << this->tvecs_hd;
}

void CalibrateKinect::saveStereo() {
    cv::FileStorage fs("calib_stereo.yml", cv::FileStorage::WRITE);

    fs << "R" << this->R << "t" << this->t;
    fs << "E" << this->E << "F" << this->F;
}

void CalibrateKinect::getChessboardCorners() {
    for(int i = 0; i < NUM_IMAGES; i++) {
        std::stringstream f_name;
        f_name << "hd_cam_image_" << i << ".png";
        cv::Mat image = cv::imread(f_name.str());
        this->getChessboardCorners(image, true, false);
    }
}

double CalibrateKinect::projectPoints() {
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(this->world_marker_corners[0], this->rvecs_hd[0], this->tvecs_hd[0], this->camera_matrix_hd, this->dist_coeff_hd, image_points);
    double rms = 0;
    std::cout<<"In Project points"<<std::endl;
    /*for(int i = 0; i < image_points.size(); i++) {
        rms += (image_points[i].x - this->marker_corners_hd[0][i].x) * (image_points[i].x - this->marker_corners_hd[0][i].x) +
                (image_points[i].y - this->marker_corners_hd[0][i].y) * (image_points[i].y - this->marker_corners_hd[0][i].y);
    }*/
    for(int i = 0; i < image_points.size(); i++) {
        std::cout<<image_points[i]<<std::endl;
    }

    return rms;
}

int CalibrateKinect::getChessboardCorners(cv::Mat& image, bool is_hd, bool save_images) {

    std::vector<cv::Point2f> pointBuf;
    bool found = cv::findChessboardCorners(image, this->chessboard_dimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::Mat draw_image;
        image.copyTo(draw_image);

        std::string count;

        if (is_hd) count = std::to_string(this->marker_corners_hd.size());
        else count = std::to_string(this->marker_corners_ir.size());

        cv::putText(draw_image, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);
        cv::drawChessboardCorners(draw_image, this->chessboard_dimension, pointBuf, found);
        cv::imshow("Checkerboard", draw_image);
        int key = cv::waitKey(1000/FPS) & 255;
        key = 110;

        if(key == 110) {
            if (is_hd) {
                this->marker_corners_hd.push_back(pointBuf);
                if(save_images) {
                    std::stringstream out_file;
                    out_file << "hd_cam_image_" << count << ".png";
                    cv::imwrite(out_file.str(), image);
                }
            }
            else {
                this->marker_corners_ir.push_back(pointBuf);
                if(save_images) {
                    std::stringstream out_file;
                    out_file << "ir_cam_image_" << count << ".png";
                    cv::imwrite(out_file.str(), image);
                }
            }
        }
    } else {
        std::string count;

        if (is_hd) count = std::to_string(this->marker_corners_hd.size());
        else count = std::to_string(this->marker_corners_ir.size());

        cv::putText(image, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);
        cv::imshow("Checkerboard", image);
        cv::waitKey(1000/FPS);
    }

    if (is_hd) return this->marker_corners_hd.size();
    else return this->marker_corners_ir.size();
}

int CalibrateKinect::getChessboardCorners(cv::Mat& image_1, cv::Mat& image_2, bool save_images) {

    std::vector<cv::Point2f> pointBuf_1, pointBuf_2;
    bool found_1 = cv::findChessboardCorners(image_1, this->chessboard_dimension, pointBuf_1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    bool found_2 = cv::findChessboardCorners(image_2, this->chessboard_dimension, pointBuf_2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (found_1 && found_2) {
        cv::Mat draw_image_1, draw_image_2;
        image_1.copyTo(draw_image_1);
        image_2.copyTo(draw_image_2);

        std::string count;

        count = std::to_string(this->marker_corners_hd_comb.size());

        cv::putText(draw_image_1, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);
        cv::drawChessboardCorners(draw_image_1, this->chessboard_dimension, pointBuf_1, found_1);
        cv::drawChessboardCorners(draw_image_2, this->chessboard_dimension, pointBuf_2, found_2);

        cv::Mat mat_array[2];
        cv::Mat frame;

        mat_array[0] = draw_image_1;
        mat_array[1] = draw_image_2;

        cv::hconcat(mat_array, 2, frame);

        cv::imshow("Checkerboard_Twin", frame);
        int key = cv::waitKey(1000/FPS) & 255;

        if(key == 110) {
            this->marker_corners_ir_comb.push_back(pointBuf_1);
            this->marker_corners_hd_comb.push_back(pointBuf_2);

            if(save_images) {
                std::stringstream out_file_1;
                out_file_1 << "ir_cam_image_comb_" << count << ".png";
                cv::imwrite(out_file_1.str(), image_1);

                std::stringstream out_file_2;
                out_file_2 << "hd_cam_image_comb_" << count << ".png";
                cv::imwrite(out_file_2.str(), image_2);
            }
        }
    } else {
        std::string count;

        count = std::to_string(this->marker_corners_hd.size());

        cv::putText(image_1, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);

        cv::Mat mat_array[2];
        cv::Mat frame;

        mat_array[0] = image_1;
        mat_array[1] = image_2;

        cv::hconcat(mat_array, 2, frame);

        cv::imshow("Checkerboard_Twin", frame);
        cv::waitKey(1000/FPS);

    }

}

cv::Mat raw_img;
cv::Mat input_ir_img;
bool dont_copy = false;

void camera_func(const sensor_msgs::Image::ConstPtr& msg){
    /**
     * Image is of type 16U with single channel
     */
    raw_img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO16)->image;

    double minVal, maxVal;
    cv::minMaxLoc(raw_img, &minVal, &maxVal);

    cv::Mat view_img;
    raw_img.convertTo(view_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    if (!dont_copy) view_img.copyTo(input_ir_img);

}

int main(int argc, char** argv) {

    /*ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle nh;
    ros::Subscriber kinect_img_sub = nh.subscribe("/camera/ir/image_rect_ir", 1, camera_func);

    ros::AsyncSpinner async_spin(4);
    async_spin.start();*/

    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle nh;
    ros::Subscriber kinect_img_sub = nh.subscribe("/camera/ir/image_rect_ir", 1, camera_func);

    ros::AsyncSpinner async_spin(4);
    async_spin.start();

    cv::Size chessboard_dimension = cv::Size(5, 7);
    cv::Size ir_img = cv::Size(640, 480);
    cv::Size hd_img = cv::Size(640, 480);
    CalibrateKinect calib(0.03, chessboard_dimension, ir_img, hd_img);

    cv::namedWindow("Checkerboard");

    int samples = 0;
    /*while(samples < NUM_IMAGES)
        if(input_ir_img.data) {
            dont_copy = true;
            samples = calib.getChessboardCorners(input_ir_img, false, true);
            dont_copy = false;
        }*/

    cv::VideoCapture cap(0);
    samples = 0;
    /*while(samples < NUM_IMAGES) {
        cv::Mat input_hd_img;
        cap >> input_hd_img;
        if(input_hd_img.data) samples = calib.getChessboardCorners(input_hd_img, true, true);
    }*/

    calib.getChessboardCorners();
    cv::destroyWindow("Checkerboard");
    calib.getIntrinsics(true);
    calib.saveIntrinsics();

    std::cout << "RMS error : " << calib.projectPoints() << std::endl;

    /*cv::namedWindow("Checkerboard_Twin");

    samples = 0;
    while (samples < NUM_IMAGES) {
        if(input_ir_img.data && !dont_copy) {
            dont_copy = true;
            cv::Mat input_hd_img;
            cap >> input_hd_img;
            if(input_hd_img.data) samples = calib.getChessboardCorners(input_ir_img, input_hd_img, true);
            dont_copy = false;
        }
    }
    calib.getStereoTx();
    calib.saveStereo();
    cv::destroyWindow("Checkerboard_Twin");*/
}
