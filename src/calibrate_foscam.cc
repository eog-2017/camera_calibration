#include "calibrate.h"

#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <foscam_cam_ros/triplets.h>

/**
  * Change this parameter to acheive better results.
  */

#define KEY "KBB1V–A386–EA49–2294–A708"
#define TAKE_IMAGE
#define NUM_IMAGES 50

#ifdef TAKE_IMAGE
#define FPS 3
#else
#define FPS 60
#endif

CalibrateFoscam::CalibrateFoscam(float calibSquareDimension, cv::Size chessboardDimension, cv::Size left_img, cv::Size right_img) {
    this->calib_square_dimension = calibSquareDimension;
    this->chessboard_dimension = chessboardDimension;
    this->left_img = left_img;
    this->right_img = right_img;

    this->camera_matrix_left = cv::Mat::eye(3, 3, CV_64F);
    this->dist_coeff_right = cv::Mat::zeros(8, 1, CV_64F);

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
CalibrateFoscam::~CalibrateFoscam(){
    this->marker_corners_left.clear();
    this->marker_corners_right.clear();
    this->world_marker_corners.clear();
}

void CalibrateFoscam::getIntrinsics(bool is_left) {

    std::cout<<"Calculating intrinsics"<<std::endl;

    if(is_left){
        double rms = cv::calibrateCamera(this->world_marker_corners, this->marker_corners_left,
                                         this->left_img, this->camera_matrix_left, this->dist_coeff_left,
                                         this->rvecs_left, this->tvecs_left);
        std::cout<<"RMS error for left Camera : " << rms << " mm" <<std::endl;

    } else {
        double rms = cv::calibrateCamera(this->world_marker_corners, this->marker_corners_right,
                                         this->right_img, this->camera_matrix_right, this->dist_coeff_right,
                                         this->rvecs_right, this->tvecs_right);
        std::cout<<"RMS error for right Camera : " << rms << " mm" <<std::endl;
    }

}

/*
void CalibrateFoscam::getStereoTx() {
    std::cout<<"Calculating Stero transformation parameters"<<std::endl;
    double rms;
    rms = cv::stereoCalibrate(this->world_marker_corners, this->marker_corners_left_comb, this->marker_corners_left_comb,
                              this->camera_matrix_left, this->dist_coeff_ir, this->camera_matrix_hd, this->dist_coeff_hd, this->ir_img,
                              this->R, this->t, this->E, this->F);
}*/

void CalibrateFoscam::saveIntrinsics(bool is_left) {
    cv::FileStorage fs("calib_intrinsic.yml", cv::FileStorage::APPEND);

    if(is_left) {

        fs << "left_camera_matrix" << this->camera_matrix_left << "left_dist_coeff" << this->dist_coeff_left;
        fs << "left_rvecs" << this->rvecs_left << "left_tvecs" << this->tvecs_left;
    } else {

        fs << "right_camera_matrix" << this->camera_matrix_right << "right_dist_coeff" << this->dist_coeff_right;
        fs << "right_rvecs" << this->rvecs_right << "right_tvecs" << this->tvecs_right;
    }

    fs.release();
}

void CalibrateFoscam::readIntrinsics(bool is_left) {
    cv::FileStorage fs("calib_intrinsic.yml", cv::FileStorage::READ);

    if(is_left) {

        fs ["left_camera_matrix"] >> this->camera_matrix_left;
        fs ["left_dist_coeff"] >> this->dist_coeff_left;
    } else {

        fs ["right_camera_matrix"] >> this->camera_matrix_right;
        fs ["right_dist_coeff"] >> this->dist_coeff_right;
    }

    fs.release();
}

void CalibrateFoscam::saveStereo() {
    cv::FileStorage fs("calib_stereo.yml", cv::FileStorage::WRITE);

    fs << "R" << this->R << "t" << this->t;
    fs << "E" << this->E << "F" << this->F;

    fs.release();
}

void CalibrateFoscam::getChessboardCorners(bool is_left) {
    if(is_left) {
        for(int i = 0; i < NUM_IMAGES; i++) {
            std::stringstream f_name;
            f_name << "left_cam_image_" << i << ".png";
            cv::Mat image = cv::imread(f_name.str());
            this->getChessboardCorners(image, is_left, false, true);
        }
    } else {
        for(int i = 0; i < NUM_IMAGES; i++) {
            std::stringstream f_name;
            f_name << "right_cam_image_" << i << ".png";
            cv::Mat image = cv::imread(f_name.str());
            this->getChessboardCorners(image, is_left, false, true);
        }
    }
}


double CalibrateFoscam::rmsError(bool is_left) {

    double rms = 0.f;

    if(is_left) {
        for (int i = 0; i < NUM_IMAGES; i++) {
            std::vector<cv::Point2f> image_points;
            std::cout<<"In project points...\n";
            cv::projectPoints(this->world_marker_corners[i],
                              this->rvecs_left[i],
                              this->tvecs_left[i],
                              this->camera_matrix_left,
                              this->dist_coeff_left,
                              image_points);

            double error;
            std::vector<cv::Point2f> orig_points = this->marker_corners_left[i];
            for(int j = 0; j < image_points.size(); j++) {
                error += cv::sqrt(cv::pow(orig_points[j].x - image_points[j].x, 2) + cv::pow(orig_points[j].y - image_points[j].y, 2));
            }
            error /= image_points.size();
            rms += error;
        }
    } else {
        for (int i = 0; i < NUM_IMAGES; i++) {
            std::vector<cv::Point2f> image_points;
            cv::projectPoints(this->world_marker_corners[i],
                              this->rvecs_right[i],
                              this->tvecs_right[i],
                              this->camera_matrix_right,
                              this->dist_coeff_right,
                              image_points);
            double error;
            std::vector<cv::Point2f> orig_points = this->marker_corners_left[i];
            for(int j = 0; j < image_points.size(); j++) {
                error += cv::sqrt(cv::pow(orig_points[j].x - image_points[j].x, 2) + cv::pow(orig_points[j].y - image_points[j].y, 2));
            }
            error /= image_points.size();
            rms += error;
        }
    }

    return rms;
}

int CalibrateFoscam::getChessboardCorners(cv::Mat& image, bool is_left, bool save_images, bool dry_run) {

    std::vector<cv::Point2f> pointBuf;
    bool found = cv::findChessboardCorners(image, this->chessboard_dimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::Mat draw_image, gray_image;
        image.copyTo(draw_image);
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);

        cv::cornerSubPix(gray_image, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        std::string count;

        if (is_left) count = std::to_string(this->marker_corners_left.size());
        else count = std::to_string(this->marker_corners_right.size());

        cv::putText(draw_image, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);
        cv::drawChessboardCorners(draw_image, this->chessboard_dimension, pointBuf, found);
        if(is_left)
            cv::imshow("Checkerboard_Left", draw_image);
        else
            cv::imshow("Checkerboard_Right", draw_image);
        int key = cv::waitKey(1000/FPS) & 255;
        // key = 110;

        if(key == 110 || dry_run) {
            if (is_left) {
                this->marker_corners_left.push_back(pointBuf);
                if(save_images) {
                    std::stringstream out_file;
                    out_file << "left_cam_image_" << count << ".png";
                    cv::imwrite(out_file.str(), image);
                }
            }
            else {
                this->marker_corners_right.push_back(pointBuf);
                if(save_images) {
                    std::stringstream out_file;
                    out_file << "right_cam_image_" << count << ".png";
                    cv::imwrite(out_file.str(), image);
                }
            }
        }
    } else {
        std::string count;

        if (is_left) count = std::to_string(this->marker_corners_left.size());
        else count = std::to_string(this->marker_corners_right.size());

        cv::putText(image, count, cv::Point2i(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.f, cv::Scalar(255,255,255), 3);
        if(is_left)
            cv::imshow("Checkerboard_Left", image);
        else
            cv::imshow("Checkerboard_Right", image);
        cv::waitKey(1000/FPS);
    }

    if (is_left) return this->marker_corners_left.size();
    else return this->marker_corners_right.size();
}

int CalibrateFoscam::getChessboardCorners(cv::Mat& image_1, cv::Mat& image_2, bool save_images) {

    std::vector<cv::Point2f> pointBuf_1, pointBuf_2;
    bool found_1 = cv::findChessboardCorners(image_1, this->chessboard_dimension, pointBuf_1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    bool found_2 = cv::findChessboardCorners(image_2, this->chessboard_dimension, pointBuf_2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (found_1 && found_2) {
        cv::Mat draw_image_1, draw_image_2;
        image_1.copyTo(draw_image_1);
        image_2.copyTo(draw_image_2);

        std::string count;

        count = std::to_string(this->marker_corners_left_comb.size());

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
            this->marker_corners_left_comb.push_back(pointBuf_1);
            this->marker_corners_right_comb.push_back(pointBuf_2);

            if(save_images) {
                std::stringstream out_file_1;
                out_file_1 << "left_cam_image_comb_" << count << ".png";
                cv::imwrite(out_file_1.str(), image_1);

                std::stringstream out_file_2;
                out_file_2 << "right_cam_image_comb_" << count << ".png";
                cv::imwrite(out_file_2.str(), image_2);
            }
        }
    } else {
        std::string count;

        count = std::to_string(this->marker_corners_left.size());

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

cv::Mat CalibrateFoscam::undistort(bool is_left, cv::Mat image){
    cv::Mat ud_image;
    if(is_left) {
        cv::Rect valid_roi;
        cv::Mat opt_mat = cv::getOptimalNewCameraMatrix(this->camera_matrix_left,
                                                             this->dist_coeff_left,
                                                             cv::Size(image.cols, image.rows),
                                                             1.f,
                                                             cv::Size(image.cols, image.rows),
                                                             &valid_roi);

        cv::undistort(image, ud_image, this->camera_matrix_left, this->dist_coeff_left);
        //ud_image = ud_image(valid_roi);

    } else {
        cv::Rect valid_roi;
        cv::Mat opt_mat = cv::getOptimalNewCameraMatrix(this->camera_matrix_right,
                                                             this->dist_coeff_right,
                                                             cv::Size(image.cols, image.rows),
                                                             1.f,
                                                             cv::Size(image.cols, image.rows),
                                                             &valid_roi);

        cv::undistort(image, ud_image, this->camera_matrix_right, this->dist_coeff_right);
        //ud_image = ud_image(*valid_roi);
    }
    return ud_image;
}

cv::Mat left_img;
cv::Mat right_img;
bool run = true;

void sighandler(int signum) {
    printf("Program Interrupted...\n");
    run = false;
    cv::destroyAllWindows();
    ros::shutdown();
    exit(1);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle nh;

    ros::ServiceClient foscam_client = nh.serviceClient<foscam_cam_ros::triplets>("fetch_images");

    signal(SIGINT, sighandler);

    ros::AsyncSpinner async_spin(4);
    async_spin.start();

    cv::Size chessboard_dimension = cv::Size(4, 7);
    cv::Size left_img_size = cv::Size(1920, 1080);
    cv::Size right_img_size = cv::Size(1920, 1080);
    CalibrateFoscam calib(0.10f, chessboard_dimension, left_img_size, right_img_size);

#ifdef TAKE_IMAGE
    int i = 0;
    while(i < NUM_IMAGES) {
        foscam_cam_ros::triplets srv;
        srv.request.key.data = KEY;
        if(foscam_client.call(srv)) {
            cv::Mat left_img;
            cv_bridge::CvImagePtr left_img_ptr = cv_bridge::toCvCopy(srv.response.left_rgb_img);

            left_img = left_img_ptr->image;
            i = calib.getChessboardCorners(left_img, true, true, false);
        }
    }
    cv::destroyAllWindows();
#else
    calib.getChessboardCorners(true);
#endif
    calib.getIntrinsics(true);
    calib.saveIntrinsics(true);
/*
#ifdef TAKE_IMAGE
    i = 0;
    while(i < NUM_IMAGES) {
        foscam_cam_ros::triplets srv;
        srv.request.key.data = KEY;
        if(foscam_client.call(srv)) {
            cv::Mat right_img;
            cv_bridge::CvImagePtr right_img_ptr = cv_bridge::toCvCopy(srv.response.right_rgb_img);

            right_img = right_img_ptr->image;
            i = calib.getChessboardCorners(right_img, false, true, false);
        }
    }
    cv::destroyAllWindows();
#else
    calib.getChessboardCorners(false);
#endif
    calib.getIntrinsics(false);
    calib.saveIntrinsics(false);*/

/*    calib.readIntrinsics(true);
    calib.readIntrinsics(false);*/

    while(run) {
        foscam_cam_ros::triplets srv;
        srv.request.key.data = KEY;
        if(foscam_client.call(srv)) {
            cv::Mat left_img, und_left_img, right_img, und_right_img;
            cv_bridge::CvImagePtr left_img_ptr = cv_bridge::toCvCopy(srv.response.left_rgb_img);
            //cv_bridge::CvImagePtr right_img_ptr = cv_bridge::toCvCopy(srv.response.right_rgb_img);

            left_img = left_img_ptr->image;
            //right_img = right_img_ptr->image;

            und_left_img = calib.undistort(true, left_img);
            //und_right_img = calib.undistort(false, right_img);

            //cv::imshow("undistorted_right_img", und_right_img);
            //cv::imshow("right_img", right_img);
            cv::imshow("undistorted_left_img", und_left_img);
            cv::imshow("left_img", left_img);
            cv::waitKey(1);
        }
    }

}
