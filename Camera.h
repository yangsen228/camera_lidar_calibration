#ifndef _CAMERA_H
#define _CAMERA_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define CORNERS_WIDTH   6;
#define CORNERS_HEIGHT  8;

#define IMAGE_WIDTH 1280;
#define IMAGE_HEIGHT 720;

class Camera {
public:
    Camera() {
    }

    void init(Mat& cameraMatrix, Mat& distCoeffs) {
        cameraMatrix_ = cameraMatrix.clone();
        distCoeffs_ = distCoeffs.clone();

        patSize_.width = CORNERS_WIDTH;
        patSize_.height = CORNERS_HEIGHT;

        imageSize_.width = IMAGE_WIDTH;
        imageSize_.height = IMAGE_HEIGHT;

        initUndistortRectifyMap(cameraMatrix_, 
                                distCoeffs_, 
                                Mat(), 
                                Mat(), 
                                imageSize_,
                                CV_16SC2, 
                                map1_, 
                                map2_);
    }

    vector<Point2f> findChessboardCorners(Mat image){
        Mat undistort_frame = image.clone();

        // remap(image, undistort_frame, map1_, map2_, INTER_LINEAR);
        undistort(image, undistort_frame, cameraMatrix_, distCoeffs_);

        imwrite("/home/fusion/project/perception/1.jpg", undistort_frame);        

        vector<Point2f> corners;
        bool ret = cv::findChessboardCorners(undistort_frame, patSize_, corners);

        if(!corners_.empty()){
            cv::drawChessboardCorners(undistort_frame, patSize_, cv::Mat(corners), ret);
        }

        // imshow("image", undistort_frame);
        // waitKey(1);

        corners_.clear();
        if(ret) {
            corners_.push_back(corners[20]);
            corners_.push_back(corners[21]);
            corners_.push_back(corners[26]);
            corners_.push_back(corners[27]);
            corners_.push_back(corners[13]);
            corners_.push_back(corners[16]);
            corners_.push_back(corners[31]);
            corners_.push_back(corners[34]);
        }
        
        return corners_;
    }

private:
    Mat cameraMatrix_;
    Mat distCoeffs_;
    Mat map1_;
    Mat map2_;
    vector<Point2f> corners_;
    Size imageSize_;
    Size patSize_;
};

#endif // _CAMERA_H