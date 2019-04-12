#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "Camera.h"

using namespace cv;
using namespace std;

vector<cv::Point2f> g_image_points;

Mat intrinsic(3, 3, CV_32F, Scalar::all(0));
Mat distCoeffs(5, 1, CV_32F, Scalar::all(0));

Camera camera;

double timestamp = 0.0;

void callback(const sensor_msgs::Image::ConstPtr& msg_cam)
{
    std::cout << msg_cam->header.stamp << std::endl;

    timestamp = msg_cam->header.stamp.toSec();
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_cam, sensor_msgs::image_encodings::BGR8);
	cv::Mat img = cv_ptr -> image;
    
    vector<cv::Point2f> test_cam;
    test_cam = camera.findChessboardCorners(img);

    std::vector<cv::Point3f> points;
    if(!test_cam.empty())
    {
        for(int i = 0; i < test_cam.size(); i++)
        {
            cout << test_cam[i].x << ',' << test_cam[i].y << endl;
        }
        g_image_points.insert(g_image_points.end(), test_cam.begin(), test_cam.end());
    }

    std::string fileName = "/home/fusion/project/perception/1/32_17/" + to_string(timestamp) + ".txt";
    std::cout << fileName << std::endl;
    ofstream ofile_image(fileName);
    //ofile.setPrecision(5);
    for(int i = 0; i < g_image_points.size(); i ++)
    {
        ofile_image << g_image_points[i].x << ',' << g_image_points[i].y << endl; 
    }
    // ofile_image.flush();
    ofile_image.close();
    g_image_points.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_cam");
    ros::NodeHandle n;

    cv::FileStorage file_read("/home/fusion/project/perception/src/cam_1.yaml", cv::FileStorage::READ);
    file_read["cameraMatrix"] >> intrinsic;
    file_read["distCoeffs"] >> distCoeffs;
    file_read.release();

    camera.init(intrinsic, distCoeffs);

    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 10, callback);

	while(true)
    {
        ros::spinOnce();
    }

	return 0;
}
