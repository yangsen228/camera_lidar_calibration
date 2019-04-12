#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
//tmp#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/time_synchronizer.h>

#include "Camera.h"
#include "detect_points.h"

using namespace cv;
using namespace message_filters;
using namespace std;

#define cam_index 2
#define lidar_index 1

// length unit: mm
// angle unit: degree
const float chess_board_length = 1150;
const float chess_board_width = 900;
const float chess_board_size = 123.0;
const float chess_board_distance = 8000.0;
const float lidar_angle_left = -60.0;
const float lidar_angle_right = 60.0;

int g_ind;
vector<cv::Point3f> g_lidar_points;
vector<cv::Point2f> g_image_points;

Mat intrinsic(3, 3, CV_32F, Scalar::all(0));
Mat distCoeffs(5, 1, CV_32F, Scalar::all(0));
Mat rvec(3, 1, CV_32F, Scalar::all(0));
Mat tvec(3, 1, CV_32F, Scalar::all(0));

vector<cv::Point3f> center_points;

Camera camera;
DetectPoints detect_points(lidar_angle_left, lidar_angle_right, 0.0, chess_board_distance, chess_board_size);

void callback(const sensor_msgs::Image::ConstPtr& msg_cam, const sensor_msgs::PointCloud2::ConstPtr& msg_lidar)
{
    //cout << msg_cam->header.stamp << endl;
    //cout << msg_lidar->header.stamp << endl;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_cam, sensor_msgs::image_encodings::BGR8);
	cv::Mat img = cv_ptr -> image;
    //imwrite("/home/fusion/project/perception/"+to_string(idx)+".png", img);
    //cv::imwrite("")
    vector<cv::Point2f> test_cam;
    test_cam = camera.findChessboardCorners(img);

    //pcl::io::savePCDFile("/home/fusion/project/perception/"+to_string(idx)+".pcd", laser);
    pcl::PointCloud<pcl::PointXYZI> lidar;
	pcl::fromROSMsg(*msg_lidar, lidar);
    detect_points.setPointCloud(lidar.makeShared());
    detect_points.setLength(chess_board_length);
    detect_points.setWidth(chess_board_width);
    std::vector<cv::Point3f> test_lidar;

    Point3f center_point;
    if(detect_points.getFeaturePoints(test_lidar) && !test_cam.empty()) {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        for(int i = 0; i < test_lidar.size(); i++) {
            x += test_lidar[i].x;
            y += test_lidar[i].y;
            z += test_lidar[i].z;
        }
        center_point.x = x / 8;
        center_point.y = y / 8;
        center_point.z = z / 8;
        
        bool append_flag = true;
        double dist = 0.0;

        if(center_points.empty()) {
            center_points.push_back(center_point);
            g_ind++;

            g_lidar_points.insert(g_lidar_points.end(), test_lidar.begin(), test_lidar.end());
            g_image_points.insert(g_image_points.end(), test_cam.begin(), test_cam.end());

            // cout << "11111 center_points.size(): " << center_points.size() << endl;

        }
        else {
            // cout << "22222 center_points.size(): " << center_points.size() << endl;
            for(int i = 0; i < center_points.size(); i++)
            {
                dist = cv::norm(Mat(center_point), Mat(center_points[i]), NORM_L2);
                if(dist < 1000.0) {
                    append_flag = false;
                    break;
                }
            }
            if(append_flag){
                g_ind++;
                cout << "dist: " << dist << endl;

                center_points.push_back(center_point);

                g_lidar_points.insert(g_lidar_points.end(), test_lidar.begin(), test_lidar.end());
                g_image_points.insert(g_image_points.end(), test_cam.begin(), test_cam.end());
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_cam");
    ros::NodeHandle n;

    cv::FileStorage file_read("/home/fusion/project/calibration/src/cam_"+to_string(cam_index)+".yaml", cv::FileStorage::READ);
    file_read["cameraMatrix"] >> intrinsic;
    file_read["distCoeffs"] >> distCoeffs;
    file_read.release();

    camera.init(intrinsic, distCoeffs);

    // time synchronization
    message_filters::Subscriber<sensor_msgs::Image> sub_1(n, "/usb_cam/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(n, "/velodyne_points_200", 1);
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), sub_1, sub_2);
	sync.registerCallback(boost::bind(&callback, _1, _2));

    g_ind = 0;
	while(g_ind < 4) {
        ros::spinOnce();
    }

    cv::solvePnP(g_lidar_points, g_image_points, intrinsic, distCoeffs, rvec, tvec, SOLVEPNP_EPNP);

    cv::FileStorage file_write("/home/fusion/project/calibration/src/lidar_"+to_string(lidar_index)+"_to_cam_"+to_string(cam_index)+".yaml", cv::FileStorage::WRITE);
    file_write << "rotation vector" << rvec;
    file_write << "translation vector" << tvec;
    file_write.release(); 

/*
    ofstream ofile_image("/home/fusion/project/perception/src/camera.txt");
    ofstream ofile_lidar("/home/fusion/project/perception/src/lidar.txt");
    //ofile.setPrecision(5);
    for(int i = 0; i < g_lidar_points.size(); i ++)
    {
        ofile_lidar << g_lidar_points[i].x << ',' << g_lidar_points[i].y << ',' << g_lidar_points[i].z << endl;
    }
    ofile_lidar.close();

    for(int i = 0; i < g_image_points.size(); i ++)
    {
        ofile_image << g_image_points[i].x << ',' << g_image_points[i].y << endl; 
    }
    ofile_image.close();
*/
    //cv::FileStorage file_write_2("/home/fusion/project/perception/src/lidar_1_to_cam_1_record.txt", cv::FileStorage::WRITE);
    //file_write_2 << "g_lidar_points" << g_lidar_points;
    //file_write_2 << "g_image_points" << g_image_points;
    //file_write_2.release(); 
    
	return 0;
}
