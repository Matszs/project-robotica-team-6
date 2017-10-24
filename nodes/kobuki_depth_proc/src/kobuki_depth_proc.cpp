#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <stdlib.h>
#include <cstdio>

using namespace std;
using namespace cv;

ros::Publisher image_new;



void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    /*sensor_msgs::Image img;
    img = *msg;
    //ROS_INFO("%s", img.encoding.c_str());
    Mat img_mat = Mat::zeros(480, 640, CV_32FC1);

    img_mat = img.data + Scalar(75, 75, 75);

    //img.data = img_mat;



    *//*for(int i=0; i < img.height; i++){
        for(int j =0; j < img.width; j++){

           ROS_INFO("img_mat.at(0,0); = [%f]", img_mat.at(0,0));
        }
    }*//*



    image_new.publish(img);*/


}

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_depth_proc node");

	ros::init(argc, argv, "kobuki_depth_proc");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("/camera/depth/image", 100, imageCallback);
	image_new = n.advertise<sensor_msgs::Image>("/depth_image_new", 100);


    ros::spin();


	ROS_INFO("Exiting the node");
	return 0;
}

