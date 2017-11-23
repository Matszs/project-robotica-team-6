#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <signal.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <kobuki_mapper/GridPoint.h>
#include <kobuki_depth/CameraPoint.h>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cstdio>

using namespace cv;
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

class Depth {

private:
	ros::NodeHandle nh;

	message_filters::Subscriber<Image> rgb_image_sub;
    message_filters::Subscriber<PointCloud2> pcl_sub;
    typedef sync_policies::ApproximateTime<Image, PointCloud2> sync_policy;
    Synchronizer<sync_policy> sync;
    ros::Publisher camera_publisher;

	Mat img_lines_color;

public:
	Depth() : rgb_image_sub(nh, "/camera/rgb/image_rect_color", 1), pcl_sub(nh, "/camera/depth_registered/points", 1), sync(sync_policy(10), rgb_image_sub, pcl_sub) {
		ROS_INFO("Kobuki_depth constructor");

		img_lines_color = Mat::zeros(480, 640, CV_8UC3);
        camera_publisher = nh.advertise<kobuki_depth::CameraPoint>("/camera_points", 10);
	    sync.registerCallback(boost::bind(&Depth::cameraCallback, this, _1, _2));
	}

	~Depth() {
		ROS_INFO("Kobuki_depth destructor");
	}

	/*void odomCallback(const nav_msgs::OdometryConstPtr& msg) {

        float positionX = msg->pose.pose.position.x;
        float positionY = msg->pose.pose.position.y;
        float rotation = msg->pose.pose.orientation.z;

        positionXGrid = (int)(positionX * 10);
        positionYGrid = (int)(positionY * 10);

        currentRotation = rotation;

        // get 2 decimals from the current rotation
        degrees = (float)((int)(currentRotation * 100)) / 100;
        // format rotation into degrees
        degrees = (degrees > 0 ? 360 - (360 * degrees / 2) : 360 * abs(degrees) / 2);


        //ROS_INFO_STREAM("Degrees: " << degrees);

    }*/

	/**
	 * Callback that is executed if there is a camera image and point cloud data available. This data is used to
	 * find the x, y, z coordinates of the object that the node is tracking.
	 *
	 * @param msg Image from the camera
	 * @param pcl Point cloud data from the camera
	 */
	void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pcl) {
		static int loopCount = 0;
		loopCount++;
		if (loopCount % 50 == 0) {
			img_lines_color = Mat::zeros(480, 640, CV_8UC3);
		}

		// Convert image for use with OpenCV
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		Mat img_HSV;
		cvtColor(cv_ptr->image, img_HSV, CV_BGR2HSV);

		Size s = img_HSV.size();

		imwrite( "/home/kobuki6/workspace/abc.jpg", img_HSV );

		pcl::PointCloud<pcl::PointXYZ> pc;
		pcl::fromROSMsg(*pcl, pc);


		pcl::PointXYZ p = pc.at(s.width / 2, s.height / 2);

		kobuki_depth::CameraPoint cameraPoint;
        cameraPoint.x = p.x;
        cameraPoint.y = p.y;
        cameraPoint.z = p.z;

        camera_publisher.publish(cameraPoint);

        ROS_INFO_STREAM("Location depth: " << p.z);



		/*int i;
		int steps = 50;
		for(i = 0; i < steps; i++) {

		    int x = s.width / steps * i;

            pcl::PointXYZ p = pc.at(x, s.height / 2);

            //ROS_INFO_STREAM("Location depth " << i << ": " << p.z);

            //if (!isnan(p.z)) {
                if(i == 25) {
                    //if(degrees > 340 || degrees < 20) {

                        // add current location as a wall to the grid.
                        *//*kobuki_mapper::GridPoint gridPoint;
                        gridPoint.x = positionXGrid + (p.z * 10);
                        gridPoint.y = positionYGrid;
                        gridPoint.type = 0;
                        grid_publisher.publish(gridPoint);*//*



                        kobuki_depth::CameraPoint cameraPoint;
                        cameraPoint.x = p.x;
                        cameraPoint.y = p.y;
                        cameraPoint.z = p.z;

                        camera_publisher.publish(cameraPoint);

                        ROS_INFO_STREAM("Location depth " << i << ": " << p.z);

                    //}
                }
           //}





            //ROS_INFO_STREAM("Location depth " << i << ": " << p.z);

        }*/

        /*if (!isnan(p.z)) {
            ROS_INFO_STREAM("><>");
        }*/
	}
};

int main(int argc, char **argv) {
	ROS_INFO("Starting the node");

	ros::init(argc, argv, "kobuki_depth");
	ROS_INFO("ros::init done");

	Depth depth;

	ros::Rate rate(30);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("Exiting the node");
	return 0;
}

