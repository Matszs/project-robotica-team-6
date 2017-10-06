#include <ros/ros.h>

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

#include <vision/TrackedPosition.h>
#include <vision/SetTrackingColours.h>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cstdio>

using namespace cv;
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

static const char *WINDOW_CONTROL = "CONTROL WINDOW";
static const int fps = 30;
// Set this to true for debugging purposes. Will display 2 images that give more information
// about the object that is being tracked.
static const bool debug = false;
// Allows you to set the minimum and maximum HSV values using a GUI if set to true
static const bool show_control_window = true;

class Tracker {

private:
	ros::NodeHandle nh;

	message_filters::Subscriber<Image> rgb_image_sub;
	message_filters::Subscriber<PointCloud2> pcl_sub;
    typedef sync_policies::ApproximateTime<Image, PointCloud2> sync_policy;
	Synchronizer<sync_policy> sync;

	ros::Subscriber set_colour_sub;

	ros::Publisher tracked_pos_pub;

	// Customizable
	int low_h, high_h, low_s, high_s, low_v, high_v;

	// System vars
	int last_x = -1;
	int last_y = -1;
	int pos_x = -1;
	int pos_y = -1;

	Mat img_lines_color = Mat::zeros(480, 640, CV_8UC3);

public:
	Tracker() : rgb_image_sub(nh, "/camera/rgb/image_rect_color", 1), pcl_sub(nh, "/camera/depth_registered/points", 1), sync(sync_policy(10), rgb_image_sub, pcl_sub) {
		ROS_INFO("Tracker constructor");

		nh.param("/vision/settings/hue_low", low_h, 0);
		nh.param("/vision/settings/hue_high", high_h, 12);
		nh.param("/vision/settings/sat_low", low_s, 75);
		nh.param("/vision/settings/sat_high", high_s, 255);
		nh.param("/vision/settings/val_low", low_v, 137);
		nh.param("/vision/settings/val_high", high_v, 255);

	    sync.registerCallback(boost::bind(&Tracker::cameraCallback, this, _1, _2));

		set_colour_sub = nh.subscribe("/vision/set_tracking_colours", 1, &Tracker::setTrackingColoursCallback, this);
		tracked_pos_pub = nh.advertise<vision::TrackedPosition>("/vision/tracked_position", 10);

		if (show_control_window) {
			namedWindow(WINDOW_CONTROL, WINDOW_AUTOSIZE);

			// hue
			cvCreateTrackbar("low_h", WINDOW_CONTROL, &low_h, 179);
			cvCreateTrackbar("high_h", WINDOW_CONTROL, &high_h, 179);
			// saturation
			cvCreateTrackbar("low_s",WINDOW_CONTROL, &low_s, 255);
			cvCreateTrackbar("high_s", WINDOW_CONTROL, &high_s, 255);
			// value
			cvCreateTrackbar("low_v", WINDOW_CONTROL, &low_v, 255);
			cvCreateTrackbar("high_v", WINDOW_CONTROL, &high_v, 255);

			cvWaitKey(1);
		}
	}

	~Tracker() {
		ROS_INFO("Tracker destructor");
		if (show_control_window) {
			destroyWindow(WINDOW_CONTROL);
		}
	}

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
		Mat img_thresholded;

		if (high_h > 180) {
			Mat img_thresholded1;
			Mat img_thresholded2;
			inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(179, high_s, high_v), img_thresholded1);
			inRange(img_HSV, Scalar(0, low_s, low_h), Scalar(high_h - 179, high_s, high_v), img_thresholded2);
			img_thresholded = img_thresholded1 | img_thresholded2;
		} else {
			inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), img_thresholded);
		}

		// morphological opening (remove small objects from the foreground)
		erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// morphological closing (fill small holes in the foreground)
		dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// Track object
		Moments oMoments = moments(img_thresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// Ignore noise
		if (dArea > 9000) {
			// calc pos
			pos_x = dM10 / dArea;
			pos_y = dM01 / dArea;

			if (last_x >= 0 && last_y >= 0 && pos_x >= 0 && pos_y >= 0 && debug) {
				// Line from prev point to new point
				line(img_lines_color, Point(pos_x, pos_y), Point(last_x, last_y), Scalar(0,0,255), 2);
			}

			last_x = pos_x;
			last_y = pos_y;
		} else {
			pos_x = -1;
			pos_y = -1;
		}

		if (debug) {
			Mat img_toShow = img_lines_color + cv_ptr->image;
			imshow("Original image", img_toShow);
			imshow("Thresholded image", img_thresholded);

			cvWaitKey(1);
		}

		pcl::PointCloud<pcl::PointXYZ> pc;
		pcl::fromROSMsg(*pcl, pc);

		if (pos_x >= 0 && pos_y >= 0) {
			pcl::PointXYZ p = pc.at(pos_x, pos_y);

			if (!isnan(p.z)) {
				vision::TrackedPosition pos;
				pos.x = p.x;
				pos.y = p.y;
				pos.z = p.z;

				tracked_pos_pub.publish(pos);

				ROS_INFO_STREAM("[CAMERA PUBLISHED] xyz: " << p.x << "|" << p.y << "|" << p.z);
			}
		}
	}

	/**
	 * Sets the minimum and maximum HSV values of the object that must be tracked.
	 *
	 * @param msg The minimum and maximum HSV values to use
	 */
	void setTrackingColoursCallback(const vision::SetTrackingColoursConstPtr& msg) {
		low_h  = msg->hue_low;
		high_h = msg->hue_high;
		low_s  = msg->sat_low;
		high_s = msg->sat_high;
		low_v  = msg->val_low;
		high_v = msg->val_high;
		ROS_INFO("Set tracking values to: {\n   hue: (%u, %u)\n   sat: (%u, %u)\n   val: (%u, %u)\n}\n",\
				msg->hue_low, msg->hue_high, msg->sat_low, msg->sat_high, msg->val_low, msg->val_high);

		pos_x = -1;
		pos_y = -1;
	}
};

int main(int argc, char **argv) {
	ROS_INFO("Starting the node");

	ros::init(argc, argv, "vision");
	ROS_INFO("ros::init done");

	Tracker tracker;

	ros::Rate rate(fps);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("Exiting the node");
	return 0;
}

