#ifndef CAMERA_CALIBRATION_HPP_
#define CAMERA_CALIBRATION_HPP_

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <Eigen/Eigen>

const string IMAGE_WINDOW = "Image Window";

class CalibrationNode
{
private:
	enum return_values
	{
		pose_robot_stored 			= 0,
		pose_robot_not_stored 		= 1,
		checkerboard_found 			= 2,
		checkerboard_not_found 		= 3,
		object_points_stored 		= 4,
		camera_calibrated			= 5,
		camera_not_calibrated		= 6
	};

	ros::NodeHandle ROSNode;

	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber calibrationImageSubscriber;

	ros::Subscriber robotPoseSubscriber;
	ros::Subscriber cameraInfoSubscriber;

	cv::Size image_size;
	cv::Size pattern;
	double patternWidth, patternHeight;

	cv::Mat image;
	cv::Mat imagePoints, objectPoints;
	cv::Mat cameraMatrix, distortionCoefficients;

	cv::vector<geometry_msgs::Pose> robotPoseVector;
	geometry_msgs::Pose robotPose;

	std::vector<Eigen::Matrix3f> rotationRB_vec;
	std::vector<Eigen::Vector3f> translationRB_vec;
	std::vector<Eigen::Matrix3f> rotationCB_vec;
	std::vector<Eigen::Vector3f> translationCB_vec;

	Eigen::Matrix3f rotationRB;
	Eigen::Vector3f translationRB;
	Eigen::Matrix3f rotationCB;
	Eigen::Vector3f translationCB;

	Eigen::Matrix3f rotationCalib;
	Eigen::Vector3f translationCalib;

	bool readPoseFlag;

public:
	CalibrationNode ();

	CalibrationNode (ros::NodeHandle&);

	~CalibrationNode ();

	void imgCallback (const sensor_msgs::ImageConstPtr&);
	void cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr&);
	void poseCallback (const geometry_msgs::PoseConstPtr&);

	static void mouseCallback (int, int, int, int, void*);

	int storeData ();
	bool generateData ();
	void performEstimation();

	//math
	Eigen::Vector3f getLogTheta(Eigen::Matrix3f);

};

#endif /* CAMERA_CALIBRATION_HPP_ */
