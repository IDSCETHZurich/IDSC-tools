/***************************************************************************

    File:           handEyeCalibration.cpp
    Author(s):      Gajamohan Mohanarajah/Ferrara Francesco
    Affiliation:    IDSC - ETH Zurich
    e-mail:         gajan@ethz.ch


 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "handEyeCalibration.hpp"
#define SIGLE_MEASUREMENT_DEBUG 0
#define ESTIMATION_DEBUG 1

using namespace Eigen;

CalibrationNode::CalibrationNode(ros::NodeHandle& n):
	ROSNode (n),
	imageTransport (n)
{

	int squares_per_column, squares_per_row;
	ROSNode.getParam ("handEyeCalibration/squares_per_column", squares_per_column);
	ROSNode.getParam ("handEyeCalibration/squares_per_row", squares_per_row);
	pattern = cv::Size_<int> (squares_per_column, squares_per_row);

	ROSNode.getParam ("handEyeCalibration/square_size", squareSize);

	calibrationImageSubscriber = imageTransport.subscribe("/camera/image_rect", 1, &CalibrationNode::imgCallback, this);
	robotPoseSubscriber = ROSNode.subscribe ("/msrCartPos", 1, &CalibrationNode::poseCallback, this);
	cameraInfoSubscriber =ROSNode.subscribe ("/camera/camera_info", 1, &CalibrationNode::cameraInfoCallback, this);

	cv::namedWindow (IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback (IMAGE_WINDOW, &CalibrationNode::mouseCallback, this);

	readPoseFlag = false;
}

CalibrationNode::~CalibrationNode ()
{
	cv::destroyWindow (IMAGE_WINDOW);
}

void CalibrationNode::imgCallback (const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr img_ptr;
	try
	{
		img_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR ("cv_bridge exception: %s", e.what ());
	}

	image = img_ptr->image;
	cv::imshow (IMAGE_WINDOW, image);
	cv::waitKey (10);
}

void CalibrationNode::poseCallback (const geometry_msgs::PoseConstPtr& msg)
{

	robotPose = *msg;

}

void CalibrationNode::mouseCallback (int event, int x, int y, int flags, void* calibrationNode)
{

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		(static_cast<CalibrationNode*> (calibrationNode))->storeData ();
		break;

	case CV_EVENT_RBUTTONDOWN:
		std::cout << "Performing Estimation..." << std::endl;
		(static_cast<CalibrationNode*> (calibrationNode))->performEstimation();
		break;
	}


}



void CalibrationNode::cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr& msg){
	std::cout << msg->distortion_model << std::endl;

	cameraMatrix = cv::Mat(3, 3, CV_64F);// Intrinsic camera matrix for the raw (distorted) images.
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			cameraMatrix.at<double>(i,j) = msg->K[i*3+j];
	std::cout << "cameraMatrix" << std::endl << cameraMatrix << std::endl;


	distortionCoefficients = cv::Mat(5, 1, CV_64F);
	for(int i=0; i<5; i++)
		distortionCoefficients.at<double>(i,1) = 0.0;

	std::cout << "distortionCoefficients" << std::endl << distortionCoefficients << std::endl;
	std::cout << "/camera/camera_info successfully read out!" << std::endl;

	cameraInfoSubscriber.shutdown();
}


int CalibrationNode::storeData ()
{
	cv::vector<cv::Point2f> corners;
	bool patternWasFound = cv::findChessboardCorners (image, pattern, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (patternWasFound)
	{
		readPoseFlag = true;
		do{
			ros::Duration(0.2).sleep();
			ROS_INFO("Waiting to read ROBOT pose ...");
		}while(readPoseFlag == false);

		cv::Mat gray_image;

		gray_image.create (image.size(), CV_8UC1);
		cv::cvtColor (image, gray_image, CV_BGR2GRAY, 0);

		cv::cornerSubPix (gray_image, corners, cv::Size (10, 10), cv::Size (-1, -1),
				cv::TermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners (image, pattern, corners, patternWasFound);
		cv::imshow (IMAGE_WINDOW, image);

		//fill in imagePoints and objectPoints
		imagePoints = cv::Mat(corners.size(), 2, CV_32F);
		objectPoints = cv::Mat(corners.size(), 3, CV_32F);

		for(int i=0; i < (int)corners.size(); i++){
			imagePoints.at<float>(i,0) = corners[i].x;
			imagePoints.at<float>(i,1) = corners[i].y;
		}

#if SIGLE_MEASUREMENT_DEBUG
		std::cout << "imagePoints" << std::endl << imagePoints << std::endl;
#endif

		//row by row, left to right in every row
		for(int i=0; i < pattern.height; i++){
			for(int j=0; j < pattern.width; j++){
				objectPoints.at<float>(i*pattern.width+j, 0) = j*squareSize;
				objectPoints.at<float>(i*pattern.width+j, 1) = i*squareSize;
				objectPoints.at<float>(i*pattern.width+j, 2) = 0.0;
			}
		}

#if SIGLE_MEASUREMENT_DEBUG
		std::cout << "objectPoints" << std::endl << objectPoints << std::endl;
#endif

		cv::Mat rvecs, tvecs;

		cv::solvePnP (objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvecs, tvecs);

#if ESTIMATION_DEBUG
		std::cout << "rvecs" << std::endl << rvecs << std::endl;
		std::cout << "tvecs" << std::endl << tvecs << std::endl;
#endif

		cv::Mat rotMat = cv::Mat(3, 3, CV_64F);
		cv::Rodrigues(rvecs, rotMat);

		for(int i=0; i < 3; i++)
			for(int j=0; j < 3; j++)
				rotationCB(i,j) = rotMat.at<double>(i,j);

		translationCB = Vector3f(tvecs.at<double>(0,0), tvecs.at<double>(0,1), tvecs.at<double>(0,2));

		rotationRB = Quaternionf(robotPose.orientation.w, robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z);
		translationRB = Vector3f(robotPose.position.x, robotPose.position.y, robotPose.position.z);

		//pushing back data into vectors
		rotationRB_vec.push_back(rotationRB);
		translationRB_vec.push_back(translationRB);
		rotationCB_vec.push_back(rotationCB);
		translationCB_vec.push_back(translationCB);

		std::cout << "Checkerboard found. Measurements Updated." << std::endl;

#if ESTIMATION_DEBUG
		std::cout << "%Adding data #" << rotationRB_vec.size() << std::endl;
		std::cout << "rotRB" << rotationRB_vec.size() << " = [ " << rotationRB << " ]; " <<std::endl;
		std::cout << "transRB" << rotationRB_vec.size() << " = [ " << translationRB << " ]; " << std::endl;
		std::cout << "rotCB" << rotationRB_vec.size() << " = [ " << rotationCB << " ]; " << std::endl;
		std::cout << "transCB" << rotationRB_vec.size() << " = [ " << translationCB << " ]; " << std::endl;
#endif

		int c = cv::waitKey ();

		if ('q'==(char)c){
            std::cout << "key 'q' pressed: pop_back called" << std::endl;
			rotationRB_vec.pop_back();
			translationRB_vec.pop_back();
			rotationCB_vec.pop_back();
			translationCB_vec.pop_back();
		}


 		return checkerboard_found;
	}
	else
	{
		std::cout << "No Checkerboard has been found !" << std::endl;
        std::cout << cv::waitKey ();

		return checkerboard_not_found;
	}
}

void CalibrationNode::performEstimation(){
	if(rotationRB_vec.size() < 5 ){
		std::cout << "Insufficient data" << std::endl;
		return;
	}

	//perform least squares estimation
	Matrix3f M;
	Matrix4f rbi, rbj, cbi, cbj;
	Matrix4f A, B;

	Matrix3f I;
	I.setIdentity();

	MatrixXf C(0,3), bA(0,1), bB(0,1);

	Vector3f ai, bi;

	VectorXf V_tmp;
	MatrixXf C_tmp;

	M.setZero();

	for(int i=0; i < (int)rotationRB_vec.size(); i++){
		for(int j=0; j < (int)rotationRB_vec.size(); j++){
			if(i!=j){
				rbi << rotationRB_vec[i] , translationRB_vec[i] ,  0, 0, 0, 1;
				rbj << rotationRB_vec[j] , translationRB_vec[j] ,  0, 0, 0, 1;
				A = rbj.inverse()*rbi;

				cbi << rotationCB_vec[i] , translationCB_vec[i] ,  0, 0, 0, 1;
				cbj << rotationCB_vec[j] , translationCB_vec[j] ,  0, 0, 0, 1;
				B = cbj*cbi.inverse();

				ai = getLogTheta(A.block(0,0,3,3));
				bi = getLogTheta(B.block(0,0,3,3));

				M += bi*ai.transpose();

				MatrixXf C_tmp = C;
				C.resize(C.rows()+3, NoChange);
				C << C_tmp,  Matrix3f::Identity() - A.block(0,0,3,3);

				V_tmp = bA;
				bA.resize(bA.rows()+3, NoChange);
				bA << V_tmp, A.block(0,3,3,1);

				V_tmp = bB;
				bB.resize(bB.rows()+3, NoChange);
				bB << V_tmp, B.block(0,3,3,1);

			}//end of if i!=j
		}
	}//end of for(.. i < rotationRB_vec.size(); ..)

#if ESTIMATION_DEBUG
	std::cout << "M = [ " << M << " ]; " << std::endl;
#endif

	EigenSolver<Matrix3f> es(M.transpose()*M);
	Matrix3cf D = es.eigenvalues().asDiagonal();
	Matrix3cf V = es.eigenvectors();

	Matrix3cf Lambda = D.inverse().array().sqrt();
	Matrix3cf Theta_X = V * Lambda * V.inverse() * M.transpose();
	std::cout << "Orientation of Camera Frame with respect to Robot tool-tip frame." << std::endl;
	std::cout << "Theta_X = [ " << Theta_X.real()  << " ]; " << std::endl;

	//Estimating translational offset
	for(int i=0; i < bB.rows()/3; i++){
		bB.block(i*3,0,3,1) = Theta_X.real()*bB.block(i*3,0,3,1);
	}
	bA = bA - bB; // this is d. saving memory

	std::cout << "Translation of Camera Frame with respect to Robot tool-tip frame." << std::endl;
    std::cout << "bX = [ " << (C.transpose()*C).inverse() * C.transpose() * bA << " ]; " << std::endl;

}

bool CalibrationNode::generateData(){
	int numberOfSamples = 20;
	Matrix3f offset_R, checkBoard_R;
	Vector3f offset_T, checkBoard_T;

	offset_R << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	offset_T << 0.17, 0.02, 0.03;

	checkBoard_R = Matrix3f::Identity();
	checkBoard_T << 1.0, 1.0, 1.0;

	Matrix3f tmp_m3f;
	Vector3f tmp_v3f;
	Matrix4f A, B, C, D; // ABC=D

	B << offset_R, offset_T, 0.0, 0.0, 0.0, 1.0;
	D << checkBoard_R, checkBoard_T, 0.0, 0.0, 0.0, 1.0;


	for(int i=1; i <= numberOfSamples; i++){
		tmp_v3f = Vector3f::Random().normalized();
		tmp_m3f = AngleAxisf((double(rand())/RAND_MAX), Vector3f::Random().normalized());

        std::cout << "rotRB" << i << " = [ " << tmp_m3f << " ]; " << std::endl;
        std::cout << "transRB" << i << " = [ " << tmp_v3f << " ]; " << std::endl;

		A << tmp_m3f, tmp_v3f, 0.0, 0.0, 0.0, 1.0;
		C = (A*B).inverse() * D;

        std::cout << "rotCB" << i << " = [ " << C.block(0,0,3,3) << " ]; " << std::endl;
        std::cout << "transCB" << i << " = [ " << C.block(0,3,3,1) << " ]; " << std::endl;

		//pushing back data into vectors
		rotationRB_vec.push_back(tmp_m3f);
		translationRB_vec.push_back(tmp_v3f);
		rotationCB_vec.push_back(C.block(0,0,3,3));
		translationCB_vec.push_back(C.block(0,3,3,1));
	}

    std::cout << "Data size: " << rotationRB_vec.size() << std::endl;

	return true;

}


Vector3f CalibrationNode::getLogTheta(Matrix3f R){

	//Assumption R is never an Identity
	float theta = acos((R.trace()-1)/2);
	Matrix3f logTheta = 0.5*theta/sin(theta)*(R-R.transpose());
	return Vector3f(logTheta(2,1), logTheta(0,2), logTheta(1,0));

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "handEyeCalibrationNode"); // initialize the CMgreenBallDetector node

	ros::NodeHandle n; // declare a node handle

	CalibrationNode CalibrationObject (n); // start the ball detector node with the node handle n

	//Debugging
	//CalibrationObject.generateData();
	//CalibrationObject.performEstimation();

	ros::spin();

	return 0;
}
