/*
 * StereoCameraOdometry.cpp
 *
 *  Created on: Mar 27, 2017
 *      Author: lyq
 */
#include "StereoCameraOdometry.h"
void readLeftAndRightImage1(int i , vector<string>& time){
	char filenamel1[200],filenamer1[200],filenamel2[200],filenamer2[200];
	sprintf(filenamel1,"/home/lyq/VIN/data/malaga-urban-dataset-extract-07/malaga-urban-dataset-extract-07_rectified_800x600_Images/%s",time[2*(i-1)].c_str());
	sprintf(filenamer1,"/home/lyq/VIN/data/malaga-urban-dataset-extract-07/malaga-urban-dataset-extract-07_rectified_800x600_Images/%s",time[2*(i-1)+1].c_str());
	sprintf(filenamel2,"/home/lyq/VIN/data/malaga-urban-dataset-extract-07/malaga-urban-dataset-extract-07_rectified_800x600_Images/%s",time[2*i].c_str());
	sprintf(filenamer2,"/home/lyq/VIN/data/malaga-urban-dataset-extract-07/malaga-urban-dataset-extract-07_rectified_800x600_Images/%s",time[2*i+1].c_str());
	leftImage1 = cv::imread(filenamel1);
	rightImage1 = cv::imread(filenamer1);
	leftImage2 = cv::imread(filenamel2);
	rightImage2 = cv::imread(filenamer2);
}

void readPreviousAndLaterImage1(int i,vector<string>& time){
	char file1[200],file2[200];
	sprintf(file1,"/home/lyq/VIN/data/malaga-urban-dataset-extract-01/malaga-urban-dataset-extract-01_rectified_800x600_Images/%s",time[2*(i-1)].c_str());
	sprintf(file2,"/home/lyq/VIN/data/malaga-urban-dataset-extract-01/malaga-urban-dataset-extract-01_rectified_800x600_Images/%s",time[2*i].c_str());
	//previousImage1 = cv::imread(file1);
	//laterImage1 = cv::imread(file2);
}

PNP_PARAMETERS pnpSolveProblem1(CAMERA_INTRINSIC_PARAMETERS& Camera,vector<Point2f>& left1,vector<Point2f>& left2,vector< vector<double> >&  landmark){
	vector<cv::Point3f> p_obj;
	vector<cv::Point2f> p_img;
	//cout<<landmark[0].size()<<endl;
	for(size_t i=0;i<landmark[0].size();i++){
		p_img.push_back(left2[i]);
		p_obj.push_back(cv::Point3f(landmark[0][i],landmark[1][i],landmark[2][i]));
	}
	double camera_matrix_data[3][3] = {{Camera.fx,0,Camera.cx},{0,Camera.fy,Camera.cy},{0,0,1}};
	cv::Mat cameraMatric(3,3,CV_64F,camera_matrix_data);
	cv::Mat rvec,tvec,inliers;
	cv::solvePnPRansac(p_obj,p_img,cameraMatric,cv::Mat(),rvec,tvec,false,100,8.0,0.99,inliers);
	PNP_PARAMETERS P;
	P.rvec = rvec;
	P.tvec = tvec;
	//cout<<"P.rvec:"<<P.rvec<<endl;
	//cout<<"P.tvec:"<<P.tvec<<endl;
	P.inliers = inliers;
	return P;
}


