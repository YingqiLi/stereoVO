//============================================================================
// Name        : IMUandVisual.cpp
// Author      : liyingqi
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================


#include "StereoCameraOdometry.h"

#define M_PI 3.1415926
cv::Mat leftImage2,rightImage2;
cv::Mat leftImage1,rightImage1;

int main() {
	ifstream myImageTimefile;
	myImageTimefile.open("/home/lyq/VIN/data/malaga-urban-dataset-extract-07/malaga-urban-dataset-extract-07_all-sensors_IMAGES.txt");
	string Index;
	vector<string> Image;
	vector<string> ImageTime;
	if(myImageTimefile.is_open()){
		while(!myImageTimefile.eof()){
			myImageTimefile>>Index;
			Image.push_back(Index);
		}
	}
	myImageTimefile.close();
	//确定相机内部参数，及两个摄像头的平移向量
	CAMERA_INTRINSIC_PARAMETERS Camera;
	Camera.cx = 404.00760;
	Camera.cy = 309.05989;
	Camera.fx = 621.18428;
	Camera.fy = 621.18428;
	Camera.focal_length = 1.000000e-03;
	Camera.Tx = 0.119471;

	vector<double> result1;
	result1.push_back(0.0);
	vector<double> result2;
	result2.push_back(0.0);
	vector<double> result3;
	result3.push_back(0.0);
	cv::Mat Rotationf=cv::Mat::zeros(3,3,CV_64FC1);
	Rotationf.row(0).col(0) = cos(180*M_PI/180);Rotationf.row(0).col(2) = -sin(180*M_PI/180);//数据集2是5.5度，数据集9是-8.2度,数据集7是-180度
	Rotationf.row(1).col(1) = 1;
	Rotationf.row(2).col(0) = sin(180*M_PI/180);Rotationf.row(2).col(2) = cos(180*M_PI/180);
	//Rotationf.row(0).col(0) = 1;
	//Rotationf.row(1).col(1) = 1;
	//Rotationf.row(2).col(2) = 1;
	for(int i=77;i<1992;i++){//数据集1为758,数据集2为1759,数据集9为950,数据集7为1992
		readLeftAndRightImage1(i-1,Image );

		//namedWindow("1",CV_WINDOW_AUTOSIZE);
		//namedWindow("2",CV_WINDOW_AUTOSIZE);
		//imshow("1",leftImage1);
	    //imshow("2",rightImage1);
	    //waitKey(20);
		//角点检测Shi-Tomasi的goodFeaturesToTrack
		vector<Point2f> left1,right1,left2,right2;//角点的位置
		Mat showleft1,showright1,showleft2,showright2;//存储灰度图
		cvtColor(leftImage1,showleft1,COLOR_RGB2GRAY);
		cvtColor(rightImage1,showright1,COLOR_RGB2GRAY);
		cvtColor(leftImage2,showleft2,COLOR_RGB2GRAY);
		cvtColor(rightImage2,showright2,COLOR_RGB2GRAY);
		goodFeaturesToTrack(showleft1,left1,500,0.1,20,Mat());
		//使用KLT跟踪，cvCalcOpticalFlowPyrLK
		vector<uchar> status1,statusl,statusr;
		vector<float> error1,errorl,errorr;
		vector<Point2f> later,rightlater;
		calcOpticalFlowPyrLK(leftImage1,rightImage1,left1,right1,status1,error1);
		calcOpticalFlowPyrLK(leftImage1,leftImage2,left1,left2,statusl,errorl);
		calcOpticalFlowPyrLK(leftImage2,rightImage2,left2,right2,statusr,errorr);
		cv::Mat lll = leftImage1.clone();
		//for(int j=0;j<left1.size();j++){
			//circle(lll,left1[j],5,Scalar(0,255,0),2,8,0);
		//}
		////for(int j=0;j<right.size();j++){
			////circle(laterImage1,later[j],5,Scalar(0,255,0),2,8,0);
		////}
		//namedWindow("3",CV_WINDOW_AUTOSIZE);
		////namedWindow("4",CV_WINDOW_AUTOSIZE);
		//imshow("3",lll);
		////imshow("4",laterImage1);
		//waitKey(25);

		vector<Point2f> left1pt,right1pt,left2pt,right2pt;
		vector< vector<double> > landmark(6),landmark1(3),landmark2(3);
		vector<Point3f> land1,land2;
		for(size_t c=0;c<left1.size();c++){
			if( status1[c]==1 && statusl[c]==1 && statusr[c]==1 ){
				/*选择关键帧*/

				/*选择关键帧*/
				double xl1 = (left1[c].x-Camera.cx)/Camera.fx;
				double yl1 = (left1[c].y-Camera.cy)/Camera.fy;
				double xr1 = (right1[c].x-Camera.cx)/Camera.fx;
				double yr1 = (right1[c].y-Camera.cy)/Camera.fy;
				double xl2 = (left2[c].x-Camera.cx)/Camera.fx;
				double yl2 = (left2[c].y-Camera.cy)/Camera.fy;
				double xr2 = (right2[c].x-Camera.cx)/Camera.fx;
				double yr2 = (right2[c].y-Camera.cy)/Camera.fy;
				double d1 = (Camera.Tx*Camera.focal_length*1000)/(xl1-xr1);
				double d2 = (Camera.Tx*Camera.focal_length*1000)/(xl2-xr2);
				if (  xl1!=xr1 && xl2!=xr2 ){
					left1pt.push_back(left1[c]);right1pt.push_back(right1[c]);
					left2pt.push_back(left2[c]);
					right2pt.push_back(right2[c]);

					landmark[2].push_back(d1);
					landmark[0].push_back( xl1*d1);
					landmark[1].push_back(yl1*d1);


					landmark[5].push_back(d2);
					landmark[3].push_back( xl2*d2);
					landmark[4].push_back(yl2*d2);
				}else{
					continue;
				}
			}

		}

		//像素点
		//cout<<"像素点："<<endl;
		//for(size_t c=0;c<landmark[0].size();c++){
			//cout<<left2pt[c].x<<","<<left2pt[c].y<<","<<right2pt[c].x<<","<<right2pt[c].y<<";"<<endl;
		//}
		//OPENCV函数

		PNP_PARAMETERS pnp;
		////pnp = ICPAlgorithm1(landmark);
		pnp = pnpSolveProblem1(Camera,left1pt,left2pt,landmark);
		Mat result,Rotation,Rotationt;
		Rodrigues(pnp.rvec,Rotation);
		//cout<<Rotation<<endl;
		//cout<<pnp.tvec<<endl;


		cv::transpose(Rotation,Rotationt);
		Rotationf = Rotationf*Rotationt;
		result = Rotationf*pnp.tvec;
		//result = pnp.tvec;
		result1.push_back((result1.back()+result.at<double>(0,0)));
		result2.push_back((result2.back() +result.at<double>(1,0)))  ;
		result3.push_back((result3.back() + result.at<double>(2,0)))  ;
		//cout<<i<<"  "<<"R="<<Rotation<<endl;
		//cout<<i<<"  "<<"t="<<pnp.tvec<<endl;
		cout<<i<<" "<<result1.back()<<" "<<result2.back()<<" "<<result3.back()<<endl;
	}

	return 0;
}
