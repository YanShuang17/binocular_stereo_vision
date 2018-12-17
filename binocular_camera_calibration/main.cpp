#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

bool image_capture=false;//定义全局变量，用于相机采集标定图像
//鼠标事件响应函数，用于相机采集标定图像（双击鼠标左键一次采集一张图像）
void on_mouse(int event,int x,int y,int flags,void* a)
{
	if(event==CV_EVENT_LBUTTONDBLCLK)
	{
		image_capture=true;
	}

}
void calobjectPoints(vector<Point3f>& obj, Size &boardSize,int squareSize);

int main(int argc, char* argv[])
{
	Size boardSize=Size(10,8);  // 标定棋盘格的内角点尺寸（如7x7）
    float squareSize=20.0;  // 标定板上黑白格子的实际边长（mm）
    int nrFrames=20;       // 用于标定的图像数目
    string outputFileName;  // 输出文件的名称
	bool showUndistorsed=true;
    //vector<string> imageList;
	vector<string> imageList1;
	vector<string> imageList2;
	Size imageSize;
	Size imageSize1;
	Size imageSize2;
    
	int calib_pattern=0;
	cout<<"这是一个双目视觉程序！"<<endl;
	cout<<"首先，请选择摄像机标定模式：1（从摄像头采集标定图像）或2（从图像序列获取标定图像）"<<endl;
	cin>>calib_pattern;

	/************************************************************************************/
	/*********************摄像机实时采集标定图像并保存到指定文件夹***********************/
	//若输入“1”，则打开相机，并利用相机预览窗口和鼠标操作采集标定图像
	//需要采集图像的数目由变量nrFrames决定，采够到nrFrames张图像，预览窗口自动关闭
	if(calib_pattern==1)
	{
		//VideoCapture inputCapture;
		VideoCapture inputCapture1;
		VideoCapture inputCapture2;
		//inputCapture.open(1);
		inputCapture2.open(2);
		inputCapture1.open(1);

		//if(!inputCapture.isOpened()==true) return -1;
		if(!inputCapture1.isOpened()==true) return -1;
		if(!inputCapture2.isOpened()==true) return -1;

		//inputCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);  
       // inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 

		inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH, 960);//设置所采集图像的分辨率大小  
        inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH, 960);  
        inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

		/*
		inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH, 640);//设置所采集图像的分辨率大小  
        inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH, 640);  
        inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		*/
		//namedWindow("标定图像采集预览窗口",CV_WINDOW_AUTOSIZE);
		namedWindow("左相机（相机1）标定图像采集预览窗口",CV_WINDOW_AUTOSIZE);
		namedWindow("右相机（相机2）标定图像采集预览窗口",CV_WINDOW_AUTOSIZE);

		//设置鼠标事件函数，用于相机采集标定图像（在指定窗口双击鼠标左键一次采集一张图像）
		//cvSetMouseCallback("标定图像采集预览窗口",on_mouse,NULL);
		cvSetMouseCallback("左相机（相机1）标定图像采集预览窗口",on_mouse,NULL);
		//Mat src_image;
		Mat src_image1;
		Mat src_image2;
		int capture_count=0;

		while(1)
		{
			//inputCapture>>src_image;
			inputCapture1>>src_image1;
			inputCapture2>>src_image2;
			//imshow("标定图像采集预览窗口",src_image);
			imshow("左相机（相机1）标定图像采集预览窗口",src_image1);
			imshow("右相机（相机2）标定图像采集预览窗口",src_image2);
			waitKey(35);
			if(image_capture==true&&capture_count<nrFrames)
			{
				//Mat cap;
				Mat cap1;
				Mat cap2;
				//inputCapture>>cap;
				inputCapture1>>cap1;
				inputCapture2>>cap2;
				char address[100];
				//拼凑标定图像存放路径并保存
				//sprintf(address,"Calibration_Image_Camera\\Image%d%s",capture_count+1,".jpg");
				//imwrite(address,cap);
				sprintf(address,"Calibration_Image_Camera\\Image_l%d%s",capture_count+1,".jpg");
				imwrite(address,cap1);
				sprintf(address,"Calibration_Image_Camera\\Image_r%d%s",capture_count+1,".jpg");
				imwrite(address,cap2);
				capture_count++;
				image_capture=false;
			}
			else if(capture_count>=nrFrames)
			{
				cout<<"标定图像采集完毕！共采集到"<<capture_count<<"张标定图像。"<<endl;
				//destroyWindow("标定图像采集预览窗口");
				destroyWindow("左相机（相机1）标定图像采集预览窗口");
				destroyWindow("右相机（相机2）标定图像采集预览窗口");
				image_capture=false;
				break;
			}
		}
	}

	/***********************************************************************************/
	/*******************将存放标定图像的路径读入到imageList（1/2）向量中****************/
	if(calib_pattern==1||calib_pattern==2)
	{
		char name[100];
		for(int i=1;i<=nrFrames;i++)
		{
			//sprintf(name,"Calibration_Image_Camera\\Image%d%s",i,".jpg");
			//imageList.push_back(name);
			sprintf(name,"Calibration_Image_Camera1\\Image_l%d%s",i,".jpg");
			imageList1.push_back(name);
			sprintf(name,"Calibration_Image_Camera1\\Image_r%d%s",i,".jpg");
			imageList2.push_back(name);
		}
	}
	//cout<<"imageList.size:"<<imageList.size()<<endl;
	cout<<"imageList1.size:"<<imageList1.size()<<endl;
	cout<<"imageList2.size:"<<imageList2.size()<<endl;

	/************************************************************************************/
	/****************标定前的准备工作：获取objectPoints和imagePoints*********************/
	//1 首先根据squareSize(棋盘格的实际边长，比如20mm)和boardSize(棋盘格的角点数，比如5x5)，利用for循环得到角点的世界坐标objectPoints（Z坐标假设为0）
	//2 利用for循环和findChessboardCorners()函数得到与角点世界坐标向量objectPoints对应的图像像素坐标向量imagePoints
	
	//vector<vector<Point2f> > imagePoints;//各个图像找到的角点的集合
	//vector<vector<Point3f> > objectPoints(1);
	vector<vector<Point2f> > imagePoints1;
	vector<vector<Point3f> > objectPoints1(1);//暂时先定义一维的objectPoints1，等确定了imagePoints的维数之后再进行扩容
	vector<vector<Point2f> > imagePoints2;
	vector<vector<Point3f> > objectPoints2(1);
	/*
	//调用calobjectPoints()函数用于得到棋盘格角点的世界坐标集
	calobjectPoints(objectPoints[0], boardSize,squareSize);

	//可通过改变变量showChessboardCorner的值来确定是否展示获取角点后的图像
	bool displayCorners = false;
	for(int i=0;i<imageList.size();i++)
	{
		Mat src=imread(imageList[i],1);
		//imshow("显示",src);
		imageSize = src.size();
		vector<Point2f> pointBuf;//某一副图像找到的角点
		bool found=findChessboardCorners( src, boardSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);//这一步比较费时
		if(found)
		{
			Mat grayimage;
            cvtColor(src, grayimage, COLOR_BGR2GRAY);
            cornerSubPix( grayimage, pointBuf, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
			imagePoints.push_back(pointBuf);
			if(displayCorners)
			{
				Mat MidImage=src.clone();
				drawChessboardCorners( MidImage, boardSize, Mat(pointBuf), found );
				imshow("角点获取情况",MidImage);
				waitKey(300);
			}
			destroyWindow("角点获取情况");
		}
	}
	objectPoints.resize(imagePoints.size(),objectPoints[0]);
	*/
	calobjectPoints(objectPoints1[0], boardSize,squareSize);

	//可通过改变变量displayCorners1的值来确定是否展示获取角点后的图像
	bool displayCorners1 = false;
	for(int i=0;i<imageList1.size();i++)
	{
		Mat src1=imread(imageList1[i],1);
		imageSize = src1.size();
		vector<Point2f> pointBuf1;
		//使用不同的FLAG变量，标定结果差别很小
		bool found1=findChessboardCorners( src1, boardSize, pointBuf1);
		//bool found1=findChessboardCorners( src1, boardSize, pointBuf1, CALIB_CB_ADAPTIVE_THRESH);
		//bool found1=findChessboardCorners( src1, boardSize, pointBuf1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);//这一步比较费时
		if(found1)
		{
			Mat grayimage1;
            cvtColor(src1, grayimage1, COLOR_BGR2GRAY);
            cornerSubPix( grayimage1, pointBuf1, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
			imagePoints1.push_back(pointBuf1);
			if(displayCorners1)
			{
				Mat MidImage1=src1.clone();
				drawChessboardCorners( MidImage1, boardSize, Mat(pointBuf1), found1 );
				imshow("左相机角点获取情况",MidImage1);
				waitKey(300);
			}
			destroyWindow("左相机角点获取情况");
		}
	}
	//利用resize()函数对objectPoints向量进行扩容
	objectPoints1.resize(imagePoints1.size(),objectPoints1[0]);

	calobjectPoints(objectPoints2[0], boardSize,squareSize);
	bool displayCorners2 = false;
	for(int i=0;i<imageList2.size();i++)
	{
		Mat src2=imread(imageList2[i],1);
		//imageSize= src2.size();
		vector<Point2f> pointBuf2;
		//使用不同的FLAG变量，标定结果差别很小
		bool found2=findChessboardCorners( src2, boardSize, pointBuf2);
		//bool found2=findChessboardCorners( src2, boardSize, pointBuf2, CALIB_CB_ADAPTIVE_THRESH);
		//bool found2=findChessboardCorners( src2, boardSize, pointBuf2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);//这一步比较费时
		if(found2)
		{
			Mat grayimage2;
            cvtColor(src2, grayimage2, COLOR_BGR2GRAY);
            cornerSubPix( grayimage2, pointBuf2, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
			imagePoints2.push_back(pointBuf2);
			if(displayCorners2)
			{
				Mat MidImage2=src2.clone();
				drawChessboardCorners( MidImage2, boardSize, Mat(pointBuf2), found2 );
				imshow("右相机角点获取情况",MidImage2);
				waitKey(300);
			}
			destroyWindow("右相机角点获取情况");
		}
	}
	objectPoints2.resize(imagePoints2.size(),objectPoints2[0]);

	/**************************************************************************************/
	/********************************进行相机标定******************************************/
	//通过calibrateCamera()函数进行相机标定
	//主要为了得到相机的内参矩阵cameraMatrix、畸变系数矩阵distCoeffs
	//另外可以通过函数返回的重投影误差大小评价相机标定的精度如何
	//这里得到的相机外参矩阵不重要

	//Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	//Mat distCoeffs = Mat::zeros(8, 1, CV_64F);//畸变系数的顺序是[k1,k2,p1,p2,k3,(k4,k5,k6)]
	Mat cameraMatrix1 = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs1 = Mat::zeros(5, 1, CV_64F);
	Mat cameraMatrix2 = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs2 = Mat::zeros(5, 1, CV_64F);
	//vector<Mat> rvecs, tvecs;
	vector<Mat> rvecs1, tvecs1;
	vector<Mat> rvecs2, tvecs2;
	/*
	double re_project_err=calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,0);
	//checkRange()函数---用于检查矩阵中的每一个元素是否在指定的一个数值区间之内
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
	if(ok)
	{
		cout<<"相机标定成功！"<<endl;
		cout<<"相机标定的重投影误差："<<re_project_err<<endl;
		cout<<"相机内参矩阵："<<endl<<cameraMatrix<<endl;
		cout<<"相机畸变系数矩阵："<<endl<<distCoeffs<<endl;
		if(showUndistorsed==true)
		{
			for(int i=0;i<imageList.size();i++)
			{
				Mat temp=imread(imageList1[i],1);;
				//利用undistort()函数得到经过畸变矫正的图像
				Mat undistort_view;
				undistort(temp, undistort_view, cameraMatrix, distCoeffs);
				imshow("原畸变图像",temp);
				imshow("畸变矫正图像",undistort_view);
				waitKey(300);
			}
			destroyWindow("原畸变图像");
			destroyWindow("畸变矫正图像");
		}
	}
	*/
	/*  
    calibrateCamera() 
    输入参数 objectPoints  角点的实际物理坐标 
             imagePoints   角点的图像坐标 
             imageSize     图像的大小 
    输出参数 
             cameraMatrix  相机的内参矩阵 
             distCoeffs    相机的畸变参数 
             rvecs         旋转矢量(外参数) 
             tvecs         平移矢量(外参数） 
    */

	//畸变系数的顺序是[k1,k2,p1,p2,k3,(k4,k5,k6)]
	//如果我们不需要K3，在初始化K3为O之后，可以使用标志CV_CALIB_FIX_K3，这样，标定函数不会改变K3的值
	//一般地，K3应设置为0，除非使用鱼眼镜头（参考《learning opencv》第十一章）
	//返回的distCoeffs1向量的长度由标志位flag决定，当flag设置为CV_CALIB_RATIONAL_MODEL时返回所有畸变参数（8个）
	//当设置成其他flag时都返回5维的畸变系数，即[k1,k2,p1,p2,k3]
	double re_project_err1=calibrateCamera(objectPoints1, imagePoints1, imageSize, cameraMatrix1, distCoeffs1, rvecs1, tvecs1,CV_CALIB_FIX_K3);
	//checkRange()函数---用于检查矩阵中的每一个元素的有效性
	bool ok1 = checkRange(cameraMatrix1) && checkRange(distCoeffs1);
	if(ok1)
	{
		cout<<"左相机标定成功！"<<endl;
		cout<<"左相机标定的重投影误差："<<re_project_err1<<endl;
		//cout<<"左相机内参矩阵："<<endl<<cameraMatrix1<<endl;
		//cout<<"左相机畸变系数矩阵："<<endl<<distCoeffs1<<endl;
	}
	double re_project_err2=calibrateCamera(objectPoints2, imagePoints2, imageSize, cameraMatrix2, distCoeffs2, rvecs2, tvecs2,CV_CALIB_FIX_K3);
	bool ok2 = checkRange(cameraMatrix2) && checkRange(distCoeffs2);
	if(ok2)
	{
		cout<<"右相机标定成功！"<<endl;
		cout<<"右相机标定的重投影误差："<<re_project_err2<<endl;
		//cout<<"右相机内参矩阵："<<endl<<cameraMatrix2<<endl;
		//cout<<"右相机畸变系数矩阵："<<endl<<distCoeffs2<<endl;
	}

	/**************************************************************************************/
	/********************************立体标定******************************************/
	//利用stereoCalibrate()函数进行立体标定，得到4个矩阵：R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
	//同时可以得到两个相机的内参矩阵和畸变系数矩阵cameraMatrix1、distCoeffs1、cameraMatrix2、distCoeffs2
	//也就是说其实可以不用事先单独给每个相机进行标定的
	//stereoCalibrate()函数内部应该是调用了calibrateCamera()函数的
	//但是也可以先调用calibrateCamera()函数先对每个相机进行标定，得到内参矩阵和畸变系数的初始值
	//然后立体标定时stereoCalibrate()函数会对内参矩阵和畸变系数进行优化，此时应加上flag：CALIB_USE_INTRINSIC_GUESS
	//如果没有事先调用过calibrateCamera()函数，请不要使用flag:CALIB_USE_INTRINSIC_GUESS，会得到很奇怪的结果
	//如果之前标定过的相机内参矩阵和畸变参数很满意，不想在立体标定时被进一步优化，可使用CV_CALIB_FIX_INTRINSIC
	//根据官方文档建议，stereoCalibrate()函数计算的参数空间的维数很高（一次性得到很多结果）
	//可能会导致某些结果发散到无意义的值，偏离正确结果，如果提前使用了calibrateCamera()函数对每个相机进行过标定
	//则可以选择将CV_CALIB_FIX_INTRINSIC应用到stereoCalibrate()函数中，这样能减少计算的参数
	//防止导致某些结果发散到无意义的值
	//CV_CALIB_FIX_INTRINSIC这个参数是否使用还需后面做进一步权衡
	Mat R=Mat::eye(3, 3, CV_64F);
	Mat T=Mat::zeros(3, 1, CV_64F);
	Mat E=Mat::zeros(3, 3, CV_64F);
	Mat F=Mat::eye(3, 3, CV_64F); //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵  
	double rms = stereoCalibrate(objectPoints1, imagePoints1, imagePoints2,  
        cameraMatrix1, distCoeffs1,  
        cameraMatrix2, distCoeffs2,  
        imageSize, R, T, E, F,  
        CV_CALIB_FIX_INTRINSIC,  
        TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 1e-5)); 
	/*double rms = stereoCalibrate(objectPoints1, imagePoints1, imagePoints2,  
        cameraMatrix1, distCoeffs1,  
        cameraMatrix2, distCoeffs2,  
        imageSize, R, T, E, F,  
        CALIB_USE_INTRINSIC_GUESS|CV_CALIB_FIX_K3,  
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6)); */
	cout << "Stereo Calibration done with RMS error = " << rms << endl;
	cout<<"左相机内参矩阵："<<endl<<cameraMatrix1<<endl;
	cout<<"左相机畸变系数矩阵："<<endl<<distCoeffs1<<endl;
	cout<<"右相机内参矩阵："<<endl<<cameraMatrix2<<endl;
	cout<<"右相机畸变系数矩阵："<<endl<<distCoeffs2<<endl;
	cout<<"R:"<<endl<<R<<endl;
	cout<<"T:"<<endl<<T<<endl;

	/**************************************************************************************/
	/********************************保存标定结果******************************************/
	//保存.xml文件时需要注意2个问题：
	//1 需要保存的Mat型变量定义时必须要初始化，否则程序编译会出错；
	//2 保存时变量的标识符命名中不能出现“.”。
	const string filename="F:\\Binocular_Stereo_Vision_Test\\Calibration_Result.xml";
	//string filename="Calibration_Result.xml";
	FileStorage fs(filename,FileStorage::WRITE);
	if(fs.isOpened())
	{
		fs << "width" << imageSize.width;
		fs << "height" << imageSize.height;
		fs << "board_width" << boardSize.width;
		fs << "board_height" << boardSize.height;
		fs << "nrFrames" << nrFrames;
		fs << "cameraMatrix1" <<cameraMatrix1;
		fs << "distCoeffs1" <<distCoeffs1;
		fs << "cameraMatrix2" <<cameraMatrix2;
		fs << "distCoeffs2" <<distCoeffs2;
		fs << "R" <<R;
		fs << "T" <<T;
		fs << "E" <<E;
		fs << "F" <<F;
		fs.release();
		cout<<"Calibration result has been saved successfully to \nF:\\Binocular_Stereo_Vision_Test\\Calibration_Result.xml"<<endl;
	}
	else
	{  
        cout << "Error: can not save the Calibration result!!!!!" << endl;  
    }
	
	cout<<"按任意键退出程序..."<<endl;
	while(1)
	{
		int a=waitKey(10);
		if(char(a)==27)
			break;
	}
	waitKey(0);
	return 0;
}
/*计算标定板上模块的实际物理坐标*/  
void calobjectPoints(vector<Point3f>& obj, Size &boardSize,int squareSize)  
{   
    for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                obj.push_back(Point3f((float)(j*squareSize), (float)(i*squareSize), 0.f));
} 