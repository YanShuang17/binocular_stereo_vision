#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include<opencv/cv.h>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
enum
{
	CAMERA=0,IMAGELIST=1
};
enum
{
	BM=1,SGBM=0
};

bool get_imagePair=false;

float X=0;
float Y=0;
float Z=0;

Size mousePoint=Size(0,0);

Size imageSize;//标定图像的分辨率（960x720）
Size boardSize;
int nrFrames=0;
Mat cameraMatrix1;//左相机内参矩阵
Mat distCoeffs1;//左相机畸变系数矩阵
Mat cameraMatrix2;//右相机内参矩阵
Mat distCoeffs2;//右相机畸变系数矩阵
Mat R, T, E, F; //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
Mat R1, R2, P1, P2, Q; //校正旋转矩阵R，投影矩阵P 重投影矩阵Q 
Mat mapx1, mapy1, mapx2, mapy2; //映射表  
Rect validROI1, validROI2; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
bool isopen=true;//用于判断相机获取图像对时是否打开
Mat imgLeft;//校正后的左图像
Mat imgRight;//校正后的右图像
Mat disparity8U;//将视差值范围投影到0-255的视差图
Mat disparity_real;//真实视差值的视差图
Mat disparity;//调用函数得到的原始视差图
int numDisparities = 7;//匹配搜索的视差范围，规定必须能被16整除
int blockSize = 7;//匹配窗口大小
int UniquenessRatio=5;
Ptr<StereoBM> bm = StereoBM::create(16,21);
Mat _3dImage;//三维坐标图
//设置鼠标回调函数on_mouse()用于输出鼠标指定点（单击左键）的视差值和三维XYZ值（单位：厘米）
void on_mouse(int event,int x,int y,int flags,void* a)
{
	if(event==CV_EVENT_LBUTTONDOWN)//按下左键
	{
		mousePoint.width=x;
		mousePoint.height=y;
		//cout<<"x="<<mousePoint.width<<'\t'<<"y="<<mousePoint.height<<endl;
	}
}
//设置鼠标回调函数on_mouse1()用于获取测距用的左右图像对（双击左键）
void on_mouse1(int event,int x,int y,int flags,void* a)
{
	if(event==CV_EVENT_LBUTTONDBLCLK)//双击左键
	{
		get_imagePair=true;
	}
}
bool readFile(string filename)
{
	FileStorage fs(filename,FileStorage::READ);
	if(fs.isOpened())
	{
		fs["width"]>>imageSize.width;
		fs["height"]>>imageSize.height;
		fs["board_width"]>> boardSize.width;
		fs["board_height"]>> boardSize.height;
		fs["nrFrames"]>>nrFrames;
		fs["cameraMatrix1"]>>cameraMatrix1;
		fs["distCoeffs1"]>>distCoeffs1;
		fs["cameraMatrix2"]>>cameraMatrix2;
		fs["distCoeffs2"]>>distCoeffs2;
		fs["R"] >> R;      
        fs["T"] >> T;
		fs["E"] >> E;      
        fs["F"] >> F;
		fs.release();
		/*
		cout<<"Succeed to read the Calibration result!!!"<<endl;
		cout<<"左相机内参矩阵："<<endl<<cameraMatrix1<<endl;
		cout<<"右相机内参矩阵："<<endl<<cameraMatrix2<<endl;
		cout<<"R:"<<endl<<R<<endl;
		cout<<"T:"<<endl<<T<<endl;
		cout<<"E:"<<endl<<E<<endl;
		cout<<"F:"<<endl<<F<<endl;
		*/
		return true;
	}
	else
	{  
        cerr<< "Error: can not open the Calibration result file!!!!!" << endl;
		return false;
    }
}
void stereo_rectify(bool useCalibrated,int getImagePair,string path1,string path2)
{
	/**************************************************************************************/
	/*
	要实现立体校正，使左右图像共平面且行对准，需要用到以下参数：
	cameraMatrix1, distCoeffs1, R1, P1
	cameraMatrix2, distCoeffs2, R2, P2
	其中内参矩阵和畸变系数通过标定程序获得，但R1、P1、R2、P2的值，opencv提供了两种方法：
	1. Hartley方法；
	这种方法称为“非标定立体校正方法”，也就是说不用通过标定获得的内参矩阵和畸变系数获取
	R1、P1、R2、P2的值，直接根据匹配点计算基础矩阵F，再进一步计算R1、P1、R2、P2。
	这种方法主要用到两个函数：findFundamentalMat()和stereoRectifyUncalibrated()
	具体的原理说明参考《Learning opencv》中文版498页。
	2. Bouguet方法 
	这种方法称为“标定立体校正方法”，它是根据立体标定获得的内参矩阵、畸变系数、R和T作为
	输入，利用stereoRectify()函数得到R1、P1、R2、P2的值。
	两种方法的选用根据bool类型的useCalibrated变量决定，
	当useCalibrated=true时，调用Bouguet方法；
	当useCalibrated=false时，调用Hartley方法；
	*/
	/**************************************************************************************/
	//当useCalibrated=true时，调用Bouguet方法
    if( useCalibrated )
    {
        //参数alpha的设置对结果影响很大
		//alpha：图像剪裁系数，取值范围是-1、0~1。
		//当取值为 0 时，OpenCV会对校正后的图像进行缩放和平移，使得remap图像只显示有效像素（即去除不规则的边角区域）,适用于机器人避障导航等应用；
		//当alpha取值为1时，remap图像将显示所有原图像中包含的像素，该取值适用于畸变系数极少的高端摄像头；
		//alpha取值在0-1之间时，OpenCV按对应比例保留原图像的边角区域像素。
		//Alpha取值为-1时，OpenCV自动进行缩放和平移
		stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q,  
                  CALIB_ZERO_DISPARITY,0,imageSize,&validROI1,&validROI2);
    }
	//当useCalibrated=false时，调用Hartley方法
    else
    {
		vector<vector<Point2f> > imagePoints1;
		vector<vector<Point2f> > imagePoints2;
		char name[100];
		for(int i=1;i<=nrFrames;i++)
		{
			//sprintf(name,"Calibration_Image_Camera\\Image%d%s",i,".jpg");
			//imageList.push_back(name);
			sprintf(name,"Calibration_Image_Camera\\Image_l%d%s",i,".jpg");
			Mat src1=imread(name,1);
			vector<Point2f> pointBuf1;
			bool found1=findChessboardCorners( src1, boardSize, pointBuf1);
			if(found1)
			{
				Mat grayimage1;
				cvtColor(src1, grayimage1, COLOR_BGR2GRAY);
				cornerSubPix( grayimage1, pointBuf1, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
				imagePoints1.push_back(pointBuf1);
			}
			sprintf(name,"Calibration_Image_Camera\\Image_r%d%s",i,".jpg");
			Mat src2=imread(name,1);
			vector<Point2f> pointBuf2;
			bool found2=findChessboardCorners( src2, boardSize, pointBuf2);
			if(found2)
			{
				Mat grayimage2;
				cvtColor(src2, grayimage2, COLOR_BGR2GRAY);
				cornerSubPix( grayimage2, pointBuf2, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
				imagePoints2.push_back(pointBuf2);
			}
		}
        vector<Point2f> allimgpt[2];
        for(int i = 0; i < nrFrames; i++ )
			copy(imagePoints1[i].begin(), imagePoints1[i].end(), back_inserter(allimgpt[0]));
		for(int i = 0; i < nrFrames; i++ )
			copy(imagePoints2[i].begin(), imagePoints2[i].end(), back_inserter(allimgpt[1]));
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix1.inv()*H1*cameraMatrix1;
        R2 = cameraMatrix2.inv()*H2*cameraMatrix2;
        P1 = cameraMatrix1;
        P2 = cameraMatrix2;
    }
	/* 
    根据stereoRectify计算出来的R和P来计算图像的映射表mapx,mapy 
    mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准 
    initUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。
	在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。 
    所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵 
    */
	initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, mapx1, mapy1);  
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, mapx2, mapy2); 
	if(getImagePair==CAMERA)
	{
		VideoCapture inputCapture1;
		VideoCapture inputCapture2;
		inputCapture2.open(2);
		inputCapture1.open(1);
		if(!inputCapture1.isOpened()==true) 
		{
			cerr<<"failed to open camera1"<<endl;
			isopen=false;
			return;
		}
		if(!inputCapture2.isOpened()==true)
		{
			cerr<<"failed to open camera2"<<endl;
			isopen=false;
			return;
		}
		
		inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH, 960);  
		inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH, 960);  
		inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		/*
		inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH, 640);  
		inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH, 640);  
		inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		*/
		namedWindow("获取图像对窗口（双击左键）",WINDOW_AUTOSIZE);
		//设置鼠标回调函数on_mouse1()用于获取三维重建用的左右图像对
		//在指定窗口双击鼠标左键一次采集一对图像
		cvSetMouseCallback("获取图像对窗口（双击左键）",on_mouse1,NULL);
		Mat src_image1;
		Mat src_image2;
		while(1)
		{
			inputCapture1>>src_image1;
			inputCapture2>>src_image2;
			imshow("获取图像对窗口（双击左键）",src_image1);
			imshow("获取图像对窗口(右图像)",src_image2);
			waitKey(35);
			if(get_imagePair==true)
			{
				Mat cap1;
				Mat cap2;
				inputCapture1>>cap1;
				inputCapture2>>cap2;
				char address[100];
				sprintf(address,"Stereo_Sample\\ImageL1%s",".jpg");
				imwrite(address,cap1);
				sprintf(address,"Stereo_Sample\\ImageR1%s",".jpg");
				imwrite(address,cap2);
				get_imagePair=false;
				cout<<"图像对采集完毕！！！！"<<endl;
				break;
			}
		}
		destroyWindow("获取图像对窗口（双击左键）");
		destroyWindow("获取图像对窗口（右图像）");
	}
	Mat grayimgLeft = imread(path1, IMREAD_GRAYSCALE );
    Mat grayimgRight = imread(path2, IMREAD_GRAYSCALE );
	Mat medianBlurLeft,medianBlurRight;
	medianBlur(grayimgLeft,medianBlurLeft,5);
	medianBlur(grayimgRight,medianBlurRight,5);
	Mat gaussianBlurLeft,gaussianBlurRight;
	GaussianBlur(medianBlurLeft,gaussianBlurLeft,Size(5,5),0,0);
	GaussianBlur(medianBlurRight,gaussianBlurRight,Size(5,5),0,0);

	//imshow("ImageL",grayimgLeft);
    //imshow("ImageR",grayimgRight);
    
    //经过remap之后，左右相机的图像已经共面并且行对准了 
	//remap(grayimgLeft, imgLeft, mapx1, mapy1, INTER_LINEAR);  
    //remap(grayimgRight, imgRight, mapx2, mapy2, INTER_LINEAR);
	remap(gaussianBlurLeft, imgLeft, mapx1, mapy1, INTER_LINEAR);  
    remap(gaussianBlurRight, imgRight, mapx2, mapy2, INTER_LINEAR);
	//imshow("ImageL_Rectified",imgLeft);
    //imshow("ImageR_Rectified",imgRight);
}
void show_rectify_performance()
{
	/**********************************************************************************/
    /***************把左右图像的校正结果显示到同一画面上进行对比*********************/ 
    Mat canvas;  
    double sf=0.7;  
    int w, h;   
    w = cvRound(imageSize.width * sf);  
    h = cvRound(imageSize.height * sf);  
    canvas.create(h, w * 2, CV_8UC1);  
    //左图像画到画布上
	//得到画布的左半部分 
    Mat canvasPart = canvas(Rect(w*0, 0, w, h));  
	//把图像缩放到跟canvasPart一样大小并映射到画布canvas的ROI区域中  
    resize(imgLeft, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     
    //右图像画到画布上 
    canvasPart = canvas(Rect(w, 0, w, h)); 
    resize(imgRight, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);  
    //画上对应的线条
    for (int i = 0; i < canvas.rows;i+=16)  
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);  
    imshow("rectified", canvas);
	/**********************************************************************************/
}
bool getDisparityRGBImage(cv::Mat& disparity, cv::Mat& disparityRGBImage, bool isColor)
{
	// 将原始视差数据的位深转换为 8 位
	cv::Mat disp8u;
	if (disparity.depth() != CV_8U)
	{
		disparity.convertTo(disp8u, CV_8U, 255.0/(numDisparities*16));
	} 
	else
	{
		disp8u = disparity;
	}

	// 转换为伪彩色图像 或 灰度图像
	if (isColor)
	{
		if (disparityRGBImage.empty() || disparityRGBImage.type() != CV_8UC3 )
		{
			disparityRGBImage = cv::Mat::zeros(disparity.rows, disparity.cols, CV_8UC3);
		}

		for (int y=0;y<disparity.rows;y++)
		{
			for (int x=0;x<disparity.cols;x++)
			{
				uchar val = disp8u.at<uchar>(y,x);
				uchar r,g,b;

				if (val==0) 
					r = g = b = 0;
				else
				{
					r = 255-val;
					g = val < 128 ? val*2 : (uchar)((255 - val)*2);
					b = val;
				}

				disparityRGBImage.at<cv::Vec3b>(y,x) = cv::Vec3b(r,g,b);
			}
		}
	} 
	else
	{
		disp8u.copyTo(disparityRGBImage);
	}

	return true;
}
void stereo_match_BM(int,void*)
{
	//minDisparity代表了匹配搜索从哪里开始,默认值为 0
	//numberOfDisparities表示最大搜索视差范围
	//举例子，要在右图中搜索左图中的点（x0,y0）的匹配点，则搜索时把右图中的（x0,y0）看做原点
	//搜索方向向左为正，搜索范围为minDisparity-numberOfDisparities。
	//若minDisparity=0，表示从右图中的点（x0,y0）开始向左搜索
	//若minDisparity<0，表示从右图中的点（x0-minDisparity,y0）开始向左搜索
	//blockSize表示匹配窗口大小。窗口越大，匹配结果的鲁棒性越强，但是精度越差，反之
	//其他状态参数暂且使用其默认值
	//opencv3.0版本下，StereoBM对象不能直接通过访问state来访问参数，
	//只能通过setter和getter方法来设置和获取参数

	bm->setUniquenessRatio(UniquenessRatio);//视差唯一性百分比
	bm->setNumDisparities(16*numDisparities+16);
	bm->setBlockSize(2*blockSize+5);
	//bm->setTextureThreshold(10);//低纹理区域的判断阈值
	//bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);//CV_STEREO_BM_NORMALIZED_RESPONSE  CV_STEREO_BM_XSOBEL
	//bm->setPreFilterSize(9);//预处理滤波器窗口大小,一般在[5，21]之间
	//bm->setPreFilterCap(31);//预处理滤波器的截断值,参数范围：1 - 31（文档中是31，但代码中是 63）

	//计算获取视差图
	bm->compute(imgLeft, imgRight, disparity);

	//注意opencv3.0版本下的BM和SGBM方法计算出的视差都是CV_16S格式的
	//参考网上根据源代码内容，得到的原始视差值需要除以16才能得到真实的视差值
	disparity.convertTo(disparity_real, CV_8U, 1.0/16);

	//将得到的视差值范围（minDisparity-（minDisparity+numDisparities））投影到(0-255)
	disparity_real.convertTo(disparity8U, CV_8U, 255.0/numDisparities);
		
	//imshow("disparity",disparity);
	imshow("disparity_real",disparity_real);
	//imshow("disparity_8U",disparity8U);
	//根据isColor的值选择是否进行深度图伪彩色显示
	Mat disparityRGBImage;
	bool isColor=false;
	bool OK=getDisparityRGBImage(disparity,disparityRGBImage,isColor);
	if(OK==true&&isColor==true)
	{
		imshow("disparityRGBImage",disparityRGBImage);
	}

	//-- Check its extreme values
	double minVal; double maxVal;
	minMaxLoc( disparity_real, &minVal, &maxVal );

	cout<<"Min disp:"<< minVal<<endl;
	cout<<"Max value:"<<maxVal<<endl;
}

int main(int argc, char* argv[])
{
	/***********************************************************************************/
	/*****************************从文件中读取相机标定结果******************************/
	const string filename="F:\\Binocular_Stereo_Vision_Test\\Calibration_Result.xml";
	bool readOK=readFile(filename);
	if(!readOK)
	{
		cerr<<"failed to readfile!"<<endl;
		return -1;
	}
	/**********************************************************************************/
	/********************************立体校正******************************************/
	bool useCalibrated=true; 
	//int getImagePair=IMAGELIST;
	int getImagePair=CAMERA;
	string path1="Stereo_Sample\\ImageL1.jpg";
	string path2="Stereo_Sample\\ImageR1.jpg";

	stereo_rectify(useCalibrated,getImagePair,path1,path2);
	if(getImagePair==CAMERA&&isopen==false)
	{
		cerr<<"failed to open camera to get imagepair!"<<endl;
		waitKey(5000);
		return -1;
	}
    //把左右图像的校正结果显示到同一画面上进行对比
    bool showRectifyPerformance=true;
	if(showRectifyPerformance)
	{
		show_rectify_performance();
	}

	/**********************************************************************************/
	/********************************立体匹配******************************************/
	//通过设置滑动条可以观察三个主要参数的变化对匹配效果的影响
	namedWindow("disparity_real", WINDOW_AUTOSIZE);
	bm->compute(imgLeft, imgRight, disparity);
	//imshow("disparity_real",disparity);
    createTrackbar("blockSize:", "disparity_real",&blockSize,8, stereo_match_BM);
    createTrackbar("UniquenessRatio:", "disparity_real", &UniquenessRatio,15, stereo_match_BM);
    createTrackbar("numDisparities:", "disparity_real", &numDisparities,25, stereo_match_BM);
	

	/**********************************************************************************/
	/********************************三维重建******************************************/
	//设置鼠标回调函数on_mouse()用于输出鼠标指定点（单击左键）的视差值和三维XYZ值（单位：厘米）
	cvSetMouseCallback("disparity_real",on_mouse,NULL);
	while(1)
	{
		if(mousePoint.area()>0)
		{
			//将鼠标指定过的点用圆圈标记出来
			circle(disparity_real,mousePoint,10,Scalar(255,255,255),3);
			imshow("disparity_real",disparity_real);

			uchar *data=disparity_real.ptr<uchar>(mousePoint.height);
			cout<<"disparity="<<(double)data[mousePoint.width]<<endl;
			//cout<<"disparity="<<disparity.at<double>(mousePoint)<<endl;
			_3dImage=Mat( disparity.rows, disparity.cols, CV_32FC3);
			reprojectImageTo3D(disparity,_3dImage,Q,true);
			//在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16
			//才能得到正确的三维坐标信息(以毫米为单位),要得到以厘米为单位的三维坐标则乘以1.6
			_3dImage=_3dImage*1.6;//得到以厘米为单位的三维坐标
			//imshow("三维图",_3dImage);
			//ReprojectTo3D()函数出来的Y坐标方向和实际相反，需要处理一下
			for (int y = 0; y < _3dImage.rows; ++y)
			{
				for (int x = 0; x < _3dImage.cols; ++x)
				{
					Point3f point = _3dImage.at<Point3f>(y,x);
					point.y = -point.y;
					_3dImage.at<Point3f>(y,x) = point;
				}
			}
			X=_3dImage.at<Vec3f>(mousePoint)[0];
			Y=_3dImage.at<Vec3f>(mousePoint)[1];
			Z=_3dImage.at<Vec3f>(mousePoint)[2];
			cout<<"X="<<X<<'\t'<<"Y="<<Y<<'\t'<<"Z="<<Z<<endl;
			mousePoint.width=0;
			mousePoint.height=0;
		}
		if((char)waitKey(10)==27)
			break;
	}
	/*
	vector<Mat> xyz;
	split(_3dImage,xyz);
	double minVal; double maxVal;
	minMaxLoc( xyz[0], &minVal, &maxVal );
	xyz[0].convertTo(xyz[0],CV_8UC1,255/(maxVal - minVal));
	minMaxLoc( xyz[1], &minVal, &maxVal );
	xyz[1].convertTo(xyz[1],CV_8UC1,255/(maxVal - minVal));
	minMaxLoc( xyz[2], &minVal, &maxVal );
	xyz[2].convertTo(xyz[2],CV_8UC1,255/(maxVal - minVal));
	//Mat _3dImage1=Mat( disparity.rows, disparity.cols, CV_8UC3 );
	//merge(xyz,_3dImage);
	imshow("xImage",xyz[0]);
	imshow("yImage",xyz[1]);
	imshow("zImage",xyz[2]);
	//imshow("_3dImage",_3dImage);
	*/
	cout<<"按任意键退出程序..."<<endl;
	waitKey(0);
	return 0;
}