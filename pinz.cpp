#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <float.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <focus/frcam.h>
#include <Aria.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image window";
 
class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  ros::Publisher pub ;
  ros::Publisher tog;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;
public:
  ImageConverter()
    : it_(nh_)
  {
 
 
      image_sub_ = it_.subscribe("/eagle_camera/disp/raw", 1, &ImageConverter::imageCb, this);
      image_pub_= it_.advertise("/bspychalski/Image",1);
 
 
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    int i,j;
    
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    //sensor_msgs::CvBridge bridge;
    //we need this object bridge for facilitating conversion from ros-img to opencv
    
    //IplImage* img = bridge.imgMsgToCv(msg,"rgb8");
    //image being converted from ros to opencv using cvbridge
    IplImage imgObj = cv_ptr->image;
    IplImage* img = &imgObj;

    IplImage* gray_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );

    cvCvtColor(img , gray_out, CV_RGB2GRAY);

     int width = 752;
     int height = 480;
     int MaxVal[width][3],a,b;
     int maks;
     int wart;
     double X,Y,ZM;


	float focalLength = 748.821716;
	float baseline = 0.060234;

	float distance;
	int   disparity;
	float numerator = focalLength * baseline;

IplImage* mapa = cvCreateImage( cvSize(600, 600), IPL_DEPTH_8U, 1 ); 
	cvCircle(mapa, cvPoint(300,600), 200, cvScalar(50),1,8,0);
	cvCircle(mapa, cvPoint(300,600), 400, cvScalar(50),1,8,0);
	cvCircle(mapa, cvPoint(300,600), 150, cvScalar(50),1,8,0);
	cvCircle(mapa, cvPoint(300,600), 100, cvScalar(50),1,8,0);

//Znajdowanie najjasniejszych punktow w kolumnach i zapisywanie ich do wektora MaxVal[]



     for(i=0;i<752;i++)
    {
     maks = 0;
	for(j=60;j<440;j++)
	{
     	wart = (int)cvGetReal2D( gray_out, j, i );
	if(wart >= maks)
       {
     	maks=wart;
     	MaxVal[i][0]=maks;
     	MaxVal[i][1]=i;
     	MaxVal[i][2]=j;
        }
	}

//filtry

if ((MaxVal[i-2][0]==0 && MaxVal[i][0]==0 && MaxVal[i-1][0]!=0)  )
{
MaxVal[i-1][0]=0;
}
if ( abs(MaxVal[i-3][0]-MaxVal[i-2][0])>40 && abs(MaxVal[i][0]-MaxVal[i-1][0])>40  )
{
MaxVal[i-1][0]=(MaxVal[i][0]+MaxVal[i-1][0])/10;
MaxVal[i-2][0]=(MaxVal[i-3][0]+MaxVal[i-2][0])/10;
}
if ( abs(MaxVal[i-2][0]-MaxVal[i-1][0])>40 && abs(MaxVal[i][0]-MaxVal[i-1][0])>40  )
{
MaxVal[i-1][0]=(MaxVal[i-1][0]+MaxVal[i-2][0])/2;
}
}


//Przeliczenie odleglosci
     for(i=0;i<752;i++)
    {
distance = 100;
disparity = MaxVal[i][0];
if(disparity != 0)
{
distance = (numerator*255) / (disparity*64);
}
//      printf("MaksVal %d %d =%d\t odleglosc %lf\n", i, MaxVal[i][2], MaxVal[i][0], distance);

if(distance < 2.5)
{
	X=(i-752/2)*0.006;
	ZM=pow(3*3+X*X,0.5);
	X=300+(X*distance*1000/(ZM*5));
	Y=600-(3*distance*1000/(ZM*5));

	cvSet2D(mapa, (int) Y, (int) X,cvScalar(255));
}


    } 
  // printf("MaksVal %d %d =%d\n", MaxVal[0][1], MaxVal[0][2], MaxVal[0][0]);
/*  cvNamedWindow("mapa", CV_WINDOW_AUTOSIZE); 
  cvMoveWindow("mapa", 100, 100);
  cvShowImage("mapa",mapa);*/
img=mapa;
   // cvShowImage( "Original image", gray_out);
    
    cv::waitKey(3);
    
    // convert to ros msg
    //sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(img, "rgb8");
    // image converted from opencv to ros for publishing
    //image_pub_.publish(out);

  cv_ptr->image = mapa;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;

    image_pub_.publish(cv_ptr->toImageMsg());
//sleep(1);
    //*/
  }
  
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::spin();



}
