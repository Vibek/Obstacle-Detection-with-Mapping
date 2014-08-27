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

    double kat;
    double odleglosc;

  ImageConverter()
    : it_(nh_)
  {
 
 
      image_sub_ = it_.subscribe("/vibek/Image", 1, &ImageConverter::imageCb, this);
 
 
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    int i,j;
    int wart, trig1=0;
    int MaxVal[600];
    
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
      }
    catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    IplImage imgObj = cv_ptr->image;
    IplImage* img = &imgObj;

int suml=0, sump=0, max=0;
//
kat = 0.;
odleglosc = 0.;
max=0;
for (i=220;i<381;i++)
{
	MaxVal[i]= 0;
	for(j=400;j<500;j++)
		{

			     	wart = (int)cvGetReal2D( img, j, i );
				if(wart > 100)
					{
						MaxVal[i]= j;
						trig1 = 1.;
					}
		}
if (max<MaxVal[i])
{
max=MaxVal[i];
}
if(i < 300)
{
suml=suml + (600 - MaxVal[i]);
}
if(i > 300)
{
sump=sump + (600 - MaxVal[i]);
}

}
//

if (trig1 == 1.)
{
	if(suml>sump)
	{
	printf("skrecam w lewo\n");
	suml = 0;
	sump = 0;
	kat= 15.;
	}

	if(suml<sump)
	{
	printf("skrecam w prawo\n");
	suml = 0;
	sump = 0;
	kat= -15.;

	}
}

if (trig1 == 0)
{
kat = 0.;
max=0;

for (i=150;i<451;i++)
{
	MaxVal[i]= 0;
	for(j=150;j<401;j++)
		{

			     	wart = (int)cvGetReal2D( img, j, i );
				if(wart > 100)
					{
						MaxVal[i]= j;

					}
		}
if (max < MaxVal[i])
{
max=MaxVal[i];
}
}
printf("naprzod\n");
odleglosc = (600 - max)*5/3;
}

if (max > 460)
{
odleglosc = - 100;
}

  cvNamedWindow("mapa2", CV_WINDOW_AUTOSIZE); 
  cvMoveWindow("mapa2", 400, 100);
  cvShowImage("mapa2",img);
   
    cv::waitKey(3);

usleep(1200);

  }
  
};



int main(int argc, char** argv)
{

double kat, odleglosc;
int trg=0;
ros::init(argc, argv, "pinz_move");
ros::NodeHandle nh("~");
Aria::init(); 
	ArRobot robot; 
	ArArgumentParser parser(&argc, argv); 
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  	parser.addDefaultArgument("-connectLaser");
  	if(!robotConnector.connectRobot())
  	{
    		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    		if(parser.checkHelpAndWarnUnparsed())
    		{
 
        	Aria::logOptions();
        	Aria::exit(1);
    		}
  	}


	if (!robotConnector.connectRobot(&robot))
	{
		printf("Unable to connect\n"); 
		Aria::exit(1);
	}

	robot.runAsync(true);

 	laserConnector.connectLasers();
 	ArUtil::sleep(400);   

  	robot.lock();
	robot.setTransVelMax(300);		// ustalenie maksymalnej predkosci liniowej robota (mm/s) 
  	robot.setTransDecel(500); 		// ustalenie maksymalnego opoznienia liniowego, z jakim robot bedzie hamowal (mm/s^2)
  	robot.setTransAccel(200);		// ustalenie maksymalnego pszyspieszenia liniowego robota (mm/s^2)
  	robot.setRotVelMax(20);		// ustalenie maksymalnej predkosci obrotowej robota (stopnie/s)
  	robot.setRotAccel(10);		// ustalenie maksymalnego pszyspieszenia katowego robota (stopnie/s^2)

  	robot.unlock();




/*	robot.lock();
    robot.setRotVel(20);
 	robot.unlock();
 	ArUtil::sleep(1000); */

  ros::Rate loop_rate(30);
    ImageConverter ic;
sleep(10);
  while (nh.ok()) {
kat=ic.kat;
odleglosc=ic.odleglosc;
if (kat == 15. || kat == -15.)
{
trg++;
}
if (kat ==0.)
{
trg = 0;
}
if (trg == 4)
{
kat=60.;
trg = 0;
}
if (robot.checkRangeDevicesCumulativePolar(-45, 45) < 950)
{
odleglosc = -200;
}
printf("laser: %lf\n", robot.checkRangeDevicesCumulativePolar(-45, 45));
	robot.lock();
	robot.setDeltaHeading(kat);
	robot.move(odleglosc);
 	robot.unlock();
 	ArUtil::sleep(1000);
    ros::spinOnce();
printf("kat= %lf\t odleglosc= %lf\t trg= %d\n",kat, odleglosc, trg);
    loop_rate.sleep();
  }




}
