#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <vector>
/**
 * Optris device interface
 */
#include "PIImager.h"

/**
 * Optris image converter
 */
#include "ImageBuilder.h"

#include <iostream>
using namespace std;
using namespace optris;

#define WIDTH    160
#define HEIGHT   120
const double min_tmp = 30;
const double max_tmp = 40;

unsigned char* _bufferThermal = NULL;
optris::PIImager* _imager;
optris::ImageBuilder* _iBuilder;
image_transport::Publisher*  thermal_pub;
    cv::Mat cv_img_bin = cv::Mat::zeros(WIDTH, HEIGHT, CV_8UC1);

void cbOnThermalFrame(unsigned short* image, unsigned int w, unsigned int h)
{
    _iBuilder->setData(w, h, image);
    if(_bufferThermal==NULL)
      _bufferThermal = new unsigned char[w * h*3];
    _iBuilder->convertTemperatureToPaletteImage(_bufferThermal);


    unsigned char* p_cv_bin = cv_img_bin.data;

    for(unsigned int i=0; i<w * h; i++)
    {       
	double temp = _iBuilder->getTemperatureAt(i);
	if(temp >= min_tmp && temp <= max_tmp) *p_cv_bin = 0xff;
	else *p_cv_bin = 0;
	++p_cv_bin;
     }	
//    memcpy(_bufferThermal, image, w*h*3);

    static int frame = 0;

  sensor_msgs::Image img_binary;
  img_binary.header.frame_id = "thermal_binary";
  img_binary.height 	       = h;
  img_binary.width 	       = w;
  img_binary.encoding        = "mono8";
  img_binary.step            = w;
  img_binary.data.resize(img_binary.height*img_binary.step);
  img_binary.header.seq      = ++frame;
  img_binary.header.stamp    = ros::Time::now();
memcpy(&img_binary.data[0], cv_img_bin.data, 160*120);
thermal_pub->publish(img_binary);

    if((frame++)%80==0)
    {
      cout << "Frame rate: " << _imager->getAverageFramerate() << " fps" << endl;

/*    cout << "data:" ;
    for(unsigned int i=0; i<w * h; i++)
  {
         
	double temp = _iBuilder->getTemperatureAt(i);
	cout << temp << " ";
  }
 	cout << endl;  */

    }
}

int main (int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <xml configuration file>" << endl;
    return -1;
  }

	ros::init (argc, argv, "thermal_image_raw_node");

  // private node handle to support command line parameters for rosrun
        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
	image_transport::Publisher  _pubThermal = it.advertise("thermal_image_raw", 1);

    	image_transport::Publisher  pubThermal_bin = it.advertise("thermal_image_bin", 1);
thermal_pub = &pubThermal_bin;
  /**
   * Initialize Optris image processing chain
   */
  _imager = new PIImager(argv[1]);
  if(_imager->getWidth()==0 || _imager->getHeight()==0)
  {
    cout << "Error: Image streams not available or wrongly configured. Check connection of camera and config file." << endl;
    return -1;
  }

  cout << "Thermal channel: " << _imager->getWidth() << "x" << _imager->getHeight() << "@" << _imager->getFramerate() << "Hz" << endl;

  unsigned char* bufferRaw = new unsigned char[_imager->getRawBufferSize()];

  _imager->setFrameCallback(cbOnThermalFrame);

  _iBuilder = new ImageBuilder();
  _iBuilder->setPaletteScalingMethod(eManual);
  _iBuilder->setPalette(eGrayBW);
  _iBuilder->setManualTemperatureRange(25.0f, 40.0f);
  _imager->startStreaming();
  /**
   * Enter endless loop in order to pass raw data to Optris image processing library.
   * Processed data are supported by the frame callback function.
   */
	unsigned int   _frame = 0;
	sensor_msgs::Image img;
  	img.header.frame_id = "thermal_image_raw";
  	img.height  = HEIGHT;
  	img.width   = WIDTH;
  	img.encoding        = "rgb8";
  	img.step            = WIDTH*3;
  	img.data.resize(img.height*img.step);
std::vector<cv::KeyPoint> keypoints;

   while(ros::ok())   
  {
    _imager->getFrame(bufferRaw);
    _imager->process(bufferRaw);

    if(_bufferThermal != NULL)
    {	    
//       	if(_pubThermal.getNumSubscribers() == 0)
 //    			continue;
		img.header.seq      = ++_frame;
  		img.header.stamp    = ros::Time::now();
		memcpy(&img.data[0], _bufferThermal, 160*120*3);
cv_bridge::CvImagePtr cv_ptr;
     cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
     cv::Mat img_processed(cv_ptr->image);
	cv::cvtColor(img_processed, img_processed, CV_RGB2GRAY);
	cv::SimpleBlobDetector::Params params;
   params.filterByColor = true;
   params.blobColor = 255;
   params.minDistBetweenBlobs = 20;
   params.filterByArea = true;
   params.minArea = 10;
   params.maxArea = 160*120;
   params.filterByCircularity = false;

   params.filterByConvexity = false;
   params.filterByInertia = false;

   keypoints.clear();
   cv::SimpleBlobDetector blob_detector(params);
   blob_detector.create("SimpleBlob");   //以前这个没加
  
   blob_detector.detect(cv_img_bin,keypoints);
cv::drawKeypoints(img_processed, keypoints, img_processed, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

	_pubThermal.publish(img);
    }	 
//    usleep(16000);  //大概55hz

    _imager->releaseFrame();
  }

  delete [] bufferRaw;
  if(_bufferThermal) delete [] _bufferThermal;

  delete _imager;
  delete _iBuilder;

  cout << "Exiting application" << endl;
}
