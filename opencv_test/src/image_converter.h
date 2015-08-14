#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
class ImageConverter
{
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	cv::Mat image_rgb;
	size_t time_delay;
public:
	ImageConverter(ros::NodeHandle &nh)
		:it_(nh)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color",1,&ImageConverter::imageCb,this);
	}

	~ImageConverter();
	void imageCb(const sensor_msgs::ImageConstPtr& msg);

	cv::Mat get_image();
	void set_delay_time(const size_t num = 10);
};

#endif
