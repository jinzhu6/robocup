#include "image_converter.h"

ImageConverter::~ImageConverter()
{

}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge input error: %s", e.what());
		return;
	}
	//cv::cvtColor(cv_ptr->image, image_rgb, CV_BGR2RGB);
	image_rgb = cv_ptr->image;
	cv::waitKey(time_delay);
}

cv::Mat ImageConverter::get_image()
{
	cv::Mat new_image = image_rgb;
	return new_image;
}

void ImageConverter::set_delay_time(const size_t num)
{
	time_delay = num;	
}
