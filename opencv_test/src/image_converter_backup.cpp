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
public:
	ImageConverter(ros::NodeHandle &nh)
		:it_(nh)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&ImageConverter::imageCb,this);
	}

	~ImageConverter()
	{

	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
		cv::cvtColor(cv_ptr->image, image_rgb, CV_BGR2RGB);
		cv::waitKey(50);
		}
	}

};
