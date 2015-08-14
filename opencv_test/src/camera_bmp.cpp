#include "image_converter.h"
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/lexical_cast.hpp>
int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_bmp");
	ros::NodeHandle nh;
	ImageConverter kinect_image(nh);
	kinect_image.set_delay_time(500);

	long file_num = 1;
	std::string directory = "/home/peng/bmp";

	cv::Mat image_mat;
	while(ros::ok())
	{
		image_mat = kinect_image.get_image(); 
		if(!image_mat.empty())
		{
			std::string fullpath = directory + "/" + boost::lexical_cast<std::string>(file_num) + ".bmp"; 
			cv::imwrite(fullpath, image_mat);	
			++file_num;
		}
		ros::spinOnce();
	}

       return 0;
}

