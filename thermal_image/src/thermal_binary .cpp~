
#include "ros/ros.h"
#include <image_transport/image_transport.h>

// #include "ImageBuilder.h"

image_transport::Publisher*  thermal_binary_pub;
double _min_tmp;
double _max_tmp;


/**
 * Callback function for receiving thermal data from thermal imager node
 * @param image      image containing raw temperature data
 */
void onThermalRawDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  static unsigned int frame = 0;

  unsigned short* data = (unsigned short*)&image->data[0];

  sensor_msgs::Image img_binary;
  img_binary.header.frame_id = "thermal_binary";
  img_binary.height 	       = image->height;
  img_binary.width 	       = image->width;
  img_binary.encoding        = "mono8";
  img_binary.step            = image->width;
  img_binary.data.resize(img_binary.height*img_binary.step);
  img_binary.header.seq      = ++frame;
  img_binary.header.stamp    = ros::Time::now();

  // generate binary image from thermal data
  for(unsigned int i=0; i<image->width*image->height; i++)
  {
     const double temp = (float(data[i]) -1000.0f)/10.0f;


        if(temp >= _min_tmp && temp <= _max_tmp) img_binary.data[i] = 0xff;
        else                  img_binary.data[i] = 0x00;

  }

  thermal_binary_pub->publish(img_binary);
}



/**
 * Main routine of optris_binary_image_node.cpp
 * @param argc
 * @param argv
 * @return
 *
 * Usage:
 * rosrun optris_drivers optris_binary_image_node _threshold=20 _invert:=false
 */
int main (int argc, char* argv[])
{
  ros::init (argc, argv, "optris_binary_image_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  // parameters for initialization
  n_.param("min_tmp", _min_tmp, 35.0);
  n_.param("max_tmp", _max_tmp, 40.0);


  // init subscribers and publishers
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("thermal_image_raw",  1, onThermalRawDataReceive);
  image_transport::Publisher pubt        = it.advertise("thermal_binary", 1);
  thermal_binary_pub = &pubt;

  ros::spin();

}
