#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_costmaps");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("clear_costmaps");
  std_srvs::Empty srv;
  //srv.request.a = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);

  ros::Rate r(1);   //10s costmap清空一次  0.1

  while(n.ok())
  {
	  if (client.call(srv))
	  {
	    ROS_INFO("call clear_costmaps service successfully");
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service clear_costmaps");
	    //return 1;
	  }
          r.sleep();
  }

  return 0;
}
