#include "ros/ros.h"
//#include "hector_worldmodel_msgs/AddObject.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include "std_srvs/Empty.h"
#include <nav_msgs/Odometry.h>
#include "hector_worldmodel_msgs/AddObject.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

float thermal_x = 0.0f , thermal_y = 0.0f , last_thermal_x = 0.0f, last_thermal_y = 0.0f;
ros::ServiceClient add_thermal_client;
ros::ServiceClient thermal_path_client;
ros::Publisher thermal_pose;

hector_worldmodel_msgs::AddObject add_srv;
//string thermal_name;
int num =1;
bool new_thermal = false;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    thermal_x = scan->ranges[205]+0.37;  //0.37是激光到本体的距离
}

bool findthermal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	new_thermal = true;	
	static int seq = 0;        
//	thermal_name += num;
	add_srv.request.object.header.seq = seq;
    	add_srv.request.object.header.frame_id = "base_link";
        add_srv.request.object.header.stamp = ros::Time::now();
        add_srv.request.map_to_next_obstacle = true;
        add_srv.request.object.info.class_id = "thermal";
        add_srv.request.object.info.name = "thermal";
        add_srv.request.object.info.object_id = num;
        add_srv.request.object.info.support = 100;
        add_srv.request.object.pose.pose.position.x = 0.1;
        add_srv.request.object.pose.pose.position.y = 0;
        add_srv.request.object.pose.pose.position.z = 0;
        add_srv.request.object.pose.pose.orientation.w = 1;
        add_srv.request.object.state.state = 1;//hector_worldmodel_msgs::ObjectState=PENDING;

   	geometry_msgs::PoseStamped thermal_position;
	thermal_position.header.seq = seq;
    	thermal_position.header.frame_id = "base_link";
        thermal_position.header.stamp = ros::Time::now();
	thermal_position.pose.position.x = last_thermal_x;
	thermal_position.pose.position.y = last_thermal_y;
	thermal_position.pose.position.z = 0;
	thermal_position.pose.orientation.w = 1;
	   thermal_pose.publish(thermal_position);
	if(((thermal_x-last_thermal_x)*(thermal_x-last_thermal_x)+(thermal_y-last_thermal_y)*(thermal_y-last_thermal_y)) < 1.0)
	{
		new_thermal = false;
        }else
	{
		ROS_INFO("find a new thermal!");	
	}
        if(new_thermal && add_thermal_client.call(add_srv))
        {
           ROS_INFO("add a thermal successfully!");

	   std_srvs::Empty change;
	   if(thermal_path_client.call(change)) ROS_INFO("change to thermal path successfully!");
	   last_thermal_x = thermal_x;
	   last_thermal_y = thermal_y;
	   ++num;
	   ++seq;
        }
        else
        {
		if(!new_thermal)
		{
			ROS_INFO("this thermal is already detect!");
		}           
		else
		{
			ROS_INFO("fail to add a thermal!");
		}

        }
	
	return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odometry)
{
    thermal_y = 0; //0.17是相对与本体右侧的偏移
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermal_find");

    ros::NodeHandle n;
    ros::Subscriber scan_sub = n.subscribe("scan" , 5 , scanCallback);
    ros::Subscriber odom_sub = n.subscribe("odom" , 1000 ,odomCallback);
    thermal_pose = n.advertise<geometry_msgs::PoseStamped>("thermal_pose",5); 
    
    ros::ServiceServer find_thermal_server = n.advertiseService("find_thermal" , findthermal);
    add_thermal_client =  n.serviceClient<hector_worldmodel_msgs::AddObject>("worldmodel/add_object");
    thermal_path_client = n.serviceClient<std_srvs::Empty>("thermal_path");

  ros::spin();

  return 0;
}
