#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
#include "std_srvs/Empty.h"
using namespace std;

#include <Connect.h>

Connection* m_connect = new Connection;
//Connection* m_connect;
bool pause_flag = false;
//int cnt = 0;


bool pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("pause service triggered/n");
	//pause_flag = !pause_flag;
	pause_flag = true;
	//cout<<pause_flag<<endl;
	return true;
}

bool continueCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("continue");
	pause_flag = false;

	return true;
}


void receiveCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
/*	if(pause_flag)
	{
		cnt++;
		if(cnt > 300)                           //暂停半分钟
		{
			cnt = 0;
			pause_flag = false;

		}
	}
*/		
	if(!pause_flag)
	{
		float vx = twist->linear.x;
		float vy = twist->linear.y;
		float vth = twist->angular.z;
		cout<<vx<<"   "<<vy<<"   "<<vth<<endl;

		float delta_x = vx * 0.1;
		float delta_y = vy * 0.1;
		float delta_th = vth * 0.1;

		//float dist = sqrt(delta_x * delta_x + delta_y * delta_y);
		float dist = delta_x;
		float width = 30.0 / 100.0;    //30.0

		float l1 = (2 * dist - width * delta_th) / 2;
		float l2 = (2 * dist + width * delta_th) / 2;

		int vel1 , vel2;

		//int vel1 = (int)(l1 *5000);
		//int vel2 = (int)(l2 *5000);    //  mm/s

		vel1 = (int)(l1 * 20000);       //         22000  
		vel2 = (int)(l2 * 20000);

//		if(fabs(vx) > fabs(vth))
//		{
//			vel1 = (int)(2500 * vx);     //(int)(2500 * vx);      //velocity to command
//			vel2 = (int)(2500 * vx);
//		}
//		else
//		{
//			vel1 = -350 * vth;                    //-350 * vth;
//			vel2 = 350 * vth;

//		}

/*		if(vx > 0.0 && vth > 0.0)
		{
			vel1 = (int)(vx * 2500);
			vel2 = (int)((vx + width * vth) * 2500);
		}
		else if(vx > 0.0 && vth <= 0.0)
		{
			vel1 = (int)((vx - width * vth) * 2500);
			vel2 = (int)(vx * 2500); 
		}
		else if(vx < 0.0 && vth > 0.0)
		{
			vel1 = (int)(vx * 2500);
			vel2 = (int)((vx - width * vth) * 2500);
		}
		else if(vx < 0.0 && vth <= 0.0)
		{
			vel1 = (int)((vx + width * vth) * 2500);
			vel2 = (int)(vx * 2500);
		}
		else
		{
			vel1 = -300 * vth;                    //-350 * vth;
			vel2 = 300 * vth;
		}
*/

		//vel1 = (int)((1000 - 300)/(200 - 80) * (vel1 - 80)) + 300;
		//vel2 = (int)((1000 - 300)/(200 - 80) * (vel2 - 80)) + 300;

		ROS_INFO("vel: left:[%f]  right:[%f]",l1,l2);

		//int vel1 = (int)(28000 * l1 -125); 
		//int vel2 = (int)(28000 * l2 -125); 	
	
		vel1 = (vel1 > 500) ? 500 : vel1;
		vel2 = (vel2 > 500) ? 500 : vel2;

		vel1 = (vel1 < -500) ? -500 : vel1;
		vel2 = (vel2 < -500) ? -500 : vel2;

		ROS_INFO("vel: left:[%d]  right:[%d]",vel1,vel2);

		char c[200];
		sprintf(c,"DRIVE {Type Direct} {Name Velocity} {Left %d} {Right %d} {Normalized 1}\r\n",vel1,vel2);
		string sc(c);
		m_connect -> sendMsg(sc);
	}
} 

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"base_controller");

	ros::NodeHandle n;

	//m_connect = new Connection;
	if(m_connect -> Oninit("192.168.1.116",10000))
	{
		cout<<"connect successfully"<<endl;
		m_connect -> sendMsg("REG {Mode WR} {Period 100}\r\n");
	}
	else
	{
		cout<<"connect unsuccessfully"<<endl;	
	}
	
	ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("cmd_vel",10,receiveCallback);	
        ros::ServiceServer service = n.advertiseService("pause" , pause);

	ros::ServiceServer continue_server = n.advertiseService("continue" , continueCallback);
	

	ros::Rate r(10);
	while(n.ok())
	{
		m_connect -> recvMsg();
		//cout<<m_connect -> rcvbuffer <<endl<<endl;
		r.sleep();
		ros::spinOnce();
	}
	
	return 0;

}

