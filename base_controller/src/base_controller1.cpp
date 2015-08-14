#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
using namespace std;

#include <Connect.h>

Connection* m_connect = new Connection;
bool isTurnBack = false;
int vel1 = 0 , vel2 = 0 , ct = 0;

void receiveCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	float vx = twist->linear.x;
	float vy = twist->linear.y;
	float vth = twist->angular.z;
	cout<<vx<<"   "<<vy<<"   "<<vth<<endl;

	float delta_x = vx * 0.1;
	float delta_y = vy * 0.1;
	float delta_th = vth * 0.1;

	float dist = sqrt(delta_x * delta_x + delta_y * delta_y);
	float width = 50.0 / 100.0;

	float l1 = (2 * dist - width * delta_th) / 2;
	float l2 = (2 * dist + width * delta_th) / 2;

	//int vel1 = (int)(l1 * 20000);
	//int vel2 = (int)(l2 * 20000);

	if(!isTurnBack && ct == 0)
	{
		if(fabs(vx) > fabs(vth))
		{
			vel1 = (int)(2500 * vx);      //velocity to command
			vel2 = (int)(2500 * vx);
		}
		else
		{
			vel1 = -350 * vth;
			vel2 = 350 * vth;

		}
	}
	else if(isTurnBack)
	{
		ct++;
	}

	if(ct >= 10 )
	{
		ct =0;
		isTurnBack =false;
	}

	if(vx < -0.1)
	{
		isTurnBack = true;

	}
	
	//ROS_INFO("vel: left:[%f]  right:[%f]",l1,l2);		
	
	vel1 = (vel1 > 1000) ? 1000 : vel1;
	vel2 = (vel2 > 1000) ? 1000 : vel2;

	vel1 = (vel1 < -1000) ? -1000 : vel1;
	vel2 = (vel2 < -1000) ? -1000 : vel2;

	ROS_INFO("vel: left:[%d]  right:[%d]",vel1,vel2);

	char c[200];
	sprintf(c,"DRIVE {Type Direct} {Name Velocity} {Left %d} {Right %d} {Normalized 1}\r\n",vel1,vel2);
	string sc(c);
	m_connect -> sendMsg(sc);
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
	
	ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("cmd_vel",100,receiveCallback);	

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

