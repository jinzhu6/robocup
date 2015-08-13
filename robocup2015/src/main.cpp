#include <ros/ros.h>
#include <Connect.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robocup2014/Int2.h>
//#include <robocup2014/Track.h>
using namespace std;

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"robocup2014");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  	tf::TransformBroadcaster odom_broadcaster;
	
        //tf::TransformBroadcaster base_broadcaster;

	//ros::Publisher track_pub = n.advertise<robocup2014::Track>("track" , 50);
	ros::Publisher sensor_pub = n.advertise<robocup2014::Int2>("sensor" , 50);

	Connection* m_connect;
	m_connect = new Connection;

	if(m_connect -> Oninit("192.168.1.116",10000))
	{
		cout<<"connect successfully"<<endl;
		m_connect ->sendMsg("REG {Mode WR} {Period 100}\r\n");
	}
	else
	{
		cout<<"connect error"<<endl;	
	}

	char* rcvbuffer;
	rcvbuffer = new char[4096 * 5]; 

	unsigned int num_readings = 511;//682;
	double laser_frequency = 10;
	double ranges[682];
	double intensities[num_readings];


      	double pose[3];
        double b_pose = 0.0;

  	double x = 0.0,b_x=0.0;
  	double y = 0.0,b_y=0.0;
  	double th = 0.0 , b_th = 0.0;
  	double roll = 0.0, pitch = 0.0;

  	double trackL = 0.0 , trackR = 0.0 , b_trackL = 0.0 , b_trackR = 0.0;
  	double distL = 0.0 , distR = 0.0 ;
  
  	double vx = 0.0;
  	double vy = 0.0;
  	double vth = 0.0;

	string b_str;
	string sub_str;

  	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();

  	long count=0;

	int p1 = 0 , p2 = 0;	
	int heat =0 ,CO2 =0;

	ros::Rate r(10.0);
	while(n.ok())
        {
		current_time = ros::Time::now();
		//ros::spinOnce();

		m_connect -> recvMsg();
		bzero(rcvbuffer , 4096*5);			
		memmove(rcvbuffer , m_connect -> rcvbuffer , m_connect -> recv_size);
		string str(rcvbuffer);
		str = b_str + str;
		//cout<<str<<endl<<endl<<endl;

		if((p1 = str.find("STA ")) < 0)
		{
			b_str = str;
			continue;                       //没有找到帧头，结束该周期数据采集
		}
		else
		{
			p2 = str.find("SYNC " , p1);
			if(p2 < 0)                      //没有找到帧尾
			{
				b_str = str;
				continue;                   
			}
			else                            //数据帧完整 , sub_str存储
			{
				sub_str = str.substr( 0 ,p2);
				b_str = str.substr(p2 , str.length() - p2);
				//ofile << sub_str <<endl<<endl<<endl;	
			}
		}

		//cout<<sub_str<<endl<<endl;

		p1=sub_str.find("{Range");         //获得帧中的激光数据
        	p2;  //？？什么意思
        	string temp;

		if(p1<0)
		{
			continue;
//		}
		else
		{
			int i;
			p1+=7;
			for(i=0;i<681;i++)	// 读取激光数据
			{
				p2=sub_str.find(",",p1);
				temp=sub_str.substr(p1,p2 - p1);
				ranges[i]=atof(temp.c_str())/1000.0;
				ranges[i]=(ranges[i]==0)?5.0:ranges[i];
				ranges[i]=(ranges[i] > 5.0)?5.0:ranges[i];
				p1=p2+1;
				//cout<<ranges[i]<<",";
			}
			p2=sub_str.find("}",p1);  //最后一个数据需要特殊处理
			temp=sub_str.substr(p1,p2 - p1);
			ranges[681]=atof(temp.c_str())/1000.0;
			ranges[681]=(ranges[681]==0)?5.0:ranges[681];
			ranges[681]=(ranges[681] > 5.0)?5.0:ranges[681];
		}

		p1 = sub_str.find("{Type Track} {Name Position}");  //获得左右履带编码器的位移
		if(p1 < 0)
		{
			continue;
		}
		else
		{
			p1 = sub_str.find("{Left" , p1);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 + 6 , p2 - p1 -6);
			trackL = atof(temp.c_str())/1000.0;

			p1 = sub_str.find("{Right" , p2);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 + 6 , p2 - p1 -6);
			trackR = atof(temp.c_str())/1000.0;
		}

		p1 = sub_str.find("{Orientation" , p2);  //获得姿航仪的数据
		if(p1 > 0)
		{
			p1 += 13;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			roll = atof(temp.c_str())/10.0*3.1415926/180.0;

			p1 = p2 + 1;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			pitch = atof(temp.c_str())/10.0*3.1415926/180.0;

			p1 = p2 + 1;
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			pose[2] = atof(temp.c_str())/10.0*3.1415926/180.0;

		}

		if(count == 0)
	    	{
			b_trackL = trackL;
			b_trackR = trackR;
			b_pose = pose[2];
	    	}

	    	if(pose[2] == 0)
	    	{
			pose[2] = b_pose;
	    	}   
	    	double delta_th = -pose[2] + b_pose;

	    	if(delta_th > 3.14)  //将姿航仪的yaw角度限制在-180到180之间（由于姿航仪角度取值范围就是-180到180）
			delta_th -= 3.1415926 * 2;
	   	 else if(delta_th < -3.14)
			delta_th += 3.1415926 * 2;

	    	th += delta_th;

	    	//cout<<"pose2:"<<pose[2] *1800 /3.1415926 <<endl;

	    	distL = trackL - b_trackL;
	    	distR = trackR - b_trackR;

	    	double delta_x = (distL + distR) / 2 * cos((th));
	    	double delta_y = (distL + distR) / 2 * sin((th));

	    	x = b_x + delta_x;
	    	y = b_y + delta_y;       
	    
	    	vx = delta_x / 0.1;
	    	vy = delta_y / 0.1;
	    	vth = delta_th / 0.1;

	    	b_x = x;
	    	b_y = y;
	   
	    	b_trackL = trackL;
	    	b_trackR = trackR;
	    	b_pose = pose[2];
/*

		p1=sub_str.find("{Pose");
		if(p1<0)
		{
			continue;
		}
		else
		{
			int i;
			p1+=5;
			for(i=0;i<2;i++)	
			{
				p2=sub_str.find(",",p1);
				temp=sub_str.substr(p1,p2 - p1);
				pose[i]=atof(temp.c_str())/1000.0;   //相对于初始位置的x,y轴向偏离
				p1=p2+1;
			}
			p2=sub_str.find("}",p1);
			temp=sub_str.substr(p1,p2 - p1);
			pose[2]= -atof(temp.c_str())/10.0*3.1415926/180.0;           //方向角
		}

		p1 = sub_str.find("{Orientation" , p2);
		if(p1 > 0)
		{
			p1 += 13;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			roll = atof(temp.c_str())/10.0*3.1415926/180.0;

			p1 = p2 + 1;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			pitch = atof(temp.c_str())/10.0*3.1415926/180.0;

//				p1 = p2 + 1;
//				p2 = sub_str.find("}" , p1);
//				temp = sub_str.substr(p1 , p2 - p1);
//				pose[2] = atof(temp.c_str())/10.0*3.1415926/180.0;

		}


		x=pose[0];
		y=pose[1];
	    	th=pose[2];

	    	if(count==0)
	    	{
			b_th=th;
	    	}

	    	double delta_x=x-b_x;
	    	double delta_y=y-b_y;
	    	double delta_th=th-b_th;

	    	vx=delta_x/0.1;
	    	vy=delta_y/0.1;
	    	vth=delta_th/0.1;

	    	b_x=x;
	    	b_y=y;
	    	b_th=th;	
*/

		p1 = sub_str.find("{CO2	");
		p1 +=5;
		if(p1 < 0)
		{
			continue;
		}
		else
		{
			//p1 = sub_str.find("{Left" , p1);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1  , p2 - p1 );
			CO2 = atoi(temp.c_str());

			p1 = sub_str.find("{Heat" , p2);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 + 6 , p2 - p1 -6);
			heat = atoi(temp.c_str());
		}




/*		robocup2014::Track track;
		track.trackL = trackL;
		track.trackR = trackR;
		track.yaw = pose[2];
		track_pub.publish(track);
*/

		robocup2014::Int2 sensor;
		sensor.heat = heat;
		sensor.CO2 = CO2;
		sensor_pub.publish(sensor);

		ros::Time scan_time = ros::Time::now();

		//populate the LaserScan message
		sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "base_laser";
		scan.angle_min = -3.14 / 2;  //-2*3.14/3;
		scan.angle_max = 3.14 / 2;   //2*3.14/3;
		scan.angle_increment = 3.14/ num_readings;  //3.14*4/3 / num_readings;
		scan.time_increment = 2*(1 / laser_frequency) / (num_readings);  //9.76562514552e-05;
		scan.scan_time = 1/laser_frequency;                              //0.10000000149;
		scan.range_min = 0.06;
		scan.range_max = 5.0;

		scan.ranges.resize(num_readings);
		for(unsigned int i = 0; i < num_readings ; ++i)
		{
		      scan.ranges[i] = ranges[i + 86];
		}

		scan_pub.publish(scan);


		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); //地图信息只需要朝向，不需要Roll Pitch

	    	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll , pitch ,th);

	    	//first, we'll publish the transform over tf
	    	geometry_msgs::TransformStamped odom_trans;
	    	odom_trans.header.stamp = current_time;
	    	odom_trans.header.frame_id = "odom";
	    	odom_trans.child_frame_id = "base_link";

	    	odom_trans.transform.translation.x = x;
	    	odom_trans.transform.translation.y = y;
	    	odom_trans.transform.translation.z = 0.0;
	    	odom_trans.transform.rotation = odom_quat;

	    	//send the transform
	    	odom_broadcaster.sendTransform(odom_trans);

		//base_link to base_laser
		//geometry_msgs::Quaternion base_quat = tf::createQuaternionMsgFromRollPitchYaw(-roll , -pitch ,0);

		//geometry_msgs::TransformStamped base_laser_trans;
	    	//base_laser_trans.header.stamp = current_time;
	    	//base_laser_trans.header.frame_id = "base_link";
	    	//base_laser_trans.child_frame_id = "base_laser";

	    	//base_laser_trans.transform.translation.x = -0.23;
	    	//base_laser_trans.transform.translation.y = -0.08;
	    	//base_laser_trans.transform.translation.z = 0.44;
	    	//base_laser_trans.transform.rotation = base_quat;

		//base_broadcaster.sendTransform(base_laser_trans);

	    	//next, we'll publish the odometry message over ROS
	    	nav_msgs::Odometry odom;
	    	odom.header.stamp = current_time;
	    	odom.header.frame_id = "odom";

	    	//set the position
	    	odom.pose.pose.position.x = x;
	    	odom.pose.pose.position.y = y;
	    	odom.pose.pose.position.z = 0.0;
	    	odom.pose.pose.orientation = odom_quat;

	    	//set the velocity
	    	odom.child_frame_id = "base_link";
	    	odom.twist.twist.linear.x = vx;
	    	odom.twist.twist.linear.y = vy;
	    	odom.twist.twist.angular.z = vth;

	    	//publish the message
	    	odom_pub.publish(odom);

	    	last_time = current_time;
	    	count++;

   		r.sleep();
	}

	//ros::spin();

	return 0;

}
