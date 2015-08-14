#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <string>
using namespace std;

#define BUFSIZE 4096*5


union semun
{
	   int val;
	   struct semid_ds *buf;
	   ushort *array;
};

/*信号量的P操作*/
void p(int semid)
{
	   struct sembuf sem_p;
	   sem_p.sem_num=0;
	   sem_p.sem_op=-1;
	   if(semop(semid,&sem_p,1)==-1)
		   printf("p operation is fail\n");
}

/*信号量的V操作*/
void v(int semid)
{
	   struct sembuf sem_v;
	   sem_v.sem_num=0;
	   sem_v.sem_op=1;
	   if(semop(semid,&sem_v,1)==-1)
	   printf("v operation is fail\n");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;


        char *addr_network;
        addr_network=new char[4096*5];

        int shmid=0;
	int semid=0;

	key_t shmkey;
	shmkey=ftok("/usr/DonotDelete/network2",0);

	key_t semkey;
	semkey=ftok("/usr/DonotDelete/network1",0);

	do
	{
		shmid=shmget(shmkey,0,0666);
		semid=semget(semkey,0,0666);

	}while((semid==-1)||(shmid==-1));

	char *addr;
	addr=(char*)shmat(shmid,0,0);
	if(addr==(char*)-1)
	{
		perror("shmat");
	}

	double pose[3];
        double b_pose = 0.0;

  double x = 0.0,b_x=0.0;
  double y = 0.0,b_y=0.0;
  double th = 0.0;
  double roll = 0.0, pitch = 0.0;

  double trackL = 0.0 , trackR = 0.0 , b_trackL = 0.0 , b_trackR = 0.0;
  double distL = 0.0 , distR = 0.0 ;
  
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  long count=0;
  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();               		   // check for incoming messages
    current_time = ros::Time::now();


	//p(semid);                                //进程间通信的信号量p、v操作
	bzero(addr_network,BUFSIZE);
	memmove(addr_network,addr,BUFSIZE);
        //cout<<addr_network<<endl;
	//v(semid);

	string str(addr_network);

/*	int p1=str.find("{Pose");
        int p2;
        string temp;

	if(p1<0)
        {
		continue;
        }
	else
	{
		int i;
		p1+=6;
		for(i=0;i<2;i++)	
		{
			p2=str.find(",",p1);
			temp=str.substr(p1,p2 - p1);
			pose[i]=atof(temp.c_str())/1000.0;   //相对于初始位置的x,y轴向偏离
			p1=p2+1;
		}
		p2=str.find("}",p1);
		temp=str.substr(p1,p2 - p1);
		pose[2]=atof(temp.c_str())/10.0*3.1415926/180.0;           //方向角
	}*/


	int p1 = str.find("{Type Track} {Name Position}");
	int p2 ;
	string temp;

	if(p1 < 0)
	{
		continue;
	}
	else
	{
		p1 = str.find("{Left" , p1);
		p2 = str.find("}" , p1);
		temp = str.substr(p1 + 6 , p2 - p1 -6);
		trackL = atof(temp.c_str())/1000.0;

		p1 = str.find("{Right" , p2);
		p2 = str.find("}" , p1);
		temp = str.substr(p1 + 6 , p2 - p1 -6);
		trackR = atof(temp.c_str())/1000.0;

	}

	p1 = str.find("{Orientation" , p2);
	

	if(p1 > 0)
	{
		p1 += 13;
		p2 = str.find("," , p1);
		temp = str.substr(p1 , p2 - p1);
		roll = atof(temp.c_str())/10.0*3.1415926/180.0;

		p1 = p2 + 1;
		p2 = str.find("," , p1);
		temp = str.substr(p1 , p2 - p1);
		pitch = atof(temp.c_str())/10.0*3.1415926/180.0;

		p1 = p2 + 1;
		p2 = str.find("}" , p1);
		temp = str.substr(p1 , p2 - p1);
		pose[2] = atof(temp.c_str())/10.0*3.1415926/180.0;

	}


    //compute odometry in a typical way given the velocities of the robot

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

    if(delta_th > 3.14)
        delta_th -= 3.1415926 * 2;
    else if(delta_th < -3.14)
        delta_th += 3.1415926 * 2;


    th += delta_th;

    cout<<"pose2:"<<pose[2] *1800 /3.1415926 <<endl;


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

//    cout<<x<<" "<<y<<" "<<th<<" "<<roll<<" "<<pitch<<" "<<endl;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
//    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll , pitch ,th);

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
}
