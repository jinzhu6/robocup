#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"
//#include "robocup2014/Track.h"
#include <math.h>
#include <Connect.h>
#include "std_srvs/Empty.h"
using namespace std;

#define NUM 682
int ranges[NUM];
int trackL , trackR , yaw;
Connection* m_connect;

bool pause_flag = false;

struct Point
{
	int X;
	int Y;
}location;     //当前机器人位置
int p_yaw = 0;      //前一周期的偏向角
int p_track_l = 0;  //前一周期左履带路程
int p_track_r = 0;  //前一周期右履带路程

int use_navigation = 1;//是否使用自动导航
int velocity_ctrl = 0;//速度附加控制



bool avoid_pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("avoid pause service triggered/n");
	//pause_flag = !pause_flag;
	pause_flag = true;
	//cout<<pause_flag<<endl;
	return true;
}

bool avoid_continueCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("avoid continue");
	pause_flag = false;

	return true;
}

/*
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	for(int i = 0;i < NUM;i++)
	{
		ranges[i] = (int)(scan->ranges[i] * 10);
	}
}

void trackCallback(const robocup2014::Track::ConstPtr& track)
{
	trackL = track ->trackL;
	trackR = track ->trackR;
	yaw = track ->yaw;
}*/

void Velocity(int vl, int vr)
{
    int v_ctrl_code = velocity_ctrl % 5;
    if (velocity_ctrl > 4)
        velocity_ctrl = 0;

    if (v_ctrl_code == 1)
    {
        vl -= 70;
        vr += 15;
    }
    else if (v_ctrl_code == 2)
    {
        vl += 15;
        vr -= 70;
    }
    else if (v_ctrl_code == 3)
    {
        vl += 35;
        vr += 35;
    }
    else if (v_ctrl_code == 4)
    {
        vl += -95;
        vr += -95;
    }

    cout<<abs(vl - vr)<<endl;

    if (abs(vl - vr) < 23)
    {
        vl *= 10.0;    //12
        vr *= 10.0;
    }
    else if (abs(vl - vr) < 32)   //32
    {
        vl = (int)(vl * 8.3);   //8.3
        vr = (int)(vr * 8.3);
        if (vl < vr)
        {
            vl = (int)(vl - abs(vr - vl) * 1);   //0.09
            vr = (int)(vr + abs(vr - vl) * 1);
        }
	else
	{
	    vl = (int)(vl + abs(vl - vr) * 1);   //0.09
            vr = (int)(vr - abs(vl - vr) * 1);
	}
    }
    else
    {
        vl = (int)(vl * 5.1);  //5.1
        vr = (int)(vr * 5.1);
        if (vl < vr)
        {
            vl = (int)(vl - abs(vr - vl) * 0.2);    //1.0
            vr = (int)(vr + abs(vr - vl) * 0.2);
        }
	else
	{
	    vl = (int)(vl + abs(vl - vr) * 0.2);    //1.0
            vr = (int)(vr - abs(vl - vr) * 0.2);
	}

    }

    vl = (int)(vl / 3);
    vr = (int)(vr / 3);

    //发送控制命令


    char ctmp[100];
    sprintf(ctmp,"DRIVE {Type Direct} {Name Velocity} {Left %d} {Right %d} {Normalized 1}\n", vl , vr);
    string str(ctmp);

    m_connect ->sendMsg(str);

}


void Position(Point loc, int yaw, int desx, int desy)          //目标点坐标                      
{
        double theta_e = 0;
        int vr,vl,vc;
        double dx,dy;
        double d_e;
        double Ka;

    	int x = loc.X;
    	int y = loc.Y;

        dx = desx - x;
        dy = desy - y;
        d_e = sqrt(dx * dx + dy * dy);//距离

    	theta_e = (180.0/3.1416 * atan2((double)(dy), (double)(dx))) - (90 + yaw / 10.0);      //偏向角北偏西为正，偏东为负
        
    	double darc = fabs(theta_e);
        //if (darc < 40)
        //    vc = 160;
        //else
	    vc = 110;

        while (theta_e > 180) theta_e -= 360;
        while (theta_e < -180) theta_e += 360;

        if (d_e > 100.0) 
	        Ka = 70.0 / 90.0;
        else if (d_e > 50)
	        Ka = 75.0 / 90.0;
        else if (d_e > 30)
	        Ka = 80.0 / 90.0;
        else if (d_e > 20)
	        Ka = 85.0 / 90.0;
        else 
	        Ka = 90.0 / 90.0;

    	if (theta_e > 95 || theta_e < -95) 
        {               
	        //if (theta_e > 80)
	        //theta_e = 80;
	        //if (theta_e < -80)
	        //theta_e = -80;
	        if (d_e < 5.0 && fabs(theta_e) < 40)
		        Ka = 0.1;
	        vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e))-0.3) - Ka * theta_e);
	        vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e))-0.3) + Ka * theta_e);

        }
        else if (theta_e < 85 && theta_e > -85)
        {
	        if (d_e < 5.0 && fabs(theta_e) < 40)
	            Ka = 0.1;
	        vl = (int)(vc * (1.0 / (1.0 + exp(-3.0 * d_e))-0.3) - Ka * theta_e);	
	        vr = (int)(vc * (1.0 / (1.0 + exp(-3.0 * d_e))-0.3) + Ka * theta_e);
	        // vr = (int)( vc - Ka * theta_e);
	        // vl = (int)( vc + Ka * theta_e);
        }
        else
        {
	        vl = (int)(-0.17 * theta_e);
	        vr = (int)(+0.17 * theta_e);
        }
    	Velocity(vl, vr);
}



void LocParse(int track_l, int track_r, int yaw)
{
    if (p_track_l != 0)
    {
        double dl = track_l - p_track_l;
        double dr = track_r - p_track_r;
        if (dl < 10 || dr < 10)
            return;
        double theta = (90 + yaw / 10.0) * 3.1416 / 180;
        double p_theta = (90 + p_yaw / 10.0) * 3.1416 / 180;
        double ds = (dl * 138 + dr * 362) / 600;
        double a_theta = (theta + p_theta) / 2;
        double dx = ds * cos(a_theta);
        double dy = ds * sin(a_theta);
        /*double r = -(dl + dr) / 2 / (theta - p_theta) + 112;
        double dx = r * (Math.Sin(p_theta) - Math.Sin(theta));
        double dy = r * (Math.Cos(theta) - Math.Cos(p_theta));*/
        location.X += (int)dx;
        location.Y += (int)dy;
        p_yaw = yaw;
    }
    p_track_l = track_l;
    p_track_r = track_r;
}


int main(int argc, char ** argv)
{
	ros::init(argc ,argv,"avoid_obs");

	ros::NodeHandle n;
//	ros::Subscriber scan_sub = n.subscribe("scan" , 5 , scanCallback);
//	ros::Subscriber track_sub = n.subscribe("track" , 10 , trackCallback);

	ros::ServiceServer service = n.advertiseService("avoid_pause" , avoid_pause);
	ros::ServiceServer continue_server = n.advertiseService("avoid_continue" , avoid_continueCallback);
	
	m_connect = new Connection;

	if(m_connect -> Oninit("192.168.1.116",10000))
	{
		cout<<"connect successfully"<<endl;
		m_connect ->sendMsg("REG {Mode WR} {Period 200}\r\n");
	}
	else
	{
		cout<<"connect error"<<endl;	
	}

	char* rcvbuffer;
	rcvbuffer = new char[4096 * 5];



	double p_des = 100000;//上一次的目标角度
	double b_des = (90.0 + yaw/10.0) /180.0 *3.14;
        int dead = 0;
        int dead_yaw = 0;
	
	location.X = 0;
  	location.Y = 0;

	string sub_str ,b_str;
	int roll ,pitch ,p1 ,p2;
	ros::Rate rate(5.0);

	usleep(10000000);

	while(n.ok())
	{
         	ros::spinOnce();

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

		p1=sub_str.find("{Range");
		string temp;

		if(p1<0)
		{
			continue;
		}
		else
		{
			int i;
			p1+=7;
			for(i=0;i<681;i++)	// 读取激光数据
			{
				p2=sub_str.find(",",p1);
				temp=sub_str.substr(p1,p2 - p1);
				ranges[i]=atoi(temp.c_str());
				ranges[i]=(ranges[i]==0)?4000:ranges[i];
				ranges[i]=(ranges[i] > 4000)?4000:ranges[i];
				p1=p2+1;
				//cout<<ranges[i]<<",";
			}
			p2=sub_str.find("}",p1);
			temp=sub_str.substr(p1,p2 - p1);
			ranges[681]=atoi(temp.c_str());
			ranges[681]=(ranges[681]==0)?4000:ranges[681];
			ranges[681]=(ranges[681] > 4000)?4000:ranges[681];
		}

		p1 = sub_str.find("{Type Track} {Name Position}");
		if(p1 < 0)
		{
			continue;
		}
		else
		{
			p1 = sub_str.find("{Left" , p1);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 + 6 , p2 - p1 -6);
			trackL = atoi(temp.c_str());

			p1 = sub_str.find("{Right" , p2);
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 + 6 , p2 - p1 -6);
			trackR = atoi(temp.c_str());
		}

		p1 = sub_str.find("{Orientation" , p2);
		if(p1 > 0)
		{
			p1 += 13;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			roll = atoi(temp.c_str());

			p1 = p2 + 1;
			p2 = sub_str.find("," , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			pitch = atoi(temp.c_str());

			p1 = p2 + 1;
			p2 = sub_str.find("}" , p1);
			temp = sub_str.substr(p1 , p2 - p1);
			yaw = atoi(temp.c_str());

		}
	    
	    try
	    {
		if(!pause_flag)
		{
			//int yaw = -data_package.Mtix.Orientation[2];
		        if (p_des == 100000)
		            p_des = (90.0 + yaw / 10.0) * 3.1416 / 180;
	   
		        LocParse(trackL, trackR, yaw);

		        int sum_r = 0;
		        int ave_r;
		        for(int i= 86;i < NUM-86;i++)
		        {
		            if (ranges[i] < 5)
		                sum_r += 4000;
		            else
		                sum_r += ranges[i];
		        }
		        ave_r = sum_r / 510 + 380;

		        double theta = (-30.0 + yaw / 10.0) * 3.1416 / 180;
		        double dtheta = 240.0 / 682 * 3.1416 / 180;
		        //PointF[] pnts = new PointF[682];
		        double p_best = 0;
		        double best = 0;
		        int p_cnt = 0;
		        int cnt = 0;
		        for(int i = 0;i <NUM;i++)
		        {
		            theta += dtheta;
		           // double dx = ranges[i] * Math.Cos(theta);
		           // double dy = ranges[i] * Math.Sin(theta);
		           // if (r < 5)
		           // {
		           //     dx = 4000 * Math.Cos(theta);
		           //     dy = 4000 * Math.Sin(theta);
		           // }
		           
		            if (ranges[i] < 5 || ranges[i] > ave_r)   //长
		            {
		                cnt++;
		                best += theta;
		                
		            }
		            else                    //短
		            {
		                if (cnt > 3)
		                {
		                    double darc = fabs(p_des - best / cnt);
		                    double darc2 = fabs((90 + yaw / 10.0) * 3.1416 / 180 - best / cnt);
		                    double k_darc = cnt * 0.5 + 25;
		                    double k_darc2 = 24;
		                    if (darc > 1.22)
		                        k_darc = 200;
		                    if (cnt > p_cnt + (int)(darc * k_darc) + (int)(darc2 * k_darc2))
		                    {
		                        p_cnt = cnt;
		                        p_best = best;
		                    }
		                    if (p_cnt < 10 && k_darc != 200)
		                    {
		                        p_cnt = cnt;
		                        p_best = best;
		                    }

				/*    if(cnt > p_cnt /(darc2*2) )
				    {
					p_cnt = cnt;
					p_best = best;

				    }*/


		                }
		                cnt = 0;
		                best = 0;
		            }
		        }

		

			if(p_cnt != 0)
			{
		        	p_des = p_best / p_cnt;
		        }
			else
				p_des = (90.0 + yaw /10.0) * 3.14 /180.0;   

			p_best = 0;   /////////////////////////
			p_cnt = 0;    /////////////////////////

			//b_des = p_des;
			cout<<"Des = "<<p_des<<"  "<<p_cnt<<endl;

		        int r_protect_fl = 0;//防碰撞保护算法
		        int r_protect_f = 0;
		        int r_protect_fr = 0;
		        int r_protect_l = 0;
		        int r_protect_r = 0;

		        if (ranges[437] == 0)
		            r_protect_fl += 4000;
		        else
		            r_protect_fl += ranges[437];
		        if (ranges[438] == 0)
		            r_protect_fl += 4000;
		        else
		            r_protect_fl += ranges[438];
		        r_protect_fl /= 2;                          //左前

		        if (ranges[301] == 0)
		            r_protect_fr += 4000;
		        else
		            r_protect_fr += ranges[301];
		        if (ranges[302] == 0)
		            r_protect_fr += 4000;
		        else
		            r_protect_fr += ranges[302];
		        r_protect_fr /= 2;                          //右前

		        if (ranges[341] == 0)
		            r_protect_f += 4000;
		        else
		            r_protect_f += ranges[341];
		        if (ranges[342] == 0)
		            r_protect_f += 4000;
		        else
		            r_protect_f += ranges[342];
		        r_protect_f /= 2;                           //前

		        if (ranges[598] == 0)
		            r_protect_l += 4000;
		        else
		            r_protect_l += ranges[598];
		        if (ranges[599] == 0)
		            r_protect_l += 4000;
		        else
		            r_protect_l += ranges[599];
		        r_protect_l /= 2;                           //左

		        if (ranges[85] == 0)
		            r_protect_r += 4000;
		        else
		            r_protect_r += ranges[85];
		        if (ranges[86] == 0)
		            r_protect_r += 4000;
		        else
		            r_protect_r += ranges[86];
		        r_protect_r /= 2;                           //右

		        int protect_fl = 658 + 180;
		        int protect_fr = 567 + 255;
		        int protect_f = 680;
		        int protect_l = 362 + 100;
		        int protect_r = 138 + 100;
		        if (velocity_ctrl >= 1 && velocity_ctrl <= 4)
		        {
		            dead = 0;
		            p_des = (90.0 + yaw / 10.0) * 3.1416 / 180;
		        }
		        else
		        {
		            //cout<<"Protected"<<endl;
		            if (r_protect_fl < protect_fl && r_protect_fr < protect_fr || r_protect_f < protect_f)
		                velocity_ctrl = 9;
		            else if (r_protect_fl < protect_fl || r_protect_l < protect_l)
		                velocity_ctrl = 7;
		            else if (r_protect_fr < protect_fr || r_protect_r < protect_r)
		                velocity_ctrl = 6;
		        }

			//cout<<"velocity_ctrl:"<<velocity_ctrl<<endl;

		        if (r_protect_fl < protect_fl + 300 && r_protect_fr < protect_fr + 300
		            && r_protect_f < protect_f + 240 && r_protect_l < protect_l + 400 && r_protect_r < protect_r + 400 && dead == 0)
		        {
		            dead = 0;
		            dead_yaw = yaw;
		            cout<<"Dead"<<endl;
		        }
		        if (abs(yaw - dead_yaw) > 1300 && dead == 1)
		        {
		            cout<<"Live"<<endl;
		            p_des = (90.0 + yaw / 10.0) * 3.1416 / 180;
		            dead = 0;
		        }
		        else if (dead == 1)
		            cout<<"dead "<<abs(yaw - dead_yaw)<<endl;

		       // Console.WriteLine("fl " + r_protect_fl + " fr " + r_protect_fr + " f " + r_protect_f);
		       // Console.WriteLine("l " + r_protect_l + " r " + r_protect_r);

		        if (use_navigation == 1)
		        {
		            if (dead == 0)
		                Position(location, yaw, location.X + (int)(25 * cos(p_des)), location.Y + (int)(25 * sin(p_des)));
		            else
		            {
		                velocity_ctrl = 0;
		                Velocity(-30, 30);    //70
		            }
		        }
		}
            }
            catch(...)
            {
                cout<<"ERR0"<<endl;
                return 0;
            }
           // start = true;

	rate.sleep();

	}

	return 0;
}


