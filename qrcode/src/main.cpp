#include <iostream>
#include <fstream>
#include <string>
#include <Magick++.h>
#include "MagickBitmapSource.h"
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/formatters.hpp>
#include <boost/tokenizer.hpp>
#include <vector>

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/inotify.h>

using namespace Magick;
using namespace std;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;

//static bool raw_dump = false;                   //这些都是常量参数，需要改变时只要改变这些就行
static bool tryHarder = false;

ofstream out;
float x = 0.0 , y = 0.0;
int counter = 0;
ros::ServiceClient client;
hector_worldmodel_msgs::AddObject srv;
vector<string> v;

string cell_result;
int res = -1;

static const int MAX_EXPECTED = 4096;

Ref<Result> decode(Ref<BinaryBitmap> image, DecodeHints hints)
{
  Ref<Reader> reader(new MultiFormatReader);
  return reader->decode(image, hints);
}

vector<Ref<Result> > decodeMultiple(Ref<BinaryBitmap> image, DecodeHints hints){
  MultiFormatReader delegate;
  GenericMultipleBarcodeReader reader(delegate);
  return reader.decodeMultiple(image,hints);
}


int test_image(Image& image, bool hybrid)
{
  uint count=0;
  Ref<Binarizer> binarizer(NULL);

  try
  {
    Ref<MagickBitmapSource> source(new MagickBitmapSource(image));
    
    if (hybrid)
    {
        binarizer = new HybridBinarizer(source);
    }
    else
    {
        binarizer = new GlobalHistogramBinarizer(source);
    }

    DecodeHints hints(DecodeHints::DEFAULT_HINT);
    hints.setTryHarder(tryHarder);
    Ref<BinaryBitmap> binary(new BinaryBitmap(binarizer));
    Ref<Result> result(decode(binary, hints));
    cell_result = result->getText()->getText();
    res = 0;
  }
  catch (ReaderException const& e)
  {
    cell_result = "zxing::ReaderException: " + string(e.what());
    res = -2;
  }
  catch (zxing::IllegalArgumentException const& e)
  {
    cell_result = "zxing::IllegalArgumentException: " + string(e.what());
    res = -3;
  }
  catch (zxing::Exception const& e)
  {
    cell_result = "zxing::Exception: " + string(e.what());
    res = -4;
  }
  catch (std::exception const& e)
  {
    cell_result = "std::exception: " + string(e.what());
    res = -5;
  }

  if(res==0)
  {
	  count=0;

	  for(int i = 0;i < v.size(); i++) //需要改进:比赛时不同地段会可能出现相同的二维码
	  {
		if(v[i] == cell_result)
		{
			count++;
			break;
		}				
	  }
              
	  if(count==0)                        //若无重复，写入.csv文件
	  {
		  v.push_back(cell_result);

		  counter++;
		  boost::posix_time::time_duration time_of_day(ros::Time::now().toBoost().time_of_day());
                  boost::posix_time::time_duration time(time_of_day.hours(), time_of_day.minutes(), time_of_day.seconds(), 0);

		  out <<counter<<","<<time<<","<<cell_result<<","<<x<<","<<y<<","<<"0.5"<<endl;

		  srv.request.object.header.frame_id = "base_link";
	          srv.request.object.header.stamp = ros::Time::now();
	          srv.request.map_to_next_obstacle = true;
	          srv.request.object.info.class_id = "qrcode";
	          srv.request.object.info.name = cell_result;

	          srv.request.object.info.support = 100;
	          srv.request.object.pose.pose.position.x = 0.1;
	          srv.request.object.pose.pose.position.y = 0;
	          srv.request.object.pose.pose.position.z = 0;
	          srv.request.object.pose.pose.orientation.w = 1;
	          srv.request.object.state.state = -1;//hector_worldmodel_msgs::ObjectState.CONFIRMED;

	    	  if(client.call(srv))
		  {
			string msg = "added qrcode , id = " + srv.response.object.info.object_id;
			cout<<msg<<endl;
		  }
	          else
	          { 
		        cout<<"fail to add qrcode to map!"<<endl;

	          }
	  }
  }

  return res;
}

int test_image_hybrid(Image& image) {
  return test_image(image, true);
}

int test_image_global(Image& image) {
  return test_image(image, false);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odometry)
{
	x = odometry->pose.pose.position.x;
	y = odometry->pose.pose.position.y;
//	ROS_INFO("Positon is : %f , %f" ,odometry ->pose.pose.position.x , odometry ->pose.pose.position.y);
}

void* GetPosition(void*)
{
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("odom" , 1000 ,odomCallback);
	client = n.serviceClient<hector_worldmodel_msgs::AddObject>("worldmodel/add_object");

	ros::spin();

	return (void*)0;
}


int main(int argc, char** argv)
{
  ros::init(argc , argv , "qrcode");

  pthread_t thread_getPosition;
  pthread_create(&thread_getPosition,NULL,GetPosition,NULL);

  sleep(5);

  boost::posix_time::ptime now = ros::Time::now().toBoost();
  boost::gregorian::date now_date(now.date());                       //格林尼治时间
  boost::posix_time::time_duration now_time(now.time_of_day().hours(), now.time_of_day().minutes(), now.time_of_day().seconds(), 0);

  out.open("/home/peng/RC2015_SEU-Jolly_Prelim1_qr.csv");
  out.seekp(0, ios::end); //每次从文件结尾处输入
  out<<"\"qr codes\""<<endl<<"\"1.0\""<<endl<<"\"SEU-Jolly\""<<endl<<"\"China\""<<endl<<"\""<<now_date<<"\""<<endl<<"\""<<now_time<<"\""<<endl<<"\"Prelim1\""<<endl<<endl<<"id,time,text,x,y,z"<<endl;

  long imageNo = 0;
  string infilename;                       //存放图片的名字，序列号.bmp,如1.bmp
  string infilename1;                      //存放图片序列号，如1
  char ch_imageNo[100];
  bool flag_navigation=false;
  char ch_subimg[50];
  string temp_subimg;

  infilename="1.bmp";
  Image image;

  string directory = "/home/peng/bmp";
  string fullpath;
  int fd , watch;
  bool isCreated = false;
  
    fd = inotify_init();
    char buf[100 * (sizeof(inotify_event) + 0)];
//    cout<<"sizeof inotify_event:"<<sizeof(inotify_event)<<endl;
    ssize_t length;	

	try
	{
	    while(ros::ok())
	    {		
			isCreated = false;
			fullpath = directory + "/" + infilename;
		        watch = inotify_add_watch(fd , directory.c_str() , IN_CREATE |IN_MOVED_TO );

	    		infilename1=infilename.substr(0,infilename.length()-4);

			if(access(fullpath.c_str(), F_OK) ==0)
			{
				//cout<<infilename<<" exists"<<endl;
				isCreated = true;
			}

			while(!isCreated)
			{
				//cout<<"waiting for "<<infilename<<endl;
				length = read(fd , buf , sizeof(buf));
				//cout<<"length:"<<length<<endl;	
				if(length < 0)
					break;
				inotify_event *event;
				for (size_t i = 0; i < static_cast<size_t>(length);i += sizeof(inotify_event) + event->len)
				{
		    			event = reinterpret_cast<inotify_event *>(&buf[i]);
				        if (event->len > 0 && infilename == event->name) 
					{
		                		//printf("The file %s was created.\n", event->name);
			        		isCreated = true;
			                        break;
					}
		                }
			
			}
			//cout<<"reading file.."<<endl;
			usleep(200000);
			if(isCreated)
				image.read(fullpath); 
			else
				break;
			
			//cout<<"finish reading .."<<endl;
	//		string expected;
	//		expected = get_expected(infilename);

			  int gresult = 1;
			  int hresult = 1;
			  hresult = test_image_hybrid(image);
			  if(res==0)
			  {
			    cout << "#"<< infilename1 <<"  " << "hybrid" <<"  "<<cell_result<<endl;
			  }
			  else
			  {
			    cout << "#"<<infilename1<<"  "<<cell_result<<endl;
			  }

			  if(hresult!=0)
			  {
				  cout<<"     hybrid failed..."<<endl;
				  gresult = test_image_global(image);
				  if(res==0)
				  {
					cout << "#"<< infilename1 <<"  " << "globle" <<"  "<<cell_result<<endl;
				  }
				  else
				  {
					cout << "#"<<infilename1<<"  "<<cell_result<<endl;
				  }

				  if(gresult!=0)
				  {
					  cout<<"     global failed..."<<endl;
				  }
			  }

			strcpy(ch_imageNo,infilename1.c_str());
			imageNo = atoi(ch_imageNo);
			imageNo++;
			sprintf(ch_imageNo,"%ld.bmp",imageNo);
			infilename=ch_imageNo;

			inotify_rm_watch(fd, watch);
			if(remove(fullpath.c_str()))
			{
				cout<<"Could not remove "<<fullpath<<endl;
			}
	    }
	}
	catch(...)
	{
		cout<<"error occured"<<endl;
	}

  out.close();
  close(fd);
  return 0;

}


