/*
 * tcp_client_text.cpp
 *
 *  Created on: 2015-5-10
 *      Author: robocup
 */
#include <stdio.h>

#include <stdlib.h>

#include <errno.h>

#include <string.h>

#include <netdb.h>

#include <sys/types.h>

#include <netinet/in.h>

#include <sys/socket.h>

#include <iostream>

#include <unistd.h>

//#include "Obvious2D.h"

#include "ros/ros.h"
#include <image_transport/image_transport.h>

#define WIDTH 160
#define HEIGHT 120

#define PORT 25000 /* Server的端口 */

#define MAXDATASIZE 57600 /*一次可以读的最大字节数 */

using namespace std;

int main(int argc, char *argv[])

{

//  	obvious::Obvious2D viewer(WIDTH, HEIGHT, "Optirs imager");
	  ros::init (argc, argv, "thermal_image_node");

  // private node handle to support command line parameters for rosrun
        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
	image_transport::Publisher  _pubThermal = it.advertise("thermal_image", 1);
	unsigned int   _frame = 0;
	
	int sockfd, numbytes;

	unsigned char buf[MAXDATASIZE];

	struct hostent *he; /* 主机信息 */

	struct sockaddr_in their_addr; /* 对方地址信息 */

	if (argc != 2) {

			fprintf(stderr,"usage: client hostname\n");

			exit(1);

	}



/* get the host info */

	if ((he=gethostbyname(argv[1])) == NULL) {

/* 注意：获取DNS信息时，显示出错需要用herror而不是perror */

		herror("gethostbyname");

		exit(1);

	}

	cout << "hostname is " << he->h_name << endl;

	if ((sockfd=socket(AF_INET,SOCK_STREAM,0))==-1) {

		perror("socket");

		exit(1);

	}

	their_addr.sin_family = AF_INET;

	their_addr.sin_port = htons(PORT); /* short, NBO */

	their_addr.sin_addr = *((struct in_addr *)he->h_addr);

	bzero(&(their_addr.sin_zero), 8); /* 其余部分设成0 */

	if (connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {

		perror("connect");

		exit(1);

	}
//	string message;
	
	int all_byte = MAXDATASIZE;
	//int nettime = 10;
	//setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&nettime, sizeof(int));

	sensor_msgs::Image img;
  	img.header.frame_id = "thermal_image";
  	img.height  = HEIGHT;
  	img.width   = WIDTH;
  	img.encoding        = "rgb8";
  	img.step            = WIDTH*3;
  	img.data.resize(HEIGHT*img.step);

	int buf_index;
	while(ros::ok()) {
/*		if (send(sockfd,message.c_str(),message.size()+1,0)==-1) {
				perror("send");
				exit(1);
			}
		cout << "send: " << message.c_str() <<endl; */
		all_byte = MAXDATASIZE;
		buf_index = 0;
		while(all_byte != 0) {

			if((numbytes = recv(sockfd, buf+buf_index, all_byte, 0)) == -1) {
				perror("recv");
				continue;
			}
			all_byte -= numbytes;
			buf_index += numbytes;
		}

		if(_pubThermal.getNumSubscribers() == 0)
     			continue;
		img.header.seq      = ++_frame;
  		img.header.stamp    = ros::Time::now();
		memcpy(&img.data[0], buf, 160*120*3);
		  _pubThermal.publish(img);
//		viewer.draw(buf, 160, 120, 3);


	//	bzero(buf, MAXDATASIZE);
		//buf[numbytes] = '\0';
		//cout << "recv: " << buf << endl;
	/*	cout << "begin:";
		for(int i = 0; i<160*120*3; ++i) {
			cout << (int)buf[i] << " ";
		}
		cout << endl;
*/
	}
	
	close(sockfd);

	return 0;

}
