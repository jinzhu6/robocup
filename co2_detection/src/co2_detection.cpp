/*
 * uart_re1.c
 *
 *  Created on: Jul 8, 2015
 *      Author: wjonuee
 */

#include "ros/ros.h"
#include <string>
#include <std_msgs/Int32.h>

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>

#define FALSE -1
#define TRUE 0

int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed){
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
    if  (speed == name_arr[i]) {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if  (status != 0) {
        perror("tcsetattr fd1");
        return;
      }
      tcflush(fd,TCIOFLUSH);
    }
  }
}

int set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
	if  ( tcgetattr( fd,&options)  !=  0) {
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE;
	/*set the numbers of data bits*/
	switch (databits)
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n"); return (FALSE);
	}
	switch (parity)
	{
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */
			break;
		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB);  /*enable odd parity*/
			options.c_iflag |= INPCK;             /* Disable parity checking */
			break;
		case 'e':
		case 'E':
			options.c_cflag |= PARENB;     /* Enable even parity */
			options.c_cflag &= ~PARODD;
			options.c_iflag |= INPCK;
			break;
		case 'S':
		case 's':                             /* Enable space parity */
		    options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;break;
		default:
			fprintf(stderr,"Unsupported parity\n");
			return (FALSE);
		}

	/*set the numbers of stop bits*/
	switch (stopbits)
	{
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;
		case 2:
			options.c_cflag |= CSTOPB;
		   break;
		default:
			 fprintf(stderr,"Unsupported stop bits\n");
			 return (FALSE);
	}
	/* Set input parity option */
	if (parity != 'n')
		options.c_iflag |= INPCK;
	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 150;
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return (FALSE);
	}
	return (TRUE);
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "co2_node");
ros::NodeHandle n;
ros::Publisher co2_pub = n.advertise<std_msgs::Int32>("co2_num",2);
	
printf("This program updates last time at %s   %s\n",__TIME__,__DATE__);
	printf("STDIO USB0\n");
	int fd;
	fd = open("/dev/ttyUSB0",O_RDWR);
	if(fd == -1)
	{
		perror("serial port error\n");
	}
	else
	{
		printf("open ");
		printf("%s",ttyname(fd));
		printf(" successfully\n");
	}

	set_speed(fd,38400);
	if (set_Parity(fd,8,1,'N') == FALSE)  {
		printf("Set Parity Error\n");
		exit (0);
	}
	//char buf[] = "fe55aa07bc010203040506073d";
	//write(fd,&buf,26);
	char buff[512];
	int nread;
	std_msgs::Int32 co2_num;
	ros::Rate loop_rate(20);
	unsigned int frame = 0;
	while(ros::ok())
	{
		if((nread = read(fd, buff, 512))>1)
		{
//			printf("\nLen: %d\n",nread);
			buff[nread+1] = '\0';
//			printf("%s",buff);
			co2_num.data = atoi(buff);
			co2_pub.publish(co2_num);
		}
		else
		{
			continue;		
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(fd);
	return 0;
}

