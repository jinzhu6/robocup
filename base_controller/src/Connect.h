#ifndef CONNECT_H_
#define CONNECT_H_
#include <iostream>
#include <fstream>
#include <strings.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <time.h>

#define buffer_size 4096*5

using namespace std;

class Connection
{
public:
	Connection();
	virtual ~Connection();
	bool recvMsg();    				//-------------- 接受数据包 ------------
	bool sendMsg(string msg);       //-------------- 发送数据（各种命令） -------------
	void closeConnection();         //-------------- 断开通信 ---------------
	bool Oninit(string,int);        //-------------- 初始化，将建立起通信连接 ------------
	void deleteBuff();				//-------------- 删除缓冲去 -------------
	void SetConnSock(int sock)
	{
		conn_sock=sock;             //-------------- 将robotframe中register命令中获得socket号传过来 ----------
	}
	int GetSock()
	{
		return conn_sock;
	}

	//wxString logShowConv;

public:
	int conn_sock;                   //------------  socket 端口号 ---------
	sockaddr_in serv_addr;
	//char *rcvbuffer=new char[buffer_size];
	char * buffer;
	int recv_size;					//------------  接受数据的字节数  变量 --------
	int send_size;					//------------  发送数据的自己数  变量 --------
	int offset;
	char *rcvbuffer;
	string logShow;					//------------- 用于存储反馈回来的Log信息 ----------
	string RT_filename;             ///////////////////////////////////////////////////////////////////


};


#endif /* CONNECT_H_ */
