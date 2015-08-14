
#include "Connect.h"

Connection::Connection()    //----------Connection 类的构造函数 ------------
{
	conn_sock=-1;
	cout<<"Connection"<<conn_sock<<endl;
	buffer=new char[buffer_size];
	rcvbuffer=new char[buffer_size];
	recv_size=0;
	offset=0;

	time_t t=time(0);                             //每一次连接之后运行的数据存放在桌面上以时间命名的文件内
	char temp[60];
	strftime(temp,sizeof(temp),"%Y_%m_%d %H:%M:%S",localtime(&t));
	RT_filename=temp;
	RT_filename="/home/peng/desktop/"+RT_filename+".txt";
	cout<<RT_filename<<endl;
	ofile1.open(RT_filename.c_str());//,ios::app);    //////////////////////////////////////////////////

}
Connection::~Connection()
{
	delete[]buffer;
	delete[]rcvbuffer;
}

//------------------ 初始化 建立socket连接  与机器人建立连接  ----------------

bool Connection::Oninit(string servip,int servport)
{
	conn_sock = socket(AF_INET,SOCK_STREAM,0); //---------创建套接字---------

	if (conn_sock == -1)
	{
		return false;
	}
	else
	{
		cout<<"socket is success"<<endl;
	}

	bzero(&serv_addr,sizeof(serv_addr));
	serv_addr.sin_family=AF_INET;
	serv_addr.sin_port=htons(servport);
	serv_addr.sin_addr.s_addr=inet_addr(servip.c_str());


	if (connect(conn_sock,(struct sockaddr*)&serv_addr,sizeof(struct sockaddr))==-1)  //------与机器人连接 --------
	{
		return false;
	}
	else
	{
		cout<<"Connect is successful!"<<endl;
	}

	return true;

}

//-----------------------------------  发送消息 ------------------
bool Connection::sendMsg(string msg)
{
	cout<<msg<<endl;
	int sendbytes=msg.length();

	if (send(conn_sock,msg.c_str(),sendbytes,0) != sendbytes)
	{
		perror("send command");
		cout<<"sendMsg"<<conn_sock<<endl;
		exit(1);
	}

	return true;
}

//-------------------------------------  接受消息包----------------
bool Connection::recvMsg()
{
	ofstream ofile;
        ofile.open("/home/peng/desktop/log.txt");
	bzero(rcvbuffer,buffer_size);
	if ((recv_size = recv(conn_sock, rcvbuffer, buffer_size, 0)) == -1)
	{
		perror("receive data");
		exit(1);
	}

	memmove(buffer+offset,rcvbuffer,recv_size);
	
	for (int i=0;i<recv_size;i++)
	{
		if (rcvbuffer[i]==0)
		{
			rcvbuffer[i]=0x20;
		}
	
		//ofile1<<rcvbuffer[i];
		
	}
	string stemp(rcvbuffer);
	ofile1 << stemp << endl <<endl<<endl;
	ofile<<stemp<<endl;
	ofile.close();
	return true;

}

void Connection::deleteBuff()
{
	ofile1.close();
	delete[]rcvbuffer;
	delete[] buffer;

}

void Connection::closeConnection()
{
	close(conn_sock);
	cout<<"Close the communication with the robot"<<endl;
	offset=0;
}
