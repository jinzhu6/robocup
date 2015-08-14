
#include "Connect.h"

Connection::Connection()    //----------Connection 类的构造函数 ------------
{
	//TODO Auto-generated constructor stub
	conn_sock=-1;
	cout<<"Connection"<<conn_sock<<endl;
	buffer=new char[buffer_size];
	rcvbuffer=new char[buffer_size];
	recv_size=0;
	offset=0;

	time_t t=time(0);                                    //每一次连接之后运行的数据存放在桌面上以时间命名的文件内
	char temp[60];
	strftime(temp,sizeof(temp),"%Y_%m_%d %H:%M:%S",localtime(&t));
	RT_filename=temp;
	RT_filename="/home/peng/desktop/"+RT_filename+".txt";
	cout<<RT_filename<<endl;


	//logShow = new string;
}
Connection::~Connection()
{
	//TODO Auto-generated constructor stub
	delete[]buffer;
	delete[]rcvbuffer;
	//delete logShow;
}

//------------------ 初始化 建立socket连接  与机器人建立连接  ----------------

bool Connection::Oninit(string servip,int servport)
{
	conn_sock = socket(AF_INET,SOCK_STREAM,0); //---------创建套接字---------

	if (conn_sock == -1)
	{
		//cout<<"conn_sock: "<<conn_sock<<endl;
		//perror("socket");
		return false;
		//exit(1);
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
		//cout<<"aaaaa"<<endl;
		//perror("connect");
		//closeConnection();
		//exit(1);
		return false;
	}
	else
	{
		cout<<"Connect is successful!"<<endl;
	}
	//cout<<"onInit   "<<conn_sock<<endl;

	return true;

}

//-----------------------------------  发送消息 ------------------
bool Connection::sendMsg(string msg)
{

	//cout<<"conn_sock  "<<conn_sock<<endl;
	cout<<msg<<endl;
	int sendbytes=msg.length();
	cout<<sendbytes<<endl;

	if (send(conn_sock,msg.c_str(),sendbytes,0) != sendbytes)
	{
		perror("send command");
		cout<<"sendMsg"<<conn_sock<<endl;
		exit(1);
	}
	//signal(SIGPIPE,SIG_IGN);
	//cout<<"ooooo";
	//cout<<conn_sock<<endl;
	return true;
}

//-------------------------------------  接受消息包----------------
bool Connection::recvMsg()
{
	//rcvbuffer=new char[buffer_size];

	//logShow->m_textCtrl_feedbackShow.Clear();
	logShow.clear();
	bzero(rcvbuffer,buffer_size);
	if ((recv_size = recv(conn_sock, rcvbuffer, buffer_size, 0)) == -1)
	{
		//cout<<"robotConnect  conn_sock"<<conn_sock<<endl;
		//cout<<"recv_size"<<recv_size<<endl;
		perror("receive data");
		exit(1);
	}

	//cout<<"the size of recv_size = "<<recv_size<<endl;
	//cout<<"the size of offset = "<<offset<<endl;
	//m_textCtrl_feedbackShow->clear();

	memmove(buffer+offset,rcvbuffer,recv_size);
	//offset+=recv_size;
	ofstream ofile;
	ofstream ofileLog;
	ofile.open("log.txt");//,ios::app);
	ofileLog.open("/home/peng/desktop/WholeLog.txt",ios::app);

	ofstream ofile1;                              //////////////////////////////////////////////////
	ofile1.open(RT_filename.c_str(),ios::app);    //////////////////////////////////////////////////



	//m_textCtrl_commandShow->AppendText(str);
	//m_textCtrl_commandShow->AppendText(_T("\n"));

	for (int i=0;i<recv_size;i++)
	{
		if (rcvbuffer[i]==0)
		{
			rcvbuffer[i]=0x20;
		}
		//logShow->m_textCtrl_feedbackShow->AppendText(rcvbuffer[i]);

		logShow.push_back(rcvbuffer[i]);
		ofile<<rcvbuffer[i];
		ofileLog<<rcvbuffer[i];
		ofile1<<rcvbuffer[i];
	}

	//cout<<"the size of string "<<logShow.size()<<endl;
	//logShowConv=wxString(logShow.c_str(),wxConvUTF8);
	//cout<<"logShowConv Size of Connection file "<<logShowConv.Len()<<endl;
	ofile.close();
	ofileLog.close();
	ofile1.close();
	//delete[]rcvbuffer;
	return true;

}

void Connection::deleteBuff()
{
	delete[]rcvbuffer;
}

void Connection::closeConnection()
{
	close(conn_sock);
	cout<<"Close the communication with the robot"<<endl;
	offset=0;
}
