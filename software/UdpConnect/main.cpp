#include <iostream>
#include <string>
#include "udpsocket.h"
#include "keyboard.h"
using namespace std;

#define CHECK_RET(q) if((q)==false){return -1;}
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cout << "Usage: ./udpcli ip port\n";
		return -1;
	}
	string srv_ip = argv[1];
	uint16_t srv_port = stoi(argv[2]);

	CUdpSocket cli_sock;
	//创建套接孄1�7
	CHECK_RET(cli_sock.Socket());
	//绑定数据(不推荄1�7)
	while(1)
	{
		cout << "client say:";
		string buf;
		int command=scanKeyboard();
        if(command=='w')
        {
            buf = "start";
            goto SEND;
        }
        if(command=='s')
        {
            buf = "stop";
            goto SEND;
        }
		 if(command=='p')
        {
            buf = "pumpPositive";
            goto SEND;
        }
		 if(command=='n')
        {
            buf = "pumpNegative";
            goto SEND;
        }
		 if(command=='u')
        {
            buf = "forward";
            goto SEND;
        }
		 if(command=='j')
        {
            buf = "back";
            goto SEND;
        }
		 if(command=='h')
        {
            buf = "left";
            goto SEND;
        }
		 if(command=='k')
        {
            buf = "right";
            goto SEND;
        }
		 if(command=='o')
        {
            buf = "goupup";
            goto SEND;
        }
         if(command=='i')
        {
            buf = "op";
            goto SEND;
        }
		 if(command=='l')
        {
            buf = "down";
            goto SEND;
        }
		 if(command=='0')
        {
            buf = "0";
            goto SEND;
        }
		 if(command=='1')
        {
            buf = "1";
            goto SEND;
        }
		 if(command=='2')
        {
            buf = "2";
            goto SEND;
        }
		 if(command=='3')
        {
            buf = "3";
            goto SEND;
        }
        if(command=='a')
        {
            buf = "left";
            goto SEND;
        }
        if(command=='d')
        {
            buf = "right";
            goto SEND;
        }
        if(command=='q')
        {
            buf = "rotateleft";
            goto SEND;
        }
        if(command=='e')
        {
            buf = "rotateright";
            goto SEND;
        }
        if(command=='c')
        {
            buf = "ChangeStancePos";
            goto SEND;
        }
        if(command=='x')
        {
            buf = "xed";
            goto SEND;
        }        

        SEND:
		CHECK_RET(cli_sock.Send(buf, srv_ip, srv_port));
		//接收数据
		buf.clear();
		// CHECK_RET(cli_sock.Recv(&buf));
		// cout << "server say: " << buf << endl;
	}
	//关闭套接孄1�7
	cli_sock.Close();
	return 0;
}
