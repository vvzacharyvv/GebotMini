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
	//鍒涘缓濂楁帴瀛�
	CHECK_RET(cli_sock.Socket());
	//缁戝畾鏁版嵁(涓嶆帹鑽�)
	while(1)
	{
		//鍙戦€佹暟鎹�
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
      
        SEND:
		CHECK_RET(cli_sock.Send(buf, srv_ip, srv_port));
		//鎺ユ敹鏁版嵁
		buf.clear();
		// CHECK_RET(cli_sock.Recv(&buf));
		// cout << "server say: " << buf << endl;
	}
	//鍏抽棴濂楁帴瀛�
	cli_sock.Close();
	return 0;
}
