//
// Created by jiyguo on 2022/3/11.
//
#include "mySocket/mySocket.h"

#define BUFFER_SIZE 1024
#define MYPORT 6680

using namespace std;

int SocketRecvDoubleArray(double* data, char* p)
{

    int n = 0;

    char s[1024];
    memset(s, 0, sizeof(s));
    char* q = s;
    while (!('\0' == *p || '\r' == *p || '\n' == *p))
    {
        q = s;
        memset(s, '\0', sizeof(s));
        if (',' == *p)
        {
            p++;
            if ('\0' == *p || '\r' == *p || '\n' == *p)
            {
                break;
            }
        }
        while (!('\0' == *p || '\r' == *p || '\n' == *p || ',' == *p))//  ȡ
        {
            *q = *p;
            q++;
            p++;
        }
        data[n] = atof(s);
        n++;
    }
    return n;
}


void controlOnRobot() {
    int sock_Client; //客户端用于通信的Socket
    char receBuf[BUFFER_SIZE]; //发送数据的缓冲区
    char sendBuf[BUFFER_SIZE]; //接受数据的缓冲区
    //初始化UDP
    sock_Client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    // cout << "socket返回值" << sock_Client << endl;
    //创建客户端用于通信的Socket
    struct sockaddr_in addr_server; //服务器的地址数据结构
    addr_server.sin_family = AF_INET;
    addr_server.sin_port = htons(MYPORT);
    addr_server.sin_addr.s_addr = inet_addr("192.168.1.99");

    std::string sendtorobot1;
    const char* sendtorobot = nullptr;
    double datafromrobot[15];

    sendtorobot1 = "0,0,0,0,0,0\n";
    sendtorobot = sendtorobot1.c_str();
    sendto(sock_Client, sendtorobot, strlen(sendtorobot), 0, (struct sockaddr *)&addr_server, sizeof(addr_server));

    int i = 100;
    while(i--){

        //接收数据并解析字符串
        memset(receBuf, 0, sizeof(receBuf));
        int last = recvfrom(sock_Client, receBuf, sizeof(receBuf), 0, nullptr,nullptr);
        int fromrobot = SocketRecvDoubleArray(datafromrobot, receBuf);

        // 获取关节角度
        double JointAngle1, JointAngle2, JointAngle3, JointAngle4, JointAngle5, JointAngle6;
        /*************************************
        获取关节角
        **************************************/
        JointAngle1 = datafromrobot[0];
        JointAngle2 = datafromrobot[1];
        JointAngle3 = datafromrobot[2];
        JointAngle4 = datafromrobot[3];
        JointAngle5 = datafromrobot[4];
        JointAngle6 = datafromrobot[5];
        cout<<"jnt:";
        for (int j = 0; j < 6; ++j) {
            cout<<datafromrobot[j]<<" , ";
        }
        cout<<endl;

        sendtorobot1 = std::to_string(0) + ',' + std::to_string(0)
                       + ',' + std::to_string(0) + ',' + std::to_string(0)
                       + ',' + std::to_string(0) + ',' + std::to_string(0) + '\n';
        sendtorobot = sendtorobot1.c_str();
        sendto(sock_Client, sendtorobot, strlen(sendtorobot), 0, (struct sockaddr *)&addr_server, sizeof(addr_server));
        sleep(1);
    }
    close(sock_Client);
}

int main()
{
    //controlOnRobot();
    MySocket mySocket;
    std::vector<double> send_msg(6,0);
    std::vector<double> resv_msg;
    if(!mySocket.SendAndResv(send_msg, resv_msg)){
        std::cerr<<"Socket error ! "<<std::endl;
    }
    cout<<"jnt:";
    for (int j = 0; j < 6; ++j) {
        cout<<resv_msg[j]<<" , ";
    }
    cout<<endl;
    return 0;
}


