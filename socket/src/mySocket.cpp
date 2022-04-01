//
// Created by jiyguo on 2022/3/11.
//

#include "mySocket/mySocket.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include <iostream>
#include <vector>
#include <string>

#include <cstring>
#include <cstdlib>

MySocket::MySocket(const char *ip, const int& port){
    sock_Client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    // cout << "socket返回值" << sock_Client << endl;
    //创建客户端用于通信的Socket
    addr_server = {};
    addr_server.sin_family = AF_INET;
    addr_server.sin_port = htons(port);
    addr_server.sin_addr.s_addr = inet_addr(ip);
}

bool MySocket::SendAndResv(const std::vector<double>& send_msg, std::vector<double>& resv_msg){
    if (send_msg.size()!=6){
        std::cerr<<"send_msg's size must be 6 ! "<<std::endl;
        return false;
    }
    std::string sendtorobot;
    sendtorobot = std::to_string(send_msg[0]) + ',' + std::to_string(send_msg[1]) + ','
            + std::to_string(send_msg[2]) + ',' + std::to_string(send_msg[3]) + ','
            + std::to_string(send_msg[4]) + ',' + std::to_string(send_msg[5]) + '\n';
    const char* send2robot = sendtorobot.c_str();
    ssize_t send_size = sendto(sock_Client, send2robot, strlen(send2robot), 0, (struct sockaddr *)&addr_server, sizeof(addr_server));
    if(send_size==-1) {
        std::cerr<<"sendto failed ! "<<std::endl;
        return false;
    }
    //接收数据并解析字符串
    memset(receBuf, 0, sizeof(receBuf));
    ssize_t recv_size = recvfrom(sock_Client, receBuf, sizeof(receBuf), 0, nullptr,nullptr);
    if(recv_size==-1) {
        std::cerr<<"recvfrom failed ! "<<std::endl;
        return false;
    }
    int fromrobot = SocketRecvDoubleArray(datafromrobot, receBuf);
    resv_msg.resize(6);
    for (int i = 0; i < 6; ++i) {
        resv_msg[i] = datafromrobot[i];
    }
    return true;
}


int MySocket::SocketRecvDoubleArray(double* data, char* p)
{
    int n = 0;
    char s[1024];
    memset(s, 0, sizeof(s));
    char* q = s;
    while (!('\0' == *p || '\r' == *p || '\n' == *p)){
        q = s;
        memset(s, '\0', sizeof(s));
        if (',' == *p) {
            p++;
            if ('\0' == *p || '\r' == *p || '\n' == *p){
                break;
            }
        }
        while (!('\0' == *p || '\r' == *p || '\n' == *p || ',' == *p)){
            *q = *p;
            q++;
            p++;
        }
        data[n] = atof(s);
        n++;
    }
    return n;
}

