//
// Created by jiyguo on 2022/3/11.
//

#ifndef SOCKET_MYSOCKET_H
#define SOCKET_MYSOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cerrno>

#define BUFFER_SIZE 1024

class MySocket {
public:
    explicit MySocket(const char *ip = "192.168.1.99", const int& port = 6680);
    bool SendAndResv(const std::vector<double>& send_msg, std::vector<double>& resv_msg);
private:
    static int SocketRecvDoubleArray(double* data, char* p);

    int sock_Client; //客户端用于通信的Socket
    char receBuf[BUFFER_SIZE]{}; //发送数据的缓冲区
    char sendBuf[BUFFER_SIZE]{}; //接受数据的缓冲区
    struct sockaddr_in addr_server{}; //服务器的地址数据结构
    double datafromrobot[15]{};
};

#endif //SOCKET_MYSOCKET_H
