#pragma once
#ifndef UDPSOCKET_H
#define UDPSOCKET_H
#include <cstdio>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>


class CUdpSocket{
private:
    int m_sSockfd;  //通道句柄

public:
    CUdpSocket():m_sSockfd(-1){};
    bool Socket();
    bool Bind(const std::string &ip, uint16_t port);
    bool Recv(std::string *buf, std::string *ip=NULL, uint16_t *port=NULL);
    bool Send(const std::string &data, std::string &ip, const uint16_t port);
    bool Close();
};
#endif