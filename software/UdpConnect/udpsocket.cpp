#include "udpsocket.h"

bool CUdpSocket::Socket()
{
    m_sSockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //ipv4, datagram transport, Udp;
			if (m_sSockfd < 0)
			{
				perror("socket error");
				return false;
			}
			return true;
}

bool CUdpSocket::Bind(const std::string &ip, uint16_t port)
{
    //define struct iPV4
    struct sockaddr_in addr;
    //give information of addr
    addr.sin_family = AF_INET;//ipv4
    addr.sin_port = htons(port);//port's data tansport
    addr.sin_addr.s_addr = inet_addr(ip.c_str());//ip's transport;
    socklen_t len = sizeof(struct sockaddr_in);
    int ret = bind(m_sSockfd, (struct sockaddr*)&addr, len);
    if (ret < 0)
    {
        perror("bind error");
        return false;
    }
    return true;

}

bool CUdpSocket::Recv(std::string *buf, std::string *ip, uint16_t *port)
{
    struct sockaddr_in peer_addr;//recive information of servent;
    socklen_t len = sizeof(struct sockaddr_in);
    char tmp[4096] = {0};
    int ret = recvfrom(m_sSockfd, tmp, 4096, 0, (struct sockaddr*)&peer_addr, &len);
    if (ret < 0)
    {
        perror("fail to receive");
        return false;
    }
    buf->assign(tmp, ret);//get information
    if (port != NULL)
    {
        *port = ntohs(peer_addr.sin_port);
    }
    if (ip != NULL)
    {
        *ip = inet_ntoa(peer_addr.sin_addr);
    }
    return true;

}

bool CUdpSocket::Send(const std::string &data, std::string &ip, const uint16_t port)
{
    	struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        socklen_t len = sizeof(struct sockaddr_in);
        int ret = sendto(m_sSockfd, data.c_str(), data.size(), 0, (struct sockaddr*)&addr, len);
        if (ret < 0)
        {
            perror("fail to send");
            return false;
        }
        return true;

}

bool CUdpSocket::Close()
{
        if (m_sSockfd > 0)
    {
        close(m_sSockfd);
        m_sSockfd = -1;
    }
    return true;
}
