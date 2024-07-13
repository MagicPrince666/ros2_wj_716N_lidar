#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <string>
#include <unistd.h>
#include <vector>
#if defined(USE_ELOQUENT_VERSION) || defined(USE_DASHING_VERSION)
#include <experimental/filesystem>
#else
#include <filesystem>
#endif
#include <memory>
#include <thread>
#include "wj_716N_lidar_protocol.h"

using namespace wj_lidar;

#define MAX_LENGTH 50000

class Async_Client
{
public:
    Async_Client(std::shared_ptr<wj_716N_lidar_protocol> protocol);
    ~Async_Client();

    bool connect(std::string ip, int port);
    bool disconnect();
    void recvData();
    void reconnect();
    bool SendData(unsigned char buf[], int length);

    bool m_bConnected;
    bool m_bReconnecting;

private:
    std::shared_ptr<wj_716N_lidar_protocol> m_pProtocol;
    std::string m_sServerIp;
    int m_iServerPort;
    boost::asio::io_service m_io;
    boost::asio::ip::tcp::endpoint m_ep;
    std::shared_ptr<boost::asio::ip::tcp::socket> m_pSocket;
    std::thread recv_thread_;

    unsigned char m_aucReceiveBuffer[MAX_LENGTH];
};

#endif // ASYNC_CLIENT_H
