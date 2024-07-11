#include "async_client.h"

Async_Client::Async_Client(wj_716N_lidar_protocol *protocol)
{
    m_pProtocol = protocol;
    m_bConnected = false;
    m_bReconnecting = false;
    m_pSocket = std::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(m_io));
    std::cout << "TCP-Connection is initialized!" << std::endl;
}

Async_Client::~Async_Client()
{
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    disconnect();
}

bool Async_Client::connect(std::string ip, int port) 
{
    try
    {
        if (m_bConnected) {
            return false;
        }
        if(m_pSocket->is_open()) {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        m_sServerIp = ip;
        m_iServerPort = port;
        m_ep = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port);
        m_pSocket->connect(m_ep,ec);
        if(ec) {
            m_bConnected = false;
            return false;
        } else {
            m_bConnected = true;
            recv_thread_ = std::thread([](Async_Client *p_this) { p_this->recvData(); }, this);
            return true;
        }
    }
    catch (std::exception &e)
    {
        return false;
    }
}

bool Async_Client::disconnect() 
{
    try
    {
        std::cout << "Disconnecting connection!" << std::endl;
        m_bConnected = false;
        if(m_pSocket->is_open()) {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        return true;
    }
    catch(std::exception& e)
    {
        return false;
    }
}

void Async_Client::recvData()
{
    try
    {
        while (m_bConnected) {
            int len = m_pSocket->read_some(boost::asio::buffer(m_aucReceiveBuffer));
            if (len > 0) {
                m_pProtocol->dataProcess(m_aucReceiveBuffer,len);
            }
        }
    } catch (std::exception e) {
    }
    m_bConnected = false;
}

void Async_Client::reconnect()
{
    m_bReconnecting = true;
    while (!m_bConnected) {
        if (m_pSocket->is_open()) {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
            std::cout << "Initializing network!" << std::endl;
        }
        sleep(3);
        std::cout << "Start reconnecting laser!" << std::endl;
        sleep(2);
        if (connect(m_sServerIp, m_iServerPort)) {
            std::cout << "Succesfully connected!" << std::endl;
            break;
        } else {
            std::cout << "Failed to reconnect!" << std::endl;
        }
        sleep(2);
    }
    m_bReconnecting = false;
}

bool Async_Client::SendData(unsigned char buf[], int length)
{
    try
    {
        if (m_pSocket->is_open() && m_bConnected)
        {
            int st = m_pSocket->send(boost::asio::buffer(buf, length));
            if (st == length) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    catch (std::exception e)
    {
        return false;
    }
}
