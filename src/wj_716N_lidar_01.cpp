#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#else
#include "rclcpp/rclcpp.hpp"
#endif
#include "async_client.h"
#include "wj_716N_lidar_protocol.h"
using namespace wj_lidar;

#ifdef BACKTRACE_DEBUG
#include <execinfo.h>

#define PRINT_SIZE_ 100

static void _signal_handler(int signum)
{
    void *array[PRINT_SIZE_];
    char **strings;

    size_t size = backtrace(array, PRINT_SIZE_);
    strings     = backtrace_symbols(array, size);

    if (strings == nullptr) {
        fprintf(stderr, "backtrace_symbols");
        exit(EXIT_FAILURE);
    }

    switch (signum) {
    case SIGSEGV:
        fprintf(stderr, "widebright received SIGSEGV! Stack trace:\n");
        break;

    case SIGPIPE:
        fprintf(stderr, "widebright received SIGPIPE! Stack trace:\n");
        break;

    case SIGFPE:
        fprintf(stderr, "widebright received SIGFPE! Stack trace:\n");
        break;

    case SIGABRT:
        fprintf(stderr, "widebright received SIGABRT! Stack trace:\n");
        break;

    default:
        break;
    }

    for (size_t i = 0; i < size; i++) {
        fprintf(stderr, "%d %s \n", i, strings[i]);
    }

    free(strings);
    // signal(signum, SIG_DFL); /* 还原默认的信号处理handler */

    exit(1);
}
#endif

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */

int main(int argc, char **argv)
{
#ifdef BACKTRACE_DEBUG
    signal(SIGPIPE, _signal_handler); // SIGPIPE，管道破裂。
    signal(SIGSEGV, _signal_handler); // SIGSEGV，非法内存访问
    signal(SIGFPE, _signal_handler);  // SIGFPE，数学相关的异常，如被0除，浮点溢出，等等
    signal(SIGABRT, _signal_handler); // SIGABRT，由调用abort函数产生，进程非正常退出
#endif

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros::init(argc, argv, "wj_716n_lidar_01");
    auto node = std::make_shared<ros::NodeHandle>();
    
    std::string hostname;
    node->getParam("wj_716n_lidar/ros__parameters/hostname", hostname);
    std::string port;
    node->getParam("wj_716n_lidar/ros__parameters/port", port);
    std::cout << "laser ip: " << hostname << ", port:" << port << std::endl;

    std::shared_ptr<wj_716N_lidar_protocol> protocol = std::make_shared<wj_716N_lidar_protocol>(node);

    auto client = std::make_shared<Async_Client>(protocol);
    protocol->heartstate = false;
    while(!client->m_bConnected) {
        ROS_INFO("Start connecting laser!");
        if(client->connect(hostname.c_str(),atoi(port.c_str()))) {
            ROS_INFO("Succesfully connected. Hello wj_716N_lidar!");
        } else {
            ROS_INFO("Failed to connect to laser. Waiting 5s to reconnect!");
        }
        if (!ros::ok()) {
            break;
        }
        ros::Duration(5).sleep();
    }
    while(ros::ok()) {
        ros::spinOnce();
        ros::Duration(2).sleep();
        if(client->m_bConnected) {
            if(protocol->heartstate) {
                protocol->heartstate = false;
            } else {
                client->m_bConnected = false;
            }
        } else {
            //reconnect
            if(!client->m_bReconnecting) {
                std::thread t(std::bind(&Async_Client::reconnect, client));
            }
        }
    }
    client->disconnect();
    ros::shutdown();
    
#else
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wj_716n_lidar_01");
    std::string hostname;
    std::string port;
    // declare ros2 param
    node->declare_parameter<std::string>("hostname", hostname);
    node->declare_parameter<std::string>("port", port);

    // get ros2 param
    node->get_parameter("hostname", hostname);
    node->get_parameter("port", port);
    std::cout << "laser ip: " << hostname << ", port:" << port << std::endl;
    std::shared_ptr<wj_716N_lidar_protocol> protocol = std::make_shared<wj_716N_lidar_protocol>(node);

    auto client = std::make_shared<Async_Client>(protocol);
    protocol->heartstate = false;
    rclcpp::Rate loop_2s_rate(2);
    while(!client->m_bConnected) {
        RCLCPP_INFO(node->get_logger(), "Start connecting laser!");
        if(client->connect(hostname.c_str(), atoi(port.c_str()))) {
            RCLCPP_INFO(node->get_logger(), "Succesfully connected. Hello wj_716N_lidar!");
        } else {
            RCLCPP_INFO(node->get_logger(), "Failed to connect to laser. Waiting 5s to reconnect!");
        }
        loop_2s_rate.sleep();
        if (!rclcpp::ok()) {
            break;
        }
    }

    while(rclcpp::ok()) {
        loop_2s_rate.sleep();
        if(client->m_bConnected) {
            if(protocol->heartstate) {
                protocol->heartstate = false;
            } else {
                client->m_bConnected = false;
            }
        } else {
            //reconnect
            if(!client->m_bReconnecting) {
                std::thread t(std::bind(&Async_Client::reconnect, client));
            }
        }
    }
    client->disconnect();

    // rclcpp::spin(node);
    rclcpp::shutdown();
#endif
}
