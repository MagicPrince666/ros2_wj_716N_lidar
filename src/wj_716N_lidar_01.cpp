#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#else
#include "rclcpp/rclcpp.hpp"
#endif
#include "async_client.h"
#include "wj_716N_lidar_protocol.h"
using namespace wj_lidar;

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */
wj_716N_lidar_protocol *protocol;
Async_Client *client;

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
void callback(wj_716N_lidar::wj_716N_lidarConfig &config,uint32_t level)
{
    protocol->setConfig(config,level);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros::init(argc, argv, "wj_716N_lidar_01");
    auto node = std::make_shared<ros::NodeHandle>("~");
    
    std::string hostname;
    node->getParam("hostname", hostname);
    std::string port;
    node->getParam("port", port);
    cout << "laser ip: " << hostname << ", port:" << port <<endl;

    auto protocol = std::make_shared<wj_716N_lidar_protocol>(node);
    dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig> server;
    dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig>::CallbackType f;
    f = std::bind(&callback, std::placeholders::_1, std::placeholders::_2);
    server.setCallback(f);

    client = new Async_Client(protocol.get());
    protocol->heartstate = false;
    while(!client->m_bConnected) {
        ROS_INFO("Start connecting laser!");
        if(client->connect(hostname.c_str(),atoi(port.c_str()))) {
            ROS_INFO("Succesfully connected. Hello wj_716N_lidar!");
        } else {
            ROS_INFO("Failed to connect to laser. Waiting 5s to reconnect!");
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
    ros::shutdown();
    
#else
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wj_716N_lidar_01");
    std::string hostname;
    std::string port;
    // declare ros2 param
    node->declare_parameter<std::string>("hostname", hostname);
    node->declare_parameter<std::string>("port", port);

    // get ros2 param
    node->get_parameter("hostname", hostname);
    node->get_parameter("port", port);
    cout << "laser ip: " << hostname << ", port:" << port <<endl;
    std::unique_ptr<wj_716N_lidar_protocol> protocol(new wj_716N_lidar_protocol(node));

    client = new Async_Client(protocol.get());
    protocol->heartstate = false;
    rclcpp::Rate loop_5s_rate(5);
    while(!client->m_bConnected) {
        RCLCPP_INFO(node->get_logger(), "Start connecting laser!");
        if(client->connect(hostname.c_str(),atoi(port.c_str()))) {
            RCLCPP_INFO(node->get_logger(), "Succesfully connected. Hello wj_716N_lidar!");
        } else {
            RCLCPP_INFO(node->get_logger(), "Failed to connect to laser. Waiting 5s to reconnect!");
        }
        loop_5s_rate.sleep();
    }

    // rclcpp::Rate loop_rate(2);
    // while(rclcpp::ok()) {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    //     if(client->m_bConnected) {
    //         if(protocol->heartstate) {
    //             protocol->heartstate = false;
    //         } else {
    //             client->m_bConnected = false;
    //         }
    //     } else {
    //         //reconnect
    //         if(!client->m_bReconnecting) {
    //             std::thread t(std::bind(&Async_Client::reconnect, client));
    //         }
    //     }
    // }

    rclcpp::spin(node);
    rclcpp::shutdown();
#endif
}
