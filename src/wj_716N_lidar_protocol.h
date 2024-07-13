#ifndef WJ_716N_LIDAR_PROTOCOL_H
#define WJ_716N_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
using LaserScanMsg = sensor_msgs::LaserScan;
#else
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
using LaserScanMsg = sensor_msgs::msg::LaserScan;
#endif

namespace wj_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
typedef struct TagDataCache
{
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
}DataCache;

class wj_716N_lidar_protocol
{
public:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    wj_716N_lidar_protocol(std::shared_ptr<ros::NodeHandle> node);
#else
    wj_716N_lidar_protocol(std::shared_ptr<rclcpp::Node> node);
#endif
    ~wj_716N_lidar_protocol() {}
    bool dataProcess(unsigned char *data,const int reclen);
    bool protocl(unsigned char *data,const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);
    void send_scan(const char *data,const int len);

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    std::shared_ptr<ros::NodeHandle> ros_node_;
    std::shared_ptr<ros::Publisher> marker_pub_;
#else
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<LaserScanMsg>::SharedPtr marker_pub_;
    // rclcpp::TimerBase::SharedPtr laser_timer_;
#endif
    LaserScanMsg scan_msg_;
    bool heartstate;

private:
    void movedata(DataCache &sdata);
    DataCache   m_sdata;
    uint32_t m_u32PreFrameNo;
    uint32_t m_u32ExpectedPackageNo;
    int m_n32currentDataNo;
    float scandata[1081];
    float scanintensity[1081];
    int total_point;
    int index_start;
    int index_end;
    int freq_scan;
};

}
#endif // WJ_716N_LIDAR_PROTOCOL_H
