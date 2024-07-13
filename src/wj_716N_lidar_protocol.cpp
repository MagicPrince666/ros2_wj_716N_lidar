#include "wj_716N_lidar_protocol.h"
#include <iostream>
#include <cmath>

namespace wj_lidar
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    wj_716N_lidar_protocol::wj_716N_lidar_protocol(std::shared_ptr<ros::NodeHandle> node)
#else
    wj_716N_lidar_protocol::wj_716N_lidar_protocol(std::shared_ptr<rclcpp::Node> node)
#endif
    : ros_node_(node)
    {
        memset(&m_sdata, 0, sizeof(m_sdata));
        std::string frame_id;
	    m_u32PreFrameNo = 0;
	    m_u32ExpectedPackageNo = 0;
	    m_n32currentDataNo = 0;
        total_point = 1081;

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
        marker_pub_ = std::make_shared<ros::Publisher>(ros_node_->advertise<LaserScanMsg>("scan", 50));
        ros::Time scan_time = ros::Time::now();
        ros_node_->getParam("wj_716n_lidar/ros__parameters/frame_id", frame_id);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/frequency_scan", freq_scan);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/min_ang", scan_msg_.angle_min);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/max_ang", scan_msg_.angle_max);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/range_min", scan_msg_.range_min);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/range_max", scan_msg_.range_max);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/angle_increment", scan_msg_.angle_increment);
        ros_node_->getParam("wj_716n_lidar/ros__parameters/time_increment", scan_msg_.time_increment); 
#else
        marker_pub_ = ros_node_->create_publisher<LaserScanMsg>("scan", 50);
        rclcpp::Time scan_time = ros_node_->get_clock()->now();
        ros_node_->declare_parameter<std::string>("frame_id", frame_id);
        ros_node_->declare_parameter<int>("frequency_scan", 1);
        ros_node_->declare_parameter<double>("min_ang", -2.35619449);
        ros_node_->declare_parameter<double>("max_ang", 2.35619449);
        ros_node_->declare_parameter<double>("range_min", 0.0);
        ros_node_->declare_parameter<double>("range_max", 30.0);
        ros_node_->declare_parameter<double>("angle_increment", 0.017453 / 4);
        ros_node_->declare_parameter<double>("time_increment", 1 / 15.00000000 / 1440);

        ros_node_->get_parameter("frame_id", frame_id);
        ros_node_->get_parameter("frequency_scan", freq_scan);
        ros_node_->get_parameter("min_ang", scan_msg_.angle_min);
        ros_node_->get_parameter("max_ang", scan_msg_.angle_max);
        ros_node_->get_parameter("range_min", scan_msg_.range_min);
        ros_node_->get_parameter("range_max", scan_msg_.range_max);
        ros_node_->get_parameter("angle_increment", scan_msg_.angle_increment);
        ros_node_->get_parameter("time_increment", scan_msg_.time_increment); 
#endif
        scan_msg_.header.stamp = scan_time;
        scan_msg_.header.frame_id = frame_id;

        scan_msg_.angle_increment = 0.017453 / 4;
        if (freq_scan == 1) //0.25°_15hz
        {
            scan_msg_.time_increment = 1 / 15.00000000 / 1440;
            total_point = 1081;
        }
        else if (freq_scan == 2) //0.25°_25hz
        {
            scan_msg_.time_increment = 1 / 25.00000000 / 1440;
            total_point = 1081;
        }

        // adjust angle_min to min_ang config param
        index_start = (scan_msg_.angle_min + 2.35619449) / scan_msg_.angle_increment;
        // adjust angle_max to max_ang config param
        index_end = 1081 - ((2.35619449 - scan_msg_.angle_max) / scan_msg_.angle_increment);
        int samples = index_end - index_start;
        scan_msg_.ranges.resize(samples);
        scan_msg_.intensities.resize(samples);

        std::cout << "frame_id:" << scan_msg_.header.frame_id << std::endl;
        std::cout << "min_ang:" << scan_msg_.angle_min << std::endl;
        std::cout << "max_ang:" << scan_msg_.angle_max << std::endl;
        std::cout << "angle_increment:" << scan_msg_.angle_increment << std::endl;
        std::cout << "time_increment:" << scan_msg_.time_increment << std::endl;
        std::cout << "range_min:" << scan_msg_.range_min << std::endl;
        std::cout << "range_max:" << scan_msg_.range_max << std::endl;
        std::cout << "samples_per_scan:" << samples << std::endl;

        std::cout << "wj_716N_lidar_protocl start success" << std::endl;
    }

    bool wj_716N_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
    {
        if (reclen > MAX_LENGTH_DATA_PROCESS) {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
            return false;
        }

        if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS) {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
            return false;
        }
        memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
        m_sdata.m_u32in += reclen;
        while (m_sdata.m_u32out < m_sdata.m_u32in) {
            if (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA) {
                unsigned l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) |
                                        (m_sdata.m_acdata[m_sdata.m_u32out + 3] << 0);
                l_u32reallen = l_u32reallen + 4;

                if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1)) {
                    if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen)) {
                        m_sdata.m_u32out += l_u32reallen;
                    } else {
                        std::cout << "continuous search frame header" << std::endl;
                        m_sdata.m_u32out++;
                    }
                }
                else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS) {
                    m_sdata.m_u32out++;
                } else {
                    break;
                }
            } else {
                m_sdata.m_u32out++;
            }
        } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

        if (m_sdata.m_u32out >= m_sdata.m_u32in) {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
        } else if (m_sdata.m_u32out < m_sdata.m_u32in && m_sdata.m_u32out != 0) {
            movedata(m_sdata);
        }
        return true;
    }

    void wj_716N_lidar_protocol::movedata(DataCache &sdata)
    {
        for (uint32_t i = sdata.m_u32out; i < sdata.m_u32in; i++) {
            sdata.m_acdata[i - sdata.m_u32out] = sdata.m_acdata[i];
        }
        sdata.m_u32in = sdata.m_u32in - sdata.m_u32out;
        sdata.m_u32out = 0;
    }

    bool wj_716N_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
    {
        if (len > 0) {
            if (checkXor(data, len)) {
                protocl(data, len);
            } else {
                return false;
            }
        } else {
            return false;
        }
        return true;
    }

    bool wj_716N_lidar_protocol::protocl(unsigned char *data, const int len)
    {
        if ((data[22] == 0x02 && data[23] == 0x02) || (data[22] == 0x02 && data[23] == 0x01)) { //command type:0x02 0x01/0X02 
            heartstate = true;
            uint32_t l_n32TotalPackage = data[80];
            uint32_t l_n32PackageNo = data[81];
            unsigned int l_u32FrameNo = (data[75] << 24) + (data[76] << 16) + (data[77] << 8) + data[78];
            int l_n32PointNum = (data[83] << 8) + data[84];
            int l_n32Frequency = data[79];

            if(l_n32Frequency != freq_scan) {
                std::cout << "The scan frequency does not match the one you setted!"<< std::endl;
                return false;
            }

            if(m_u32PreFrameNo != l_u32FrameNo) {
                m_u32PreFrameNo = l_u32FrameNo;
                m_u32ExpectedPackageNo = 1;
                m_n32currentDataNo = 0;
            }

            if (l_n32PackageNo == m_u32ExpectedPackageNo && m_u32PreFrameNo == l_u32FrameNo) {
                if(data[82] == 0x00) { //Dist
                    for (int j = 0; j < l_n32PointNum; j++) {
                        scandata[m_n32currentDataNo] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                       ((unsigned char)data[86 + j * 2]);
                        scandata[m_n32currentDataNo] /= 1000.0;
                        scanintensity[m_n32currentDataNo] = 0;
                        if (scandata[m_n32currentDataNo] > scan_msg_.range_max || scandata[m_n32currentDataNo] < scan_msg_.range_min || scandata[m_n32currentDataNo] == 0) {
                            scandata[m_n32currentDataNo] = NAN;
                        }
                        m_n32currentDataNo++;
                    }
                    m_u32ExpectedPackageNo++;
                } else if(data[82] == 0x01 && m_n32currentDataNo >= total_point) { //intensities
                    for (int j = 0; j < l_n32PointNum; j++) {
                        scanintensity[m_n32currentDataNo - total_point] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                                          ((unsigned char)data[86 + j * 2]);
                        m_n32currentDataNo++;
                    }
                    m_u32ExpectedPackageNo++;
                }

                if(m_u32ExpectedPackageNo - 1 == l_n32TotalPackage) {

                    for (int i = index_start; i < index_end; i++) {
                        scan_msg_.ranges[i - index_start] = scandata[i];
                        if(scandata[i - index_start] == NAN) {
                            scan_msg_.intensities[i - index_start] = 0;
                        } else {
                            scan_msg_.intensities[i - index_start] = scanintensity[i];
                        }
                    }

                    
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
                    ros::Time scan_time = ros::Time::now();
#else
                    rclcpp::Time scan_time = ros_node_->get_clock()->now();
#endif
                    scan_msg_.header.stamp = scan_time;
                    marker_pub_->publish(scan_msg_);
                }
            } 
            return true;
        } else {
            return false;
        }
    }

    bool wj_716N_lidar_protocol::checkXor(unsigned char *recvbuf, int recvlen)
    {
        int i = 0;
        unsigned char check = 0;
        unsigned char *p = recvbuf;
        int len;
        if (*p == 0xFF) {
            p = p + 2;
            len = recvlen - 6;
            for (i = 0; i < len; i++) {
                check ^= *p++;
            }
            p++;
            if (check == *p) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

}
