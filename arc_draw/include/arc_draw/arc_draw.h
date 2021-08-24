#ifndef ARC_DRAW_H
#define ARC_DRAW_H
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
class LaserScan
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher arc_draw_publisher_; // 声明一个Publisher

public:
    LaserScan();
    ~LaserScan();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};
#endif