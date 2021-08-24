#include "arc_draw.h"
// 构造函数
LaserScan::LaserScan() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Arc Draw Started.\033[0m");

    // 将雷达的回调函数与订阅的topic进行绑定   laser_scan scan
    laser_scan_subscriber_ = node_handle_.subscribe("scan", 1, &LaserScan::ScanCallback, this);
    // 将提取后的点发布到 arc_draw 这个topic
    arc_draw_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("arc_draw", 1, this);
}

LaserScan::~LaserScan()
{
}