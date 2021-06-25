#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<map>

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

void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    sensor_msgs::LaserScan arc_draw;
    arc_draw.header = scan_msg->header;
    arc_draw.angle_min = scan_msg->angle_min;
    arc_draw.angle_max = scan_msg->angle_max;
    arc_draw.angle_increment = scan_msg->angle_increment;
    arc_draw.range_min = scan_msg->range_min;
    arc_draw.range_max = scan_msg->range_max;
    arc_draw.ranges.resize(1081);
    float new_scan[1081]; 
    float new_scan_filter[1081];
    for(int i=0;i<1081;i++)
    {
        new_scan[i]=scan_msg->ranges[i];
    }

 
    int i=1;
    int count=0;
    while(i<1081)
    {
        if(    (   (new_scan[i-1]) - (new_scan[i]) >1 )   &&   (  (new_scan[i])-(new_scan[i+1])<0.02  )   )
            //判断筒的最右端
             {    
                i++;
                count=i;
                while(   (  (new_scan[i-1]) - (new_scan[i])   <      0.02   )    &&  (  (new_scan[i]) - (new_scan[i-1])   <   0.02  )   )
                    {
                             //arc_draw.ranges[i-1]=new_scan[i-1];
                            new_scan_filter[i-1]=new_scan[i-1];
                             i++;
                    } 
               if(   (   new_scan_filter[i-1]-   new_scan_filter[count-1]    <0.03 )    ||    (   new_scan_filter[count-1]    -  new_scan_filter[i-1]     <0.03    )      )
               {
                   for(int k=count;k<i;k++)
                   {
                       arc_draw.ranges[k-1]=new_scan_filter[i-1];
                   }
               }        
            }
        i++;
    }
     arc_draw_publisher_.publish(arc_draw);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arc_draw_node"); // 节点的名字
    LaserScan laser_scan;
    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}