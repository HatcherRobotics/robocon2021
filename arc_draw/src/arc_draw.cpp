#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<map>
#include<math.h>
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
   
    for (int j = 0; j < 1081;j++)
    {
         new_scan_filter[j] = 0;
    }
     for (int j = 0; j < 1081;j++)
    {
        if (!std::isfinite(scan_msg->ranges[j]))
        {
            continue;
        }
    }
    for(int j=0;j<1081;j++)
    {
        new_scan[j]=scan_msg->ranges[j];
     //   std::cout<< new_scan[i]<<std::endl;
    }

    int i=440;
    int start_point=0;
    int end_point=0;
    /*float result[10];//最多有10个筒
    int bucket_number=0;//数到筒的个数*/
    float detect_distance[6];
     int detect_number=0;
    while(i<640)// i代表线的序号，总共有1081条线(0--1080)
    {
        if(    (   new_scan[i-1]) - (new_scan[i]) >1       )
        {    
                
                 i++;
                start_point=i-1;
                do
                 {
                new_scan_filter[i-1]=new_scan[i-1];
                i++;
                 }while (     ! ((new_scan[i]-new_scan[i-1] )  >  1 )     );
                end_point=i-1;
                for(int k=start_point;k<end_point;k++)
                {
                    arc_draw.ranges[k]=new_scan_filter[k];       
                }
                detect_distance[detect_number]=new_scan_filter[(start_point+end_point)/2];
                if(end_point-start_point>5)
                {
                        std::cout<< "detect_angle   "<<(start_point+end_point)/2*0.25-135<<"degrees   "<<detect_distance[detect_number]<<std::endl;    
                }        
         }
         i++;
        detect_number++;
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