#include<map>
#include<math.h>
#include "arc_draw.h"
#include "send_value.h"


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
    float new_scan_filter[1081]={0};
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
        if(scan_msg->ranges[j]<10&&(scan_msg->ranges[j]>1))
        {
            new_scan[j]=scan_msg->ranges[j];     
        }
        else
        {
            new_scan[j]=15;
        } 
    }
    int i=400;  
    int start_point=0;
    int end_point=0;
    int mid_point=0;
    float detect_distance[50]={0},distance_1[50]={0}, distance_2[50]={0}, distance_3[50]={0};//直接给出三只箭的直线距离，无需换算
    float interval_1 = 0.045;//雷达与第一支箭的间距
    float interval_2 = 0.2+0.045;//雷达与第二支箭的间距
    float interval_3 = 0.2+0.045+0.2; //雷达与第三只箭的间距
    int detect_number=0;
    float detect_angle[50]={0},angle_1[50]={0},angle_2[50]={0},angle_3[50]={0};//直接给出三只箭与筒的角度，无需换算
    
    while(i<720)// i代表线的序号，总共有1081条线(0--1080)
    {
        //int length=0;
        if( ( (new_scan[i-1]) - (new_scan[i]) ) > 0.05 )
        {    
            detect_number++;
            i++;
            start_point=i-1;
            do
            {
                new_scan_filter[i-1]=new_scan[i-1];
                i++;
            //    length++;                  //此处可改为i-2
            }while (    ( ! ((new_scan[i]-new_scan[i-2] )  > 0.05 ) )  && (i<720)  );//改1
            end_point=i-1;
            
                for(int k=start_point;k<end_point+1;k++)
                {
                    arc_draw.ranges[k]=new_scan_filter[k];       
                }                                                       
            mid_point=(start_point+end_point)/2;
            detect_distance[detect_number-1]=new_scan_filter[mid_point];
            
            detect_angle[detect_number-1]=90-(  (900-mid_point)*0.25   );//改四
    
            distance_1[detect_number-1]=sqrt( pow(interval_1,2.0)+pow(detect_distance[detect_number-1],2.0)-2*interval_1*detect_distance[detect_number-1]*cos(detect_angle[detect_number-1]*180/3.1416));
            distance_3[detect_number-1]=sqrt( pow(interval_3,2.0)+pow(detect_distance[detect_number-1],2.0)-2*interval_3*detect_distance[detect_number-1]*cos(detect_angle[detect_number-1]*180/3.1416)); 
            float cos_beta_1 = (pow(interval_1,2.0)+pow(distance_1[detect_number-1],2.0)-pow(detect_distance[detect_number-1],2.0))/(2*interval_1*distance_1[detect_number-1]);
            float cos_beta_3 = (pow(interval_3,2.0)+pow(distance_3[detect_number-1],2.0)-pow(detect_distance[detect_number-1],2.0))/(2*interval_3*distance_3[detect_number-1]);
            float sin_theta=sin(detect_angle[detect_number-1]*3.1415926/180);
            if(sin_theta>1)     
            {
                sin_theta=1.0;
            }
            float sin_beta_1=detect_distance[detect_number-1]*sin_theta/distance_1[detect_number-1];
            float sin_beta_3=detect_distance[detect_number-1]*sin_theta/distance_3[detect_number-1];
            if(sin_beta_1>1)
            {
                sin_beta_1=1.0;
            }
            if(sin_beta_3>1)
            {
                sin_beta_3=1.0;
            }
            if(cos_beta_1>=0)//beta为锐角，arcsin(sth.)
            {
                angle_1[detect_number-1]=90-asin(sin_beta_1)*180/3.1415926;
            }
            else if(cos_beta_1<0)//beta为钝角，180-arcsin(sth.)
            {
                angle_1[detect_number-1]=asin(sin_beta_1)*180/3.1415926-90;       
            }
            if(cos_beta_3>=0)//beta为锐角，arcsin(sth.)
            {
                angle_3[detect_number-1]=90-asin(sin_beta_3)*180/3.1415926;
            }
            else if(cos_beta_3<0)//beta为钝角，180-arcsin(sth.)
            {
                angle_3[detect_number-1]=asin(sin_beta_3)*180/3.1415926-90;
            }
            int check_point= int(detect_distance[detect_number-1]);
            if(    (    (check_point<1)||(check_point>10)||(start_point>=end_point)) &&  (detect_number>0)      )//改2
            {              //改9
                
                detect_distance[detect_number-1]=0;             
                detect_number--;
            }       
        }
        else
        {
            i++;
        }      
        
        
    }
    for(int k=0;k<10;k++)
    {
        std::cout<<"angle"<<k<<"  "<<detect_angle[k]<<std::endl;
        std::cout<<"distance"<<k<<"  "<<detect_distance[k]<<std::endl;
    } 
        //distance_1,2,3和angle1,2,3,均为大小为5的float数组
        if (!serial_por.IsOpen())
        {
            std::cout<<"No"<<std::endl;
            serial_por.Open("/dev/ttyUSB0");
            serial_por.SetBaudRate(SerialStreamBuf::BAUD_115200);
            serial_por.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
            serial_por.SetNumOfStopBits(1);
            serial_por.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
        }
        else
        {
            std::cout<<"Yes"<<std::endl;
        }
        send_value(angle_1,distance_1);
        send_value(angle_2,distance_2);
        send_value(angle_3,distance_3); 
    arc_draw_publisher_.publish(arc_draw);
}
