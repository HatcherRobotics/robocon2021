#ifndef SEND_VALUE_H
#define SEND_VALUE_H
#include<SerialStream.h>
using namespace LibSerial;
SerialStream serial_por;

void send_value(float value_angle[5],float value_distance[5])
{
    char a[2]={char(0x03),char(0xfc)};//帧头
    char b[2]={char(0xfc),char(0x03)};//帧尾
    char c[1]={0x23};//#
    char d[1]={0x20};//空格
    char* distance=new char[7];
    char* angle   =new char[2];
    int float_to_int[5];
    for(int j=0;j<5;j++)
    {
        float_to_int[j]=value_angle[j];
    }
    for(int k=0;k<5;k++)
    {
        serial_por.write(a,2);
        sprintf(angle,"%02d",float_to_int[k]);
        serial_por.write(angle,sizeof(angle));
        serial_por.write(b,2);

        serial_por.write(a,2);
        sprintf(distance,"%7.4f",value_distance[k]);
        serial_por.write(distance,sizeof(distance));
        serial_por.write(b,2);

        serial_por.write(a,2);
        serial_por.write(c,1);
        serial_por.write(b,2);
    }

    serial_por.write(a,2);
    serial_por.write(d,2);
    serial_por.write(b,2);
}
#endif