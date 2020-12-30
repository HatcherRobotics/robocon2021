#include<iostream>
#include<cmath>
#define radius 152
#define pi 3.1415926
using namespace std;
int main()
{
    cout<<"Hello cpp!"<<endl;
    int distance;
    cout<<"Input the value of the distance(mm):";
    cin>>distance;
    double theta;
    theta=2* atan2(radius,distance);
    double theta_d;
    theta_d=theta/pi*180;
    cout<<"The value of the angle is\t"<<theta_d<<endl;
    return 0;
}
