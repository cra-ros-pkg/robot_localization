#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>

int i = 1;


void callback(const nav_msgs::Odometry::ConstPtr& msg)
   {
       i++;
    std::ofstream file;
    file.open("Odometry-data.txt", std::ios_base::app | std::ios_base::out);
    file<<msg->pose.pose.position.x<<" "<<msg->pose.pose.position.y<<" "<<msg->twist.twist.angular.z<<"\n";
    std::cout<<"Got data: "<<sin(msg->twist.twist.angular.z)<<"\n";
     std::cout<<i<<"\n";
    if(i%50 == 0){
        i=0;
    double x = msg->pose.pose.position.x, y = msg->pose.pose.position.y, yaw = msg->twist.twist.angular.z;
    double to_x, to_y;
    to_x = x + 15*cos(yaw);
    to_y = y + 15*sin(yaw);
	FILE * gnuplotPipe = popen("gnuplot ", "w");
    fprintf(gnuplotPipe, "unset label\n");
    fprintf(gnuplotPipe, "set xlabel 'Position on x-axis(in metres)'\n");
    fprintf(gnuplotPipe, "set title 'Raw wheel odometry data'\n");
    fprintf(gnuplotPipe, "set ylabel 'Position on y-axis(in metres)'\n");
    fprintf(gnuplotPipe,"set xrange [-150:150]\n");
    fprintf(gnuplotPipe, "set yrange [-150:150]\n");
    fprintf(gnuplotPipe, "set arrow from %f, %f to %f, %f\n", x, y, to_x, to_y);
    fprintf(gnuplotPipe, "plot 'Odometry-data.txt' with lines\n");
    fprintf(gnuplotPipe, "set term png\n");
    fprintf(gnuplotPipe, "set output 'bin/plot%f.png'\n", x);
    fprintf(gnuplotPipe, "replot\n");
	fflush(gnuplotPipe);
   }
    }


int main(int argc, char **argv){
    ros::init(argc, argv, "plot_pub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/wheel_odometry", 1000, callback);
  
    ros::spin();

    return 0;
}


