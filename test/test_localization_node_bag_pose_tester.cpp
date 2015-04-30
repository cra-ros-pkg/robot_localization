#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

nav_msgs::Odometry filtered_;

void filterCallback(const nav_msgs::OdometryConstPtr &msg)
{
  filtered_ = *msg;
}

TEST (BagTest, PoseCheck)
{
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  double finalX = 0;
  double finalY = 0;
  double finalZ = 0;
  double tolerance = 0;
  bool outputFinalPosition = false;
  std::string finalPositionFile;

  nhLocal.getParam("final_x", finalX);
  nhLocal.getParam("final_y", finalY);
  nhLocal.getParam("final_z", finalZ);
  nhLocal.getParam("tolerance", tolerance);
  nhLocal.param("output_final_position", outputFinalPosition, false);
  nhLocal.param("output_location", finalPositionFile, std::string("test.txt"));

  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
  }

  if(outputFinalPosition)
  {
    try
    {
      std::ofstream posOut;
      posOut.open(finalPositionFile.c_str(), std::ofstream::app);
      posOut << filtered_.pose.pose.position.x << " " << filtered_.pose.pose.position.y << " " << filtered_.pose.pose.position.z << std::endl;
      posOut.close();
    }
    catch(...)
    {
      ROS_ERROR_STREAM("Unable to open output file.\n");
    }
  }

  double xDiff = filtered_.pose.pose.position.x - finalX;
  double yDiff = filtered_.pose.pose.position.y - finalY;
  double zDiff = filtered_.pose.pose.position.z - finalZ;

  EXPECT_LT(::sqrt(xDiff*xDiff + yDiff*yDiff + zDiff*zDiff), tolerance);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "localization_node-bag-pose-tester");
  ros::Time::init();

  // Give ekf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  return RUN_ALL_TESTS();
}

