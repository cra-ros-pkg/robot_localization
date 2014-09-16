#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf/tf.h>

nav_msgs::Odometry filtered_;
bool msgReceived_;

void filterCallback(const nav_msgs::OdometryConstPtr &msg)
{
  filtered_ = *msg;
  msgReceived_ = true;
}

TEST (InterfacesTest, BasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input0", 5);
  ros::Publisher setPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;
  
  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  while(!msgReceived_)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();
    odom.header.seq++;
    ros::Duration(0.1).sleep();
  }

  // Now check the values from the callback
  EXPECT_EQ(filtered_.pose.pose.position.x, odom.pose.pose.position.x);
  EXPECT_EQ(filtered_.pose.pose.position.y, 0); // Configuration for this variable for this sensor is false
  EXPECT_EQ(filtered_.pose.pose.position.z, odom.pose.pose.position.z);

  EXPECT_EQ(filtered_.pose.covariance[0], 2);
  EXPECT_EQ(filtered_.pose.covariance[7], 1); // Configuration for this variable for this sensor is false
  EXPECT_EQ(filtered_.pose.covariance[14], 2);

  msgReceived_ = false;
  bool poseChanged = false;

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.orientation.w = 1;
  pose.header.frame_id = "odom";

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  while(!poseChanged)
  {
    pose.header.stamp = ros::Time::now();
    setPosePub.publish(pose);
    pose.header.seq++;
    ros::spinOnce();

    poseChanged = (filtered_.pose.pose.position.x == pose.pose.pose.position.x) &&
                  (filtered_.pose.pose.position.y == pose.pose.pose.position.y) &&
                  (filtered_.pose.pose.position.z == pose.pose.pose.position.z) &&
                  (filtered_.pose.pose.orientation.x == pose.pose.pose.orientation.x) &&
                  (filtered_.pose.pose.orientation.y == pose.pose.pose.orientation.y) &&
                  (filtered_.pose.pose.orientation.z == pose.pose.pose.orientation.z) &&
                  (filtered_.pose.pose.orientation.w == pose.pose.pose.orientation.w);

    ros::Duration(0.1).sleep();
  }
}

TEST (InterfacesTest, DifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input1", 5);
  ros::Publisher setPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;

  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;

  odom.pose.pose.orientation.w = 1;

  odom.pose.covariance[21] = 0.2;
  odom.pose.covariance[28] = 0.2;
  odom.pose.covariance[35] = 0.2;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // No guaranteeing that the zero state
  // we're expecting to see here isn't just
  // a result of zeroing it out previously,
  // so check 10 times in succession.
  size_t zeroCount = 0;
  while(zeroCount < 10)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    EXPECT_EQ(filtered_.pose.pose.position.x, 0);
    EXPECT_EQ(filtered_.pose.pose.position.y, 0);
    EXPECT_EQ(filtered_.pose.pose.position.z, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.x, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.y, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.z, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.w, 1);

    ros::Duration(0.1).sleep();

    zeroCount++;
    odom.header.seq++;
  }

  // Now feed it a position that is near the first
  // one. As this sensor is differential, its position
  // variables should yield values close to (1, 2, -3)
  odom.pose.pose.position.x = 21;
  odom.pose.pose.position.y = 12.0;
  odom.pose.pose.position.z = -43.0;

  // ...but only if we give the measurement a tiny covariance
  odom.pose.covariance[0] = 1e-6;
  odom.pose.covariance[7] = 1e-6;
  odom.pose.covariance[14] = 1e-6;

  bool poseChanged = false;

  while(!poseChanged)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);

    ros::spinOnce();

    poseChanged = (::fabs(filtered_.pose.pose.position.x - 1) < 1e-4) &&
                  (::fabs(filtered_.pose.pose.position.y - 2) < 1e-4) &&
                  (::fabs(filtered_.pose.pose.position.z + 3) < 1e-4) &&
                  (filtered_.pose.pose.orientation.x == 0) &&
                  (filtered_.pose.pose.orientation.y == 0) &&
                  (filtered_.pose.pose.orientation.z == 0) &&
                  (filtered_.pose.pose.orientation.w == 1);

    ros::Duration(0.1).sleep();
    odom.header.seq++;
  }

  poseChanged = false;

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.orientation.w = 1;
  pose.header.frame_id = "odom";

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  while(!poseChanged)
  {
    pose.header.stamp = ros::Time::now();
    setPosePub.publish(pose);
    pose.header.seq++;
    ros::spinOnce();

    poseChanged = (filtered_.pose.pose.position.x == pose.pose.pose.position.x) &&
                  (filtered_.pose.pose.position.y == pose.pose.pose.position.y) &&
                  (filtered_.pose.pose.position.z == pose.pose.pose.position.z) &&
                  (filtered_.pose.pose.orientation.x == pose.pose.pose.orientation.x) &&
                  (filtered_.pose.pose.orientation.y == pose.pose.pose.orientation.y) &&
                  (filtered_.pose.pose.orientation.z == pose.pose.pose.orientation.z) &&
                  (filtered_.pose.pose.orientation.w == pose.pose.pose.orientation.w);

    ros::Duration(0.1).sleep();
  }
}

TEST (InterfacesTest, VelocityTest)
{


}

int main(int argc, char **argv)
{
  msgReceived_ = false;

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "ekf_navigation_node-test-interfaces");
  ros::Time::init();

  // Give ekf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  return RUN_ALL_TESTS();
}
