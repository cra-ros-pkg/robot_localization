#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf/tf.h>

nav_msgs::Odometry filtered_;

void resetFilter()
{
  ros::NodeHandle nh;
  ros::Publisher setPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 5);

  bool poseChanged = false;

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.orientation.w = 1;
  pose.header.frame_id = "odom";
  for(size_t ind = 0; ind < 36; ind+=7)
  {
    pose.pose.covariance[ind] = 1e-6;
  }

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

void filterCallback(const nav_msgs::OdometryConstPtr &msg)
{
  filtered_ = *msg;
}

TEST (InterfacesTest, OdomPoseBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input0", 5);

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

  for(size_t i = 0; i < 10; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    odom.header.seq++;

    /*double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(filtered_.pose.pose.orientation, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    std::cerr << filtered_.pose.pose.position.x << " " << filtered_.pose.pose.position.y << " " << filtered_.pose.pose.position.z << "\n  " << roll << " " << pitch << " " << yaw << "\n";*/
  }


  // Now check the values from the callback
  EXPECT_EQ(filtered_.pose.pose.position.x, odom.pose.pose.position.x);
  EXPECT_EQ(filtered_.pose.pose.position.y, 0); // Configuration for this variable for this sensor is false
  EXPECT_EQ(filtered_.pose.pose.position.z, odom.pose.pose.position.z);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7], 0.25); // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[14], 0.5);

  resetFilter();
}

TEST (InterfacesTest, OdomTwistBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input2", 5);
  ros::Publisher setPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = M_PI / 2.0;

  odom.twist.covariance[0] = 1e-6;
  odom.twist.covariance[7] = 1e-6;
  odom.twist.covariance[14] = 1e-6;
  odom.twist.covariance[21] = 1e-6;
  odom.twist.covariance[28] = 1e-6;
  odom.twist.covariance[35] = 1e-6;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  for(size_t i = 0; i < 10; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    odom.header.seq++;

    /*double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(filtered_.pose.pose.orientation, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    std::cerr << filtered_.pose.pose.position.x << " " << filtered_.pose.pose.position.y << " " << filtered_.pose.pose.position.z << "\n  " << roll << " " << pitch << " " << yaw << "\n";*/
  }

  odom.twist.twist.linear.x = 0.5;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  for(size_t i = 0; i < 10; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    odom.header.seq++;
  }

  //std::cerr << filtered_;
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 0.5), 0.2);

  resetFilter();
}

TEST (InterfacesTest, PoseBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 20.0;
  pose.pose.pose.position.y = 10.0;
  pose.pose.pose.position.z = -40.0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;

  for(size_t ind = 0; ind < 36; ind+=7)
  {
    pose.pose.covariance[ind] = 1e-6;
  }

  pose.header.frame_id = "odom";

  bool poseChanged = false;

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  while(!poseChanged)
  {
    pose.header.stamp = ros::Time::now();
    posePub.publish(pose);
    pose.header.seq++;
    ros::spinOnce();

    poseChanged = ::fabs(filtered_.pose.pose.position.x - pose.pose.pose.position.x) < 1e-5 &&
                  ::fabs(filtered_.pose.pose.position.z - pose.pose.pose.position.z) < 1e-5;

    ros::Duration(0.1).sleep();
  }

  // Now check the values from the callback
  EXPECT_LT(filtered_.pose.pose.position.y, 1e-7); // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[0], 0.1);
  EXPECT_LT(filtered_.pose.covariance[7], 2.0);
  EXPECT_LT(filtered_.pose.covariance[14], 0.1);

  poseChanged = false;

  resetFilter();
}

TEST (InterfacesTest, TwistBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher twistPub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/twist_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 0;
  twist.twist.twist.linear.y = 0;
  twist.twist.twist.linear.z = 0;
  twist.twist.twist.angular.x = M_PI / 2.0;
  twist.twist.twist.angular.y = 0;
  twist.twist.twist.angular.z = 0;

  twist.twist.covariance[0] = 1e-6;
  twist.twist.covariance[7] = 1e-6;
  twist.twist.covariance[14] = 1e-6;
  twist.twist.covariance[21] = 1e-6;
  twist.twist.covariance[28] = 1e-6;
  twist.twist.covariance[35] = 1e-6;

  twist.header.frame_id = "base_link";

  for(size_t i = 0; i < 10; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    twist.header.seq++;

    /*double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(filtered_.pose.pose.orientation, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    std::cerr << filtered_.pose.pose.position.x << " " << filtered_.pose.pose.position.y << " " << filtered_.pose.pose.position.z << "\n  " << roll << " " << pitch << " " << yaw << "\n";*/
  }

  twist.twist.twist.linear.x = 0;
  twist.twist.twist.linear.y = -0.5;
  twist.twist.twist.linear.z = 0;
  twist.twist.twist.angular.x = 0;
  twist.twist.twist.angular.y = 0;
  twist.twist.twist.angular.z = 0;

  for(size_t i = 0; i < 10; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    twist.header.seq++;
  }

  EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 0.5), 0.2);

  resetFilter();
}

TEST (InterfacesTest, OdomDifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input1", 5);
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
  while(zeroCount++ < 10)
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

    odom.header.seq++;
  }

  // Now feed it a position that is near the first
  // one. As this sensor is differential, its position
  // variables should yield values close to (1, 2, -3)
  odom.pose.pose.position.x = 21;
  odom.pose.pose.position.y = 12.0;
  odom.pose.pose.position.z = -43.0;

  // ...but only if we give the measurement a tiny covariance
  for(size_t ind = 0; ind < 36; ind+=7)
  {
    odom.pose.covariance[ind] = 1e-6;
  }

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

  resetFilter();
}

TEST (InterfacesTest, PoseDifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_input1", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 20.0;
  pose.pose.pose.position.y = 10.0;
  pose.pose.pose.position.z = -40.0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;

  for(size_t ind = 0; ind < 36; ind+=7)
  {
    pose.pose.covariance[ind] = 1e-6;
  }

  pose.header.frame_id = "odom";

  bool poseChanged = false;

  // No guaranteeing that the zero state
  // we're expecting to see here isn't just
  // a result of zeroing it out previously,
  // so check 10 times in succession.
  size_t zeroCount = 0;
  while(zeroCount++ < 10)
  {
    pose.header.stamp = ros::Time::now();
    posePub.publish(pose);
    pose.header.seq++;
    ros::spinOnce();

    EXPECT_EQ(filtered_.pose.pose.position.x, 0);
    EXPECT_EQ(filtered_.pose.pose.position.y, 0);
    EXPECT_EQ(filtered_.pose.pose.position.z, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.x, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.y, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.z, 0);
    EXPECT_EQ(filtered_.pose.pose.orientation.w, 1);

    ros::Duration(0.1).sleep();
  }

  resetFilter();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "ekf_navigation_node-test-interfaces");
  ros::Time::init();

  // Give ekf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  return RUN_ALL_TESTS();
}
