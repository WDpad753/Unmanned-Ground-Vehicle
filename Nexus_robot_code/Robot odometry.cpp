#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

double vx, vy, vth;
double OdomTurnMultiplier = 0.95;
double dist_l;
double dist_r;
double WheelSeparation = 0.3;

void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   vx = msg->linear.x;
   vy = msg->linear.y;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "Robot_Odom");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber sub = n.subscribe("nexus_odom", 1, OdomCallback);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(n.ok())

{
    ros::spinOnce();               // check for incoming messages

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    
    //Initialise distance:
    dist_l = dt*vx;
    dist_r = dt*vy;

    // Mean distance:
    double mean_xy = (dist_l + dist_r) / 2;
    double mean_th = (((dist_r - dist_l) / WheelSeparation)) * OdomTurnMultiplier;

    double delta_x = mean_xy * cosf(th);
    double delta_y = mean_xy * sinf(th);
    double delta_th = mean_th;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //Current velocity:
    double spd = mean_xy / dt;
    double ang_spd = mean_th / dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = spd;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = ang_spd;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
 ros::spin();
}