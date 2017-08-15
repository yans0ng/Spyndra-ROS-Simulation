#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <std_msgs/Float64.h>
#include <math.h>

int main(int argc, char **argv)
{
  double freq = 60; //Hz
  ros::init(argc, argv, "harmonic_walk");

  ros::NodeHandle n;
  ros::Publisher j1 = n.advertise<std_msgs::Float64>("/spyndra/joint1_position_controller/command",freq);
  ros::Publisher j2 = n.advertise<std_msgs::Float64>("/spyndra/joint2_position_controller/command",freq);
  ros::Publisher j3 = n.advertise<std_msgs::Float64>("/spyndra/joint3_position_controller/command",freq);
  ros::Publisher j4 = n.advertise<std_msgs::Float64>("/spyndra/joint4_position_controller/command",freq);
  ros::Publisher j5 = n.advertise<std_msgs::Float64>("/spyndra/joint5_position_controller/command",freq);
  ros::Publisher j6 = n.advertise<std_msgs::Float64>("/spyndra/joint6_position_controller/command",freq);
  ros::Publisher j7 = n.advertise<std_msgs::Float64>("/spyndra/joint7_position_controller/command",freq);
  ros::Publisher j8 = n.advertise<std_msgs::Float64>("/spyndra/joint8_position_controller/command",freq);

  ros::Rate loop_rate(freq);
  std_msgs::Float64 j1_cmd, j2_cmd, j3_cmd, j4_cmd, j5_cmd, j6_cmd, j7_cmd, j8_cmd;

  // Initialize position
  j1_cmd.data = 0;
  j2_cmd.data = 0;
  j3_cmd.data = 0;
  j4_cmd.data = 0;
  j5_cmd.data = 0;
  j6_cmd.data = 0;
  j7_cmd.data = 0;
  j8_cmd.data = 0;

  ROS_INFO_STREAM("Wait 3 secs to initialize");
  for (int i = 0; i < 3 * freq; ++i)
  {
    j1.publish(j1_cmd);
    j2.publish(j2_cmd);
    j3.publish(j3_cmd);
    j4.publish(j4_cmd);
    j5.publish(j5_cmd);
    j6.publish(j6_cmd);
    j7.publish(j7_cmd);
    j8.publish(j8_cmd);
    loop_rate.sleep();
  }
  //ros::Duration(3.0).sleep();

  const double phase = M_PI/4.0;
  ros::Time begin = ros::Time::now();
  ROS_INFO_STREAM("Begin walking");
  while (ros::ok())
  {
    double t = (begin - ros::Time::now()).toSec();
    // Update command
    j1_cmd.data = 0.2 * sin(t) + 0.35;
    j2_cmd.data = 0.1 * sin(t) + 0.3;
    j3_cmd.data = 0.2 * sin(t+phase) + 0.35;
    j4_cmd.data = 0.1 * sin(t+phase) + 0.3;
    j5_cmd.data = 0.2 * sin(t+2.*phase) + 0.35;
    j6_cmd.data = 0.1 * sin(t+2.*phase) + 0.3;
    j7_cmd.data = 0.2 * sin(t+3.*phase) + 0.35;
    j8_cmd.data = 0.1 * sin(t+3.*phase) + 0.3;

    j1.publish(j1_cmd);
    j2.publish(j2_cmd);
    j3.publish(j3_cmd);
    j4.publish(j4_cmd);
    j5.publish(j5_cmd);
    j6.publish(j6_cmd);
    j7.publish(j7_cmd);
    j8.publish(j8_cmd);

    // Output joint angles
    //std::ostringstream strs;
    //strs << "femur_angle = " << femur_angle << "tibia_angle = " << tibia_angle;
    //ROS_INFO_STREAM(strs.str());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
