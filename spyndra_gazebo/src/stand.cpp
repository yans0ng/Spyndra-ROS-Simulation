#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <vector>
//#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basicmove");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(1);

  // message declarations
  sensor_msgs::JointState joint_state;  // Joint angles

  // NOTE: reference frame must be the fixed frame of RViz 
  // If you uncomment the following line, you should choose "femur_1" as fixed frame
  // of RViz
  //t1_trans.header.frame_id = "femur_1";
  geometry_msgs::TransformStamped f1_trans; // TF of femur
  f1_trans.header.frame_id = "base_link";
  f1_trans.child_frame_id = "femur_1";

  geometry_msgs::TransformStamped t1_trans; // TF of tibia
  t1_trans.header.frame_id = "base_link";
  t1_trans.child_frame_id = "tibia_1";

  geometry_msgs::TransformStamped f2_trans; // TF of femur
  f2_trans.header.frame_id = "base_link";
  f2_trans.child_frame_id = "femur_2";

  geometry_msgs::TransformStamped t2_trans; // TF of tibia
  t2_trans.header.frame_id = "base_link";
  t2_trans.child_frame_id = "tibia_2";

  geometry_msgs::TransformStamped f3_trans; // TF of femur
  f3_trans.header.frame_id = "base_link";
  f3_trans.child_frame_id = "femur_3";

  geometry_msgs::TransformStamped t3_trans; // TF of tibia
  t3_trans.header.frame_id = "base_link";
  t3_trans.child_frame_id = "tibia_3";

  geometry_msgs::TransformStamped f4_trans; // TF of femur
  f4_trans.header.frame_id = "base_link";
  f4_trans.child_frame_id = "femur_4";

  geometry_msgs::TransformStamped t4_trans; // TF of tibia
  t4_trans.header.frame_id = "base_link";
  t4_trans.child_frame_id = "tibia_4";

  // robot state
  const double degree = M_PI/180;
  const double lf = 0.068, lt = 0.126;  // length of fumur and tibia
  double angle1 = 0.0, angle2 = 0.0, inc1 = degree, inc2 = 2.0*degree;


  while (ros::ok())
  {
    // update joint state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.name[0] = "motor_f1";
    joint_state.position[0] = angle1;
    joint_state.name[1] = "motor_t1"; // NOTE: angle2 is irrelavent to TF.
    joint_state.position[1] = angle2;
    joint_state.name[2] = "motor_f2";
    joint_state.position[2] = angle1;
    joint_state.name[3] = "motor_t2"; // NOTE: angle2 is irrelavent to TF.
    joint_state.position[3] = angle2;
    joint_state.name[4] = "motor_f3";
    joint_state.position[4] = angle1;
    joint_state.name[5] = "motor_t3"; // NOTE: angle2 is irrelavent to TF.
    joint_state.position[5] = angle2;
    joint_state.name[6] = "motor_f4";
    joint_state.position[6] = angle1;
    joint_state.name[7] = "motor_t4"; // NOTE: angle2 is irrelavent to TF.
    joint_state.position[7] = angle2;

    // update transform
    f1_trans.header.stamp = ros::Time::now();

    // leg1
    f1_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    f1_trans.transform.translation.x = lf;//lf*cos(-angle1);
    f1_trans.transform.translation.z = 0;//lf*sin(-angle1);
    f1_trans.transform.translation.y = 0;

    t1_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    t1_trans.transform.translation.x = lf + lt*cos(-angle1);
    t1_trans.transform.translation.z = lt*sin(-angle1);
    t1_trans.transform.translation.y = 0;

    // leg2
    f2_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    f2_trans.transform.translation.x = lf;//lf*cos(-angle1);
    f2_trans.transform.translation.z = 0;//lf*sin(-angle1);
    f2_trans.transform.translation.y = 0;

    t2_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    t2_trans.transform.translation.x = lf + lt*cos(-angle1);
    t2_trans.transform.translation.z = lt*sin(-angle1);
    t2_trans.transform.translation.y = 0;

    // leg3
    f3_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    f3_trans.transform.translation.x = lf;//lf*cos(-angle1);
    f3_trans.transform.translation.z = 0;//lf*sin(-angle1);
    f3_trans.transform.translation.y = 0;

    t3_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    t3_trans.transform.translation.x = lf + lt*cos(-angle1);
    t3_trans.transform.translation.z = lt*sin(-angle1);
    t3_trans.transform.translation.y = 0;

    // leg4
    f4_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    f4_trans.transform.translation.x = lf;//lf*cos(-angle1);
    f4_trans.transform.translation.z = 0;//lf*sin(-angle1);
    f4_trans.transform.translation.y = 0;

    t4_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, angle1, 0);
    t4_trans.transform.translation.x = lf + lt*cos(-angle1);
    t4_trans.transform.translation.z = lt*sin(-angle1);
    t4_trans.transform.translation.y = 0;

    // sned the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(f1_trans);
    broadcaster.sendTransform(t1_trans);
    broadcaster.sendTransform(f2_trans);
    broadcaster.sendTransform(t2_trans);
    broadcaster.sendTransform(f3_trans);
    broadcaster.sendTransform(t3_trans);
    broadcaster.sendTransform(f4_trans);
    broadcaster.sendTransform(t4_trans);

    // create a new robot state
    angle1 = angle1 + inc1;
    angle2 = angle2 + inc2;
    if (angle1>=0.5 || angle1<=1e-15)
      inc1 = -inc1;
    if (angle2>=0.5 || angle2<=1e-15)
      inc2 = -inc2;

    std::ostringstream strs;
    strs << "angle 1 = "<< angle1 << ", angle 2 = " << angle2;
    ROS_INFO_STREAM(strs.str());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
