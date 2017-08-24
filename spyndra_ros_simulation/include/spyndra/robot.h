#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "spyndra/gait_generator.h"

#ifndef SPYNDRA_ROBOT_H
#define SPYNDRA_ROBOT_H
namespace spyndra
{
class Robot
{
public:
  Robot(){}
  ~Robot(){}
  virtual GaitGenerator gait_generator(const std::string& method, const std::string& filename) = 0;
  // TODO: write sensor interface
  //virtual Sensor sensor(const std::string& type ) = 0;

  // TODO: write shared function here
  void send_pulse(int jt, float angle);

protected:
  ros::V_Publisher controllers;
};

void Robot::send_pulse( int jt, float angle )
{
  std_msgs::Float64 cmd;
  cmd.data = angle;
  controllers[jt].publish( cmd );
  ros::spinOnce();
}

} // namespace spyndra

#endif
