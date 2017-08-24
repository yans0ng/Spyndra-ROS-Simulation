/* simulator.h */
#include <ros/ros.h>
#include "spyndra/robot.h"
#include <std_msgs/Float64.h>
#include <string>
#include "spyndra/sensor.h"

#ifndef SPYNDRA_ROBOT_SIMULATOR_H
#define SPYNDRA_ROBOT_SIMULATOR_H
namespace spyndra
{
class Simulator : Robot
{
public:
  Simulator(int argc, char **argv, int freq );
  Simulator( const Simulator& rhs ){}
  ~Simulator(){}

  GaitGenerator gait_generator(const std::string& method, const std::string& filename);
  //Sensor sensor(int argc, char**argv, const std::string& sensor_type);
};


/* simulator.cpp */
Simulator::Simulator(int argc, char **argv, int freq = 10 )
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;
  ros::Publisher j1 = n.advertise<std_msgs::Float64>("/spyndra/joint1_position_controller/command",freq);
  ros::Publisher j2 = n.advertise<std_msgs::Float64>("/spyndra/joint2_position_controller/command",freq);
  ros::Publisher j3 = n.advertise<std_msgs::Float64>("/spyndra/joint3_position_controller/command",freq);
  ros::Publisher j4 = n.advertise<std_msgs::Float64>("/spyndra/joint4_position_controller/command",freq);
  ros::Publisher j5 = n.advertise<std_msgs::Float64>("/spyndra/joint5_position_controller/command",freq);
  ros::Publisher j6 = n.advertise<std_msgs::Float64>("/spyndra/joint6_position_controller/command",freq);
  ros::Publisher j7 = n.advertise<std_msgs::Float64>("/spyndra/joint7_position_controller/command",freq);
  ros::Publisher j8 = n.advertise<std_msgs::Float64>("/spyndra/joint8_position_controller/command",freq);
  controllers = ros::V_Publisher({j1, j2, j3, j4, j5, j6, j7, j8});
}

GaitGenerator Simulator::gait_generator(const std::string& method, const std::string& filename)
{
  spyndra::GaitGenerator g(method, filename);
  g.set_controllers( controllers );
  return g;
}

/*
Sensor Simulator::sensor(int argc, char** argv, const std::string& sensor_type )
{
  Sensor s(argc, argv, sensor_type);
  return s;
}
*/
} // namespace spyndra

#endif
