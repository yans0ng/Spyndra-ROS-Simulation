#include <ros/ros.h>
#include <vector>
#include <string>
#include <list>
#include <fstream>
#include <iostream>
#include "csv_gait_generator.h"

namespace ros
{
class Spyndra
{
public:
  Spyndra(){}
  Spyndra(const Spyndra& rhs);
  ~Spyndra(){}

  ros::CsvGaitGenerator GaitGenerator(std::string method);

private:
  ros::V_Publisher controllers;
};

ros::CsvGaitGenerator Spyndra::GaitGenerator(std::string method)
{
  //if (method == "csv")
  CsvGaitGenerator c(controllers);
  return c;
}
}
