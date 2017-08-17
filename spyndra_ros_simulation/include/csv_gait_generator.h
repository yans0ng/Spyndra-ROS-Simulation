#include <ros/ros.h>
#include <vector>
#include <string>
#include <list>
#include <fstream>
#include <iostream>
#include <std_msgs/Float64.h>
//#include "csv_gait_generator.cpp"

namespace ros
{
class CsvGaitGenerator
{
public:
  CsvGaitGenerator(){};
  CsvGaitGenerator(const ros::V_Publisher& joint_publishers);
  CsvGaitGenerator(const CsvGaitGenerator& rhs);
  ~CsvGaitGenerator(){};

  // Override virtaul function
  void next_step();
  bool is_over();

  void set_csv_file( char* filename );
  void print_gait();

private:
  ros::V_Publisher controllers;
  std::list<std::vector<double> > gait;
  std::list<std::vector<double> >::iterator curr;
};

// TODO: use reference link
CsvGaitGenerator::CsvGaitGenerator(const ros::V_Publisher& joint_publishers)
{ controllers = joint_publishers; } 

void CsvGaitGenerator::set_csv_file( char* filename )
{
  std::string delimeter = " ";
  // If filename is a string
  // std::ifstream csvfile( filename.c_str())
  std::ifstream csvfile( filename );
  std::string line, token;

  if ( csvfile.is_open())
  {
    // Omit the first line
    std::getline( csvfile, line );

    // Loop through each line
    while ( std::getline ( csvfile, line ) )
    {
      size_t last = 0, next = 0;
      // Omit the first 6 fields (IMU measurements)
      for ( int i = 0; i < 7; ++i)
        next = line.find(delimeter, next+1);
      last = next + 1;

      std::vector<double> step(8);
      for ( int i = 0; i < 8; ++i)
      {
        // Separate tokens
        next = line.find(delimeter, last);
        token = line.substr(last, next-last);

        step.push_back( atof(token.c_str()) );
      }
      gait.push_back(step);
    }
  }

  curr = gait.begin();
}

void CsvGaitGenerator::print_gait()
{
  for (std::list<std::vector<double> >::iterator it = gait.begin(); it != gait.end(); ++it)
  {
    for (std::vector<double>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2)
      std::cout << *it2 << ' ';
    std::cout << '\n';
  }
}

void CsvGaitGenerator::next_step()
{
  curr++;
  for ( int i = 0; i < 8; ++i )
  {
    std_msgs::Float64 cmd;
    cmd.data = (*curr)[i];
    controllers[i].publish(cmd);
  }
}

bool CsvGaitGenerator::is_over()
{ return curr == gait.end(); }

}
