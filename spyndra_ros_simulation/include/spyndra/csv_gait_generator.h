/* CsvGaitGenerator.h */
#include "gait_generator.h"
#include <std_msgs/Float64.h>
#include <string>
#include <fstream>
#include <iostream>
#include <spyndra/util.h>

namespace spyndra
{
class CsvGaitGenerator : GaitGenerator
{
public:
  CsvGaitGenerator(){}
  CsvGaitGenerator( const std::string filename );
  CsvGaitGenerator( ros::V_Publisher vp );
  CsvGaitGenerator( const CsvGaitGenerator& rhs){}
  ~CsvGaitGenerator(){}

  void foo() override;
  void next_step() override;
  bool is_over() override;
  void set_csv_file( std::string filename);
  void print_gait();

private:
  std::list<std::vector<double> >::iterator curr;
};

CsvGaitGenerator::CsvGaitGenerator( const std::string filename)
{ set_csv_file( filename ); }

CsvGaitGenerator::CsvGaitGenerator( ros::V_Publisher vp )
{ controllers = vp; }

void CsvGaitGenerator::next_step()
{
  ROS_INFO_STREAM("Next csv step");
  curr++;
  for (int i = 0; i < 8; ++i)
  {
    std_msgs::Float64 cmd;
    cmd.data = (*curr)[i];
    controllers[i].publish(cmd);
    std::cout << "Joint " << i << ": " << cmd.data << ' ';
  }
  std::cout << '\n';
  ros::spinOnce();
}

void CsvGaitGenerator::set_csv_file( std::string filename )
{
  std::string delimeter = " ";
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

      std::vector<double> step;
      for ( int i = 0; i < 8; ++i)
      {
        // Separate tokens
        next = line.find(delimeter, last);
        token = line.substr(last, next-last);

        //step.push_back( atof(token.c_str()) / 180. - 1.5);
        step.push_back( cmd_to_rad(i, atof(token.c_str() ) ) );

        last = next + 1;
      }
      gait.push_back(step);
    }
  }

  curr = gait.begin();
}

bool CsvGaitGenerator::is_over()
{ return curr == gait.end(); }

void CsvGaitGenerator::print_gait()
{
  for (std::list<std::vector<double> >::iterator it = gait.begin(); it != gait.end(); ++it)
  {
    for (std::vector<double>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2)
      std::cout << *it2 << ' ';
    std::cout << '\n';
  }
}

}
