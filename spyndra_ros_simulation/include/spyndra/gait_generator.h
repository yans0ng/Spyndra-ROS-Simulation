/* gait_generator.h 
 *
 * Inteface of gait generator. 
 * Currently 4 types of gait generator are being constructed.
 *
 * 1. csv_gait_generator
 * 2. gaussian_process_generator
 * 3. spline_walk_generator
 * 4. sine_walk_generator
 */

#include <ros/ros.h>
#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <iostream>
#include <spyndra/util.h>
#include <std_msgs/Float64.h>
#include <sstream>

namespace spyndra
{
template <class T>
class GaitGenerator
{
public:
  GaitGenerator(){}
  GaitGenerator(const ros::V_Publisher& vp):controllers(vp){}
  //GaitGenerator(const ros::V_Publisher& vp, std::string method, std::string filename):controllers(vp)
  GaitGenerator(const ros::V_Publisher& vp, std::string filename):controllers(vp)
  {
    static_cast<T*>(this)->set_csv_file( filename );
  }
  ~GaitGenerator(){}
    
  // Abstract Functions
  void next_step()
  { static_cast<T*>(this)->next_step_impl(); }
  bool is_over()
  { return static_cast<T*>(this)->is_over_impl(); }

  // Shared Funtions
  void print_gait();

protected:
  ros::V_Publisher controllers;
  std::list<std::vector<double> > gait;
};

template <class T>
void GaitGenerator<T>::print_gait()
{
  for (auto it = gait.begin(); it != gait.end(); ++it)
  {
    for (auto it2 = it->begin(); it2 != it->end(); ++it2)
      std::cout << *it2 << ' ';
    std::cout << '\n';
  }
}

class CsvGaitGenerator : public GaitGenerator<CsvGaitGenerator>
{
public:
  CsvGaitGenerator(){}
  ~CsvGaitGenerator(){}
  void next_step_impl();
  bool is_over_impl();
  void set_csv_file( std::string filename );
private:
  std::list<std::vector<double> >::iterator curr;
};

void CsvGaitGenerator::next_step_impl()
{
  std::stringstream ss;
  curr++;
  for (int i = 0; i < 8 ; ++i)
  {
    std_msgs::Float64 cmd;
    cmd.data = (*curr)[i];
    controllers[i].publish(cmd);
    ss << "Joint" << i << ":" << cmd.data << " "; 
  }
  ROS_INFO_STREAM( ss.str() );
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

bool CsvGaitGenerator::is_over_impl()
{ return curr == gait.end(); }

class RandomGaitGenerator : public GaitGenerator<RandomGaitGenerator>
{
public:
  RandomGaitGenerator(){}
  ~RandomGaitGenerator(){}
};

}
