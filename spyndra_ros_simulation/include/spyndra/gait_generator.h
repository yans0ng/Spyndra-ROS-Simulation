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

namespace spyndra
{
class GaitGenerator
{
public:
  GaitGenerator(){}
  GaitGenerator(const GaitGenerator& rhs){}
  ~GaitGenerator(){}
    
  virtual void foo() = 0;
  virtual void next_step() = 0;
  virtual bool is_over() = 0;

protected:
  ros::V_Publisher controllers;
  std::list<std::vector<double> > gait;
};
}
