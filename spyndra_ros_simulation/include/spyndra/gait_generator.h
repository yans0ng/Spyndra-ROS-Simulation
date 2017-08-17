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

namespace spyndra
{
class GaitGenerator
{
public:
  GaitGenerator(){}
  GaitGenerator(const GaitGenerator& rhs){}
  ~GaitGenerator(){}
    
  virtual void foo() = 0;
};
}
