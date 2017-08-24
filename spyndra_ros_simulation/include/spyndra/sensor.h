/* sensor.h */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector> 
#include <string>
#include <list>
#include <fstream>

#ifndef SPYNDRA_SENSOR_H
#define SPYNDRA_SENSOR_H
namespace spyndra
{
template <class T, typename M>
class Sensor
{
public:
  Sensor(){}
  //SensorImpl( int argc, char** argv){};
  ~Sensor(){}
  M measure()
  { return static_cast<T*>(this)->measure_impl(); }
  void save_csv( std::string filename )
  { static_cast<T*>(this)->save_csv_impl( filename ); }

protected:
  std::list<M> data;
};

} // namespace spyndra

#endif
