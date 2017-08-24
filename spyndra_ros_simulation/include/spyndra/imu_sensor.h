/* ImuSensor.h */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include "spyndra/sensor.h"

#ifndef SPYNDRA_IMUSENSOR_H
#define SPYNDRA_IMUSENSOR_H
namespace spyndra
{
class ImuSensor : public Sensor<ImuSensor, std::vector<double> >
{
public:
  ImuSensor(){}
  ImuSensor( int argc, char **argv);
  ~ImuSensor(){};

private:
  ros::Subscriber imu_sub;
  std::vector<double> measure_impl();
  double x_, y_, z_, roll_, pitch_, yaw_;
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);
  void save_csv_impl( std::string filename );

friend Sensor<ImuSensor, std::vector<double> >;
};

/* imu_sensor.cpp */
ImuSensor::ImuSensor( int argc, char **argv)
{
  ros::init(argc, argv, "spyndra/imu_sensor");
  ros::NodeHandle n;
  imu_sub = n.subscribe("/imu",100,&ImuSensor::imu_callback,this);
}

std::vector<double> ImuSensor::measure_impl()
{
  std::vector<double> measure;
  measure.push_back(x_);
  measure.push_back(y_);
  measure.push_back(z_);
  measure.push_back(roll_);
  measure.push_back(pitch_);
  measure.push_back(yaw_);
  data.push_back( measure );
  return measure;
}

void ImuSensor::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  // Update RPY
  geometry_msgs::Quaternion o = msg->orientation;
  tf::Quaternion q (o.x, o.y, o.z, o.w );
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // Update acceleration
  geometry_msgs::Vector3 a = msg->linear_acceleration;
  x_ = a.x;
  y_ = a.y;
  z_ = a.z;
}

void ImuSensor::save_csv_impl( std::string filename )
{
  std::string delimeter = ",";
  std::ofstream csvfile( filename );
  if (csvfile.is_open() )
  {
    for (auto it = data.begin(); it != data.end(); ++it)
    {
      for (int i = 0; i < 5; ++i )
        csvfile << (*it)[i] << delimeter;
      csvfile << (*it)[5] << '\n';
    }
    csvfile.close();
  }
  else std::cout << "Unable to open file\n";
}

} // namespace spyndra
#endif
