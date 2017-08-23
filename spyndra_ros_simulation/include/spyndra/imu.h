/* imu.h */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector> // TODO: replace with customized msg

#ifndef SPYNDRA_IMU_H
#define SPYNDRA_IMU_H
namespace spyndra
{
class Imu
{
public:
  Imu(int argc, char **argv);
  ~Imu(){}
  std::vector<double> measure_impl();
private:
  double x, y, z, roll, pitch, yaw;
  ros::Subscriber imu_sub;
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);
};

std::vector<double> Imu::measure_impl()
{
  std::vector<double> measure;
  measure.push_back(x);
  measure.push_back(y);
  measure.push_back(z);
  measure.push_back(roll);
  measure.push_back(pitch);
  measure.push_back(yaw);
  return measure;
}

/* imu.cpp */
Imu::Imu(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;
  imu_sub = n.subscribe("/imu",100,&Imu::imu_callback, this);
}

void Imu::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  // Update RPY
  geometry_msgs::Quaternion o = msg->orientation;
  tf::Quaternion q (o.x, o.y, o.z, o.w );
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // update acceleration
  geometry_msgs::Vector3 a = msg->linear_acceleration;
  x = a.x;
  y = a.y;
  z = a.z;
}
} // namespace spyndra

#endif

