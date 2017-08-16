#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <list>
#include <vector>
#include <fstream>

// IMU output variables
double x,y,z,roll,pitch,yaw;

/* Process Imu_msg to update output variables */
void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  // Update RPY
  geometry_msgs::Quaternion o = msg->orientation;
  tf::Quaternion q (o.x, o.y, o.z, o.w );
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // Update acceleration
  geometry_msgs::Vector3 a = msg->linear_acceleration;
  x = a.x;
  y = a.y;
  z = a.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;
  ros::Subscriber imu = n.subscribe("/imu",100,imu_callback);

  // TODO: you can change rate here
  ros::Rate loop_rate(1);

  std::list<std::vector<double> > data;
  while (ros::ok())
  {
    // Store data
    double r[6] =  {x, y, z, roll, pitch, yaw};
    std::vector<double> row(r, r+sizeof(r)/sizeof(double)); // iterator constructor
    data.push_back(row);

    ROS_INFO("x, y, z = [%f,%f,%f] roll, pitch, yaw = [%f,%f,%f]", x,y,z, roll, pitch, yaw);

    loop_rate.sleep();
    ros::spinOnce();
  }

  // Output csv file
  // TODO: you can change output file here
  std::ofstream csvfile ("test.csv");
  if (csvfile.is_open())
  {
    for ( std::list<std::vector<double> >::iterator it=data.begin(); it != data.end(); ++it)
    {
      for (std::vector<double>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2)
      {
        csvfile << *it2 << ' ';
      }
      csvfile << '\n';
    }
    csvfile.close();
  }

  return 0;
}
