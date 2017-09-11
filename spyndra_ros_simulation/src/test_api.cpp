#include "spyndra/spyndra.h"

int main(int argc, char **argv)
{
  spyndra::Simulator sim( argc, argv );
  spyndra::GaitGenerator gait = sim.gait_generator("csv", "walk.csv");
  spyndra::Sensor<spyndra::ImuSensor, std::vector<double> >* i;
  spyndra::ImuSensor* iptr = new spyndra::ImuSensor(argc, argv);
  i = static_cast<spyndra::Sensor<spyndra::ImuSensor, std::vector<double> >* >(iptr);
  ros::Rate loop_rate(3);
  while (ros::ok() and !gait.is_over() )
  {
    std::vector<double> measure = i->measure();
    std::stringstream ss;
    ss << "Measure: ";
    for (int i = 0; i < 6; ++i)
      ss << measure[i] << " ";
    ROS_INFO_STREAM(ss.str());
    //s.move(1, -0.5 );
    gait.next_step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2.0).sleep(); // sleep for half a second

  i->save_csv("output.csv");
  return 0;
}
