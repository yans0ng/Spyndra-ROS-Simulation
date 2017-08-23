#include "spyndra/spyndra.h"

int main(int argc, char **argv)
{
  spyndra::Simulator sim( argc, argv );
  spyndra::GaitGenerator gait = sim.gait_generator("csv", "walk.csv");
  gait.print_gait();
  gait.next_step();
  spyndra::Imu i(argc, argv);
  ros::Rate loop_rate(5);
  while (ros::ok() and !gait.is_over() )
  {
    std::vector<double> measure = i.measure_impl();
    //std::stringstream ss;
    std::cout << "Measure: ";
    for (int i = 0; i < 6; ++i)
      std::cout << measure[i] << " ";
    std::cout << '\n';
    //ROS_INFO_STREAM(ss.str());
    //s.move(1, -0.5 );
    gait.next_step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  /*
  //spyndra::GaitGenerator<spyndra::CsvGaitGenerator> cg("csv", "walk.csv");
  spyndra::GaitGenerator<spyndra::CsvGaitGenerator> cg = s.gait_generator("walk.csv");
  //cg.print_gait();
  
  */
  return 0;
}
