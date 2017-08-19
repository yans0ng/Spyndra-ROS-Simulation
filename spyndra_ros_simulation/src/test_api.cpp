#include "spyndra/spyndra.h"

int main(int argc, char **argv)
{
  spyndra::Imu i(argc, argv);
  spyndra::Simulator s( argc, argv );
  //spyndra::GaitGenerator<spyndra::CsvGaitGenerator> cg("csv", "walk.csv");
  spyndra::GaitGenerator<spyndra::CsvGaitGenerator> cg = s.gait_generator("walk.csv");
  //cg.print_gait();
  ros::Rate loop_rate(5);
  while (ros::ok() and !cg.is_over() )
  {
    std::vector<double> measure = i.measure_impl();
    //std::stringstream ss;
    std::cout << "Measure: ";
    for (int i = 0; i < 6; ++i)
      std::cout << measure[i] << " ";
    std::cout << '\n';
    //ROS_INFO_STREAM(ss.str());
    //s.move(1, -0.5 );
    cg.next_step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
