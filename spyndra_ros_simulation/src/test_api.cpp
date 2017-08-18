#include "spyndra/spyndra.h"

int main(int argc, char **argv)
{
  spyndra::Simulator s( argc, argv );
  //ros::spin();
  spyndra::CsvGaitGenerator cg = s.gait_generator();
  cg.set_csv_file( "walk.csv" );
  cg.print_gait();
  ros::Rate loop_rate(5);
  while (ros::ok() and !cg.is_over() )
  {
    //s.move(1, -0.5 );
    cg.next_step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
