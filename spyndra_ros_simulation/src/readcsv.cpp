#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
  // ROS setup
  ros::init(argc, argv, "readcsv");
  ros::NodeHandle n;
  //ros::Time::init();
  int freq = 10;
  ros::Rate loop_rate(freq);

  // Controllers
  ros::Publisher j1 = n.advertise<std_msgs::Float64>("/spyndra/joint1_position_controller/command",freq);
  ros::Publisher j2 = n.advertise<std_msgs::Float64>("/spyndra/joint2_position_controller/command",freq);
  ros::Publisher j3 = n.advertise<std_msgs::Float64>("/spyndra/joint3_position_controller/command",freq);
  ros::Publisher j4 = n.advertise<std_msgs::Float64>("/spyndra/joint4_position_controller/command",freq);
  ros::Publisher j5 = n.advertise<std_msgs::Float64>("/spyndra/joint5_position_controller/command",freq);
  ros::Publisher j6 = n.advertise<std_msgs::Float64>("/spyndra/joint6_position_controller/command",freq);
  ros::Publisher j7 = n.advertise<std_msgs::Float64>("/spyndra/joint7_position_controller/command",freq);
  ros::Publisher j8 = n.advertise<std_msgs::Float64>("/spyndra/joint8_position_controller/command",freq);

  std::vector<ros::Publisher> controllers;
  controllers.resize(8);
  controllers[0] = j1;
  controllers[1] = j2;
  controllers[2] = j3;
  controllers[3] = j4;
  controllers[4] = j5;
  controllers[5] = j6;
  controllers[6] = j7;
  controllers[7] = j8;
  std::vector<ros::Publisher>::iterator itr = controllers.begin();

  // Read csv file
  std::string delimeter = " ";
  std::ifstream csvfile ("walk.csv");
  std::string line, token;

  if (csvfile.is_open())
  {
    // Omit the first line
    std::getline( csvfile, line);

    // Loop through each line
    while ( std::getline ( csvfile, line) )
    {
      ROS_INFO_STREAM("Next line");
      size_t last = 0, next = 0;

      // Omit the first 6 fields (IMU measurements)
      for (int i = 0; i < 7; ++i)
        next = line.find(delimeter,next+1);
      last = next + 1;

      // Borrow from https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
      //while ( (next = line.find(delimeter,last)) != std::string::npos )
      // Send 8 motor commands
      for (int i = 0; i < 8; ++i)
      {
        // Separate tokens
        next = line.find(delimeter, last);
        token = line.substr(last, next-last);

        // Instantiate command
        std_msgs::Float64 cmd;
        //TODO: need correct scale
        cmd.data = atof(token.c_str()) /360 - 0.5; 
        (*itr).publish(cmd);
        ROS_INFO_STREAM(cmd.data);

        last = next + 1;
      }

      itr = controllers.begin();

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  else std::cout << "Unable to open file\n";

  return 0;
}
