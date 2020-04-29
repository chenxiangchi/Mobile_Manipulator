#include <tf_transfrom/ft_tool.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  FT_transform ft_pub;
  ros::Rate Rate(500);
  while(ros::ok())
  {
    Rate.sleep();
  }
}