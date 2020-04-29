#include <tf_transfrom/ft_tool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_subscriber");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    FT_subscriber ft_sub;
    ft_sub.spin();
    ros::spin();
}