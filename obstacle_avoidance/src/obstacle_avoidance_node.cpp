#include<obstacle_avoidance/obstacle_avoidance.h>
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "obstacle_avoidance", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ObstacleAvoidance test;
    test.init();
    test.spin();
}