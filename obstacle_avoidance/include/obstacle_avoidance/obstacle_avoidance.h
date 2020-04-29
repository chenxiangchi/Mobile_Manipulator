#ifndef obstacle_avoidance
#define obstacle_avoidance

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <obstacle_msgs/obstacle.h>
class ObstacleAvoidance
{

public:
    //
    // The length of center point to fowark/back point 
    float car_length;
    // The circle radius of obstacle detection
    float car_radius;
    // The angle increment
    double angle;
    // PI
    double pi;
    // Radius of detection area
    double Rmin;
    double Rmax;
    // Ranges to be ignored
    int Rfl;
    int Rfr;
    // Ranges to be detected
    int Rbl;
    int Rbr;
    // Gain coefficient Kf =  kf / ((Ranges - (Rr - Rl)) * Rmin)
    double Kf;
    // Gain coefficient Kt
    double Kt;
    // loop rate;
    ros::Rate loop_rate;
    // 
    geometry_msgs::Twist compensated_twist;
    // Laser scan data 
    sensor_msgs::LaserScan laser_scan;
    // Oringinal wrench data
    geometry_msgs::WrenchStamped wrench_data;
    // Compensated wrench to be published
    geometry_msgs::WrenchStamped wrench_compensation;
    // Obstacle info
    obstacle_msgs::obstacle obstacle_info;
    // Subscribe scan_info and regist callback
    void scanCallback(const sensor_msgs::LaserScanConstPtr &input);
    // Subsriber wrench_data
    void wrenchCallback(const geometry_msgs::WrenchStamped &input);
    // init
    void init();
    // spin
    void spin();

    ObstacleAvoidance():loop_rate(125)
    {   
        angle = 0.00581718236208; 
        pi = 3.14159250259; 
        Rmin = 0.7;
        Rmax = 2;
        Rfl = 457;
        Rfr = 624;
        Rbl = 100;
        Rbr = 980;
        Kf = 0.005;
        Kt = 0.00005;
    }

private:
    // Create nodehandle
    ros::NodeHandle nh;
    // 
    ros::Subscriber laser_sub;
    //
    ros::Subscriber wrench_sub;
    //
    ros::Publisher  wrench_compensation_pub;
    //
    ros::Publisher  obstacle_pub;
    //
    ros::Publisher  twist_pub;
};

#endif