//This tf_transform is prepared for the transformation from ATI FT_SENSOR to UR5 tool frame.
#ifndef ft_tool
#define ft_tool

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <geometry_msgs/WrenchStamped.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

/*
 *Class FT_transform for transform from FT sensor to tool frame
 */
class FT_transform
{

public:
  // Transform quaternion form tool0 to sensor
  const tf::Quaternion basic_quaternion;
  // Transform Vector form tool0 to sensor
  const tf::Vector3 basic_vector;
  // Storage of tool0 pose
  tf::Quaternion tf_quaternion;
  // Storage of transform from tool0 to sensor
  tf::Transform basic_transform;
  // Storage of transform from world to tool0
  tf::Transform transform;
  // TF broadcasted
  tf::TransformBroadcaster ft_bro;
  geometry_msgs::PoseStamped current_pose;

  // Initialize the tranform as default(0,0.707107,0,0.707107)
  FT_transform():basic_quaternion(0, 0.707107, 0, 0.707107),basic_vector(0,0,0)
  {
    init();
  }
  // Publish transfrom
  void publishTF();

protected:
  // Create a nodehandle
  ros::NodeHandle n_ft;
  // Initialize the FT_transform
  void init();
  // Recieve UR5 tool0 pose
  void poseCallback(const geometry_msgs::Pose &msg);

private:
  ros::Subscriber tool_pose_sub;
};


/*
 *Class FT_subscriber to transform the wrench
 */
class FT_subscriber
{

public:
  // Listen to the ft_publisher
  tf::TransformListener ft_lis;
  tf::StampedTransform ft_transform;
  
  geometry_msgs::WrenchStamped wrench_data;
  // Recieved wrench_force
  geometry_msgs::Vector3Stamped wrench_force;
  // Revieved wrench_torque
  geometry_msgs::Vector3Stamped wrench_torque;
  // Transformed_force
  geometry_msgs::Vector3Stamped transformed_force;
  // Transformed_torque
  geometry_msgs::Vector3Stamped transformed_torque;
  // Const Threshold 
  geometry_msgs::Wrench Threshold;
  // The error from the .. sensor
  geometry_msgs::Wrench wrench_error;
  // Const Cerror
  geometry_msgs::Wrench Cerror;
  // Transform Rate
  ros::Rate rate;
  // Filter Factor
  double wrench_filter_factor;
  Vector6d wrench_external;
  Vector6d wrench_external_old;

  FT_subscriber() : rate(500), ft_lis(ros::Duration(10)),wrench_filter_factor(1)
  {
    init();
  }
  // Initialize the FT_subscriber
  void init();
  // Spin  
  void spin();
protected:
  ros::NodeHandle n_ftp;
  // Subscribe the data from sensor
  ros::Subscriber ft_wrench_sub;
  // Publish transformed wrench data
  ros::Publisher ft_wrench_pub; 
  
private: 
  // Transform wrench_force
  void transformWrench(const tf::TransformListener &listener);
  // Reciving wrench data
  void wrenchCallback(const geometry_msgs::WrenchStamped &_wrench);
  // Low pass Filter
  void lowpassFilter();
  // Zero the oringinal data of the ft_300_sensor
  void setZero(geometry_msgs::WrenchStamped _wrench_data);
  // Set Threshold for the bias
  geometry_msgs::Wrench setThres(geometry_msgs::Wrench _wrench_data,const geometry_msgs::Wrench _Threshold);

};

class Gravity_compensation
{
};

#endif
