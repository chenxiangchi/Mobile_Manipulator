#include <obstacle_avoidance/obstacle_avoidance.h>

void ObstacleAvoidance::init()
{
  laser_sub = nh.subscribe("/scan_filtered", 1, &ObstacleAvoidance::scanCallback, this);
  wrench_sub = nh.subscribe("/transformed_wrench_data", 100, &ObstacleAvoidance::wrenchCallback, this);
  wrench_compensation_pub = nh.advertise<geometry_msgs::WrenchStamped>("/compensated_wrench", 125);
  obstacle_pub = nh.advertise<obstacle_msgs::obstacle>("/obstacle_info",125);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/compensated_twist",1);
  ROS_INFO_STREAM("Waiting for connection ");
  while (!laser_sub.getNumPublishers() && !wrench_sub.getNumPublishers() && !wrench_compensation_pub.getNumSubscribers() && !obstacle_pub.getNumSubscribers())
    ;
  ROS_INFO_STREAM("Connection established"); 
}

void ObstacleAvoidance::scanCallback(const sensor_msgs::LaserScanConstPtr &input)
{ 
  // From -3.14159250259 to 3.14159250259 by 0.00581718236208
  // Totally 1082 ranges
  // range_min = 0.009999999
  // range_max = 20
  // Total ranges calculated = 1082 - 166 = 916
  // Obstacle info 
  //
  obstacle_info.forward = 0;
  obstacle_info.back = 0;
  obstacle_info.left = 0;
  obstacle_info.right = 0;
  obstacle_info.left_rotate = 0;
  geometry_msgs::WrenchStamped wrench_count;
  geometry_msgs::Twist twist_count;
  for(int count = 0 ; count <= input->ranges.size(); count++)
   {
     
     double theta = count * angle - pi; 
     double x = input->ranges[count] * cos(theta);
     double y = input->ranges[count] * sin(theta);
     wrench_count.header.stamp = ros::Time::now();
     //std::cout << x << "," << y << std::endl;
     // Obstacle avoidance
     // 
       //obstacle in the back
       if(x < 0.7 && x > 0.6 && y < 0.35 && y > -0.35 && obstacle_info.forward == 0)
       {
         obstacle_info.forward = 1;
         ROS_WARN_STREAM("Obstacle too near in the forward!!");

       }
       if(x < -0.6 && x >-0.9 && y > -0.35 && y < 0.35 && obstacle_info.back == 0)
       { 
         obstacle_info.back = 1;
         ROS_WARN_STREAM("Obstacle too near in the back!!");
       }
       //obstacle in the left
       if (x < 0.6 && x > -0.6 && y < 0.65 && y > 0.35 && obstacle_info.left == 0)
       { 
         obstacle_info.left = 1;
         ROS_WARN_STREAM("Obstacle too near in the left!!");
       }
       //obstacle in the right
       if (x < 0.6 && x > -0.6 && y < -0.35 && y > -0.65 && obstacle_info.right == 0)
       { 
         obstacle_info.right = 1;
         ROS_WARN_STREAM("Obstacle too near in the right!!");
       }

       if((x < 0.7 && x > 0.6 && y < 0.3 && y > -0.3) || (x > -0.7 && x < -0.6 && y < 0.3 && y > -0.3) 
           || (x < 0.6 && x > -0.6 && y < 0.4 && y > 0.3) || (x < 0.6 && x > -0.6 && y < -0.3 && y > -0.4))
       {
         obstacle_info.left_rotate = 1;
         ROS_WARN_STREAM("Obstacle too near around!!");

       }



     // Force Torque compensation
 /*     if(input->ranges[count] <= Rmax && input->ranges[count] >= Rmin && (!(y > -0.35 && y < 0.35 && x > 0.6)))
     {
       

       wrench_count.wrench.force.x += Kf * fabs(wrench_data.wrench.force.x) * cos(count * angle) / pow(input->ranges[count],4);

       wrench_count.wrench.force.y += - Kf * fabs(wrench_data.wrench.force.y) * sin(count * angle) / pow(input->ranges[count],4);
      //std::cout << wrench_data.wrench.force.x <<std::endl;
      //std::cout << wrench_count.wrench.force.x << std::endl;
      // wrench_compensation.wrench.torque.z 
      }
      */
        if(input->ranges[count] <= 1.0 && input->ranges[count] >= Rmin && (!(y > -0.35 && y < 0.35 && x > 0.6)))
        {
          twist_count.linear.x += 0.005 * cos(count * angle) / pow(input->ranges[count],6);
          twist_count.linear.y +=  0.005 * sin(count * angle) / pow(input->ranges[count],6);

        }
     
   }

  /* if( fabs(wrench_compensation.wrench.force.x) > fabs(wrench_data.wrench.force.x))
      wrench_compensation.wrench.force.x = wrench_data.wrench.force.x;
   if( fabs(wrench_compensation.wrench.force.y) > fabs(wrench_data.wrench.force.y))
      wrench_compensation.wrench.force.y = wrench_data.wrench.force.y;*/
      wrench_compensation = wrench_count;
      compensated_twist = twist_count;
      if(compensated_twist.linear.x > 0.05)
      {
         compensated_twist.linear.x = 0.05;
      }
       if(compensated_twist.linear.x < -0.05)
      {
         compensated_twist.linear.x = -0.05;
      }
      if(compensated_twist.linear.y > 0.05)
      {
         compensated_twist.linear.y = 0.05;
      }
      if(compensated_twist.linear.y < -0.05)
      {
         compensated_twist.linear.y = -0.05;
      }
}

void ObstacleAvoidance::wrenchCallback(const geometry_msgs::WrenchStamped &input)
{
  wrench_data = input;    
  std::cout << wrench_data << std::endl;
}

void ObstacleAvoidance::spin()
{ 
  while(nh.ok())
  {
  wrench_compensation_pub.publish(wrench_compensation);
  obstacle_pub.publish(obstacle_info);
  twist_pub.publish(compensated_twist);
  loop_rate.sleep();
  }
}