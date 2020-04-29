
#include <tf_transfrom/ft_tool.h>
/* 
**FT_transform Functions
*/
void FT_transform::publishTF()
{   
        // Attention: If set tf_bro as a class member,the node won't response to the ft_publisher!
        
        ft_bro.sendTransform(
            tf::StampedTransform(
                transform,
                ros::Time::now(), "world0", "ft_sensor"));
        //std::cout << "Publishing TF from world to ft_sensor"  << std::endl;
}

void FT_transform::init()
{   
    tool_pose_sub = n_ft.subscribe("/ur_arm/tool_pos", 100, &FT_transform::poseCallback, this);
    while(!tool_pose_sub.getNumPublishers());
}

void FT_transform::poseCallback(const geometry_msgs::Pose &msg)
{ 
    current_pose.pose = msg;   
    tf_quaternion.setW(msg.orientation.w);
    tf_quaternion.setX(msg.orientation.x);
    tf_quaternion.setY(msg.orientation.y);
    tf_quaternion.setZ(msg.orientation.z);
    tf_quaternion = basic_quaternion * tf_quaternion;
    transform.setOrigin(basic_vector);
    transform.setRotation(tf_quaternion);
    publishTF();
}


/* 
**FT_subscriber Functions
*/
void FT_subscriber::transformWrench(const tf::TransformListener &listener)
{

    // transform wrench_force
    wrench_force.header.frame_id = "ft_sensor";
    wrench_force.header.stamp = ros::Time(0);
    wrench_force.vector.x = wrench_data.wrench.force.x;
    wrench_force.vector.y = wrench_data.wrench.force.y;
    wrench_force.vector.z = wrench_data.wrench.force.z;

    // transform wrench_torque
    wrench_torque.header.frame_id = "ft_sensor";
    wrench_torque.header.stamp = ros::Time(0);
    wrench_torque.vector.x = wrench_data.wrench.torque.x;
    wrench_torque.vector.y = wrench_data.wrench.torque.y;
    wrench_torque.vector.z = wrench_data.wrench.torque.z;

    try
    {   
       
        //ft_lis.waitForTransform("world0", "ft_sensor", ros::Time(0), ros::Duration(1));
        ft_lis.transformVector("world0", wrench_force, transformed_force);
        ft_lis.transformVector("world0", wrench_torque, transformed_torque);
        ROS_INFO("oringinal_force: (%.5f, %.5f. %.5f) -----> transformed: (%.5f, %.5f, %.5f) at time %.2f",
                 wrench_force.vector.x, wrench_force.vector.y, wrench_force.vector.z,
                 transformed_force.vector.x, transformed_force.vector.y, transformed_force.vector.z, transformed_force.header.stamp.toSec());
        ROS_INFO("oringinal_torque: (%.5f, %.5f. %.5f) -----> transformed: (%.5f, %.5f, %.5f) at time %.2f",
                 wrench_torque.vector.x, wrench_torque.vector.y, wrench_torque.vector.z,
                 transformed_torque.vector.x, transformed_torque.vector.y, transformed_torque.vector.z, transformed_torque.header.stamp.toSec());

    }
    catch (tf::TransformException &ex)
    {
        //ROS_ERROR("Received an exception trying to transform wrench from \"world0\" to \"ft_sensor\": %s", ex.what());
         ROS_INFO_STREAM("Waiting for TF");
    }
    wrench_data.wrench.force.x = transformed_force.vector.x;
    wrench_data.wrench.force.y = transformed_force.vector.y;
    wrench_data.wrench.force.z = transformed_force.vector.z;
    wrench_data.wrench.torque.x = transformed_torque.vector.x;
    wrench_data.wrench.torque.y = transformed_torque.vector.y;
    wrench_data.wrench.torque.z = transformed_torque.vector.z;
}

void FT_subscriber::wrenchCallback(const geometry_msgs::WrenchStamped &_wrench)
{
    // Obtain wrench_data(Need to be setZero and filtered)
    wrench_data = _wrench;
    // Filt high frequency data
    lowpassFilter();
    // Compensate for the temperature drift
    setZero(wrench_data);
    // Set threshold
    wrench_data.wrench = setThres(wrench_data.wrench,Threshold);
    // Transform wrench_data to ft_sensor frame
    transformWrench(ft_lis);
    // compensate for calculate error
    wrench_data.wrench = setThres(wrench_data.wrench,Cerror);
    // Publish Transformed wrench data
    ft_wrench_pub.publish(wrench_data);
}

void FT_subscriber::lowpassFilter()
{
    wrench_external << wrench_data.wrench.force.x, wrench_data.wrench.force.y,
        wrench_data.wrench.force.z, wrench_data.wrench.torque.x,
        wrench_data.wrench.torque.y, wrench_data.wrench.torque.z;
    wrench_external << (1 - wrench_filter_factor) * wrench_external_old +
                           wrench_filter_factor * wrench_external;
    wrench_data.wrench.force.x = wrench_external[0];
    wrench_data.wrench.force.y = wrench_external[1];
    wrench_data.wrench.force.z = wrench_external[2];
    wrench_data.wrench.torque.x = wrench_external[3];
    wrench_data.wrench.torque.y = wrench_external[4];
    wrench_data.wrench.torque.z = wrench_external[5];
    wrench_external_old = wrench_external;
}

void FT_subscriber::setZero(geometry_msgs::WrenchStamped _wrench_data)
{
    wrench_data.wrench.force.x -= wrench_error.force.x;
    wrench_data.wrench.force.y -= wrench_error.force.y;
    wrench_data.wrench.force.z -= wrench_error.force.z;
    wrench_data.wrench.torque.x -= wrench_error.torque.x;
    wrench_data.wrench.torque.y -= wrench_error.torque.y;
    wrench_data.wrench.torque.z -= wrench_error.torque.z;
    //std::cout << wrench_data << std::endl;
}

geometry_msgs::Wrench FT_subscriber::setThres(geometry_msgs::Wrench _wrench_data, const geometry_msgs::Wrench _Threshold)
{
    if (fabs(_wrench_data.force.x) < _Threshold.force.x)
        _wrench_data.force.x = 0;
    if (fabs(_wrench_data.force.y) < _Threshold.force.y)
        _wrench_data.force.y = 0;
    if (fabs(_wrench_data.force.z) < _Threshold.force.z)
        _wrench_data.force.z = 0;
    if (fabs(_wrench_data.torque.x) < _Threshold.torque.x)
        _wrench_data.torque.x = 0;
    if (fabs(_wrench_data.torque.y) < _Threshold.torque.y)
        _wrench_data.torque.y = 0;
    if (fabs(_wrench_data.torque.z) < _Threshold.torque.z)
        _wrench_data.torque.z = 0;
    return _wrench_data;
}

void FT_subscriber::init()
{   


    // n_ftp.getParm(Threshold);
    Threshold.force.x = 5;
    Threshold.force.y = 5;
    Threshold.force.z = 5;
    Threshold.torque.x = 0.2;
    Threshold.torque.y = 0.2;
    Threshold.torque.z = 0.2;
    // n_ftp.getParm(Cerror);
    // Avoid the Calculate error
    Cerror.force.x = 5;
    Cerror.force.y = 5;
    Cerror.force.z = 5;
    Cerror.torque.x = 0.5;
    Cerror.torque.y = 0.5;
    Cerror.torque.z = 0.5;

    
    //Define the subscribers and publishers
    ft_wrench_sub = n_ftp.subscribe("netft_data", 100, &FT_subscriber::wrenchCallback, this);
    ft_wrench_pub = n_ftp.advertise<geometry_msgs::WrenchStamped>("transformed_wrench_data", 125);

    while (ft_wrench_sub.getNumPublishers())
        ;
    ROS_INFO_STREAM("Connected to the ft_publihser");
    // Wait for wrench data obtained
    while (!wrench_error.force.x)
    {
        wrench_error = wrench_data.wrench;
    }
    ROS_INFO_STREAM("Obtain wrench data");
}

void FT_subscriber::spin()
{
    while (ros::ok())
    {
        //std::cout << wrench_data << std::endl;
        rate.sleep();
    }
}