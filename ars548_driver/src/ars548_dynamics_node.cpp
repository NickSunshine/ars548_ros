#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh;

    ROS_INFO("ars548_dynamics_node started.");

    ros::Rate loop_rate(20); // 20 Hz

    while (ros::ok())
    {
        // Your control logic here

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}