
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

sensor_msgs::Imu latest_imu_msg;
std::mutex imu_mutex;
bool imu_msg_received = false;

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    latest_imu_msg = *msg;
    imu_msg_received = true;
    // Further IMU data processing here
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh;

    ROS_INFO("ars548_dynamics_node started.");

    std::string imu_topic;
    nh.param<std::string>("yaw_rate_topic", imu_topic, "");

    ros::Subscriber imu_sub;
    if (imu_topic.empty()) {
        ROS_ERROR("Parameter 'yaw_rate_topic' is not set. Skipping IMU subscription.");
    } else {
        imu_sub = nh.subscribe(imu_topic, 1, IMUCallback);
        ROS_INFO_STREAM("Subscribed to IMU topic: " << imu_topic);
    }

    ros::Rate loop_rate(20); // 20 Hz


    while (ros::ok())
    {
        // Example: Access the latest IMU data safely
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            if (imu_msg_received) {
                double yaw_rate = latest_imu_msg.angular_velocity.z;
                double longitudinal_accel = latest_imu_msg.linear_acceleration.x;
                double lateral_accel = latest_imu_msg.linear_acceleration.y;
                // Use yaw_rate, longitudinal_accel, and lateral_accel or other IMU data here
                ROS_INFO_STREAM("[Loop] Latest yaw rate: " << yaw_rate
                                << ", longitudinal accel: " << longitudinal_accel
                                << ", lateral accel: " << lateral_accel);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}