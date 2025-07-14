
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <mutex>

sensor_msgs::Imu latest_imu_msg;
std_msgs::Float64 latest_speed_msg;
dbw_mkz_msgs::SteeringReport latest_steering_msg;
std::mutex imu_mutex;
std::mutex speed_mutex;
std::mutex steering_mutex;
bool imu_msg_received = false;
bool speed_msg_received = false;
bool steering_msg_received = false;

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    latest_imu_msg = *msg;
    imu_msg_received = true;
    // Further IMU data processing here
}

void SpeedCallback(const std_msgs::Float64::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(speed_mutex);
    latest_speed_msg = *msg;
    speed_msg_received = true;
    // Further speed data processing here
}

void SteeringCallback(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(steering_mutex);
    latest_steering_msg = *msg;
    steering_msg_received = true;
    // Further steering data processing here
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh;

    ROS_INFO("ars548_dynamics_node started.");

    std::string imu_topic;
    std::string speed_topic;
    std::string steering_topic;
    nh.param<std::string>("imu_topic", imu_topic, "");
    nh.param<std::string>("speed_topic", speed_topic, "");
    nh.param<std::string>("steering_topic", steering_topic, "");

    ros::Subscriber imu_sub;
    if (imu_topic.empty()) {
        ROS_ERROR("Parameter 'imu_topic' is not set. Skipping IMU subscription.");
    } else {
        imu_sub = nh.subscribe(imu_topic, 1, IMUCallback);
        ROS_INFO_STREAM("Subscribed to IMU topic: " << imu_topic);
    }

    ros::Subscriber speed_sub;
    if (speed_topic.empty()) {
        ROS_ERROR("Parameter 'speed_topic' is not set. Skipping speed subscription.");
    } else {
        speed_sub = nh.subscribe(speed_topic, 1, SpeedCallback);
        ROS_INFO_STREAM("Subscribed to speed topic: " << speed_topic);
    }

    ros::Subscriber steering_sub;
    if (steering_topic.empty()) {
        ROS_ERROR("Parameter 'steering_topic' is not set. Skipping steering subscription.");
    } else {
        steering_sub = nh.subscribe(steering_topic, 1, SteeringCallback);
        ROS_INFO_STREAM("Subscribed to steering topic: " << steering_topic);
    }

    ros::Rate loop_rate(20); // 20 Hz

    while (ros::ok())
    {
        // Example: Access the latest IMU data safely
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            if (imu_msg_received) {
                double yaw_rate = latest_imu_msg.angular_velocity.z * 180.0 / M_PI; // Convert to deg/s
                double longitudinal_accel = latest_imu_msg.linear_acceleration.x; // m/s^2
                double lateral_accel = latest_imu_msg.linear_acceleration.y; // m/s^2
                ROS_INFO_STREAM("[Loop] Latest yaw rate (deg/s): " << yaw_rate
                                << ", longitudinal accel (m/s^2): " << longitudinal_accel
                                << ", lateral accel (m/s^2): " << lateral_accel);
            }
        }

        // Example: Access the latest speed data safely
        {
            std::lock_guard<std::mutex> lock(speed_mutex);
            if (speed_msg_received) {
                double speed = latest_speed_msg.data * 3.6; // Convert to km/h
                ROS_INFO_STREAM("[Loop] Latest speed: " << speed << " km/h");
            }
        }

        // Example: Access the latest steering data safely
        {
            std::lock_guard<std::mutex> lock(steering_mutex);
            if (steering_msg_received) {
                // Convert steering wheel angle rad to steering front axle degrees (14.81:1 ratio)
                double front_wheel_angle_deg = latest_steering_msg.steering_wheel_angle * (180.0 / M_PI) / 14.81; 
                ROS_INFO_STREAM("[Loop] Latest front wheel angle: " << front_wheel_angle_deg << " degrees");
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}