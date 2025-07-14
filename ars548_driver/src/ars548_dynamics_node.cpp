#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/Gear.h>

#include <mutex>

sensor_msgs::Imu latest_imu_msg;
std_msgs::Float64 latest_speed_msg;
dbw_mkz_msgs::SteeringReport latest_steering_msg;
dbw_mkz_msgs::GearReport latest_gear_report_msg;
uint8_t latest_gear = dbw_mkz_msgs::Gear::NONE;
DrivingDirection latest_direction = DrivingDirection::Standstill;
std::mutex imu_mutex;
std::mutex speed_mutex;
std::mutex steering_mutex;
std::mutex gear_mutex;
bool imu_msg_received = false;
bool speed_msg_received = false;
bool steering_msg_received = false;
bool gear_msg_received = false;

// Enum for driving direction
enum class DrivingDirection {
    Standstill = 0,
    Forward = 1,
    Backward = 2
};

// Helper function to convert DrivingDirection to string
std::string directionToString(DrivingDirection dir)
{
    switch (dir) {
        case DrivingDirection::Standstill: return "Standstill";
        case DrivingDirection::Forward: return "Forward";
        case DrivingDirection::Backward: return "Backward";
        default: return "Unknown";
    }
}

// Helper function to convert dbw_mkz_msgs::Gear enum to string
std::string gearToString(uint8_t gear)
{
    switch (gear) {
        case dbw_mkz_msgs::Gear::NONE: return "NONE";
        case dbw_mkz_msgs::Gear::PARK: return "PARK";
        case dbw_mkz_msgs::Gear::REVERSE: return "REVERSE";
        case dbw_mkz_msgs::Gear::NEUTRAL: return "NEUTRAL";
        case dbw_mkz_msgs::Gear::DRIVE: return "DRIVE";
        case dbw_mkz_msgs::Gear::LOW: return "LOW";
        default: return "UNKNOWN";
    }
}

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

void GearCallback(const dbw_mkz_msgs::GearReport::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(gear_mutex);
    latest_gear_report_msg = *msg;
    gear_msg_received = true;
    // Further gear data processing here
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh;

    ROS_INFO("ars548_dynamics_node started.");

    std::string imu_topic;
    std::string speed_topic;
    std::string steering_topic;
    double steering_gear_ratio = 14.81;
    std::string gear_topic;
    nh.param<std::string>("imu_topic", imu_topic, "");
    nh.param<std::string>("speed_topic", speed_topic, "");
    nh.param<std::string>("steering_topic", steering_topic, "");
    nh.param<std::string>("gear_topic", gear_topic, "");
    nh.param<double>("steering_gear_ratio", steering_gear_ratio, 14.81);

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

    ros::Subscriber gear_sub;
    if (gear_topic.empty()) {
        ROS_ERROR("Parameter 'gear_topic' is not set. Skipping gear subscription.");
    } else {
        gear_sub = nh.subscribe(gear_topic, 1, GearCallback);
        ROS_INFO_STREAM("Subscribed to gear topic: " << gear_topic);
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
                // Convert steering wheel angle rad to steering front axle degrees using parameterized gear ratio
                double front_wheel_angle_deg = latest_steering_msg.steering_wheel_angle * (180.0 / M_PI) / steering_gear_ratio;
                ROS_INFO_STREAM("[Loop] Latest front wheel angle: " << front_wheel_angle_deg << " degrees (gear ratio: " << steering_gear_ratio << ")");
            }
        }


        // Example: Access the latest gear data safely
        {
            std::lock_guard<std::mutex> lock(gear_mutex);
            if (gear_msg_received) {
                latest_gear = latest_gear_report_msg.gear;
                std::string gear_str = gearToString(latest_gear);
                // Map gear to driving direction
                switch (latest_gear) {
                    case dbw_mkz_msgs::Gear::DRIVE:
                    case dbw_mkz_msgs::Gear::LOW:
                        latest_direction = DrivingDirection::Forward;
                        break;
                    case dbw_mkz_msgs::Gear::REVERSE:
                        latest_direction = DrivingDirection::Backward;
                        break;
                    case dbw_mkz_msgs::Gear::PARK:
                    case dbw_mkz_msgs::Gear::NEUTRAL:
                    case dbw_mkz_msgs::Gear::NONE:
                    default:
                        latest_direction = DrivingDirection::Standstill;
                        break;
                }
                ROS_INFO_STREAM("[Loop] Latest gear state: " << gear_str << " (" << static_cast<int>(latest_gear) << ") | Driving direction: " << directionToString(latest_direction) << " (" << static_cast<int>(latest_direction) << ")");
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}