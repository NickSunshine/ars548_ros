#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/Gear.h>
#include <mutex>

template<typename T>
struct ThreadSafeData {
    T msg;
    std::mutex mtx;
    bool received = false;
};

enum class DrivingDirection {
    Standstill = 0,
    Forward = 1,
    Backward = 2
};

std::string directionToString(DrivingDirection dir)
{
    switch (dir) {
        case DrivingDirection::Standstill: return "Standstill";
        case DrivingDirection::Forward: return "Forward";
        case DrivingDirection::Backward: return "Backward";
        default: return "Unknown";
    }
}

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

int main(int argc, char** argv)
{
    ThreadSafeData<sensor_msgs::Imu> imu_data;
    ThreadSafeData<std_msgs::Float64> speed_data;
    ThreadSafeData<dbw_mkz_msgs::SteeringReport> steering_data;
    ThreadSafeData<dbw_mkz_msgs::GearReport> gear_data;

    uint8_t latest_gear = dbw_mkz_msgs::Gear::NONE;
    DrivingDirection latest_direction = DrivingDirection::Standstill;

    // Lambda callback for IMU
    auto imu_callback = [&](const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_data.mtx);
        imu_data.msg = *msg;
        imu_data.received = true;
        // Further IMU data processing here
    };

    // Lambda callback for Speed
    auto speed_callback = [&](const std_msgs::Float64::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(speed_data.mtx);
        speed_data.msg = *msg;
        speed_data.received = true;
        // Further speed data processing here
    };

    // Lambda callback for Steering
    auto steering_callback = [&](const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(steering_data.mtx);
        steering_data.msg = *msg;
        steering_data.received = true;
        // Further steering data processing here
    };

    // Lambda callback for Gear
    auto gear_callback = [&](const dbw_mkz_msgs::GearReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(gear_data.mtx);
        gear_data.msg = *msg;
        gear_data.received = true;
        // Further gear data processing here
    };
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
        imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imu_callback);
        ROS_INFO_STREAM("Subscribed to IMU topic: " << imu_topic);
    }

    ros::Subscriber speed_sub;
    if (speed_topic.empty()) {
        ROS_ERROR("Parameter 'speed_topic' is not set. Skipping speed subscription.");
    } else {
        speed_sub = nh.subscribe<std_msgs::Float64>(speed_topic, 1, speed_callback);
        ROS_INFO_STREAM("Subscribed to speed topic: " << speed_topic);
    }

    ros::Subscriber steering_sub;
    if (steering_topic.empty()) {
        ROS_ERROR("Parameter 'steering_topic' is not set. Skipping steering subscription.");
    } else {
        steering_sub = nh.subscribe<dbw_mkz_msgs::SteeringReport>(steering_topic, 1, steering_callback);
        ROS_INFO_STREAM("Subscribed to steering topic: " << steering_topic);
    }

    ros::Subscriber gear_sub;
    if (gear_topic.empty()) {
        ROS_ERROR("Parameter 'gear_topic' is not set. Skipping gear subscription.");
    } else {
        gear_sub = nh.subscribe<dbw_mkz_msgs::GearReport>(gear_topic, 1, gear_callback);
        ROS_INFO_STREAM("Subscribed to gear topic: " << gear_topic);
    }

    ros::Rate loop_rate(20); // 20 Hz


    std::string node_name = ros::this_node::getName();

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        std::string stamp_str = std::to_string(now.sec) + "." + std::to_string(now.nsec);

        // Example: Access the latest IMU data safely
        {
            std::lock_guard<std::mutex> lock(imu_data.mtx);
            if (imu_data.received) {
                double yaw_rate = imu_data.msg.angular_velocity.z * 180.0 / M_PI; // Convert to deg/s
                double longitudinal_accel = imu_data.msg.linear_acceleration.x; // m/s^2
                double lateral_accel = imu_data.msg.linear_acceleration.y; // m/s^2
                ROS_INFO_STREAM("[" << node_name << "] [" << stamp_str << "] [Loop] Latest yaw rate (deg/s): " << yaw_rate
                                << ", longitudinal accel (m/s^2): " << longitudinal_accel
                                << ", lateral accel (m/s^2): " << lateral_accel);
            }
        }

        // Example: Access the latest speed data safely
        {
            std::lock_guard<std::mutex> lock(speed_data.mtx);
            if (speed_data.received) {
                double speed = speed_data.msg.data * 3.6; // Convert to km/h
                ROS_INFO_STREAM("[" << node_name << "] [" << stamp_str << "] [Loop] Latest speed: " << speed << " km/h");
            }
        }

        // Example: Access the latest steering data safely
        {
            std::lock_guard<std::mutex> lock(steering_data.mtx);
            if (steering_data.received) {
                // Convert steering wheel angle rad to steering front axle degrees using parameterized gear ratio
                double front_wheel_angle_deg = steering_data.msg.steering_wheel_angle * (180.0 / M_PI) / steering_gear_ratio;
                ROS_INFO_STREAM("[" << node_name << "] [" << stamp_str << "] [Loop] Latest front wheel angle: " << front_wheel_angle_deg << " degrees (gear ratio: " << steering_gear_ratio << ")");
            }
        }

        // Example: Access the latest gear data safely
        {
            std::lock_guard<std::mutex> lock(gear_data.mtx);
            if (gear_data.received) {
                latest_gear = gear_data.msg.gear;
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
                ROS_INFO_STREAM("[" << node_name << "] [" << stamp_str << "] [Loop] Latest gear state: " << gear_str << " (" << static_cast<int>(latest_gear) << ") | Driving direction: " << directionToString(latest_direction) << " (" << static_cast<int>(latest_direction) << ")");
            }
        }

        // TODO:  Handle packaging up outgoing data to UDP message to sensor

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}