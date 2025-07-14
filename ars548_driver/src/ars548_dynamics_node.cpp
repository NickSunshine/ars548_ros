
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/Gear.h>
#include <mutex>

// UDP socket includes (from ars548_driver.hpp)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>


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
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh("~");
    ROS_INFO("ars548_dynamics_node started.");

    // --- IMU Setup ---
    ThreadSafeData<sensor_msgs::Imu> imu_data;
    // Lambda callback for IMU
    auto imu_callback = [&](const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_data.mtx);
        imu_data.msg = *msg;
        imu_data.received = true;
        // Further IMU data processing here
    };
    std::string imu_topic;
    nh.param<std::string>("imu_topic", imu_topic, "");
    ros::Subscriber imu_sub;
    if (imu_topic.empty()) {
        ROS_ERROR("Parameter 'imu_topic' is not set. Skipping IMU subscription.");
    } else {
        imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imu_callback);
        ROS_INFO_STREAM("Subscribed to IMU topic: " << imu_topic);
    }
    // --- End IMU Setup ---
    
    // --- Speed Setup ---
    ThreadSafeData<std_msgs::Float64> speed_data;
    // Lambda callback for Speed
    auto speed_callback = [&](const std_msgs::Float64::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(speed_data.mtx);
        speed_data.msg = *msg;
        speed_data.received = true;
        // Further speed data processing here
    };
    std::string speed_topic;
    nh.param<std::string>("speed_topic", speed_topic, "");
    ros::Subscriber speed_sub;
    if (speed_topic.empty()) {
        ROS_ERROR("Parameter 'speed_topic' is not set. Skipping speed subscription.");
    } else {
        speed_sub = nh.subscribe<std_msgs::Float64>(speed_topic, 1, speed_callback);
        ROS_INFO_STREAM("Subscribed to speed topic: " << speed_topic);
    }
    // --- End Speed Setup ---

    // --- Steering Setup ---
    ThreadSafeData<dbw_mkz_msgs::SteeringReport> steering_data;
    // Lambda callback for Steering
    auto steering_callback = [&](const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(steering_data.mtx);
        steering_data.msg = *msg;
        steering_data.received = true;
        // Further steering data processing here
    };
    std::string steering_topic;
    double steering_gear_ratio = 14.81;
    nh.param<std::string>("steering_topic", steering_topic, "");
    ros::Subscriber steering_sub;
    if (steering_topic.empty()) {
        ROS_ERROR("Parameter 'steering_topic' is not set. Skipping steering subscription.");
    } else {
        steering_sub = nh.subscribe<dbw_mkz_msgs::SteeringReport>(steering_topic, 1, steering_callback);
        ROS_INFO_STREAM("Subscribed to steering topic: " << steering_topic);
    }
    // --- End Steering Setup ---

    // --- Gear Setup ---
    ThreadSafeData<dbw_mkz_msgs::GearReport> gear_data;
    uint8_t latest_gear = dbw_mkz_msgs::Gear::NONE;
    DrivingDirection latest_direction = DrivingDirection::Standstill;
    // Lambda callback for Gear
    auto gear_callback = [&](const dbw_mkz_msgs::GearReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(gear_data.mtx);
        gear_data.msg = *msg;
        gear_data.received = true;
        // Further gear data processing here
    };
    std::string gear_topic;
    nh.param<std::string>("gear_topic", gear_topic, "");
    nh.param<double>("steering_gear_ratio", steering_gear_ratio, 14.81);
    ros::Subscriber gear_sub;
    if (gear_topic.empty()) {
        ROS_ERROR("Parameter 'gear_topic' is not set. Skipping gear subscription.");
    } else {
        gear_sub = nh.subscribe<dbw_mkz_msgs::GearReport>(gear_topic, 1, gear_callback);
        ROS_INFO_STREAM("Subscribed to gear topic: " << gear_topic);
    }
    // --- End Gear Setup ---

    // --- UDP Socket Initialization ---
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        perror("UDP socket creation failed");
        return 1;
    }

    // Set up source address (local IP/port)
    struct sockaddr_in src_addr;
    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.sin_family = AF_INET;
    std::string src_ip;
    int src_port;
    nh.param<std::string>("radarCfgSrcIP", src_ip, std::string("10.13.1.166"));
    nh.param<int>("radarCfgSrcPort", src_port, 42401);
    src_addr.sin_port = htons(src_port);
    src_addr.sin_addr.s_addr = inet_addr(src_ip.c_str());

    // Bind the socket to the source address
    if (bind(udp_sock, (struct sockaddr*)&src_addr, sizeof(src_addr)) < 0) {
        perror("UDP socket bind failed");
        close(udp_sock);
        return 1;
    }

    // Set up destination address (sensor IP/port)
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    std::string dest_ip;
    int dest_port;
    nh.param<std::string>("radarCfgDstIP", dest_ip, std::string("10.13.1.113"));
    nh.param<int>("radarCfgDstPort", dest_port, 42101);
    dest_addr.sin_port = htons(dest_port);
    dest_addr.sin_addr.s_addr = inet_addr(dest_ip.c_str());
    // --- End UDP Socket Initialization ---   

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
                latest_gear = gear_data.msg.state.gear;
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
        // --- UDP Message Packaging and Sending (Pseudocode) ---
        // 1. [Initialization, outside loop]
        //    - Create and open a UDP socket (once, before the loop).
        //    - Set up source and destination sockaddr_in structs.
        //    - Bind the socket to the source address/port if required.
        //
        // 2. [Inside main loop, at desired send rate]
        //    - For each of the six messages to send:
        //        a. Serialize the message into a temporary buffer.
        //        b. Ensure each message starts with its own "Header 1st Part" (ServiceID, MethodID, PayloadLength, etc.).
        //    - Concatenate all six serialized message buffers into a single send buffer.
        //    - Check that the total buffer size does not exceed the UDP payload limit.
        //    - Send the combined buffer in a single sendto() call to the sensor.
        //
        // 3. [Shutdown, after loop]
        //    - Close the UDP socket.
        //
        // // Note: Use mutexes or thread-safe access if any message data is updated from callbacks.
        // // Note: If any message is optional, check if it should be included before serializing/concatenating.
        // --- End UDP Message Packaging and Sending (Pseudocode) ---
        ros::spinOnce();
        loop_rate.sleep();
    }

    // --- UDP Socket Shutdown ---
    close(udp_sock);
    // --- End UDP Socket Shutdown ---

    return 0;
}