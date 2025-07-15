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

#include "ars548_data.h"

template<typename T>
struct ThreadSafeData {
    T msg;
    std::mutex mtx;
    bool received = false;
};

enum class MotionState {
    Standstill = 0,
    Forward = 1,
    Backward = 2
};

std::string motionStateToString(MotionState state)
{
    switch (state) {
        case MotionState::Standstill: return "Standstill";
        case MotionState::Forward: return "Forward";
        case MotionState::Backward: return "Backward";
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

// --- Helper for float serialization (network order) ---
static uint32_t serializeFloat32(float value) {
    uint32_t temp;
    memcpy(&temp, &value, sizeof(temp));
    return htonl(temp);
}

// --- Packs AccelerationLateralCoG message into buffer ---
static size_t PackAccelerationLateralCoG(char* buffer, size_t offset, float lateral_accel) {
    AccelerationLateralCoG accelLatCog = {};
    accelLatCog.ServiceID = htons(0);
    accelLatCog.MethodID = htons(321);
    accelLatCog.PayloadLength = htonl(32);
    accelLatCog.AccelerationLateral = lateral_accel;

    uint32_t serialized_value;
    memcpy(buffer + offset, &accelLatCog.ServiceID, sizeof(accelLatCog.ServiceID));
    offset += sizeof(accelLatCog.ServiceID);
    memcpy(buffer + offset, &accelLatCog.MethodID, sizeof(accelLatCog.MethodID));
    offset += sizeof(accelLatCog.MethodID);
    memcpy(buffer + offset, &accelLatCog.PayloadLength, sizeof(accelLatCog.PayloadLength));
    offset += sizeof(accelLatCog.PayloadLength);
    serialized_value = serializeFloat32(accelLatCog.AccelerationLateralErrAmp);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &accelLatCog.AccelerationLateralErrAmp_InvalidFlag, sizeof(accelLatCog.AccelerationLateralErrAmp_InvalidFlag));
    offset += sizeof(accelLatCog.AccelerationLateralErrAmp_InvalidFlag);
    memcpy(buffer + offset, &accelLatCog.QualifierAccelerationLateral, sizeof(accelLatCog.QualifierAccelerationLateral));
    offset += sizeof(accelLatCog.QualifierAccelerationLateral);
    serialized_value = serializeFloat32(accelLatCog.AccelerationLateral);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &accelLatCog.AccelerationLateral_InvalidFlag, sizeof(accelLatCog.AccelerationLateral_InvalidFlag));
    offset += sizeof(accelLatCog.AccelerationLateral_InvalidFlag);
    memcpy(buffer + offset, &accelLatCog.AccelerationLateralEventDataQualifier, sizeof(accelLatCog.AccelerationLateralEventDataQualifier));
    offset += sizeof(accelLatCog.AccelerationLateralEventDataQualifier);
    memcpy(buffer + offset, &accelLatCog.Reserved1, sizeof(accelLatCog.Reserved1));
    offset += sizeof(accelLatCog.Reserved1);
    memcpy(buffer + offset, &accelLatCog.Reserved2, sizeof(accelLatCog.Reserved2));
    offset += sizeof(accelLatCog.Reserved2);
    memcpy(buffer + offset, &accelLatCog.Reserved3, sizeof(accelLatCog.Reserved3));
    offset += sizeof(accelLatCog.Reserved3);
    return offset;
}

// --- Packs AccelerationLongitudinalCoG message into buffer ---
static size_t PackAccelerationLongitudinalCoG(char* buffer, size_t offset, float longitudinal_accel) {
    AccelerationLongitudinalCoG accelLongCog = {};
    accelLongCog.ServiceID = htons(0);
    accelLongCog.MethodID = htons(322);
    accelLongCog.PayloadLength = htonl(32);
    accelLongCog.AccelerationLongitudinal = longitudinal_accel;

    uint32_t serialized_value;
    memcpy(buffer + offset, &accelLongCog.ServiceID, sizeof(accelLongCog.ServiceID));
    offset += sizeof(accelLongCog.ServiceID);
    memcpy(buffer + offset, &accelLongCog.MethodID, sizeof(accelLongCog.MethodID));
    offset += sizeof(accelLongCog.MethodID);
    memcpy(buffer + offset, &accelLongCog.PayloadLength, sizeof(accelLongCog.PayloadLength));
    offset += sizeof(accelLongCog.PayloadLength);
    serialized_value = serializeFloat32(accelLongCog.AccelerationLongitudinalErrAmp);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &accelLongCog.AccelerationLongitudinalErrAmp_InvalidFlags, sizeof(accelLongCog.AccelerationLongitudinalErrAmp_InvalidFlags));
    offset += sizeof(accelLongCog.AccelerationLongitudinalErrAmp_InvalidFlags);
    memcpy(buffer + offset, &accelLongCog.QualifierAccelerationLongitudinal, sizeof(accelLongCog.QualifierAccelerationLongitudinal));
    offset += sizeof(accelLongCog.QualifierAccelerationLongitudinal);
    serialized_value = serializeFloat32(accelLongCog.AccelerationLongitudinal);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &accelLongCog.AccelerationLongitudinal_InvalidFlag, sizeof(accelLongCog.AccelerationLongitudinal_InvalidFlag));
    offset += sizeof(accelLongCog.AccelerationLongitudinal_InvalidFlag);
    memcpy(buffer + offset, &accelLongCog.AccelerationLongitudinalEventDataQualifier, sizeof(accelLongCog.AccelerationLongitudinalEventDataQualifier));
    offset += sizeof(accelLongCog.AccelerationLongitudinalEventDataQualifier);
    memcpy(buffer + offset, &accelLongCog.Reserved1, sizeof(accelLongCog.Reserved1));
    offset += sizeof(accelLongCog.Reserved1);
    memcpy(buffer + offset, &accelLongCog.Reserved2, sizeof(accelLongCog.Reserved2));
    offset += sizeof(accelLongCog.Reserved2);
    memcpy(buffer + offset, &accelLongCog.Reserved3, sizeof(accelLongCog.Reserved3));
    offset += sizeof(accelLongCog.Reserved3);
    return offset;
}

// --- Packs DrivingDirection message into buffer ---
static size_t PackDrivingDirection(char* buffer, size_t offset, MotionState latest_direction) {
    DrivingDirection drivingDir = {};
    drivingDir.ServiceID = htons(0);
    drivingDir.MethodID = htons(325);
    drivingDir.PayloadLength = htonl(22);
    drivingDir.DrivingDirectionConfirmed = static_cast<uint8_t>(latest_direction);

    memcpy(buffer + offset, &drivingDir.ServiceID, sizeof(drivingDir.ServiceID));
    offset += sizeof(drivingDir.ServiceID);
    memcpy(buffer + offset, &drivingDir.MethodID, sizeof(drivingDir.MethodID));
    offset += sizeof(drivingDir.MethodID);
    memcpy(buffer + offset, &drivingDir.PayloadLength, sizeof(drivingDir.PayloadLength));
    offset += sizeof(drivingDir.PayloadLength);
    memcpy(buffer + offset, &drivingDir.DrivingDirectionUnconfirmed, sizeof(drivingDir.DrivingDirectionUnconfirmed));
    offset += sizeof(drivingDir.DrivingDirectionUnconfirmed);
    memcpy(buffer + offset, &drivingDir.DrivingDirectionConfirmed, sizeof(drivingDir.DrivingDirectionConfirmed));
    offset += sizeof(drivingDir.DrivingDirectionConfirmed);
    memcpy(buffer + offset, &drivingDir.Reserved1, sizeof(drivingDir.Reserved1));
    offset += sizeof(drivingDir.Reserved1);
    memcpy(buffer + offset, &drivingDir.Reserved2, sizeof(drivingDir.Reserved2));
    offset += sizeof(drivingDir.Reserved2);
    memcpy(buffer + offset, &drivingDir.Reserved3, sizeof(drivingDir.Reserved3));
    offset += sizeof(drivingDir.Reserved3);
    return offset;
}

// --- Packs SteeringAngleFrontAxle message into buffer ---
static size_t PackSteeringAngleFrontAxle(char* buffer, size_t offset, float front_wheel_angle_deg) {
    SteeringAngleFrontAxle steeringAngle = {};
    steeringAngle.ServiceID = htons(0);
    steeringAngle.MethodID = htons(327);
    steeringAngle.PayloadLength = htonl(32);
    steeringAngle.SteeringAngleFrontAxle = front_wheel_angle_deg;

    uint32_t serialized_value;
    memcpy(buffer + offset, &steeringAngle.ServiceID, sizeof(steeringAngle.ServiceID));
    offset += sizeof(steeringAngle.ServiceID);
    memcpy(buffer + offset, &steeringAngle.MethodID, sizeof(steeringAngle.MethodID));
    offset += sizeof(steeringAngle.MethodID);
    memcpy(buffer + offset, &steeringAngle.PayloadLength, sizeof(steeringAngle.PayloadLength));
    offset += sizeof(steeringAngle.PayloadLength);
    memcpy(buffer + offset, &steeringAngle.QualifierSteeringAngleFrontAxle, sizeof(steeringAngle.QualifierSteeringAngleFrontAxle));
    offset += sizeof(steeringAngle.QualifierSteeringAngleFrontAxle);
    serialized_value = serializeFloat32(steeringAngle.SteeringAngleFrontAxleErrAmp);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &steeringAngle.SteeringAngleFrontAxleErrAmp_InvalidFlag, sizeof(steeringAngle.SteeringAngleFrontAxleErrAmp_InvalidFlag));
    offset += sizeof(steeringAngle.SteeringAngleFrontAxleErrAmp_InvalidFlag);
    serialized_value = serializeFloat32(steeringAngle.SteeringAngleFrontAxle);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &steeringAngle.SteeringAngleFrontAxle_InvalidFlag, sizeof(steeringAngle.SteeringAngleFrontAxle_InvalidFlag));
    offset += sizeof(steeringAngle.SteeringAngleFrontAxle_InvalidFlag);
    memcpy(buffer + offset, &steeringAngle.SteeringAngleFrontAxleEventDataQualifier, sizeof(steeringAngle.SteeringAngleFrontAxleEventDataQualifier));
    offset += sizeof(steeringAngle.SteeringAngleFrontAxleEventDataQualifier);
    memcpy(buffer + offset, &steeringAngle.Reserved1, sizeof(steeringAngle.Reserved1));
    offset += sizeof(steeringAngle.Reserved1);
    memcpy(buffer + offset, &steeringAngle.Reserved2, sizeof(steeringAngle.Reserved2));
    offset += sizeof(steeringAngle.Reserved2);
    memcpy(buffer + offset, &steeringAngle.Reserved3, sizeof(steeringAngle.Reserved3));
    offset += sizeof(steeringAngle.Reserved3);
    return offset;
}

// --- Packs VelocityVehicle message into buffer ---
static size_t PackVelocityVehicle(char* buffer, size_t offset, float speed) {
    VelocityVehicle velocity = {};
    velocity.ServiceID = htons(0);
    velocity.MethodID = htons(323);
    velocity.PayloadLength = htonl(28);
    velocity.VelocityVehicle = speed;

    uint32_t serialized_value;
    memcpy(buffer + offset, &velocity.ServiceID, sizeof(velocity.ServiceID));
    offset += sizeof(velocity.ServiceID);
    memcpy(buffer + offset, &velocity.MethodID, sizeof(velocity.MethodID));
    offset += sizeof(velocity.MethodID);
    memcpy(buffer + offset, &velocity.PayloadLength, sizeof(velocity.PayloadLength));
    offset += sizeof(velocity.PayloadLength);
    memcpy(buffer + offset, &velocity.StatusVelocityNearStandstill, sizeof(velocity.StatusVelocityNearStandstill));
    offset += sizeof(velocity.StatusVelocityNearStandstill);
    memcpy(buffer + offset, &velocity.QualifierVelocityVehicle, sizeof(velocity.QualifierVelocityVehicle));
    offset += sizeof(velocity.QualifierVelocityVehicle);
    memcpy(buffer + offset, &velocity.VelocityVehicleEventDataQualifier, sizeof(velocity.VelocityVehicleEventDataQualifier));
    offset += sizeof(velocity.VelocityVehicleEventDataQualifier);
    serialized_value = serializeFloat32(velocity.VelocityVehicle);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &velocity.VelocityVehicle_InvalidFlag, sizeof(velocity.VelocityVehicle_InvalidFlag));
    offset += sizeof(velocity.VelocityVehicle_InvalidFlag);
    memcpy(buffer + offset, &velocity.Reserved1, sizeof(velocity.Reserved1));
    offset += sizeof(velocity.Reserved1);
    memcpy(buffer + offset, &velocity.Reserved2, sizeof(velocity.Reserved2));
    offset += sizeof(velocity.Reserved2);
    memcpy(buffer + offset, &velocity.Reserved3, sizeof(velocity.Reserved3));
    offset += sizeof(velocity.Reserved3);
    return offset;
}

// --- Packs Yaw_Rate message into buffer ---
static size_t PackYawRate(char* buffer, size_t offset, float yaw_rate) {
    Yaw_Rate yawRate = {};
    yawRate.ServiceID = htons(0);
    yawRate.MethodID = htons(326);
    yawRate.PayloadLength = htonl(32);
    yawRate.YawRate = yaw_rate;

    uint32_t serialized_value;
    memcpy(buffer + offset, &yawRate.ServiceID, sizeof(yawRate.ServiceID));
    offset += sizeof(yawRate.ServiceID);
    memcpy(buffer + offset, &yawRate.MethodID, sizeof(yawRate.MethodID));
    offset += sizeof(yawRate.MethodID);
    memcpy(buffer + offset, &yawRate.PayloadLength, sizeof(yawRate.PayloadLength));
    offset += sizeof(yawRate.PayloadLength);
    serialized_value = serializeFloat32(yawRate.YawRateErrAmp);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &yawRate.YawRateErrAmp_InvalidFlag, sizeof(yawRate.YawRateErrAmp_InvalidFlag));
    offset += sizeof(yawRate.YawRateErrAmp_InvalidFlag);
    memcpy(buffer + offset, &yawRate.QualifierYawRate, sizeof(yawRate.QualifierYawRate));
    offset += sizeof(yawRate.QualifierYawRate);
    serialized_value = serializeFloat32(yawRate.YawRate);
    memcpy(buffer + offset, &serialized_value, sizeof(serialized_value));
    offset += sizeof(serialized_value);
    memcpy(buffer + offset, &yawRate.YawRate_InvalidFlag, sizeof(yawRate.YawRate_InvalidFlag));
    offset += sizeof(yawRate.YawRate_InvalidFlag);
    memcpy(buffer + offset, &yawRate.YawRateEventDataQualifier, sizeof(yawRate.YawRateEventDataQualifier));
    offset += sizeof(yawRate.YawRateEventDataQualifier);
    memcpy(buffer + offset, &yawRate.Reserved1, sizeof(yawRate.Reserved1));
    offset += sizeof(yawRate.Reserved1);
    memcpy(buffer + offset, &yawRate.Reserved2, sizeof(yawRate.Reserved2));
    offset += sizeof(yawRate.Reserved2);
    memcpy(buffer + offset, &yawRate.Reserved3, sizeof(yawRate.Reserved3));
    offset += sizeof(yawRate.Reserved3);
    return offset;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ars548_dynamics_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(20); // 20 Hz
    ROS_INFO_STREAM("Started node: " << ros::this_node::getName());

    // --- IMU Setup ---
    ThreadSafeData<sensor_msgs::Imu> imu_data;
    float yaw_rate = 0.0;
    float longitudinal_accel = 0.0;
    float lateral_accel = 0.0;
    // Lambda callback for IMU
    auto imu_callback = [&](const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_data.mtx);
        imu_data.msg = *msg;
        imu_data.received = true;
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
    float speed = 0.0;
    // Lambda callback for Speed
    auto speed_callback = [&](const std_msgs::Float64::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(speed_data.mtx);
        speed_data.msg = *msg;
        speed_data.received = true;
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
    float front_wheel_angle_deg = 0.0;
    // Lambda callback for Steering
    auto steering_callback = [&](const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(steering_data.mtx);
        steering_data.msg = *msg;
        steering_data.received = true;
    };
    std::string steering_topic;
    float steering_gear_ratio = 14.81f;
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
    MotionState latest_direction = MotionState::Standstill;
    // Lambda callback for Gear
    auto gear_callback = [&](const dbw_mkz_msgs::GearReport::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(gear_data.mtx);
        gear_data.msg = *msg;
        gear_data.received = true;
    };
    std::string gear_topic;
    nh.param<std::string>("gear_topic", gear_topic, "");
    nh.param<float>("steering_gear_ratio", steering_gear_ratio, 14.81f);
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

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        std::string stamp_str = std::to_string(now.sec) + "." + std::to_string(now.nsec);

        // Access the latest IMU data safely
        {
            std::lock_guard<std::mutex> lock(imu_data.mtx);
            if (imu_data.received) {
                yaw_rate = imu_data.msg.angular_velocity.z * 180.0 / M_PI; // Convert to deg/s
                longitudinal_accel = imu_data.msg.linear_acceleration.x; // m/s^2
                lateral_accel = imu_data.msg.linear_acceleration.y; // m/s^2
                ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [" << stamp_str << "] [Loop] Latest yaw rate (deg/s): " << yaw_rate
                                << ", longitudinal accel (m/s^2): " << longitudinal_accel
                                << ", lateral accel (m/s^2): " << lateral_accel);
            }
        }

        // Access the latest speed data safely
        {
            std::lock_guard<std::mutex> lock(speed_data.mtx);
            if (speed_data.received) {
                speed = speed_data.msg.data * 3.6; // Convert to km/h
                ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [" << stamp_str << "] [Loop] Latest speed: " << speed << " km/h");
            }
        }

        // Access the latest steering data safely
        {
            std::lock_guard<std::mutex> lock(steering_data.mtx);
            if (steering_data.received) {
                // Convert steering wheel angle rad to steering front axle degrees using parameterized gear ratio
                front_wheel_angle_deg = steering_data.msg.steering_wheel_angle * (180.0 / M_PI) / steering_gear_ratio;
                ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [" << stamp_str << "] [Loop] Latest front wheel angle: " << front_wheel_angle_deg << " degrees (gear ratio: " << steering_gear_ratio << ")");
            }
        }

        // Access the latest gear data safely
        {
            std::lock_guard<std::mutex> lock(gear_data.mtx);
            if (gear_data.received) {
                latest_gear = gear_data.msg.state.gear;
                std::string gear_str = gearToString(latest_gear);
                // Map gear to motion state
                switch (latest_gear) {
                    case dbw_mkz_msgs::Gear::DRIVE:
                    case dbw_mkz_msgs::Gear::LOW:
                        latest_direction = MotionState::Forward;
                        break;
                    case dbw_mkz_msgs::Gear::REVERSE:
                        latest_direction = MotionState::Backward;
                        break;
                    case dbw_mkz_msgs::Gear::PARK:
                    case dbw_mkz_msgs::Gear::NEUTRAL:
                    case dbw_mkz_msgs::Gear::NONE:
                    default:
                        latest_direction = MotionState::Standstill;
                        break;
                }
                ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [" << stamp_str << "] [Loop] Latest gear state: " << gear_str << " (" << static_cast<int>(latest_gear) << ") | Motion state: " << motionStateToString(latest_direction) << " (" << static_cast<int>(latest_direction) << ")");
            }
        }

        // --- Pack messages into buffer ---
        char buffer[1024];
        size_t offset = 0;
        offset = PackAccelerationLateralCoG(buffer, offset, lateral_accel);
        offset = PackAccelerationLongitudinalCoG(buffer, offset, longitudinal_accel);
        offset = PackDrivingDirection(buffer, offset, latest_direction);
        offset = PackSteeringAngleFrontAxle(buffer, offset, front_wheel_angle_deg);
        offset = PackVelocityVehicle(buffer, offset, speed);
        offset = PackYawRate(buffer, offset, yaw_rate);

        // --- Send UDP datagram ---
        ssize_t bytes_sent = sendto(udp_sock, buffer, offset, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (bytes_sent < 0) {
            perror("UDP sendto failed");
        } else {
            ROS_DEBUG_STREAM("Sent " << bytes_sent << " bytes to sensor at " << inet_ntoa(dest_addr.sin_addr) << ":" << ntohs(dest_addr.sin_port));
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(udp_sock);

    return 0;
}