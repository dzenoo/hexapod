/** 
 *  Copyright 2016
 *  @file    serial_node.cpp
 *  @date    3/18/2016  
 *  @version 1.0 
 *  
 *  @brief Serial communication beetwen this node and XBee/Arduino
 *
 *  @section DESCRIPTION
 *  
 *  Credits goes to Gary Servin
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
// #include "../msg/Imu.msg"
#include <string>
#include <vector>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

sensor_msgs::Imu convert_to_Imu_msg(const std::string &str) {
    // Message template:
    // XquaternionxYquaternionyZquaternionzWquaternionwAaccxBaccyCacczDgyroxEgyroyFgyrozHheadingLaltutiudeS
    sensor_msgs::Imu message;
    std::vector<std::size_t> found{str.find('X'), str.find('Y'), str.find('Z'),
                                   str.find('W'), str.find('A'), str.find('B'),
                                   str.find('C'), str.find('D'), str.find('E'),
                                   str.find('F'), str.find('H'), str.find('L'),
                                   str.find('S')};

    // quaternion
    message.orientation.x = std::stod(str.substr(found[0]+1, found[1]-found[0]-1));
    message.orientation.y = std::stod(str.substr(found[1]+1, found[2]-found[1]-1));
    message.orientation.z = std::stod(str.substr(found[2]+1, found[3]-found[2]-1));
    message.orientation.w = std::stod(str.substr(found[3]+1, found[4]-found[3]-1));

    // linear_acceleration
    message.linear_acceleration.x = std::stod(str.substr(found[4]+1, found[5]-found[4]-1));
    message.linear_acceleration.y = std::stod(str.substr(found[5]+1, found[6]-found[5]-1));
    message.linear_acceleration.z = std::stod(str.substr(found[6]+1, found[7]-found[6]-1));

    // angular_velocity
    message.angular_velocity.x = std::stod(str.substr(found[7]+1, found[8]-found[7]-1));
    message.angular_velocity.y = std::stod(str.substr(found[8]+1, found[9]-found[8]-1));
    message.angular_velocity.z = std::stod(str.substr(found[9]+1, found[10]-found[9]-1));
    /*
    nisam jos odlucio sta s ovim
    message.heading = std::stof(str.substr(foundH+1,foundA-foundH-1));
    message.altitude = std::stof(str.substr(foundA+1,foundE-foundA-1));*/
    return message;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_ros_xbee");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("ros_to_xbee", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<sensor_msgs::Imu>("xbee_to_ros", 1000);

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        ros::spinOnce();

        if (ser.available()) {
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            sensor_msgs::Imu msg = convert_to_Imu_msg(result.data);
            ROS_INFO_STREAM("Publishing: " << msg);
            read_pub.publish(msg);
        }
        loop_rate.sleep();
    }
}
