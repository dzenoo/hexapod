/** 
 *  Copyright 2016
 *  @file    main_node.cpp
 *  @author  Dzenan Lapandic
 *  @date    3/18/2016  
 *  @version 1.0 
 *  
 *  @brief Control one leg of a hexapod robot over serial communication
 *
 *  @section DESCRIPTION
 *  
 *
 */

 /*
Serial.print('/r');
Serial.print('/n');

 */
#include <iKine.h>

#include <ros/ros.h>


#include <math>

#include <custom_msgs/Imu>
#include <std_msgs/String>
#include <std_msgs/Empty>
#include <Vector3d>

#include <iostream>


//=========================================================================
//                     Pins of servo motor
//=========================================================================

//                Kukovi:
#define pDKuk 18  // Prednji Desni Kuk
#define pLKuk 0   // Prednji Lijevi Kuk
#define sDKuk 26  // Srednji Desni Kuk
#define sLKuk 5   // Srednji Lijevi Kuk
#define zDKuk 29  // Zadnji Desni Kuk
#define zLKuk 13  // Zadnji Lijevi Kuk
int servoJoints1[6]={pDKuk, sLKuk, zDKuk, pLKuk, sDKuk, zLKuk};

//                 Koljena:
#define pDKolj 17  // Prednje Desno Koljeno
#define pLKolj 1   // Prednje Lijevo Koljeno
#define sDKolj 27  // Srednje Desno Koljeno
#define sLKolj 11  // Srednje Lijevo Koljeno
#define zDKolj 30  // Zadnje Desno Koljeno
#define zLKolj 14  // Zadnje Lijevo Koljeno
int servoJoints2[6]={pDKolj, sLKolj, zDKolj, pLKolj, sDKolj, zLKolj};

//                 Clanci:
#define pDCl 16   // Prednji Desni Clanak
#define pLCl 2    // Prednji Lijevi Clanak
#define sDCl 20   // Srednji Desni Clanak
#define sLCl 4    // Srednji Lijevi Clanak
#define zDCl 31   // Zadnji Desni Clanak
#define zLCl 15   // Zadnji Lijevi Clanak
int servoJoints3[6]={pDCl, sLCl, zDCl, pLCl, sDCl, zLCl};

int pins[3][6]{
    {pDKuk, sLKuk, zDKuk, pLKuk, sDKuk, zLKuk},
    {pDKolj, sLKolj, zDKolj, pLKolj, sDKolj, zLKolj},
    {pDCl, sLCl, zDCl, pLCl, sDCl, zLCl}
};

void write_callback(const sensor_msgs::Imu& msg) {
    ROS_INFO_STREAM("Received data from IMU");
}

int main(int argc, char const *argv[]) {
    ros::init(argc, argv, "serial_ros_xbee");
    ros::NodeHandle nh;

    ros::Subscriber read_sub = nh.subscribe("xbee_to_ros", 1000, write_callback);
    ros::Publisher write_pub = nh.advertise<std_msgs::String>("ros_to_xbee", 1000);

    while (ros::ok()) {
        int i, x, y, z;
        std::cout << "Choose the leg to control: ";
        std::cin >> i;
        std::cout << "Position of the end-effector " << i << ": x y z" << std::endl;
        std::cin >> x >> y >> z;

        geometry_msgs::Vector3 leg_position;
        leg_position.x = x;
        leg_position.y = y;
        leg_position.z = z;

        geometry_msgs::Vector3 angles;

        geometry_msgs::Vector3 motor_pulse;
        try {
            angles = iKine::inverseKine(leg_position);
            motor_pulse = iKine::anglesToMotorPulse(angles);
        }
        catch (...) {
            std::cout << "Try another positon of the end-effector" << std::endl;
            continue;
        }

        std_msgs::String command;
        command.data = "#";
        command.data += std::to_string(servoJoints1[i]);
        command.data += " P";
        command.data += std::to_string(std::static_cast<int>(motor_pulse.x));

        command.data += " #";
        command.data += std::to_string(servoJoints2[i]);
        command.data += " P";
        command.data += std::to_string(std::static_cast<int>(motor_pulse.y));

        command.data += " #";
        command.data += std::to_string(servoJoints3[i]);
        command.data += " P";
        command.data += std::to_string(std::static_cast<int>(motor_pulse.z));

        command.data += "K";

        std::cout << "Command: " << command.data << std::endl;
        std::cout << "Angles: " << angles.x << "|" << angles.y << "|" << angles.z << std::endl;
        std::cout << "Pulse: " << motor_pulse.x << "|" << motor_pulse.y << "|" << motor_pulse.z << std::endl;

        write_pub.publish(command);

        ros::spinOnce();
    }

    return 0;
}


