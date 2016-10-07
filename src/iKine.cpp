/** 
 *  Copyright 2016
 *  @file    inverseKine.cpp
 *  @author  Dzenan Lapandic
 *  @date    3/18/2016  
 *  @version 1.0 
 *  
 *  @brief Inverse kinematics for legs of hexapod robot
 *
 *  @section DESCRIPTION
 *  
 *
 */

#include <iKine>

#include <ros/ros.h>

#include <vector>

#include "Vector3d.h"


double coxa = 4.7;
double femur = 8.3;
double tibia = 12.6;

Vector3d COM{0, 0, 0};
std::vector<double> leg_dimensions{coxa, femur, tibia};
std::vector<Vector3d> COM_to_leg;

double chasis_radius = 5;

Vector3d polarToCartesian(double r, double alpha) {
    Vector3d v{r*cos(alpha), r*sin(alpha), 0};
    return v;
}
/*
for(int i=30; i <=360; i+=60)
    COM_to_leg.push_back(polarToCartesian(chasis_radius,i));

std::vector<Vector3d> legs_positions;
std::vector<Vector3d> legs_angles;
std::vector<Vector3d> motor_pulse;

std::vector<Vector3d> inverseKineToMotorP(const std::vector<Vector3d> &positions){
    std::vector<Vector3d> vector;
    for(int i=0; i<6; i++){
        Vector3d v = positions[i] - COM - COM_to_leg[i];
        vector.push_back(anglesToMotorPulse(inverseKine(v)));
    }
    return vector;
}
*/
geometry_msgs::Vector3 InverseKine(const geometry_msgs::Vector3 &leg_pos) {
    double L1 = sqrt(leg_pos.x*leg_pos.x + leg_pos.y*leg_pos.y);
    double z_offset = -leg_pos.z;
    double L = sqrt(z_offset * z_offset + (L1 - coxa)^2);
    double gama = atan(leg_pos.x, leg_pos.y);
    double beta = acos((L^2-tibia^2-femur^2)/(-2*tibia*femur));
    double alpha = acos(z_offset/L) + acos((tibia^2-femur^2-L^2)/(2*femur*L));

    if (std::isnan(alpha) || std::isnan(beta) || std::isnan(gama))
        throw "Error";

    geometry_msgs::Vector3 angles;
    angles.x = alpha;
    angles.y = beta;
    angles.z = gama;
    return angles;
}

geometry_msgs::Vector3 anglesToMotorPulse(const geometry_msgs::Vector3 &angles) {
    geometry_msgs::Vector3 values;
    values.x = angles.x*(2000/pi)+500;
    values.y = angles.y*(2000/pi)+500;
    values.z = angles.z*(2000/pi)+500;
    return values;
}





