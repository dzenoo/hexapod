/** 
 *  Copyright 2016
 *  @file    inverseKine.h
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

#ifndef HEXAPOD_IKINE_H
#define HEXAPOD_IKINE_H

#include "Vector3d.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>

#include <vector>

double coxa = 4.7;
double femur = 8.3;
double tibia = 12.6;

std::vector<double> leg_dimensions{coxa, femur, tibia};
std::vector<Vector3d> COM_to_leg;

double chasis_radius;

Vector3d polarToCartesian(double r, double alpha);

geometry_msgs::Vector3 inverseKine(const geometry_msgs::Vector3 &leg_position);

geometry_msgs::Vector3 anglesToMotorPulse(const geometry_msgs::Vector3 &angles);
#endif  //  HEXAPOD_IKINE_H
