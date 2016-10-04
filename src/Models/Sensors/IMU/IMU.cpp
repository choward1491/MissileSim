//
//  IMU.cpp
//  MissileSim
//
//  Created by Christian J Howard on 5/2/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "IMU.hpp"


IMU::IMU(){
    trueAccel = 0;
    trueAngVel = 0;
}
IMU::~IMU(){
    
}
void IMU::initialize(){
    
}
void IMU::setupPrintData(){
    
}
void IMU::update(){
    imu_accel = *trueAccel;
    imu_angvel = *trueAccel;
}
void IMU::setAccelerationSource( const vec3 & accel ){
    trueAccel = &accel;
}
void IMU::setAngularVelocitySource( const vec3 & angVel ){
    trueAngVel = &angVel;
}

vec3 IMU::getAccel() const{
    return imu_accel;
}
vec3 IMU::getAngVel() const{
    return imu_angvel;
}
