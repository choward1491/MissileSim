//
//  EOM.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//


#ifndef _EOM_IMPL_HPP_
#define _EOM_IMPL_HPP_

#include "EquationsOfMotion.hpp"
#include "ModelState.hpp"
#include "vec3.hpp"
#include "Earth.hpp"
#include "Gravity84.hpp"
#include "CoordTransforms.hpp"

/*
 int numDims_;
 Quaternion q;
 vec3 omega;
 vec3 pos;
 vec3 vel;
 */
#define HEADER template<class Type>
#define EOM EquationsOfMotion<Type>

namespace ops = vec3_ops;

HEADER
void EOM::addForceContributor( ForceContributor & force ){
    forces.push_back(&force);
}

HEADER
void EOM::addMomentContributor( MomentContributor & moment ){
    moments.push_back(&moment);
}

HEADER
EOM::EquationsOfMotion():initial_pos(){
    numDims_ = 4 + 3*3;
}

HEADER
void EOM::setInertia( mat3 & I_, mat3 & Iinv_ ){
    I = &I_;
    Iinv = &Iinv_;
}

HEADER
void EOM::setMass( double & mass_ ){
    mass = &mass_;
}

HEADER
void EOM::setInitialLatLong( const LatLongAlt & init_coord ){
    initial_pos = init_coord;
    current_pos = initial_pos;
}

HEADER
void EOM::setEOM_State( ModelState & eom_state ){
    state = &eom_state;
}

HEADER
void EOM::updateComponents(){
    ModelState & s = *state;
    pos = ops::equal(s[0], s[1], s[2]);
    current_pos = transforms::convertECEFtoLLA(pos);
    vel = ops::equal(s[3], s[4], s[5]);
    q[0] = s[6];
    q[1] = s[7];
    q[2] = s[8];
    q[3] = s[9];
    q.normalize();
    omega = ops::equal(s[10], s[11], s[12]);
}

HEADER
void EOM::operator()(double t, ModelState & dudt ){
    
    // position derivative
    dudt[0] = vel[0];
    dudt[1] = vel[1];
    dudt[2] = vel[2];
    
    // velocity derivative
    double g = Earth::gravity::obtainGravityWithCoordinate(current_pos);
    vec3 totAccel = 0, totAccelENU;
    vec3 gravity = ops::equal(0, 0, g); // in NED frame
    getTotalForce( t, totAccel ); // in body frame
    totAccel = totAccel/(*mass);
    
    
    // add gravity and rotate to ECEF
    totAccelENU   = transforms::NEDtoENU( q.rotate(totAccel) + gravity );
    mat3 enu2ecef = transforms::ENUtoECEF_Matrix(current_pos);
    
    // compute net acceleration
    vec3 rotVel = vel + ops::cross(Earth::omega, pos);
    totAccel   = enu2ecef*totAccelENU // external accel sum
               + 2.0 * ops::cross(Earth::omega, vel) // coriolis
               + ops::cross(Earth::omega, ops::cross(Earth::omega, rotVel)); // centripital
    accel = totAccel; // used for sensors/external models

    dudt[3] = totAccel[0];
    dudt[4] = totAccel[1];
    dudt[5] = totAccel[2];
    
    // attitude derivative
    Quaternion dqdt = q.getDerivative(omega);
    dudt[6] = dqdt[0];
    dudt[7] = dqdt[1];
    dudt[8] = dqdt[2];
    dudt[9] = dqdt[3];
    
    // omega derivative
    vec3 totMoment = 0;
    getTotalMoment(t, totMoment); // in body frame
    vec3 dwdt = (*Iinv)*(totMoment - ops::cross(omega, (*I)*omega));
    dudt[10] = dwdt[0];
    dudt[11] = dwdt[1];
    dudt[12] = dwdt[2];
    
}

HEADER
void EOM::getTotalForce( double time, vec3 & total ){
    static_cast<Type*>(this)->getExternalForceSum( time, total );
}

HEADER
void EOM::getTotalMoment( double time, vec3 & total ){
    static_cast<Type*>(this)->getExternalMomentSum( time, total );
}

HEADER
int EOM::numDims() const {
    return numDims_;
}

#undef HEADER
#undef EOM

#endif