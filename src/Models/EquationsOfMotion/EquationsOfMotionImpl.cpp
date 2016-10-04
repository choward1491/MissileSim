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
#include "math3d_define.hpp"
#include "Earth.hpp"
#include "Gravity84.hpp"
#include "CoordTransforms.hpp"
#include "ForceContributor.hpp"
#include "MomentContributor.hpp"


#define HEADER
#define EOM EquationsOfMotion


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

const vec3 & EOM::getPos() const {
    return pos;
}
const vec3 & EOM::getVel() const {
    return vel;
}
const vec3 & EOM::getAccel() const {
    return accel;
}
const quat & EOM::getAttitude() const {
    return q;
}
const vec3 & EOM::getAngularVel() const {
    return omega;
}
const LatLongAlt & EOM::getInitialCoord() const {
    return initial_pos;
}
const LatLongAlt & EOM::getCurrentCoord() const {
    return current_pos;
}

HEADER
void EOM::updateComponents(){
    ModelState & s = *state;
    pos[0] = s[0];
    pos[1] = s[1];
    pos[2] = s[2];
    vel[0] = s[3];
    vel[1] = s[4];
    vel[2] = s[5];
    q[0] = s[6];
    q[1] = s[7];
    q[2] = s[8];
    q[3] = s[9];
    q.normalize();
    omega[0] = s[10];
    omega[1] = s[11];
    omega[2] = s[12];
    current_pos = transforms::convertECEFtoLLA(pos);
}

HEADER
void EOM::operator()(double t, ModelState & dudt ){
    
    vec3 totAccelBody, totAccelENU, totAccelECEF, totMomentBody;
    
    // get total external moments and forces
    getTotalForceAndMoments( t, totAccelBody, totMomentBody );
    
    // velocity derivative
    double g = Earth::gravity::obtainGravityWithCoordinate(current_pos);
    vec3 gravity(0, 0, g);              // in NED frame
    double m = (*mass);
    totAccelBody[0] /= m; totAccelBody[1] /= m; totAccelBody[2] /= m;
    
    
    // add gravity and get ENU to ECEF rotation matrix
    totAccelENU   = transforms::NEDtoENU( q.rotate(totAccelBody) + gravity );
    mat3 enu2ecef = transforms::ENUtoECEF_Matrix(current_pos);
    
    // compute net acceleration in ECEF, including coriolis and centripital forces
    vec3 rotVel = vel + Earth::omega.cross(pos);
    totAccelECEF= enu2ecef*totAccelENU                          // external accel sum
               + Earth::omega.cross(vel)*2.0                    // coriolis
               + Earth::omega.cross(Earth::omega.cross(rotVel));// centripital
    
    accel = totAccelECEF; // used for sensors/external models

    // set position derivative
    dudt[0] = vel[0];
    dudt[1] = vel[1];
    dudt[2] = vel[2];
    
    // set derivative of velocity
    dudt[3] = accel[0];
    dudt[4] = accel[1];
    dudt[5] = accel[2];
    
    // set attitude derivative
    quat dqdt = q.getDerivative(omega);
    dudt[6] = dqdt[0];
    dudt[7] = dqdt[1];
    dudt[8] = dqdt[2];
    dudt[9] = dqdt[3];
    
    // set angular velocity derivative
    vec3 dwdt = (*Iinv)*(totMomentBody - omega.cross((*I)*omega));
    dudt[10] = dwdt[0];
    dudt[11] = dwdt[1];
    dudt[12] = dwdt[2];
    
    
}

HEADER
void EOM::getTotalForceAndMoments(  double time, vec3 & totalForceBody, vec3 & totalMomentBody ){
    vec3 force, moment, r, tmp;
    for(int i = 0; i < forces.size(); ++i){
        forces[i]->getForce( time, force );
        totalForceBody[0] += force[0];
        totalForceBody[1] += force[1];
        totalForceBody[2] += force[2];
        
        forces[i]->getLocation(time, r);
        
        tmp = r.cross(force);
        totalMomentBody[0] += tmp[0];
        totalMomentBody[1] += tmp[1];
        totalMomentBody[2] += tmp[2];
    }
    
    for(int i = 0; i < moments.size(); ++i ){
        moments[i]->getMoment(time, moment);
        totalMomentBody[0] += moment[0];
        totalMomentBody[1] += moment[1];
        totalMomentBody[2] += moment[2];
    }
}

HEADER
int EOM::numDims() const {
    return numDims_;
}

#undef HEADER
#undef EOM

#endif
