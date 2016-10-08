//
//  TargetModel.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/25/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "TargetModel.hpp"
#include "MissileModel.hpp"
#include "CoordTransforms.hpp"
#include "Constants.hpp"
#include "SimState.hpp"

/*
 // Target Missile will go after
 MissileModel * target;
 
 // Rigid Body Dynamics
 EquationsOfMotion eom;
 TargetMassProps massprops;
 */

    
TargetModel::TargetModel() {
    model_name = "target";
    eom.setEOM_State(state);
    eom.useGravity(false);
}
    

void TargetModel::initialize() {
    // init rigid body
    massprops.initialize();
    eom.setMass(massprops.mass);
    eom.setInertia(massprops.I, massprops.Iinv);
    
    // init pos, velocity, attitude
    speed_const = generator->rand()*60 + 5;
    LatLongAlt start(0,0,0);
    eom.setInitialLatLong(start);
    vec3 startECEF = transforms::convertLLAtoECEF(start);
    vec3 velENU(0, speed_const * 0.44704, 0);
    vec3 velECEF = transforms::ENUtoECEF_Matrix(start)*velENU;
    quat q0(generator->rand()*2.0*3.1415, vec3(0.0, 0, 1.0) );
    
    // init rigid body
    massprops.initialize();
    eom.setMass(massprops.mass);
    eom.setInertia(massprops.I, massprops.Iinv);
    
    // initial state condition
    state[0] = startECEF[0];
    state[1] = startECEF[1];
    state[2] = startECEF[2];
    state[3] = velECEF[0];
    state[4] = velECEF[1];
    state[5] = velECEF[2];
    state[6] = q0[0];
    state[7] = q0[1];
    state[8] = q0[2];
    state[9] = q0[3];
    state[10] = 0;
    state[11] = 0;
    state[12] = 0;

}
    

void TargetModel::setupPrintData() {
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().latitude, "TargLatitude" );
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().longitude, "TargLongitude" );
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().altitude, "TargAltitude" );
}
    

int TargetModel::numDims() const {
    return eom.numDims();
}
    

void TargetModel::update() {
    vec3 velENU(0, speed_const * 0.44704, 0);
    vec3 velECEF = transforms::ENUtoECEF_Matrix(eom.getCurrentCoord())*velENU;
    state[3] = velECEF[0];
    state[4] = velECEF[1];
    state[5] = velECEF[2];
    eom.updateComponents();
    
}

void TargetModel::operator()( double time , ModelState & dqdt ) {
    eom(time,dqdt);
    massprops.update_(time);
}

void TargetModel::setMissile( const MissileModel & missile_ ) {
    missile = &missile_;
}
double TargetModel::getAltitude() const {
    return eom.getCurrentCoord().getAltitudeInMeters();
}
double TargetModel::getMass() const {
    return massprops.mass;
}
const LatLongAlt & TargetModel::getCurrentCoord() const {
    return eom.getCurrentCoord();
}
const vec3 & TargetModel::getPos() const {
    return eom.getPos();
}
const vec3 & TargetModel::getVel() const {
    return eom.getVel();
}
