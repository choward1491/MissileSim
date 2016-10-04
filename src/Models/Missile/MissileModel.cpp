//
//  MissileModel.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "MissileModel.hpp"
#include "MissileSim.hpp"
#include "CoordTransforms.hpp"
#include "Constants.hpp"

MissileModel::MissileModel(){
    model_name = "missile";
    eom.setEOM_State(state);
    eom.addForceContributor(motor);
    
    /*imu.setAccelerationSource(eom.getAccel());
    imu.setAngularVelocitySource(eom.getAngularVel());
    gps.setLatLongSource(eom.getCurrentCoord());
    
    a_commands.setMissile(*this);
    eom.addForceContributor(a_commands);
    eom.addMomentContributor(a_commands);*/
}


void MissileModel::initialize(){
    
    // init pos, velocity, attitude
    LatLongAlt start(0,0,10000);
    eom.setInitialLatLong(start);
    vec3 startECEF = transforms::convertLLAtoECEF(start);
    vec3 velENU(0, 0, 0);
    vec3 velECEF = transforms::ENUtoECEF_Matrix(start)*velENU;
    quat q0(-Constants::pi/6.0, vec3(0.0, 0, 1.0) );
    
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
    state[12] = 0.174532925199433;
    
    // initialize auxilary variables based
    // on initialized states
    update();
}

void MissileModel::addSubModels( MissileSim & msim ){
    //msim.addDiscrete(&imu, 600);
    //msim.addDiscrete(&gps, 1);
}


void MissileModel::update(){
    
    eom.updateComponents();
    /*EulerAngles angles = eom.getAttitude().getEulerAngles();
    roll  = angles[0]* Constants::rad2deg;
    pitch = angles[1]* Constants::rad2deg;
    yaw   = angles[2]* Constants::rad2deg;
    P     = state[10]* Constants::rad2deg;
    Q     = state[11]* Constants::rad2deg;
    R     = state[12]* Constants::rad2deg;*/
}

void MissileModel::setupPrintData(){
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().latitude, "Latitude" );
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().longitude, "Longitude" );
    simState->dataPrinter.addVariableToPrint(&eom.getCurrentCoord().altitude, "Altitude" );
    simState->dataPrinter.addVariableToPrint(&roll, "Roll" );
    simState->dataPrinter.addVariableToPrint(&pitch, "Pitch" );
    simState->dataPrinter.addVariableToPrint(&yaw, "Yaw" );
    simState->dataPrinter.addVariableToPrint(&P, "P" );
    simState->dataPrinter.addVariableToPrint(&Q, "Q" );
    simState->dataPrinter.addVariableToPrint(&R, "R" );
}

double MissileModel::getAltitude() const{
    return eom.getCurrentCoord().altitude;
}

double MissileModel::getMass() const {
    return massprops.mass;
}


int MissileModel::numDims() const{
    return eom.numDims();
}


void MissileModel::operator()( double time , ModelState & dqdt ){
    eom(time,dqdt);
    massprops.update_(time);
}


