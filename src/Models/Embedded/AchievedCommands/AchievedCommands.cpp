//
//  AchievedCommands.cpp
//  MissileSim
//
//  Created by Christian J Howard on 9/30/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "AchievedCommands.hpp"
#include "MissileModel.hpp"
#include "ProNav.hpp"
#include "CoordTransforms.hpp"
#include "Gravity84.hpp"
#include <math.h>
#include "TargetModel.hpp"



AchievedCommands::AchievedCommands():missile(0) {
    
}
void AchievedCommands::setMissile( MissileModel & missile_ ) {
    missile = &missile_;
}
void AchievedCommands::getForce( double time, vec3 & outForceBody ) {
    LatLongAlt null;
    const TargetModel & targ = *missile->target;
    const LatLongAlt & start = missile->eom.getCurrentCoord();
    const quat & q = missile->eom.getAttitude();
    quat P;
    vec3 accelENU;
    pronav::R       = CoordTransforms::getRelativeENU(start, targ.eom.getCurrentCoord()) ;
    mat3 ecef2enu   = CoordTransforms::ECEFtoENU_Matrix(start);
    pronav::V       = ecef2enu * (targ.eom.getVel() - missile->eom.getVel());
    pronav::computeCommandedAccel(accelENU);
    vec3 g( 0, 0, -Earth::Gravity84::obtainGravityWithCoordinate(start) );
    accelENU = (accelENU + g)*missile->getMass();
    P.setVectorPart( CoordTransforms::ENUtoNED(accelENU) );
    outForceBody = (q.getInverse() * P * q).getVectorPart();
    const double G = 9.81;
    const double MAX_FORCE = 5.0*G*missile->getMass();
    outForceBody[0] = sign(outForceBody[0])*std::min(MAX_FORCE, fabs(outForceBody[0]));
    outForceBody[1] = sign(outForceBody[1])*std::min(MAX_FORCE, fabs(outForceBody[1]));
    outForceBody[2] = sign(outForceBody[2])*std::min(MAX_FORCE, fabs(outForceBody[2]));
}

void AchievedCommands::getMoment( double time, vec3 & outMomentBody ) {
    
}

void AchievedCommands::getLocation( double time, vec3 & locBody ) {
    locBody[0] = locBody[1] = locBody[2] = 0.0;
}
