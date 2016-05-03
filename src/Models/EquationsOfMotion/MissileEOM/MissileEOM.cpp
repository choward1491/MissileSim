//
//  MissileEOM.cpp
//  MissileSim
//
//  Created by Christian J Howard on 5/2/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "MissileEOM.hpp"

MissileEOM::MissileEOM(){
    
}
void MissileEOM::getExternalForceSum(double time, vec3 & forceSum ){
    double mass = *(this->mass);
    forceSum[0] = 10.0 * mass;
}
void MissileEOM::getExternalMomentSum(double time, vec3 & momentSum ){
    
}