//
//  TargetMassProps.cpp
//  MissileSim
//
//  Created by Christian J Howard on 10/4/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "TargetMassProps.hpp"

void TargetMassProps::initialize_(){
    mass = 1.0;
    I(0,0) = 1.0; I(1,1) = 1.0; I(2,2) = 1.0;
    Iinv(0,0) = 1.0/I(0,0); Iinv(1,1) = 1.0/I(1,1); Iinv(2,2) = 1.0/I(2,2);
}
void TargetMassProps::update_(double time){
    
}

double & TargetMassProps::inertia(int row, int col) {
    return I(row,col);
}
const double & TargetMassProps::inertia(int row, int col) const {
    return I(row,col);
}
void TargetMassProps::computeInverseInertia() {
    
}
