//
//  MissileMassProps.cpp
//  MissileSim
//
//  Created by Christian J Howard on 5/1/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "MissileMassProps.hpp"


void MissileMassProps::initialize_(){
    mass = 50.0;
    I(0,0) = 3.0; I(1,1) = 3.0; I(2,2) = 3.0;
    Iinv(0,0) = 1.0/I(0,0); Iinv(1,1) = 1.0/I(1,1); Iinv(2,2) = 1.0/I(2,2);
}
void MissileMassProps::update_(double time){
    
}