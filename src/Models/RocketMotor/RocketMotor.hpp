//
//  RocketMotor.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/25/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef RocketMotor_hpp
#define RocketMotor_hpp

#include <stdio.h>
#include "ForceContributor.hpp"
#include "MomentContributor.hpp"


class RocketMotor : public ForceContributor {
public:
    RocketMotor():motor_loc(-1,0,0){
        
    }
    
    virtual void getForce( double time, vec3 & outForceBody ) {
        outForceBody[0] = 10.0 ;
    }
    
    virtual void getLocation( double time, vec3 & locBody ) {
        locBody = motor_loc;
    }
    
private:
    vec3 motor_loc;
    
    
};


#endif /* RocketMotor_hpp */
