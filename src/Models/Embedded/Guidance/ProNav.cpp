//
//  ProNav.cpp
//  MissileSim
//
//  Created by Christian J Howard on 9/30/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "ProNav.hpp"


namespace pronav {
    void computeCommandedAccel( vec3 & outAccelBody ) {
        vec3 omega = R.cross(V);
        double r2 = R.magnitudeSquared();
        omega[0] /= r2;
        omega[1] /= r2;
        omega[2] /= r2;
        
        outAccelBody = V.cross(omega) * gain;
        
    }
    
}


vec3 pronav::R;
vec3 pronav::V;
double pronav::gain = 3.0;
