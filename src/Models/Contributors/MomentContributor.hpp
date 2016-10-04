//
//  MomentContributor.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef MomentContributor_h
#define MomentContributor_h

#include "math3d_define.hpp"

class MomentContributor {
public:
    virtual void getMoment(double time, vec3 & outMomentBody) = 0;
};

#endif /* MomentContributor_h */
