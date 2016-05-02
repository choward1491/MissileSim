//
//  ForceContributor.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef ForceContributor_h
#define ForceContributor_h

#include "vec3.hpp"

class ForceContributor {
public:
    virtual vec3 getForce( double time ) = 0;
};

#endif /* ForceContributor_h */
