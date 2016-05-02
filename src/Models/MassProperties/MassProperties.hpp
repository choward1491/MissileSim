//
//  MassProperties.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/30/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef MassProperties_hpp
#define MassProperties_hpp

#include <stdio.h>
#include "vec3.hpp"

class EquationsOfMotion;
typedef la::FastMat<double, 3, 3> mat3;


template<class Type>
class MassProperties {
public:
    void initialize();
    void update( double time );
    
    
protected:
    friend class EquationsOfMotion;
    
    mat3 I;
    mat3 Iinv;
    double mass;
    
};

#include "MassPropertiesImpl.hpp"

#endif /* MassProperties_hpp */
