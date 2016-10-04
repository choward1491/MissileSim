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
#include "math3d_define.hpp"

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
