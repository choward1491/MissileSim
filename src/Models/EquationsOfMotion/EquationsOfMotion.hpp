//
//  EquationsOfMotion.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef EquationsOfMotion_hpp
#define EquationsOfMotion_hpp

#include <stdio.h>
#include <vector>
#include "Quaternion.hpp"
#include "LatLongAlt.hpp"

class ModelState;
class MissileModel;
class TargetModel;
class ForceContributor;
class MomentContributor;

typedef la::FastMat<double, 3, 3> mat3;


template<class Type>
class EquationsOfMotion {
public:
    
    EquationsOfMotion();
    void setInitialLatLong( const LatLongAlt & init_coord );
    void setEOM_State( ModelState & eom_state );
    void operator()(double t, ModelState & dqdt );
    void addForceContributor( ForceContributor & force );
    void addMomentContributor( MomentContributor & moment );
    int  numDims() const;
    void updateComponents();
    void setInertia( mat3 & I_, mat3 & Iinv_ );
    void setMass( double & mass );
    
    
protected:
    friend class MissileModel;
    friend class TargetModel;
    
    int numDims_;
    
    vec3 pos;
    vec3 vel;
    vec3 accel;
    Quaternion q;
    vec3 omega;
    
    LatLongAlt initial_pos;
    LatLongAlt current_pos;
    ModelState * state;
    
    mat3 * I, *Iinv;
    double * mass;
    
    std::vector<ForceContributor*> forces;
    std::vector<MomentContributor*> moments;
    void getTotalForce( double time, vec3 & total );
    void getTotalMoment( double time, vec3 & total );
    
};


#include "EquationsOfMotionImpl.hpp"


#endif /* EquationsOfMotion_hpp */
