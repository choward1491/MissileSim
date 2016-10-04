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
#include "math3d_define.hpp"

class ModelState;
class MissileModel;
class TargetModel;
class ForceContributor;
class MomentContributor;


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
    
    const vec3 & getPos() const;
    const vec3 & getVel() const;
    const vec3 & getAccel() const;
    const quat & getAttitude() const;
    const vec3 & getAngularVel() const;
    const LatLongAlt & getInitialCoord() const;
    const LatLongAlt & getCurrentCoord() const;
    
    
protected:
    
    int numDims_;
    
    vec3 pos;
    vec3 vel;
    vec3 accel;
    quat q;
    vec3 omega;
    
    LatLongAlt initial_pos;
    LatLongAlt current_pos;
    ModelState * state;
    
    mat3 * I, *Iinv;
    double * mass;
    
    std::vector<ForceContributor*> forces;
    std::vector<MomentContributor*> moments;
    void getTotalForceAndMoments( double time, vec3 & totalForceBody, vec3 & totalMomentBody );
    
};


//#include "EquationsOfMotionImpl.hpp"


#endif /* EquationsOfMotion_hpp */
