//
//  MissileSim.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef MissileSim_hpp
#define MissileSim_hpp

#include <stdio.h>
#include "MissileModel.hpp"
#include "TargetModel.hpp"
#include "uniform_sim.hpp"
#include "runge_kutta4.hpp"
#include "time_step.hpp"

typedef double num_type;

class MissileSim : public sim::uniform<num_type, integrate::rk4 >{
public:
    
    MissileSim();
    
    bool isMonteCarloDone();
    void linkModelsToSim();         // method to link models to sim
    void connectModelsTogether();
    bool finishedSimulation();      // method to return whether the sim has finished
    void finalizeMonteCarloRun();   // method to finalize a monte carlo run
    void finalize();
    
private:
    
    Timer timer;
    time_step<num_type> ts;
    MissileModel missile;
    TargetModel target;
    
};

#endif /* MissileSim_hpp */
