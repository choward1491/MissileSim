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
#include "Simulator.hpp"
#include "MissileModel.hpp"
#include "Timer.hpp"
#include "RungeKutta4.hpp"
#include "ExplicitTrapezoidal.hpp"

class MissileSim : public Simulator<MissileSim,RungeKutta4>{
public:
    
    MissileSim();
    
    void _linkModelsToSim( SimState & state );
    void _connectModelsTogether();
    bool _finishedSimulation( SimState & state ) const;
    void _finalizeMonteCarloRun();
    void _finalize();
    
private:
    
    Timer timer;
    TimeStep tstep;
    MissileModel missile;
    
};

#endif /* MissileSim_hpp */
