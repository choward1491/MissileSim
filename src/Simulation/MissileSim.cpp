//
//  MissileSim.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "MissileSim.hpp"

MissileSim::MissileSim(){
    timer.start();
    std::string historyFile("/Users/christianjhoward/history.txt");
    setSimHistoryPath(historyFile);
    state.printFrequency = 60;
    numMC = 1;
    writeSimHistory = false;
}


void MissileSim::_linkModelsToSim( SimState & state ){
    addDiscrete(&tstep, 1000);
    addDynamics(&missile);
    missile.addSubModels(*this);
}

void MissileSim::_connectModelsTogether(){
    
}

bool MissileSim::_finishedSimulation( SimState & state ) const{
    //return missile.getAltitude() <= 0.0;
    return getTime() > 3600;
}
void MissileSim::_finalizeMonteCarloRun(){
    printf("Finished #%i Monte Carlo run!\n",static_cast<int>(getCompletedMC()));
}
void MissileSim::_finalize(){
    printf("Finished!\n");
    timer.stop();
    
    printf("Simulation complete after %lf seconds\n",timer.getDuration());
}
