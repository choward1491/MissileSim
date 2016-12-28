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
    this->setSimHistoryRate(10);
    this->setSimHistoryPath("test_history.txt");
    this->getIntegrator().setTolerance(1e-10);
}

bool MissileSim::isMonteCarloDone() {
    return (this->getCompletedMC() == 10);
}
void MissileSim::linkModelsToSim(){
    this->addDynamics(missile);
    this->addDynamics(target);
    missile.addSubModels(*this);
    this->addDiscrete(ts, 1000.0);
}
void MissileSim::connectModelsTogether() {
    missile.setTarget(target);
    target.setMissile(missile);
}
bool MissileSim::finishedSimulation(){
    return missile.getAltitude() <= 0.0;
}
void MissileSim::finalizeMonteCarloRun(){
    printf("Finished #%i Monte Carlo run!\n",static_cast<int>(getCompletedMC()));
    vec3 delta = missile.getPos()-target.getPos();
    delta[2] = 0;
    double miss = delta.magnitude();
    printf("Miss(%i) = %lf\n",static_cast<int>(getCompletedMC()),miss);
}
void MissileSim::finalize(){
    printf("Finished!\n");
    timer.stop();
    printf("Simulation complete after %lf seconds\n",timer.getDuration());
}
