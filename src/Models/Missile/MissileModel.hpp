//
//  MissileModel.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef MissileModel_hpp
#define MissileModel_hpp

#include <stdio.h>
#include "DynamicModel.hpp"
#include "EquationsOfMotion.hpp"
#include "EulerAngles.hpp"
#include "MissileMassProps.hpp"

class MissileSim;
class TargetModel;

class MissileModel : public DynamicModel {
public:
    
    MissileModel();
    
    
    /*!
     * This method is used to do initialization
     * that needs to be done every time a sim is run
     * but could benefit from occuring after all the
     * proper model constructions and linking has occurred.
     * A good example could be initializing this model
     * based on data in other models in the simulation.
     * Another use of this function is to ensure each
     * monte carlo sim run is initialized properly
     *
     * \params None
     * \returns None
     */
    virtual void initialize();
    
    
    /*!
     *
     * Method that allows a model to have some
     * data saved to a sim history file
     *
     */
    virtual void setupPrintData();
    
    
    /*!
     * In the event this model
     * is based on differential
     * equations, this method returns
     * the number of dimensions
     * in the ODE equations
     *
     * \params None
     * \returns Number of Dimensions in System of ODEs
     */
    virtual int numDims() const;
    
    
    
    
    /*!
     * Method to add models used within this model
     * to the model list of the simulation. The point
     * is partially so that multiple missiles can be
     * added to the system easily
     */
    void addSubModels( MissileSim & msim );
    
    
    
    /*!
     * This method will be used to update
     * any variables or states within the
     * dynamic object that aren't explicitly
     * time integrated. It could be as simple
     * as updating a variable that is a unit
     * conversion different than the one being
     * integrated. This happens right after the
     * time integration is complete.
     *
     * \params None
     * \returns None
     */
    virtual void update();
    
    /*!
     * This is a method that represents
     * the system of differential equations
     * and outputs the rate of change of the
     * variables in the equations
     *
     * \params dxdt A vector that will hold the rate of hange
     of the diffeq variables. This vector
     will be modified in this method and in
     turn make this input also the output
     * \returns None
     */
    virtual void operator()( double time , ModelState & dqdt );
    
    
    
    void setTarget( const TargetModel & target ) const;
    double getAltitude() const;
    
    
    
    
private:
    double roll, pitch, yaw;
    double P, Q, R;
    EquationsOfMotion eom;
    MissileMassProps massprops;
    TargetModel * target;
    
};


#endif /* MissileModel_hpp */
