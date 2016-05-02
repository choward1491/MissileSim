//
//  Atmosphere.cpp
//  MissileSim
//
//  Created by Christian J Howard on 11/26/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#include "Atmosphere.hpp"


atmosphere::state atmosphere::getAtmosphereState( double h ){
    
    state output;
    
    if( h >= 25000 ){       // upper stratosphere
        output.temperature  = -131.21 + 0.00299 * h;
        output.pressure     = 2.488 * pow((output.temperature + 273.1)/216.6, -11.388);
    }else if( h >= 11000 ){ // lower stratosphere
        output.temperature  = -56.46;
        output.pressure     = 22.65 * exp(1.73 - 0.000157 * h);
    }else{                  // troposphere
        output.temperature  = 15.04 - 0.00649 * h;
        output.pressure     = 101.29 * pow((output.temperature + 273.1)/288.08, 5.256);
    }
    
    output.density      = output.pressure / (0.2869 * (output.temperature + 273.1) );
    return output;
    
}
double atmosphere::convertTempToKelvin( double temp ){
    return temp + 273.15;
}
double atmosphere::convertTempToFahrenheit( double temp ){
    return temp * 9.0 / 5.0 + 32;
}
