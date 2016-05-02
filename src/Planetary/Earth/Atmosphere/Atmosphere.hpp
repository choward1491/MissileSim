//
//  Atmosphere.hpp
//  MissileSim
//
//  Created by Christian J Howard on 11/26/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#ifndef Atmosphere_hpp
#define Atmosphere_hpp

#include <math.h>

namespace atmosphere {
    
    struct state {
        double temperature; // in celsius
        double pressure;    // in kPa
        double density;     // in kg/m^3
    };
    
    state getAtmosphereState( double altitudeInMeters );
    double convertTempToKelvin( double tempInCelsius );
    double convertTempToFahrenheit( double tempInCelsius );
    
    
};

#endif /* Atmosphere_hpp */
