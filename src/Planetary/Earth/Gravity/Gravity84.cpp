//
//  Gravity84.cpp
//  MissileSim
//
//  Created by Christian J Howard on 11/28/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#include "Gravity84.hpp"
#include "Constants.hpp"
#include "Earth.hpp"
#include "LatLongAlt.hpp"

namespace Earth {

    double Gravity84::obtainGravityWithLatitudeDegrees( double alt, double latitude ){
        double lat = latitude * Constants::deg2rad;
        return obtainGravityWithLatitudeRadians(alt, lat);
    }
    double Gravity84::obtainGravityWithLatitudeRadians( double alt , double latitude ){
        double st = sin(latitude);
        double st2 = st*st;
        double g0 = Ge*(1 + k*st2)/sqrt(1.0 - e2*st2);
        double q = re / (re + alt);
        return g0*q*q;
    }
    
    double Gravity84::obtainGravityWithCoordinate( const LatLongAlt & coord ){
        return Gravity84::obtainGravityWithLatitudeRadians( coord.getAltitudeInMeters() ,
                                                            coord.getLatitudeInRadians() );
    }

}