//
//  Gravity84.hpp
//  MissileSim
//
//  Created by Christian J Howard on 11/28/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#ifndef Gravity84_hpp
#define Gravity84_hpp

#include <math.h>
struct LatLongAlt;

namespace Earth {

    namespace Gravity84 {
        double obtainGravityWithLatitudeDegrees( double altInMeters, double latitude );
        double obtainGravityWithLatitudeRadians( double altInMeters, double latitude );
        double obtainGravityWithCoordinate( const LatLongAlt & coord );
    };
    
    namespace gravity = Gravity84;
    
}

#endif /* Gravity84_hpp */
