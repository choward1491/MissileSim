//
//  CoordTransforms.hpp
//  MissileSim
//
//  Created by Christian J Howard on 11/25/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#ifndef CoordTransforms_hpp
#define CoordTransforms_hpp

#include "LatLongAlt.hpp"
#include "math3d_define.hpp"
typedef LatLongAlt LLA;
typedef vec3 ECEF;
typedef vec3 ENU;
typedef vec3 NED;




namespace CoordTransforms {

    ENU  NEDtoENU( const NED & vec );
    NED  ENUtoNED( const ENU & vec );
    ECEF convertLLAtoECEF( const LLA & coord );
    ENU  getRelativeENU( const LLA & startCoord, const LLA & endCoord );
    ENU  getRelativeENU( const LLA & startCoord, const ECEF & startPos, const ECEF & endPos );
    ECEF getFinalECEF( const LLA & startCoord, const ENU & relENU );
    LLA  convertECEFtoLLA( const ECEF & posECEF );
    mat3 ENUtoECEF_Matrix( const LLA & startCoord );
    mat3 ECEFtoENU_Matrix( const LLA & startCoord );
    
    
};

namespace transforms = CoordTransforms;

#endif /* CoordTransforms_hpp */
