//
//  CoordTransforms.cpp
//  MissileSim
//
//  Created by Christian J Howard on 11/25/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#include "CoordTransforms.hpp"
#include "Constants.hpp"
#include "Earth.hpp"
#include <math.h>
#include "vec3.hpp"

using namespace Earth;


double NormalComputation( const double & latitude ){
    double st = sin(latitude);
    return a/sqrt(1.0 - e2*st*st);
}

ENU  CoordTransforms::NEDtoENU( const NED & vec ){
    return ops::equal(vec[1], vec[0], -vec[2]);
}
NED  CoordTransforms::ENUtoNED( const ENU & vec ){
    return ops::equal(vec[1], vec[0], -vec[2]);
}

ECEF CoordTransforms::convertLLAtoECEF( const LLA & coord ){
    double clat = cos(coord.getLatitudeInRadians());
    double slat = sin(coord.getLatitudeInRadians());
    double clong = cos(coord.getLongitudeInRadians());
    double slong = sin(coord.getLongitudeInRadians());
    double N = NormalComputation(coord.getLatitudeInRadians());
    double totalHeight = N + coord.getAltitudeInMeters();
    
    ECEF out;
    out[0] = totalHeight * clat * clong;
    out[1] = totalHeight * clat * slong;
    out[2] = (N*(1-e2) + coord.getAltitudeInMeters()) * slat;
    
    return out;
    
}
ENU  CoordTransforms::getRelativeENU( const LLA & startCoord, const LLA & endCoord ){
    ECEF startPos   = convertLLAtoECEF(startCoord);
    ECEF endPos     = convertLLAtoECEF(endCoord);
    return getRelativeENU(startCoord, startPos, endPos);
}
ENU  CoordTransforms::getRelativeENU( const LLA & startCoord, const ECEF & startPos, const ECEF & endPos ){
    double clat = cos(startCoord.getLatitudeInRadians());
    double slat = sin(startCoord.getLatitudeInRadians());
    double clong = cos(startCoord.getLongitudeInRadians());
    double slong = sin(startCoord.getLongitudeInRadians());
    
    ECEF dX = endPos - startPos;
    
    ENU out;
    out[0] = dX[1]*clong - dX[0]*slong;
    out[1] = dX[2]*clat - slat*(dX[0]*clong + dX[1]*slong);
    out[2] = clat*(dX[0]*clong + dX[1]*slong) + dX[2]*slat;
    
    return out;
}
ECEF CoordTransforms::getFinalECEF( const LLA & startCoord, const ENU & relENU ){
    double clat = cos(startCoord.getLatitudeInRadians());
    double slat = sin(startCoord.getLatitudeInRadians());
    double clong = cos(startCoord.getLongitudeInRadians());
    double slong = sin(startCoord.getLongitudeInRadians());
    
    ECEF startPos   = convertLLAtoECEF(startCoord);
    ECEF dx;
    dx[0] = -relENU[0]*slong - relENU[1]*slat*clong + relENU[2]*clat*clong;
    dx[1] = relENU[0]*clong - relENU[1]*slat*slong + relENU[2]*clat*slong;
    dx[2] = relENU[1]*clat + relENU[2]*slat;
    
    return startPos + dx;
    
}

mat3 CoordTransforms::ENUtoECEF_Matrix( const LLA & startCoord ){
    double lat = startCoord.latitude;
    double lng = startCoord.longitude;
    double clat = cos(lat);
    double slat = sin(lat);
    double clong = cos(lng);
    double slong = sin(lng);
    mat3 out;
    out(0,0) = -slong; out(0,1) = -slat*clong; out(0,2) = clat*clong;
    out(1,0) = clong;  out(1,1) = -slat*slong; out(1,2) = clat*slong;
    out(2,0) = 0.0;    out(2,1) = clat;        out(2,2) = slat;
    return out;
}
mat3 CoordTransforms::ECEFtoENU_Matrix( const LLA & startCoord ){
    double clat = cos(startCoord.getLatitudeInRadians());
    double slat = sin(startCoord.getLatitudeInRadians());
    double clong = cos(startCoord.getLongitudeInRadians());
    double slong = sin(startCoord.getLongitudeInRadians());
    mat3 out;
    out(0,0) = -slong; out(1,0) = -slat*clong; out(2,0) = clat*clong;
    out(0,1) = clong;  out(1,1) = -slat*slong; out(2,1) = clat*slong;
    out(0,2) = 0.0;    out(1,2) = clat;        out(2,2) = slat;
    return out;
    
}

LLA  CoordTransforms::convertECEFtoLLA( const ECEF & finalECEF ){
    double X = finalECEF[0], Y = finalECEF[1], Z = finalECEF[2];
    /*double p = sqrt( X*X + Y*Y );
    double theta = atan2(Z*a,p*b);
    double lng = atan2(Y,X);
    double lat = atan2(Z + ep2*b*pow(sin(theta),3.0),p - e2*a*pow(cos(theta),3.0));
    double alt = p/cos(lat) - NormalComputation(lat);*/
    
    
    double p = sqrt( X*X + Y*Y );
    double p2 = p*p;
    double e4 = e2*e2;
    double a2 = a*a;
    double zi = (1-e2)*Z*Z/(a2);
    double rho = (p2/a2 + zi - e4)/6.0;
    double rho2 = rho*rho;
    double rho3 = rho*rho2;
    double s = 0.25*e4*zi*p2/a2;
    double t = pow(rho3 + s + sqrt(s*(s+2.0*rho3)), 1.0/3.0);
    double u = rho + t + rho2 / t;
    double v = sqrt(u*u + e4*zi);
    double w = e2*(u + v - zi)/(2*v);
    double k = 1 + e2*(sqrt(u+v+w*w) + w)/(u+v);
    double Zk = Z*k;
    double lat = atan2(k*Z, p);
    double alt = sqrt(p2 + Zk*Zk)*(1.0/k - (1-e2))/e2;
    double lng = atan2(Y,X);
    
    
    
    LLA out;
    out.setLatitudeInRadians(lat);
    out.setLongitudeInRadians(lng);
    out.setAltitudeInMeters(alt);
    
    return out;
}



