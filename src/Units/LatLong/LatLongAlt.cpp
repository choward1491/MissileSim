//
//  LatLong.cpp
//  MissileSim
//
//  Created by Christian J Howard on 11/25/15.
//  Copyright © 2015 Christian Howard. All rights reserved.
//

#include "LatLongAlt.hpp"
#include "Constants.hpp"

LatLongAlt::LatLongAlt( double latInDegrees, double longInDegrees, double alt ){
    setLatitudeInDegrees(latInDegrees);
    setLongitudeInDegrees(longInDegrees);
    altitude = alt;
}

double LatLongAlt::getLatitudeInDegrees() const{
    return latitude*Constants::rad2deg;
}
double LatLongAlt::getLatitudeInRadians() const{
    return latitude;
}
double LatLongAlt::getLongitudeInDegrees() const{
    return longitude*Constants::rad2deg;
}
double LatLongAlt::getLongitudeInRadians() const{
    return longitude;
}

void LatLongAlt::setLatitudeInDegrees( double latDegrees ){
    latitude = latDegrees*Constants::deg2rad;
}
void LatLongAlt::setLatitudeInRadians( double latRadians ){
    latitude = latRadians;
}
void LatLongAlt::setLongitudeInDegrees( double longDegrees ){
    longitude = longDegrees*Constants::deg2rad;
}
void LatLongAlt::setLongitudeInRadians( double longDegrees ){
    longitude = longDegrees;
}