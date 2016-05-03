//
//  GPS.cpp
//  MissileSim
//
//  Created by Christian J Howard on 5/2/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#include "GPS.hpp"

GPS::GPS(){
    truePos = 0;
}
GPS::~GPS(){
    
}
void GPS::initialize(){
    
}
void GPS::setupPrintData(){
    
}
void GPS::update(){
    gps_pos = *truePos;
}
void GPS::setLatLongSource( LatLongAlt & pos ){
    truePos = &pos;
}
LatLongAlt GPS::getPos() const{
    return gps_pos;
}


//LatLongAlt * truePos;
//LatLongAlt gps_pos;