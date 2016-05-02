//
//  MassProperties.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/30/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//


#ifndef MassPropertiesImpl_hpp
#define MassPropertiesImpl_hpp

#include "MassProperties.hpp"

template<class Type>
void MassProperties<Type>::initialize(){
    static_cast<Type*>(this)->initialize_();
}

template<class Type>
void MassProperties<Type>::update( double time ){
    static_cast<Type*>(this)->update_(time);
}



#endif