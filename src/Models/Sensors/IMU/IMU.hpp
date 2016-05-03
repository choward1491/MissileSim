//
//  IMU.hpp
//  MissileSim
//
//  Created by Christian J Howard on 5/2/16.
//
//  The MIT License (MIT)
//    Copyright © 2016 Christian Howard. All rights reserved.
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//
//

#ifndef IMU_hpp
#define IMU_hpp

#include "vec3.hpp"
#include "DiscreteModel.hpp"

class IMU : public DiscreteModel {
public:
    
    IMU();
    ~IMU();
    virtual void initialize();
    virtual void setupPrintData();
    virtual void update();
    void setAccelerationSource( vec3 & accel );
    void setAngularVelocitySource( vec3 & angVel );
    vec3 getAccel() const;
    vec3 getAngVel() const;
    
    
private:
    vec3 * trueAccel;
    vec3 * trueAngVel;
    vec3 imu_accel;
    vec3 imu_angvel;
};

#endif /* IMU_hpp */
