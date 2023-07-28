//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#ifndef RRHELPER_H
#define RRHELPER_H

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace rrlib
{
Eigen::Vector4d RotationMatrixToQuaternions( const Eigen::Matrix3d &R )
{
    Eigen::Vector4d Q;
    
    double arg_1, arg_2;
    
    arg_1 = R(2, 1) - R(1, 2);
    arg_2 = R(0, 0) - R(1, 1) - R(2, 2) + 1.0;
    if (arg_2 < 0.0) { arg_2 = 0.0; }
    Q(0) = 0.5 * ( ( arg_1 >= 0.0 ) - ( arg_1 < 0.0 ) ) * sqrt( arg_2 );
    
    arg_1 = R(0, 2) - R(2, 0);
    arg_2 = R(1, 1) - R(0, 0) - R(2, 2) + 1.0;
    if (arg_2 < 0.0) { arg_2 = 0.0; }
    Q(1) = 0.5 * ( ( arg_1 >= 0.0 ) - ( arg_1 < 0.0 ) ) * sqrt( arg_2 );
    
    arg_1 = R(1, 0) - R(0, 1);
    arg_2 = R(2, 2) - R(0, 0) - R(1, 1) + 1.0;
    if (arg_2 < 0.0) { arg_2 = 0.0; }
    Q(2) = 0.5 * ( ( arg_1 >= 0.0 ) - ( arg_1 < 0.0 ) ) * sqrt( arg_2 );
    
    arg_2 = R(0, 0) + R(1, 1) + R(2, 2) + 1.0;
    if (arg_2 < 0.0) { arg_2 = 0.0; }
    Q(3) = 0.5 * sqrt( arg_2 );
    
    //~ Note that the sgn(x) function here is non-standard:
    //~ sgn(x) = 1.0 for x >= 0.0 and sgn(x) = -1.0 for x < 0.0,
    //~ so it is implemented as:
    //~ sgn(x) = (x >= 0.0) - (x < 0.0)
    
    return Q;
}

Eigen::Matrix3d QuaternionsToRotationMatrix( const Eigen::Vector4d &Q )
{
    Eigen::Matrix3d R;
    
    double x = Q(0);
    double y = Q(1);
    double z = Q(2);
    double w = Q(3);
    
    R << 2.0 * (w * w + x * x) - 1.0,   2.0 * (x * y - w * z),      2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),          2.0 * (w*w + y*y) - 1.0,    2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),          2.0 * (y * z + w * x),      2.0 * (w * w + z * z) - 1.0;
    
    return R;
}
}   //namespace rrlib

#endif /* RRHELPER_H */
