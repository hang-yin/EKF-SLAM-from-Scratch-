// implement functions defined in rigid2d.hpp

#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace rigid2d
{
    // constructor
    Transform2D::Transform2D()
    {
        // initialize to identity matrix
        T[0][0] = 1.0;
        T[0][1] = 0.0;
        T[0][2] = 0.0;
        T[1][0] = 0.0;
        T[1][1] = 1.0;
        T[1][2] = 0.0;
        T[2][0] = 0.0;
        T[2][1] = 0.0;
        T[2][2] = 1.0;
    }

    // constructor
    Transform2D::Transform2D(double
    theta,
    Vector
