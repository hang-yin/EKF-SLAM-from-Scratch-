// implement functions defined in rigid2d.hpp

#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char first_char = is.peek();
        if (first_char == '[')
        {
            is.get();
            is >> v.x >> v.y;
        }
        else
        {
            is >> v.x >> v.y;
        }
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        char first_char = is.peek();
        char temp[10];
        Vector2D temp_v;
        double temp_theta;
        if (first_char =='d'){
            is.get(temp, 6);
            is >> temp_theta;
            temp_theta = deg2rad(temp_theta);
            is.get(temp, 4);
            is >> temp_v.x;
            is.get(temp, 4);
            is >> temp_v.y;
            tf = Transform2D(temp_v, temp_theta);
        }
        else{
            is >> temp_theta >> temp_v.x >> temp_v.y;
            tf = Transform2D(temp_v, deg2rad(temp_theta));
        }
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs *= rhs;
    }

}