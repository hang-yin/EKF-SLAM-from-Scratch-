// implement functions defined in rigid2d.hpp

#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{
    Transform2D::Transform2D(){
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }

    Transform2D::Transform2D(Vector2D trans){
        x = trans.x;
        y = trans.y;
        theta = 0.0;
    }

    Transform2D::Transform2D(double radians){
        x = 0.0;
        y = 0.0;
        theta = radians;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        x = trans.x;
        y = trans.y;
        theta = radians;
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D temp;
        temp.x = x + v.x*cos(theta) - v.y*sin(theta);
        temp.y = y + v.x*sin(theta) + v.y*cos(theta);
        return temp;
    }

    Twist2D Transform2D::operator()(Twist2D twist) const{
        Twist2D temp;
        temp.w = twist.w;
        temp.x = twist.x*cos(theta) - twist.y*sin(theta) + y*twist.w;
        temp.y = twist.x*sin(theta) + twist.y*cos(theta) - x*twist.w;
        return temp;
    }

    Transform2D Transform2D::inv() const{
        Transform2D temp;
        temp.theta = -theta;
        temp.x = -x*cos(theta) - y*sin(theta);
        temp.y = x*sin(theta) - y*cos(theta);
        return temp;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        x = x + rhs.x*cos(theta) - rhs.y*sin(theta);
        y = y + rhs.x*sin(theta) + rhs.y*cos(theta);
        theta = theta + rhs.theta;
        return *this;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs *= rhs;
    }

    Vector2D Transform2D::translation() const{
        Vector2D temp;
        temp.x = x;
        temp.y = y;
        return temp;
    }

    double Transform2D::rotation() const{
        return theta;
    }

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
        Vector2D temp_v;
        double temp_theta;

        if (first_char =='d'){
            std::string temp1, temp2, temp3;
            is >> temp1 >> temp_theta >> temp2 >> temp_v.x >> temp3 >> temp_v.y;
            tf = Transform2D(temp_v, deg2rad(temp_theta));
        }
        else{
            is >> temp_theta >> temp_v.x >> temp_v.y;
            tf = Transform2D(temp_v, deg2rad(temp_theta));
        }
        is.get();
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t){
        os << "[" << t.w << " " << t.x << " " << t.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t){
        char first_char = is.peek();
        if (first_char == '[')
        {
            is.get();
            is >> t.w >> t.x >> t.y;
        }
        else
        {
            is >> t.w >> t.x >> t.y;
        }
        return is;
    }

    Vector2D normalize(Vector2D v){
        Vector2D temp;
        double mag = sqrt(v.x*v.x + v.y*v.y);
        temp.x = v.x/mag;
        temp.y = v.y/mag;
        return temp;
    }

}