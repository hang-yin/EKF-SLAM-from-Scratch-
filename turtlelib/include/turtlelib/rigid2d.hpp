#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath> // contains standard C math library

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg * PI / 180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    /// \brief turns any angle into the equivalent angle in the interval (-pi, pi]
    /// \param angle - the angle to normalize, in radians
    /// \return the equivalent angle in the interval (-pi, pi]
    constexpr double normalize_angle(double rad){
        double temp = std::fmod(rad, 2*PI);
        if(temp > PI){
            temp -= 2*PI;
        }
        else if(temp <= -PI){
            temp += 2*PI;
        }
        return temp;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief Default constructor
        Vector2D() = default;

        /// \brief Construct a Vector2D from x and y coordinates
        /// \param x - the x coordinate
        /// \param y - the y coordinate
        Vector2D(double x, double y);

        /// \brief += operator overload for Vector2D
        /// \param v - the vector to add
        /// \return a reference to the modified vector
        Vector2D & operator+=(const Vector2D & v);

        /// \brief -= operator overload for Vector2D
        /// \param v - the vector to subtract
        /// \return a reference to the modified vector
        Vector2D & operator-=(const Vector2D & v);

        /// \brief *= operator overload for Vector2D
        /// \param s - the scalar to multiply by
        /// \return a reference to the modified vector
        Vector2D & operator*=(double s);

        /// \brief dot product of two vectors
        /// \param v1 - the first vector
        /// \param v2 - the second vector
        /// \return the dot product of the two vectors
        double dot(Vector2D v1, Vector2D v2);

        /// \brief magnitude of a vector
        /// \param v - the vector
        /// \return the magnitude of the vector
        double magnitude(Vector2D v);

        /// \brief compute the angle between two vectors
        /// \param v1 - the first vector
        /// \param v2 - the second vector
        /// \return the angle between the two vectors
        double angle(Vector2D v1, Vector2D v2);
    };

    /// \brief + operator overload for Vector2D
    /// \param v - the vector to add
    /// \return a new vector that is the sum of the two vectors
    Vector2D operator+(Vector2D v1, Vector2D v2);

    /// \brief - operator overload for Vector2D
    /// \param v - the vector to subtract
    /// \return a new vector that is the difference of the two vectors
    Vector2D operator-(Vector2D v1, Vector2D v2);

    /// \brief * operator overload for Vector2D
    /// \param s - the scalar to multiply by
    /// \return a new vector that is the product of the vector and the scalar
    Vector2D operator*(Vector2D v, double s);

    /// \brief * operator overload for Vector2D, in another order
    /// \param s - the scalar to multiply by
    /// \return a new vector that is the product of the vector and the scalar
    Vector2D operator*(double s, Vector2D v);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief A Twist in 2D
    struct Twist2D
    {
        /// \brief the rotational component
        double w = 0.0;
        /// \brief the translational component
        double x = 0.0;
        /// \brief the translational component
        double y = 0.0;
    };

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {

    private:
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D
        /// \param twist - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D twist) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief output a Twist2D as [x y w]
    /// \param os - the output stream
    /// \param t - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a Twist2D
    /// \param is - the input stream
    /// \param t [out] - the twist to read
    std::istream & operator>>(std::istream & is, Twist2D & t);

    /// \brief normalize a Vector2D
    /// \param v - the vector to normalize
    /// \return a normalized version of the vector
    Vector2D normalize(Vector2D v);

    /// \brief compute the transformation of a rigid body following a constant twist for one time unit
    /// \param twist - the twist to follow
    /// \return the transform that results from following the twist for one time unit
    Transform2D integrate_twist(Twist2D twist);

}

#endif
