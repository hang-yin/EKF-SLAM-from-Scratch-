#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"

TEST_CASE( "Rotation and Translation", "[transform]" ) { // Hughes, Katie
   float my_x = 2;
   float my_y = 3;
   float my_ang = turtlelib::PI;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   REQUIRE( 1  == 1 );
   REQUIRE( Ttest.rotation() == my_ang);
   REQUIRE( Ttest.translation().x == my_x);
   REQUIRE( Ttest.translation().y == my_y);
}

TEST_CASE( "Inverse", "[transform]" ) { // Hughes, Katie
   float my_x = 0.;
   float my_y = 1.;
   float my_ang = turtlelib::PI/2;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Transform2D Ttest_inv = Ttest.inv();
   REQUIRE( (Ttest.inv()).rotation() == -my_ang);
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().x, -1.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().y,  0.0, 1.0e-5) );
   // REQUIRE( Ttest_inv.translation().y == 0.0 );
}

TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Yin, Hang
    float my_x = 2;
    float my_y = 3;
    float my_ang = turtlelib::PI;
    turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
    turtlelib::Vector2D v = {2,2};
    turtlelib::Vector2D result = Ttest(v);
    REQUIRE( turtlelib::almost_equal(result.x, 0.0, 1.0e-5) );
    REQUIRE( turtlelib::almost_equal(result.y, 1.0, 1.0e-5) );
}

TEST_CASE( "Operator () for Twist2D", "[transform]" ) { // Yin, Hang
    float my_x = 2;
    float my_y = 3;
    float my_ang = turtlelib::PI;
    turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
    turtlelib::Twist2D twist = {2,2,2};
    turtlelib::Twist2D result = Ttest(twist);
    REQUIRE( turtlelib::almost_equal(result.w, 2.0, 1.0e-5) );
    REQUIRE( turtlelib::almost_equal(result.x, 4.0, 1.0e-5) );
    REQUIRE( turtlelib::almost_equal(result.y, -6.0, 1.0e-5) );
}
