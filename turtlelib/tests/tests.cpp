#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <fstream>

using namespace turtlelib;

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

TEST_CASE("Rotation()","[transform]"){ // Marno, Nel
   double test_rot = 30.0;
   Transform2D T_test{test_rot};
   REQUIRE(T_test.rotation() == test_rot);
}

TEST_CASE("Translation()","[transform]"){ // Marno, Nel
   double test_x = 4.20;
   double test_y = 6.9;
   Transform2D T_test{{test_x,test_y}};
   REQUIRE(T_test.translation().x == test_x);
   REQUIRE(T_test.translation().y == test_y);
}

TEST_CASE("operator()(Vector2D v)","[transform]") // Marno, Nel
{
   double test_rot = PI/2.0;
   double test_x = 0.0;
   double test_y = 1.0;
   Transform2D T_ab{{test_x,test_y}, test_rot};
   Vector2D v_b{1, 1};
   Vector2D v_a = T_ab(v_b);
   REQUIRE(almost_equal(v_a.x, -1.0));
   REQUIRE(almost_equal(v_a.y, 2.0));
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel
{
   double test_rot = PI/2.0;
   double test_x = 0.0;
   double test_y = 1.0;
   Transform2D T_ab{{test_x,test_y}, test_rot};
   Twist2D V_b{1, 1, 1};
   Twist2D V_a = T_ab(V_b);
   REQUIRE(almost_equal(V_a.w, 1.0));
   REQUIRE(almost_equal(V_a.x, 0.0));
   REQUIRE(almost_equal(V_a.y, 1.0));
}

TEST_CASE( "Rotation", "[transform]") { // Rintaroh, Shima
   double theta = turtlelib::PI/4;
   turtlelib::Transform2D trans = turtlelib::Transform2D(theta);
   REQUIRE( trans.rotation() == theta);
}

TEST_CASE( "Translation", "[transform]") { // Rintaroh, Shima
   turtlelib::Vector2D v;
   v.x = 5.5;
   v.y = 7.2;
   turtlelib::Transform2D trans = turtlelib::Transform2D(v);
   REQUIRE( trans.translation().x == v.x);
   REQUIRE( trans.translation().y == v.y);
}

TEST_CASE( " Vector2D Operator()", "[transform]") { // Rintaroh, Shima
   turtlelib::Vector2D v;
   v.x = 5;
   v.y = 6;
   double theta = turtlelib::PI/4;
   turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
   turtlelib::Vector2D vec = {3, 4};
   turtlelib::Vector2D vector = trans(vec);
   REQUIRE( turtlelib::almost_equal(vector.x, 4.29289, 1.0e-4));
   REQUIRE( turtlelib::almost_equal(vector.y, 10.9497, 1.0e-4));
}

TEST_CASE("Twist2D Operator()", "[transform]") { // Rintaroh, Shima
   turtlelib::Vector2D v;
   v.x = 5;
   v.y = 6;
   double theta = turtlelib::PI/4;
   turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
   turtlelib::Twist2D t = {3, 4, 5};
   turtlelib::Twist2D twist = trans(t);
   REQUIRE(turtlelib::almost_equal(twist.w, 3));
   REQUIRE(turtlelib::almost_equal(twist.x, 17.2929, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(twist.y, -8.63604, 1.0e-5));
}

TEST_CASE( "Translation1", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Vector2D vec;
   vec.x = 1.0;
   vec.y = 3.4;
   turtlelib::Transform2D tf = turtlelib::Transform2D(vec);
   REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
   REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

TEST_CASE( "Rotation1", "[transform]" ) // Ava, Zahedi
{
   double phi = 90;
   turtlelib::Transform2D tf = turtlelib::Transform2D(phi);
   REQUIRE( turtlelib::almost_equal(tf.rotation(), phi, 0.00001) );
}

TEST_CASE( "Stream insertion operator <<", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Vector2D vec;
   vec.x = 1.0;
   vec.y = 3.4;
   double phi = 0.0;
   turtlelib::Transform2D tf = turtlelib::Transform2D(vec, phi);
   std::string str = "deg: 0 x: 1 y: 3.4";
   std::stringstream sstr;
   sstr << tf;
   REQUIRE( sstr.str() == str );
}

TEST_CASE( "Stream extraction operator >>", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Transform2D tf = turtlelib::Transform2D();
   std::stringstream sstr;
   sstr << "deg: 90 x: 1 y: 3.4";
   sstr >> tf;
   REQUIRE( turtlelib::almost_equal(tf.rotation(), turtlelib::deg2rad(90), 0.00001) );
   REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
   REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

TEST_CASE("operator *=", "[transform]"){    //Megan, Sindelar
   turtlelib::Vector2D trans_ab = {1,2};
   double rotate_ab = 0;
   turtlelib::Transform2D T_ab_1 = {trans_ab, rotate_ab};      //T_ab's are all the same,
   turtlelib::Transform2D T_ab_2 = {trans_ab, rotate_ab};      //but, need different vars
   turtlelib::Transform2D T_ab_3 = {trans_ab, rotate_ab};      //b/c getting overwritten otherwise
   turtlelib::Vector2D trans_bc = {3,4};
   double rotate_bc = turtlelib::PI/2;
   turtlelib::Transform2D T_bc = {trans_bc, rotate_bc};
   REQUIRE(turtlelib::almost_equal((T_ab_1*=T_bc).translation().x, 4.0));
   REQUIRE(turtlelib::almost_equal((T_ab_2*=T_bc).translation().y, 6.0));
   REQUIRE(turtlelib::almost_equal((T_ab_3*=T_bc).rotation(), (turtlelib::PI/2)));
}

TEST_CASE("Output stream operator <<", "[transform]"){    //Megan, Sindelar
   std::stringstream ss;                                   //create an object of stringstream
   std::streambuf* old_cout_streambuf = std::cout.rdbuf();     //pointer that holds the output
   std::cout.rdbuf(ss.rdbuf());        //redirect output stream to a stringstream
   turtlelib::Transform2D T_m = {{4, 3}, 0};
   std::cout << T_m << std::endl;
   std::cout.rdbuf(old_cout_streambuf);    //restore the original stream buffer
   REQUIRE(ss.str() == "deg: 0 x: 4 y: 3\n");
}

TEST_CASE("Input stream operator >>, [transform]"){     //Megan Sindelar
   std::stringstream ss("deg: 0 x: 4 y: 5");            //create an object of string stream
   std::streambuf* old_cin_streambuf = std::cin.rdbuf();   //pointer that holds the input
   std::cin.rdbuf(ss.rdbuf());                     //redirect input stream to a stringstream
   std::string input1, input2, input3, input4, input5, input6;     // variable to store input
   std::cin >> input1;                              //user input (before whitespace)
   std::cin >> input2;                              //user input (before whitespace)
   std::cin >> input3;                              //user input (before whitespace)
   std::cin >> input4;                              //user input (before whitespace)
   std::cin >> input5;                              //user input (before whitespace)
   std::cin >> input6;                              //user input (before whitespace)
   std::cin.rdbuf(old_cin_streambuf);              //restore the original stream buffer
   std::string input;
   input = input1 + " " + input2 + " " + input3 + " " + input4 + " " + input5 + " " + input6;
   REQUIRE(input == "deg: 0 x: 4 y: 5");
}

TEST_CASE("Rotation and Translation1", "[transform]"){   //Megan Sindelar
   turtlelib::Vector2D trans = {1,2};
   double rotate = 90;
   turtlelib::Transform2D T = {trans, rotate};
   REQUIRE(turtlelib::almost_equal(T.translation().x, trans.x));
   REQUIRE(turtlelib::almost_equal(T.translation().y, trans.y));
   REQUIRE(turtlelib::almost_equal(T.rotation(), rotate));
}

TEST_CASE("Inverse() for Twist2D", "[transform]") { // Dilan Wijesinghe
   float x = 3.0;
   float y = 4.0;
   float theta = turtlelib::PI / 4.0;
   turtlelib::Transform2D TfTest = {turtlelib::Vector2D {x, y}, theta};
   turtlelib::Transform2D res = TfTest.inv();
 
   REQUIRE( turtlelib::almost_equal(res.rotation(), -theta, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(res.translation().x, -4.94975, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(res.translation().y, -0.707107, 1.0e-5) );
}

TEST_CASE("Operator *= for Transform2D", "[transform]") { // Dilan Wijesinghe
   float x = 3.0;
   float y = 4.0;
   float theta = turtlelib::PI / 4.0;
   turtlelib::Transform2D TfTest = {turtlelib::Vector2D {x, y}, theta};
   TfTest *= TfTest;
   REQUIRE( turtlelib::almost_equal(TfTest.rotation(), 1.5708, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(TfTest.translation().x, 2.29289, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(TfTest.translation().y, 8.44975, 1.0e-5) );
}

TEST_CASE("Operator>> for Transform2D", "[transform]") { // Dilan Wijesinghe
   std::stringstream input;
   turtlelib::Transform2D TfTest;
   input << "90 2 3";
   input >> TfTest;
   REQUIRE( turtlelib::almost_equal(TfTest.rotation(), turtlelib::deg2rad(90), 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(TfTest.translation().x, 2.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(TfTest.translation().y, 3.0, 1.0e-5) );
}

TEST_CASE( "operator()", "[Transform2D]" ) { // Shantao Cao
   turtlelib::Transform2D trantest;
   turtlelib::Vector2D testvec;
   testvec.x = 0;
   testvec.y = 1;
   double testdeg;
   testdeg = turtlelib::PI/2;
   trantest = turtlelib::Transform2D(testvec,testdeg);
   turtlelib::Vector2D testvec2 = {1, 1}, testans;
   testans = trantest(testvec2);
   // std::cout<<trantest;
   // std::cout<<testvec2;
   // std::cout<<testans;
   REQUIRE(turtlelib::almost_equal(testans.x, -1, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(testans.y, 2, 1.0e-5));
}

TEST_CASE( "inv()", "[Transform2D]" ) { // Shantao Cao
   turtlelib::Transform2D trantest, tranans;
   turtlelib::Vector2D testvec;
   testvec.x = 0;
   testvec.y = 1;
   double testdeg;
   testdeg = turtlelib::PI/2;
   trantest = turtlelib::Transform2D(testvec,testdeg);
   tranans = trantest.inv();
   REQUIRE(turtlelib::almost_equal(tranans.rotation(), -testdeg, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans.translation().x, -1, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans.translation().y, 0, 1.0e-5));
}

TEST_CASE( "operator*=", "[Transform2D]" ) { // Shantao Cao
   turtlelib::Transform2D trantest1, trantest2,tranans;
   turtlelib::Vector2D testvec1,testvec2;
   testvec1.x = 0;
   testvec1.y = 1;
   testvec2.x = 1;
   testvec2.y = 0;
   double testdeg;
   testdeg = turtlelib::PI/2;
   trantest1 = turtlelib::Transform2D(testvec1,testdeg);
   trantest2 = turtlelib::Transform2D(testvec2,testdeg);
   tranans = trantest1*trantest2;
   REQUIRE(turtlelib::almost_equal(tranans.rotation(), 2*testdeg, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans.translation().x, 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans.translation().y, 2, 1.0e-5));
}

TEST_CASE( "translation and rotation", "[Transform2D]" ) { // Shantao Cao
   turtlelib::Transform2D trantest1;
   turtlelib::Vector2D testvec1;
   testvec1.x = 0;
   testvec1.y = 1;
   double testdeg;
   testdeg = turtlelib::PI/2;
   trantest1 = turtlelib::Transform2D(testvec1,testdeg);
   REQUIRE(turtlelib::almost_equal(trantest1.rotation(), testdeg, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(trantest1.translation().x, testvec1.x, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(trantest1.translation().y, testvec1.y, 1.0e-5));
}

TEST_CASE("Rotation()1","[transform]"){ //Ghosh, Ritika
   double test_ang = turtlelib::PI/4;
   turtlelib::Vector2D test_vec = {3.5, 2.5};
   turtlelib::Transform2D Ttest1, Ttest2(test_ang), Ttest3(test_vec), Ttest4(test_vec, test_ang);
   REQUIRE(Ttest1.rotation() == 0.0);
   REQUIRE(Ttest2.rotation() == test_ang);
   REQUIRE(Ttest3.rotation() == 0.0);
   REQUIRE(Ttest4.rotation() == test_ang);
}

TEST_CASE("operator <<","[transform]"){ //Ghosh, Ritika
   turtlelib::Transform2D Ttest={turtlelib::Vector2D{0,1}, turtlelib::PI/2};
   std::ostringstream os;
   os << Ttest;
   REQUIRE(os.str()=="deg: 90 x: 0 y: 1");
}

TEST_CASE("operator >>","[transform]"){ //Ghosh, Ritika
   turtlelib::Transform2D Ttest1, Ttest2;
   std::istringstream is1("deg: 90 x: 0 y:1"), is2("45 1 0");
   is1 >> Ttest1;
   is2 >> Ttest2;
   REQUIRE(turtlelib::almost_equal(Ttest1.rotation(), turtlelib::PI/2));
   REQUIRE(turtlelib::almost_equal(Ttest2.rotation(), turtlelib::PI/4));
}

// test case for normalize_angle function
// test this function with the following input: pi, -pi, 0, -pi/4, 3pi/2, -5pi/2
TEST_CASE("normalize_angle function", "[transform]") { //Yin, Hang
   double testdeg1, testdeg2, testdeg3, testdeg4, testdeg5, testdeg6;
   testdeg1 = turtlelib::PI;
   testdeg2 = -turtlelib::PI;
   testdeg3 = 0;
   testdeg4 = -turtlelib::PI/4;
   testdeg5 = 3*turtlelib::PI/2;
   testdeg6 = -5*turtlelib::PI/2;
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg1), turtlelib::PI, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg2), turtlelib::PI, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg3), 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg4), -turtlelib::PI/4, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg5), -turtlelib::PI/2, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(turtlelib::normalize_angle(testdeg6), -turtlelib::PI/2, 1.0e-5));
}

// test case for integrate_twist function
// test this function with a pure translation, a pure rotation, and a combination of translation and rotation
TEST_CASE("integrate_twist function", "[transform]") { //Yin, Hang
   turtlelib::Twist2D twisttest1, twisttest2, twisttest3;
   twisttest1.x = 1;
   twisttest1.y = 0;
   twisttest1.w = 0;
   twisttest2.x = 0;
   twisttest2.y = 0;
   twisttest2.w = turtlelib::PI/2;
   twisttest3.x = 1;
   twisttest3.y = 1;
   twisttest3.w = turtlelib::PI/4;
   turtlelib::Transform2D tranans1, tranans2, tranans3;
   tranans1 = turtlelib::integrate_twist(twisttest1);
   tranans2 = turtlelib::integrate_twist(twisttest2);
   tranans3 = turtlelib::integrate_twist(twisttest3);
   REQUIRE(turtlelib::almost_equal(tranans1.translation().x, 1, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans1.translation().y, 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans1.rotation(), 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans2.translation().x, 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans2.translation().y, 0, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans2.rotation(), turtlelib::PI/2, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans3.translation().x, 0.527393, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans3.translation().y, 1.27324, 1.0e-5));
   REQUIRE(turtlelib::almost_equal(tranans3.rotation(), turtlelib::PI/4, 1.0e-5));
}
