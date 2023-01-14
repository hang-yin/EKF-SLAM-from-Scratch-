#include "rigid2d.hpp"
#include <iostream>

/// \file
/// \brief Prompt user to enter two transforms, a vector, and a twist to show relevant information

using namespace std;


int main(){

    // PART1: Transform2D user input

    cout << "Enter transform T_{a,b}:" << endl;
    turtlelib::Transform2D T_ab;
    cin >> T_ab;

    cout << "Enter transform T_{b,c}:" << endl;
    turtlelib::Transform2D T_bc;
    cin >> T_bc;

    cout << "T_{a_b} = " << T_ab << endl;
    
    turtlelib::Transform2D T_ba = T_ab.inv();
    cout << "T_{b_a} = " << T_ba << endl;

    cout << "T_{b_c} = " << T_bc << endl;

    turtlelib::Transform2D T_cb = T_bc.inv();
    cout << "T_{c_b} = " << T_cb << endl;

    turtlelib::Transform2D T_ac = T_ab*T_bc;
    cout << "T_{a_c} = " << T_ac << endl;

    turtlelib::Transform2D T_ca = T_ac.inv();
    cout << "T_{c_a} = " << T_ca << endl;

    // PART2: Vector2D user input
    turtlelib::Vector2D v_b;
    cout << "Enter vector v_b:" << endl;
    cin >> v_b;

    turtlelib::Vector2D v_bhat = normalize(v_b);
    cout << "v_bhat: " << v_bhat << endl;

    turtlelib::Vector2D v_a = T_ab(v_b);
    cout << "v_a: " << v_a << endl;

    cout << "v_b: " << v_b << endl;

    turtlelib::Vector2D v_c = T_cb(v_b);
    cout << "v_c: " << v_c << endl;

    // PART3: Twist2D user input
    cout << "Enter twist V_b:" << endl;
    turtlelib::Twist2D V_b;
    cin >> V_b;

    turtlelib::Twist2D V_a = T_ab(V_b);
    cout << "V_a " << V_a << endl;
    cout << "V_b " << V_b << endl;
    turtlelib::Twist2D V_c = T_cb(V_b);
    cout << "V_c " << V_c << endl;

}