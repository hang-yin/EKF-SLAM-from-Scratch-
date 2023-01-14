# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

# Frame Main Output
```
Enter transform T_{a,b}:
deg: 90 x: 0 y: 1
Enter transform T_{b,c}:
deg: 90 x: 1 y: 0
T_{a_b} = deg: 90 x: 0 y: 1
T_{b_a} = deg: -90 x: -1 y: -6.12323e-17
T_{b_c} = deg: 90 x: 1 y: 0
T_{c_b} = deg: -90 x: -6.12323e-17 y: 1
T_{a_c} = deg: 180 x: 6.12323e-17 y: 2
T_{c_a} = deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 6.12323e-17]
Enter twist V_b:
1 1 1
V_a [1 1.11022e-16 1]
V_b [1 1 1]
V_c [1 2 -1]
```