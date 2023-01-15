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


   Design 1: A standalone normalize function
   - Pros: the independence of this function makes it easier to be tested and reused
   - Cons: you lose control of private fields for the Vector2D object you are trying to normalize; your code become less structured as this function is now not encapsulated in any classes

   Design 2: A member function within the Vector2D class
   - Pros: encapsulation gives you control over the Vector2D objects and their behaviors
   - Cons: if we want to use the normalize functionality outside of this class, we need to write separate functions with the exact same logic

   Design 3: Operator overload for the Vector2D class
   - Pros: makes normalize call look clean; encapsulation is maintained
   - Cons: it might be counter-intuitive to use an operator overload for normalization

   At this point, I'm implementing the normalize functionality as a standalone function for testing purposes. But given time and the necessity to refine my library, I would probably implement it within a class so that the function is encapsulated. 

2. What is the difference between a class and a struct in C++?
   - While a class has both data and methods, a struct usually only contains data
   - The data within a class is usually not accessible by users for encapsulation purposes, but data within a struct is always accessible to users

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

   Vector2D is a struct because it is a simple collection of data. Transform2D is a class because it has both data and methods that need to operate on its data. One C++ core guideline states that "use class if the class has an invariant; use struct if the data members can vary independently." This guideline is reflected here as members in a 2D vectors can vary independently as desired but the data members within a 2D transform are dependent. Another core guideline states that "use class rather than struct if any member is non-public." This makes sense for Transform2D as the matrix needs to be private to protect its data entries. 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    A core guideline states "by default, declare single-argument constructors explicit." This explains why the single-argument constructors are declared explicit. This guideline is here to help avoid unintended conversions. 


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   The function inv() is const because it does not modify the argument that is passed to this function. However, the operator*= modifies the provided argument, which makes it not suitable for a const declaration. One core guideline states that "by default, make member functions const." That's why we make the inv() function const here. 



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