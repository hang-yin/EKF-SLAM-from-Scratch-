# Feature-based EKF-SLAM from Scratch

## Video Demo

https://user-images.githubusercontent.com/60046203/230535520-b80a9883-18a1-43e9-b8e3-b3b580309c6c.mp4

## Simulation Environment from Scratch

For this project, a simulation environment was created using ROS2 and Rviz as a visualization tool. A ROS2 node was developed to simulate robot position and landmarks to test various components of this project, including the EKF SLAM algorithm and landmark detection. Collision detection between the simulated robot and obstacles was also implemented.

<img width="1072" alt="slam-simulation" src="https://user-images.githubusercontent.com/60046203/230535013-86ca066a-7840-423c-99fc-e3f931086778.png">


## Kinematics and 2D Rigid Body Transformations

Throughout the project, helper libraries were developed in C++ to handle calculations related to differential drive kinematics, 2D rigid body transformations, and other tasks. These libraries can be found in the turtlelib directory of the repository.

## Extended Kalman Filter SLAM

The primary component of this project is the implementation of feature-based extended Kalman filter simultaneous localization and mapping ([EKF-SLAM](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)). The EKF-SLAM algorithm consisted of three steps: initialization, prediction, and update. At each timestep, odometry and sensor measurements were used to estimate the state of the robot and landmarks. The prediction step updated the estimate of the full state vector and propagated uncertainty using the linearized state transition model. The update step involved computing the theoretical measurement given the current state estimate, the Kalman gain, the posterior state update, and the posterior covariance.

![slam-ekf](https://user-images.githubusercontent.com/60046203/230535031-41649eb0-030f-45b6-b1c7-61e37e1e8c4f.gif)

## Landmark Detection

To detect cylindrical landmarks, laser scan data was processed using a machine learning approach that combined unsupervised (lidar point clustering) and supervised (circular regression) learning. The resulting clusters were filtered using a [circle-fitting algorithm](https://projecteuclid.org/journals/electronic-journal-of-statistics/volume-3/issue-none/Error-analysis-for-circle-fitting-algorithms/10.1214/09-EJS419.full) to eliminate false positives.

<img width="863" alt="slam-circle" src="https://user-images.githubusercontent.com/60046203/230535050-a3452a44-a8ea-4d86-8820-1caf6b58a7bd.png">

## Reference
 - Ali Al-Sharadqah. Nikolai Chernov. "Error analysis for circle fitting algorithms." Electron. J. Statist. 3 886 - 911, 2009. https://doi.org/10.1214/09-EJS419
