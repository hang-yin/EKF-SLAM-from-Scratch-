# EKF SLAM Package

# Demo: EKF-SLAM in real world

https://user-images.githubusercontent.com/60046203/224433130-1fed25ba-89fe-4ed9-8a1b-0daec8c5b489.mp4

The last logged pose:
| Method       | X coordinate | Y coordinate | theta  |
|--------------|--------------|--------------|--------|
| Odometry     | 0.023393     | 0.004747     | -0.367 |
| EKF-SLAM     | -0.007048    | -0.022957    | -0.171 |

# Demo: EKF-SLAM with unknown data association in simulation

[uda_slam_demo.webm](https://user-images.githubusercontent.com/60046203/222990268-22c5cda8-28e6-411e-997d-e82988ee43ad.webm)

The last logged pose:
| Method       | X coordinate | Y coordinate |
|--------------|--------------|--------------|
| Ground Truth | 0.120515     | 0.026262     |
| Odometry     | 0.232572     | 0.004832     |
| EKF-SLAM     | 0.128980     | 0.025990     |

Pose error:
| Method      | Pose Error |
|-------------|------------|
| Odometry    | 0.114088   |
| EKF-SLAM    | 0.008469   |

# Demo: EKF-SLAM with known landmark location in simulation

https://user-images.githubusercontent.com/60046203/221379997-dfaca644-33cf-44dd-b627-c01b6a9aba9b.mov

# Instruction
To run EKF-SLAM with known landmark location:
- run `ros2 launch nuslam slam.launch.xml cmd_src:=teleop`

To run EKF-SLAM with unknown data association:
- run `ros2 launch nuslam unknown_data_assoc.launch.xml cmd_src:=teleop`

To run EKF-SLAM on a real Turtlebot3:
- on the Turtlebot, run `ros2 launch nuslam turtlebot_bringup.launch.xml`
- on your PC, run `ros2 launch nuslam pc_bringup.launch.xml`
