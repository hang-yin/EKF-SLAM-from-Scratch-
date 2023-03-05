# EKF SLAM Package

# Demo: EKF-SLAM with unknown data association

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

# Demo: EKF-SLAM with known landmark location

https://user-images.githubusercontent.com/60046203/221379997-dfaca644-33cf-44dd-b627-c01b6a9aba9b.mov

# Instruction
To run EKF-SLAM with known landmark location:
- run `ros2 launch nuslam slam.launch.xml cmd_src:=teleop`

To run EKF-SLAM with unknown data association:
- run `ros2 launch nuslam unknown_data_assoc.launch.xml cmd_src:=teleop`
