#include <rclcpp/rclcpp.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <armadillo>
#include <cmath>

using namespace std::chrono_literals;

class LandmarksNode : public rclcpp::Node
{
public:
  LandmarksNode()
    : Node("landmarks")
  {
    // Declare parameters
    declare_parameter("min_lidar_range", 0.12);
    declare_parameter("max_lidar_range", 3.5);
    declare_parameter("angle_increment", 0.0174532924);
    lidar_range_max_ = get_parameter("max_lidar_range").as_double();
    lidar_range_min_ = get_parameter("min_lidar_range").as_double();
    lidar_angle_increment_ = get_parameter("angle_increment").as_double();

    // Initialize publishers, subscribers, and timers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(
        &LandmarksNode::scan_callback,
        this,
        std::placeholders::_1));
    landmark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fitted_landmarks", 10);

    clusters_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/clusters", 10);
    /*
    auto rate = 5.0;
    main_timer_ = this->create_wall_timer(1s/rate, std::bind(&LandmarksNode::main_timer_callback, this));
    */
  }

private:
  // Declare variables
  double lidar_range_max_;
  double lidar_range_min_;
  double lidar_angle_increment_;
  // rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;

  // Declare functions
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // remember that laser scan is published at 5Hz in nusim!
  {
    // Get the scan data
    // cast msg->ranges to std::vector<double>
    // std::vector<double> ranges = msg->ranges;
    std::vector<double> ranges(msg->ranges.begin(), msg->ranges.end());
    std::vector<double> angles;
    for (int i = 0; i < int(ranges.size()); i++) {
      angles.push_back(msg->angle_min + i * msg->angle_increment);
    }

    // Find the landmarks
    std::vector<std::vector<double>> landmarks = find_landmarks(ranges, angles);

    // Publish the landmarks
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < int(landmarks.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_scan";
      marker.header.stamp = msg->header.stamp;
      marker.ns = "landmarks";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = landmarks[i][0];
      marker.pose.position.y = landmarks[i][1];
      marker.pose.position.z = 0.125;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
    }
    landmark_pub_->publish(marker_array);
  }

  std::vector<std::vector<double>> find_landmarks(std::vector<double> ranges, std::vector<double> angles)
  {
    // Step1: cluster points into groups
    std::vector<std::vector<double>> clusters;
    double threshold = 0.05;
    std::vector<double> curr_cluster;

    for (int i = 0; i < int(ranges.size()); i++) {
      if (ranges[i] < lidar_range_max_ && ranges[i] > lidar_range_min_) {
        // if curr_cluster is empty, add the first point
        if (curr_cluster.size() == 0) {
          curr_cluster.push_back(ranges[i]);
          curr_cluster.push_back(angles[i]);
        }
        // if curr_cluster is not empty, check if the point is close to the last point in the cluster
        else {
          // given range and angle for current point and previous point in cluster, calculate euclidean distance
          double prev_x = curr_cluster[curr_cluster.size() - 2] * cos(curr_cluster[curr_cluster.size() - 1]);
          double prev_y = curr_cluster[curr_cluster.size() - 2] * sin(curr_cluster[curr_cluster.size() - 1]);
          double curr_x = ranges[i] * cos(angles[i]);
          double curr_y = ranges[i] * sin(angles[i]);
          double dist = sqrt(pow(curr_x - prev_x, 2) + pow(curr_y - prev_y, 2));
          // RCLCPP_INFO(this->get_logger(), "dist: %f", dist);
          if (dist < threshold) {
            curr_cluster.push_back(ranges[i]);
            curr_cluster.push_back(angles[i]);
          }
          else {
            clusters.push_back(curr_cluster);
            curr_cluster.clear();
            curr_cluster.push_back(ranges[i]);
            curr_cluster.push_back(angles[i]);
          }
        }
      }
    }
    // push the last cluster to clusters
    clusters.push_back(curr_cluster);

    // check if the first and last points are close
    // if so, combine the two clusters
    double dist = abs(ranges[0] - curr_cluster[curr_cluster.size() - 2]);
    if (dist < threshold){
      // combine the two clusters
      std::vector<double> combined_cluster;
      // push the first cluster to the combined cluster
      for (int i = 0; i < int(clusters[0].size()); i++) {
        combined_cluster.push_back(clusters[0][i]);
      }
      // push the last cluster to the combined cluster
      for (int i = 0; i < int(curr_cluster.size()); i++) {
        combined_cluster.push_back(curr_cluster[i]);
      }
      // remove the first and last clusters
      clusters.erase(clusters.begin());
      clusters.pop_back();
      // push the combined cluster to the clusters
      clusters.push_back(combined_cluster);
    }

    // discard clusters that have less than 3 points
    for (int i = 0; i < int(clusters.size()); i++) {
      if (clusters[i].size() < 6) {
        clusters.erase(clusters.begin() + i);
        i--;
      }
    }


    // publish all clusters for TESTING
    /*
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < int(clusters.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_scan";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "clusters";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      for (int j = 0; j < int(clusters[i].size()); j += 2) {
        geometry_msgs::msg::Point p;
        p.x = clusters[i][j] * cos(clusters[i][j + 1]);
        p.y = clusters[i][j] * sin(clusters[i][j + 1]);
        p.z = 0;
        marker.points.push_back(p);
      }
      marker_array.markers.push_back(marker);
    }
    clusters_pub_->publish(marker_array);
    */

    // Step2: implement circle fitting algorithm to detect circles
    std::vector<std::vector<double>> circles;
    for (int i = 0; i < int(clusters.size()); i++){
      // Step2.0: convert cluster points to cartesian coordinates
      std::vector<double> cluster_x;
      std::vector<double> cluster_y;
      for (int j = 0; j < int(clusters[i].size()); j += 2) {
        cluster_x.push_back(clusters[i][j] * cos(clusters[i][j + 1]));
        cluster_y.push_back(clusters[i][j] * sin(clusters[i][j + 1]));
      }
      // Step2.1: calculate centroid
      double x_sum = std::accumulate(std::begin(cluster_x), std::end(cluster_x), 0.0);
      double centroid_x =  x_sum / cluster_x.size();
      double y_sum = std::accumulate(std::begin(cluster_y), std::end(cluster_y), 0.0);
      double centroid_y =  y_sum / cluster_y.size();
      // Step2.2: shift coordinates so that centroid is at origin
      for (int j = 0; j < int(cluster_x.size()); j++) {
        cluster_x[j] -= centroid_x;
        cluster_y[j] -= centroid_y;
      }
      // Step2.3: calculate z
      std::vector<double> z;
      for (int j = 0; j < int(cluster_x.size()); j++) {
        z.push_back(pow(cluster_x[j], 2) + pow(cluster_y[j], 2));
      }
      // Step2.4: calculate mean of z
      double z_sum = std::accumulate(std::begin(z), std::end(z), 0.0);
      double mean_z =  z_sum / z.size();
      // Step2.5: calculate the data matrix Z
      arma::vec all_ones = arma::ones<arma::vec>(cluster_x.size());
      arma::vec z_arma(z.data(), z.size());
      arma::vec cluster_x_arma(cluster_x.data(), cluster_x.size());
      arma::vec cluster_y_arma(cluster_y.data(), cluster_y.size());
      arma::mat Z0 = arma::join_rows(z_arma, cluster_x_arma);
      arma::mat Z1 = arma::join_rows(cluster_y_arma, all_ones);
      arma::mat Z = arma::join_rows(Z0, Z1);
      // Step2.6: calculate moment matrix M
      arma::mat M = (1/(cluster_x.size()))*(Z.t()) * (Z);
      // Step2.7: form the constraint matrix
      arma::mat H = arma::zeros<arma::mat>(4, 4);
      H(0, 0) = 8.0*mean_z;
      H(1, 1) = 1;
      H(2, 2) = 1;
      H(0, 3) = 2;
      H(3, 0) = 2;
      // Step2.8: computer inverse of H
      arma::mat H_inv = arma::zeros<arma::mat>(4, 4);
      H_inv(1, 1) = 1;
      H_inv(2, 2) = 1;
      H_inv(3, 3) = -2.0*(mean_z);
      H_inv(0, 3) = 1.0/2.0;
      H_inv(3, 0) = 1.0/2.0;
      // Step2.9: compute the singular value decomposition of Z
      arma::mat U;
      arma::vec sigma;
      arma::mat V;
      arma::svd(U, sigma, V, Z);
      // Step2.10: check if the last singular value is small
      arma::vec A;
      if (sigma.back() < pow(10, -12)){
        A = V.col(3);
      }
      // Step2.11: the second case for A
      else{
        arma::mat sigma_mat = arma::diagmat(sigma);
        arma::mat Y = V * sigma_mat * V.t();
        arma::mat Q = Y * H_inv * Y;
        arma::vec eigen_vec;
        arma::mat eigen_mat;
        arma::eig_sym(eigen_vec, eigen_mat, Q);
        arma::vec A_s;
        for (int j = 0; j < int(eigen_vec.size()); j++){
          if (eigen_vec(j) > 0){
            A_s = eigen_mat.col(j);
            break;
          }
        }
        A = arma::solve(Y, A_s);
      }
      // Step2.12: calculate the circle parameters
      double a = (-A.at(1))/(2*A.at(0));
      double b = (-A.at(2))/(2*A.at(0));
      double r = sqrt((pow(A.at(1), 2) + pow(A.at(2), 2) - (4.0*A.at(0)*A.at(3)))/(4*pow(A.at(0), 2)));
      // Step2.13: calculate the circle center
      double center_x = a + centroid_x;
      double center_y = b + centroid_y;
      // lastly, push the circle parameters to the circles vector
      if(r < 0.05 && r > 0.01){
        std::vector<double> circle;
        circle.push_back(center_x);
        circle.push_back(center_y);
        circle.push_back(r);
        circles.push_back(circle);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Number of circles detected: %d", int(circles.size()));
    // log all circle information
    for (int i = 0; i < int(circles.size()); i++){
      RCLCPP_INFO(this->get_logger(), "Circle %d: center_x = %f, center_y = %f, radius = %f", i, circles[i][0], circles[i][1], circles[i][2]);
    }

    // return empty list of landmarks
    std::vector<std::vector<double>> landmarks;
    return landmarks;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarksNode>());
  rclcpp::shutdown();
  return 0;
}
