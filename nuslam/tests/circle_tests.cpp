#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <stdexcept>
#include <armadillo>
#include <cmath>

std::vector<std::vector<double>> clustering(std::vector<double> ranges, std::vector<double> angles)
{
  std::vector<std::vector<double>> clusters;
  double threshold = 0.05;
  std::vector<double> curr_cluster;

  double lidar_range_max_ = 3.5;
  double lidar_range_min_ = 0.12;

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
        double prev_x = curr_cluster[curr_cluster.size() - 2] *
          cos(curr_cluster[curr_cluster.size() - 1]);
        double prev_y = curr_cluster[curr_cluster.size() - 2] *
          sin(curr_cluster[curr_cluster.size() - 1]);
        double curr_x = ranges[i] * cos(angles[i]);
        double curr_y = ranges[i] * sin(angles[i]);
        double dist = sqrt(pow(curr_x - prev_x, 2) + pow(curr_y - prev_y, 2));
        // RCLCPP_INFO(this->get_logger(), "dist: %f", dist);
        if (dist < threshold) {
          curr_cluster.push_back(ranges[i]);
          curr_cluster.push_back(angles[i]);
        } else {
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
  if (dist < threshold) {
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

  return clusters;
}

std::vector<std::vector<double>> circular_regression(std::vector<std::vector<double>> clusters)
{
  // Step2: implement circle fitting algorithm to detect circles
  std::vector<std::vector<double>> circles;
  for (int i = 0; i < int(clusters.size()); i++) {
    // Step2.0: convert cluster points to cartesian coordinates
    std::vector<double> cluster_x;
    std::vector<double> cluster_y;
    for (int j = 0; j < int(clusters[i].size()); j += 2) {
      cluster_x.push_back(clusters[i][j] * cos(clusters[i][j + 1]));
      cluster_y.push_back(clusters[i][j] * sin(clusters[i][j + 1]));
    }
    // experiment with a check condition
    if (cluster_x.size() < 4) {
      continue;
    }
    // Step2.1: calculate centroid
    double x_sum = std::accumulate(std::begin(cluster_x), std::end(cluster_x), 0.0);
    double centroid_x = x_sum / cluster_x.size();
    double y_sum = std::accumulate(std::begin(cluster_y), std::end(cluster_y), 0.0);
    double centroid_y = y_sum / cluster_y.size();
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
    double mean_z = z_sum / z.size();
    // Step2.5: calculate the data matrix Z
    arma::vec all_ones = arma::ones<arma::vec>(cluster_x.size());
    arma::vec z_arma(z.data(), z.size());
    arma::vec cluster_x_arma(cluster_x.data(), cluster_x.size());
    arma::vec cluster_y_arma(cluster_y.data(), cluster_y.size());
    arma::mat Z0 = arma::join_rows(z_arma, cluster_x_arma);
    arma::mat Z1 = arma::join_rows(cluster_y_arma, all_ones);
    arma::mat Z = arma::join_rows(Z0, Z1);
    // Step2.6: calculate moment matrix M
    arma::mat M = (1 / (cluster_x.size())) * (Z.t()) * (Z);

    // Step2.7: form the constraint matrix
    arma::mat H = arma::zeros<arma::mat>(4, 4);
    H(0, 0) = 8.0 * mean_z;
    H(1, 1) = 1;
    H(2, 2) = 1;
    H(0, 3) = 2;
    H(3, 0) = 2;
    // Step2.8: computer inverse of H
    arma::mat H_inv = arma::zeros<arma::mat>(4, 4);
    H_inv(1, 1) = 1;
    H_inv(2, 2) = 1;
    H_inv(3, 3) = -2.0 * (mean_z);
    H_inv(0, 3) = 1.0 / 2.0;
    H_inv(3, 0) = 1.0 / 2.0;
    // Step2.9: compute the singular value decomposition of Z
    arma::mat U;
    arma::vec sigma;
    arma::mat V;
    arma::svd(U, sigma, V, Z);

    // Step2.10: check if the last singular value is small
    arma::vec A;
    if (sigma.back() < pow(10, -12)) {
      A = V.col(3);
    }
    // Step2.11: the second case for A
    else {
      arma::mat sigma_mat = arma::diagmat(sigma);
      arma::mat Y = V * sigma_mat * V.t();   // matrix multiplication: incompatible matrix dimensions 4x4 and 3x3 when there are only 2 circles
      arma::mat Q = Y * H_inv * Y;

      arma::vec eigen_vec;
      arma::mat eigen_mat;
      arma::eig_sym(eigen_vec, eigen_mat, Q);
      arma::vec A_s;
      for (int j = 0; j < int(eigen_vec.size()); j++) {
        if (eigen_vec(j) > 0) {
          A_s = eigen_mat.col(j);
          break;
        }
      }
      A = arma::solve(Y, A_s);
    }

    // Step2.12: calculate the circle parameters
    double a = (-A.at(1)) / (2 * A.at(0));
    double b = (-A.at(2)) / (2 * A.at(0));
    double r =
      sqrt(
      (pow(
        A.at(1),
        2) + pow(A.at(2), 2) - (4.0 * A.at(0) * A.at(3))) / (4 * pow(A.at(0), 2)));
    // Step2.13: calculate the circle center
    double center_x = a + centroid_x;
    double center_y = b + centroid_y;

    /*

    // Step3: circle classification, helps to avoid false positives
    double p1_x = cluster_x.at(0);
    double p1_y = cluster_y.at(0);
    double p2_x = cluster_x.at(int(cluster_x.size()) - 1);
    double p2_y = cluster_y.at(int(cluster_y.size()) - 1);
    std::vector<double> angles;
    for (int j = 1; j < int(cluster_x.size() - 1); j++) {
      double p_x = cluster_x.at(j);
      double p_y = cluster_y.at(j);
      // angle is the angle between the line p1p and the line p2p
      double dot_product = (p1_x - p_x) * (p2_x - p_x) + (p1_y - p_y) * (p2_y - p_y);
      double magnitude1 = sqrt(pow(p1_x - p_x, 2) + pow(p1_y - p_y, 2));
      double magnitude2 = sqrt(pow(p2_x - p_x, 2) + pow(p2_y - p_y, 2));
      double angle = acos(dot_product / (magnitude1 * magnitude2));
      angles.push_back(angle);
    }
    // find mean and standard deviation of angles
    double angle_sum = std::accumulate(std::begin(angles), std::end(angles), 0.0);
    double mean_angle = angle_sum / angles.size();
    // convert mean_angle from radians to degrees
    double angle_degree = mean_angle * 180.0 / turtlelib::PI;
    double sq_sum = std::inner_product(angles.begin(), angles.end(), angles.begin(), 0.0);
    double stdev_angle = std::sqrt(sq_sum / angles.size() - mean_angle * mean_angle);

    // lastly, push the circle parameters to the circles vector
    if (r < 0.05 && r > 0.01 && stdev_angle < 0.1 && angle_degree > 90 && angle_degree < 160) {
      std::vector<double> circle;
      circle.push_back(center_x);
      circle.push_back(center_y);
      circle.push_back(r);
      circles.push_back(circle);
    }
    */
    std::vector<double> circle;
    circle.push_back(center_x);
    circle.push_back(center_y);
    circle.push_back(r);
    circles.push_back(circle);
  }
  return circles;
}


TEST_CASE("Circle fitting") {

  // define a list of 2D points
  std::vector<std::vector<double>> points1 = {{1, 7}, {2, 6}, {5, 8}, {7, 7}, {9, 5}, {3, 7}};
  // convert 2D points to two double vectors ranges and angles from the origin
  std::vector<double> ranges1, angles1;
  for (auto & point : points1) {
    ranges1.push_back(std::sqrt(point[0] * point[0] + point[1] * point[1]));
    angles1.push_back(std::atan2(point[1], point[0]));
  }

  std::vector<std::vector<double>> clusters1 = clustering(ranges1, angles1);
  std::vector<std::vector<double>> circles1 = circular_regression(clusters1);

  REQUIRE(circles1.size() == 1);
  REQUIRE(circles1[0].size() == 3);
  REQUIRE(turtlelib::almost_equal(circles1[0][0], 4.615482, 1.0e-4));
  REQUIRE(turtlelib::almost_equal(circles1[0][1], 2.807354, 1.0e-4));
  REQUIRE(turtlelib::almost_equal(circles1[0][2], 4.8275, 1.0e-4));

  std::vector<std::vector<double>> points2 = {{-1, 0}, {-0.3, -0.06}, {0.3, 0.1}, {1, 0}};
  std::vector<double> ranges2, angles2;
  for (auto & point : points2) {
    ranges2.push_back(std::sqrt(point[0] * point[0] + point[1] * point[1]));
    angles2.push_back(std::atan2(point[1], point[0]));
  }

  std::vector<std::vector<double>> clusters2 = clustering(ranges2, angles2);
  std::vector<std::vector<double>> circles2 = circular_regression(clusters2);

  REQUIRE(circles2.size() == 1);
  REQUIRE(circles2[0].size() == 3);
  REQUIRE(turtlelib::almost_equal(circles2[0][0], 0.4908357, 1.0e-4));
  REQUIRE(turtlelib::almost_equal(circles2[0][1], -22.15212, 1.0e-4));
  REQUIRE(turtlelib::almost_equal(circles2[0][2], 22.17979, 1.0e-4));
}
