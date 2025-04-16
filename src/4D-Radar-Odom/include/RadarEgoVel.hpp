#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <iostream>
#include "RadarEgoVelConfig.hpp"
#include <random>
namespace rio {

  // Define a type alias for a vector of size 11
  using Vector11 = Eigen::Matrix<double, 11, 1>;

  // Struct to hold indices for accessing radar data fields
  struct RadarEgoVelocityEstimatorIndices {
    uint x_r = 0;
    uint y_r = 1;
    uint z_r = 2;
    uint snr_db = 3;
    uint doppler = 4;
    uint range = 5;
    uint azimuth = 6;
    uint elevation = 7;
    uint normalized_x = 8;
    uint normalized_y = 9;
    uint normalized_z = 10;
  };


// Convert a Vector11 item to RadarPointCloudType using the provided indices
  RadarPointCloudType toRadarPointCloudType(const Vector11& item, const RadarEgoVelocityEstimatorIndices& idx) {
    RadarPointCloudType point;
    point.x = item[idx.x_r];
    point.y = item[idx.y_r];
    point.z = item[idx.z_r];
    point.doppler = -item[idx.doppler];
    point.intensity = item[idx.snr_db];
    return point;
  }

  class RadarEgoVel {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RadarEgoVel(const RadarEgoVelocityEstimatorConfig& config) : config_(config) {
      setRansacIter();
    }

    // Main function to estimate ego velocity from radar data
    bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg,
                  const double& pitch,
                  const double& roll,
                  const double& yaw,
                  bool is_holonomic,
                  Eigen::Vector3d& v_r,
                  Eigen::Vector3d& sigma_v_r,
                  sensor_msgs::msg::PointCloud2& inlier_radar_msg,
                  sensor_msgs::msg::PointCloud2& outlier_radar_msg,
                  Eigen::Vector3d& w_r);

  private:
    const RadarEgoVelocityEstimatorIndices idx_; // Indices for data access
    RadarEgoVelocityEstimatorConfig config_;     // Configuration parameters
    uint ransac_iter_ = 0;                       // Number of RANSAC iterations

    // Set the number of RANSAC iterations based on the configuration
    void setRansacIter() {
      ransac_iter_ = static_cast<uint>(
        std::log(1.0 - config_.success_prob) /
        std::log(1.0 - std::pow(1.0 - config_.outlier_prob, config_.N_ransac_points))
      );
    }  
    // Perform RANSAC-based estimation
    bool solve3DFullRansac(
      const Eigen::MatrixXd& radar_data,
      const double& pitch,
      const double& roll,
      const double& yaw,
      bool is_holonomic,
      Eigen::Vector3d& v_r,
      Eigen::Vector3d& sigma_v_r,
      std::vector<uint>& inlier_idx_best,
      std::vector<uint>& outlier_idx_best,
      Eigen::Vector3d& w_r  // <- Add this parameter
    );

    // Solve for non-holonomic vehicles
    bool solve3DFull_not_holonomic(
      const Eigen::MatrixXd& radar_data,
      const double pitch,
      const double roll,
      const double yaw,
      Eigen::Vector3d& v_r
    );

    // Solve for holonomic vehicles
    bool solve3DFull_holonomic(
      const Eigen::MatrixXd& radar_data,
      const double pitch,
      const double roll,
      Eigen::Vector3d& v_r
    );

    // 6DoF estimator for non-holonomic vehicles
    bool solve6Dof_not_holonomic(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r);

    // 6DoF estimator for holonomic vehicles (same as above, but you may want to use a different model if needed)
    bool solve6Dof_holonomic(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r);
  };

  // Implementation of the estimate function
  bool RadarEgoVel::estimate(
    const sensor_msgs::msg::PointCloud2& radar_scan_msg,
    const double& pitch,
    const double& roll,
    const double& yaw,
    bool is_holonomic,
    Eigen::Vector3d& v_r,
    Eigen::Vector3d& sigma_v_r,
    sensor_msgs::msg::PointCloud2& inlier_radar_msg,
    sensor_msgs::msg::PointCloud2& outlier_radar_msg,
    Eigen::Vector3d& w_r) {

    // Create point clouds for radar data and inliers/outliers
    auto radar_scan = std::make_unique<pcl::PointCloud<RadarPointCloudType>>();
    auto radar_scan_inlier = std::make_unique<pcl::PointCloud<RadarPointCloudType>>();
    auto radar_scan_outlier = std::make_unique<pcl::PointCloud<RadarPointCloudType>>(); 
    bool success = false;

    // Convert ROS message to PCL point cloud
    pcl::fromROSMsg(radar_scan_msg, *radar_scan);

    std::vector<Vector11> valid_targets;

    std::cout << "[RadarEgoVel] Total radar points: " << radar_scan->size() << std::endl;

    // Filter and preprocess radar targets
    for (uint i = 0; i < radar_scan->size(); ++i) {
      const auto& target = radar_scan->at(i);
      const double r = Eigen::Vector3d(target.x, target.y, target.z).norm();

      double azimuth = std::atan2(target.y, target.x);
      double elevation = std::atan2(std::sqrt(target.x * target.x + target.y * target.y), target.z) - M_PI_2;

      // Apply thresholds and filters
      if (
        r > config_.min_dist &&
        r < config_.max_dist &&
        target.intensity > config_.min_db &&
        std::fabs(azimuth) < angles::from_degrees(config_.azimuth_thresh_deg) &&
        std::fabs(elevation) < angles::from_degrees(config_.elevation_thresh_deg)
      ) {
        Vector11 v_pt;
        v_pt << target.x, target.y, target.z,         // 0, 1, 2
                target.intensity,                     // 3
                -target.doppler * config_.doppler_velocity_correction_factor,                // 4
                r, azimuth, elevation,                // 5, 6, 7
                target.x / r, target.y / r, target.z / r; // 8, 9, 10
        valid_targets.emplace_back(v_pt);
      }
    }

    std::cout << "[RadarEgoVel] Valid targets after filtering: " << valid_targets.size() << std::endl;
    if (!valid_targets.empty()) {
      std::cout << "[RadarEgoVel] Example valid target: x=" << valid_targets[0][0]
                << " y=" << valid_targets[0][1]
                << " z=" << valid_targets[0][2]
                << " intensity=" << valid_targets[0][3]
                << " doppler=" << valid_targets[0][4] << std::endl;
    }

    double min_doppler = std::numeric_limits<double>::max();
    double max_doppler = std::numeric_limits<double>::lowest();
    double sum_doppler = 0.0;
    for (const auto& v_pt : valid_targets) {
        double d = v_pt[idx_.doppler];
        min_doppler = std::min(min_doppler, d);
        max_doppler = std::max(max_doppler, d);
        sum_doppler += d;
    }
    std::cout << "[RadarEgoVel] Doppler stats: min=" << min_doppler
              << " max=" << max_doppler
              << " mean=" << (sum_doppler / valid_targets.size()) << std::endl;

    // Proceed if we have enough valid targets
    if (valid_targets.size() > 2) {
      std::vector<double> v_dopplers;
      for (const auto& v_pt : valid_targets)
        v_dopplers.emplace_back(std::fabs(v_pt[idx_.doppler]));

      // Calculate median of Doppler velocities
      const size_t n = v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage);
      std::nth_element(v_dopplers.begin(), v_dopplers.begin() + n, v_dopplers.end());
      const auto median = v_dopplers[n];

      std::cout << "[RadarEgoVel] Median Doppler: " << median << std::endl;

      if (median < config_.thresh_zero_velocity) {
        std::cout << "[RadarEgoVel] Median Doppler below threshold, assuming zero velocity." << std::endl;
        // Assume zero velocity if median Doppler is below threshold
        v_r = Eigen::Vector3d(0, 0, 0);
        w_r = Eigen::Vector3d(0, 0, 0); // <-- Always set w_r to zero here
        sigma_v_r = Eigen::Vector3d(
          config_.sigma_zero_velocity_x,
          config_.sigma_zero_velocity_y,
          config_.sigma_zero_velocity_z
        );
        // Collect inliers
        for (const auto& item : valid_targets)
          if (std::fabs(item[idx_.doppler]) < config_.thresh_zero_velocity)
            radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
        success = true;
      } else {
        // Prepare radar data for RANSAC
        Eigen::MatrixXd radar_data(valid_targets.size(), 4);
        uint idx = 0;
        for (const auto& v_pt : valid_targets)
          radar_data.row(idx++) = Eigen::Vector4d(
            v_pt[idx_.normalized_x],
            v_pt[idx_.normalized_y],
            v_pt[idx_.normalized_z],
            v_pt[idx_.doppler]
          );
        std::vector<uint> inlier_idx_best;
        std::vector<uint> outlier_idx_best;
         // Perform RANSAC-based estimation
        std::cout << "[RadarEgoVel] Running RANSAC with " << valid_targets.size() << " targets." << std::endl;
        success = solve3DFullRansac(
          radar_data, pitch, roll, yaw, is_holonomic,
          v_r, sigma_v_r, inlier_idx_best, outlier_idx_best, w_r
        );
        std::cout << "[RadarEgoVel] RANSAC result: " << (success ? "success" : "failure") << std::endl;
        std::cout << "[RadarEgoVel] Inliers: " << inlier_idx_best.size() << ", Outliers: " << outlier_idx_best.size() << std::endl;

        // Collect inliers and outliers
        for (const auto& idx : inlier_idx_best)
          radar_scan_inlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
        for (const auto& idx : outlier_idx_best)
          radar_scan_outlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
        // NOTE: w_r is set by solve6Dof_* inside RANSAC
      }
    } else {
      std::cout << "[RadarEgoVel] Not enough valid targets for estimation." << std::endl;
      v_r = Eigen::Vector3d(0, 0, 0);
      w_r = Eigen::Vector3d(0, 0, 0); // <-- Always set w_r to zero here too
      sigma_v_r = Eigen::Vector3d(
        config_.sigma_zero_velocity_x,
        config_.sigma_zero_velocity_y,
        config_.sigma_zero_velocity_z
      );
    }

    // Prepare inlier point cloud message
    radar_scan_inlier->height = 1;
    radar_scan_inlier->width = radar_scan_inlier->size();
    pcl::PCLPointCloud2 tmp_inlier;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_inlier, tmp_inlier);
    pcl_conversions::fromPCL(tmp_inlier, inlier_radar_msg);
    inlier_radar_msg.header = radar_scan_msg.header;

    // Prepare outlier point cloud message
    radar_scan_outlier->height = 1;
    radar_scan_outlier->width = radar_scan_outlier->size();
    pcl::PCLPointCloud2 tmp_outlier;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_outlier, tmp_outlier);
    pcl_conversions::fromPCL(tmp_outlier, outlier_radar_msg);
    outlier_radar_msg.header = radar_scan_msg.header;

    return success;
  }
bool RadarEgoVel::solve3DFullRansac(const Eigen::MatrixXd& radar_data, const double& pitch, const double& roll, const double& yaw, bool is_holonomic, Eigen::Vector3d& v_r, Eigen::Vector3d& sigma_v_r, std::vector<uint>& inlier_idx_best, std::vector<uint>& outlier_idx_best, Eigen::Vector3d& w_r) {
    // Matrix with radar data
    Eigen::MatrixXd H_all(radar_data.rows(), 3);
    H_all.col(0) = radar_data.col(0);
    H_all.col(1) = radar_data.col(1);
    H_all.col(2) = radar_data.col(2);
    const Eigen::VectorXd y_all = radar_data.col(3);

    std::vector<uint> idx(radar_data.rows());
    for (uint k = 0; k < radar_data.rows(); ++k) idx[k] = k;
    
    std::random_device rd;
    std::mt19937 g(rd());

    // Only proceed if there are enough points to perform RANSAC
    if (radar_data.rows() >= config_.N_ransac_points) {
        for (uint k = 0; k < ransac_iter_; ++k) {
            std::shuffle(idx.begin(), idx.end(), g);
            Eigen::MatrixXd radar_data_iter;
            radar_data_iter.resize(config_.N_ransac_points, 4);

            for (uint i = 0; i < config_.N_ransac_points; ++i)
                radar_data_iter.row(i) = radar_data.row(idx.at(i));
            bool rtn = false;
            // Call to solve3DFull with the new equations
            if(!is_holonomic){
               rtn = solve3DFull_not_holonomic(radar_data_iter, pitch, roll, yaw, v_r);
            }else{
               rtn = solve3DFull_holonomic(radar_data_iter, pitch, roll, v_r);
            }
            
            if (rtn) {
                Eigen::VectorXd err(radar_data.rows());
                for (int i = 0; i < radar_data.rows(); ++i) {
                  if(!is_holonomic){
                    // Calculation of expected_vr using the new equations:
                   double expected_vr = v_r.x() * H_all(i, 0) * std::cos(pitch) * std::cos(yaw) +
                                   v_r.y() * H_all(i, 1) * std::cos(pitch) * std::sin(yaw) +
                                   v_r.z() * H_all(i, 2) * std::sin(pitch);
                    err(i) = std::abs(y_all(i) - expected_vr);
                    }else{
                    // Calculation of expected_vr using the new equations:
                    double expected_vr = H_all(i, 0) * std::cos(pitch) + 
                                         H_all(i, 1) * std::cos(roll) +  
                                         H_all(i, 2) * (std::sin(pitch) * std::sin(roll));
                    err(i) = std::abs(y_all(i) - expected_vr);
                    }
                }

                // Identification of inliers and outliers
                std::vector<uint> inlier_idx;
                std::vector<uint> outlier_idx;
                for (uint j = 0; j < err.rows(); ++j) {
                    if (err(j) < config_.inlier_thresh)
                        inlier_idx.emplace_back(j);
                    else
                        outlier_idx.emplace_back(j);
                }

                // Adjust to limit the proportion of outliers
                //if (float(outlier_idx.size()) / (inlier_idx.size() + outlier_idx.size()) > 0.05) {
                //    inlier_idx.insert(inlier_idx.end(), outlier_idx.begin(), outlier_idx.end());
                //    outlier_idx.clear();
                //}

                // Update the best inlier and outlier indices
                if (inlier_idx.size() > inlier_idx_best.size()) {
                    inlier_idx_best = inlier_idx;
                }
                if (outlier_idx.size() > outlier_idx_best.size()) {
                    outlier_idx_best = outlier_idx;
                }
            }
        }
    }

    // Solve using the final inliers
    if (!inlier_idx_best.empty()) {
        // Require at least 10% of points as inliers for a valid result
        if (inlier_idx_best.size() < std::max(10u, static_cast<uint>(radar_data.rows() * 0.1))) {
            std::cout << "[RadarEgoVel] Not enough inliers for a valid estimate: " << inlier_idx_best.size() << std::endl;
            return false;
        }
        Eigen::MatrixXd radar_data_inlier(inlier_idx_best.size(), 4);
        for (uint i = 0; i < inlier_idx_best.size(); ++i)
            radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));
        bool rtn = false;
        if(!is_holonomic){
            rtn = solve6Dof_not_holonomic(radar_data_inlier, v_r, w_r);
        }else{
            rtn = solve6Dof_holonomic(radar_data_inlier, v_r, w_r);
        }
        // Store or output w_r as needed
        return rtn;
    }

    return false;
}
bool RadarEgoVel::solve3DFull_not_holonomic(const Eigen::MatrixXd& radar_data, const double pitch, const double roll, const double yaw, Eigen::Vector3d& v_r) {
    // Initialize H_all with the first three columns of radar_data
    Eigen::MatrixXd H_all(radar_data.rows(), 3);
    H_all.col(0) = radar_data.col(0); 
    H_all.col(1) = radar_data.col(1);  
    H_all.col(2) = radar_data.col(2); 
    const Eigen::VectorXd y_all = radar_data.col(3);

    // Prepare matrices for solving the linear system
    Eigen::MatrixXd A(radar_data.rows(), 2);  
    Eigen::VectorXd b(radar_data.rows());

    // Populate A and b based on radar measurements and orientation angles
    for (int i = 0; i < H_all.rows(); ++i) {
        double term = H_all(i, 0) * std::cos(pitch) * std::cos(yaw) +  
                      H_all(i, 1) * std::cos(pitch) * std::sin(yaw) + 
                      H_all(i, 2) * std::sin(pitch);                  
        A(i, 0) = term;
        b(i) = y_all(i);
    }

    // Solve for velocity v using least squares
    double v = (A.transpose() * A).ldlt().solve(A.transpose() * b)(0);
   
    // Compute the velocity components in the x, y, and z directions
    v_r.x() = v * std::cos(pitch) * std::cos(yaw);  
    v_r.y() = v * std::cos(pitch) * std::sin(yaw);  
    v_r.z() = v * std::sin(pitch);                 

    return true;
}

bool RadarEgoVel::solve3DFull_holonomic(const Eigen::MatrixXd& radar_data, const double pitch, const double roll, Eigen::Vector3d& v_r) {
    // Define the rotation matrix for pitch (rotation around the y-axis)
    Eigen::Matrix3d R_pitch;
    R_pitch << std::cos(pitch), 0, std::sin(pitch),
               0, 1, 0,
               -std::sin(pitch), 0, std::cos(pitch);

    // Define the rotation matrix for roll (rotation around the x-axis)
    Eigen::Matrix3d R_roll;
    R_roll << 1, 0, 0,
              0, std::cos(roll), -std::sin(roll),
              0, std::sin(roll), std::cos(roll);

    // Compute the combined rotation matrix by applying roll and then pitch
    Eigen::Matrix3d R = R_roll * R_pitch;

    // Extract the normal vectors (H) and measurements (y) from radar_data
    Eigen::MatrixXd H_all = radar_data.leftCols(3); 
    Eigen::VectorXd y_all = radar_data.col(3);    

    // Rotate the normal vectors to align with the vehicle's orientation
    Eigen::MatrixXd H_transformed = H_all * R.transpose();

    // Solve the linear system H_transformed * v_r = y_all for the velocity vector v_r
    v_r = (H_transformed.transpose() * H_transformed).ldlt().solve(H_transformed.transpose() * y_all);

    return true;
}

// 6DoF estimator for non-holonomic vehicles
bool RadarEgoVel::solve6Dof_not_holonomic(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r) {
    int N = radar_data.rows();
    std::cout << "[RadarEgoVel] 6DoF solver starting with " << N << " points" << std::endl;
    
    Eigen::MatrixXd A(N, 6);
    Eigen::VectorXd b(N);

    for (int i = 0; i < N; ++i) {
        // n: normalized direction vector for Doppler projection
        Eigen::Vector3d r(radar_data(i, 0), radar_data(i, 1), radar_data(i, 2));
        Eigen::Vector3d n = r.normalized();  // Use normalized direction for Doppler
        
        // Skew-symmetric matrix for cross product
        Eigen::Matrix3d r_cross;
        r_cross << 0, -r(2), r(1),
                   r(2), 0, -r(0),
                   -r(1), r(0), 0;
        
        // Fill A: [n^T, n^T * r_cross]
        A.block<1,3>(i,0) = n.transpose();
        A.block<1,3>(i,3) = (n.transpose() * r_cross);
        
        b(i) = radar_data(i,3); // Doppler
    }

    // Add regularization for numerical stability
    Eigen::MatrixXd ATA = A.transpose() * A;
    double reg = 1e-2;  // Small regularization factor
    ATA.block<3,3>(3,3) += Eigen::Matrix3d::Identity() * reg;
    
    // Solve with regularization
    Eigen::VectorXd x = ATA.ldlt().solve(A.transpose() * b);
    v_r = x.segment<3>(0);
    w_r = x.segment<3>(3);
    
    std::cout << "[RadarEgoVel] 6DoF solution: v = [" << v_r.x() << ", " << v_r.y() << ", " << v_r.z() 
              << "], w = [" << w_r.x() << ", " << w_r.y() << ", " << w_r.z() << "]" << std::endl;
    return true;
}

// 6DoF estimator for holonomic vehicles (same as above, but you may want to use a different model if needed)
bool RadarEgoVel::solve6Dof_holonomic(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r) {
    return solve6Dof_not_holonomic(radar_data, v_r, w_r);
}

}
