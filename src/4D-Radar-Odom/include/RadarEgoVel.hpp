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

    // 2D planar model for estimating [vx, vy, ωz]
    bool solve2DPlanar(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r);
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

    // Convert ROS message to PCL point cloud
    auto radar_scan = std::make_unique<pcl::PointCloud<RadarPointCloudType>>();
    pcl::fromROSMsg(radar_scan_msg, *radar_scan);

    std::vector<Vector11> all_targets;

    //std::cout << "[RadarEgoVel] Total radar points: " << radar_scan->size() << std::endl;

    // Use all radar targets without filtering
    for (uint i = 0; i < radar_scan->size(); ++i) {
        const auto& target = radar_scan->at(i);
        double r = Eigen::Vector3d(target.x, target.y, target.z).norm();

        Vector11 v_pt;
        v_pt << target.x, target.y, target.z,         // 0, 1, 2
                target.intensity,                     // 3
                -target.doppler * config_.doppler_velocity_correction_factor, // 4
                r,                                   // 5
                0.0, 0.0,                            // 6, 7 (azimuth, elevation unused)
                target.x / r, target.y / r, target.z / r; // 8, 9, 10
        all_targets.emplace_back(v_pt);
    }

    //std::cout << "[RadarEgoVel] Using all targets: " << all_targets.size() << std::endl;
    //if (!all_targets.empty()) {
     //   std::cout << "[RadarEgoVel] Example target: x=" << all_targets[0][0]
     //             << " y=" << all_targets[0][1]
     //             << " z=" << all_targets[0][2]
     //             << " intensity=" << all_targets[0][3]
     //             << " doppler=" << all_targets[0][4] << std::endl;
    //}

    // Prepare radar data for estimation
    Eigen::MatrixXd radar_data(all_targets.size(), 4);
    uint idx = 0;
    for (const auto& v_pt : all_targets)
        radar_data.row(idx++) = Eigen::Vector4d(
            v_pt[idx_.normalized_x],
            v_pt[idx_.normalized_y],
            v_pt[idx_.normalized_z],
            v_pt[idx_.doppler]
        );

    // Run estimation if enough points
    bool success = false;
    if (all_targets.size() > 2) {
        std::vector<uint> inlier_idx_best, outlier_idx_best;
        success = solve3DFullRansac(
            radar_data, pitch, roll, yaw, is_holonomic,
            v_r, sigma_v_r, inlier_idx_best, outlier_idx_best, w_r
        );
        //std::cout << "[RadarEgoVel] RANSAC result: " << (success ? "success" : "failure") << std::endl;
    } else {
        //std::cout << "[RadarEgoVel] Not enough points for estimation." << std::endl;
        v_r = Eigen::Vector3d(0, 0, 0);
        w_r = Eigen::Vector3d(0, 0, 0);
        sigma_v_r = Eigen::Vector3d(
            config_.sigma_zero_velocity_x,
            config_.sigma_zero_velocity_y,
            config_.sigma_zero_velocity_z
        );
    }

    // Output all points as inliers (since no filtering)
    auto radar_scan_inlier = std::make_unique<pcl::PointCloud<RadarPointCloudType>>(*radar_scan);
    radar_scan_inlier->height = 1;
    radar_scan_inlier->width = radar_scan_inlier->size();
    pcl::PCLPointCloud2 tmp_inlier;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_inlier, tmp_inlier);
    pcl_conversions::fromPCL(tmp_inlier, inlier_radar_msg);
    inlier_radar_msg.header = radar_scan_msg.header;

    // No outliers
    auto radar_scan_outlier = std::make_unique<pcl::PointCloud<RadarPointCloudType>>();
    radar_scan_outlier->height = 1;
    radar_scan_outlier->width = 0;
    pcl::PCLPointCloud2 tmp_outlier;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_outlier, tmp_outlier);
    pcl_conversions::fromPCL(tmp_outlier, outlier_radar_msg);
    outlier_radar_msg.header = radar_scan_msg.header;

    return success;
  }

bool RadarEgoVel::solve3DFullRansac(
    const Eigen::MatrixXd& radar_data,
    const double& pitch,
    const double& roll,
    const double& yaw,
    bool is_holonomic,
    Eigen::Vector3d& v_r,
    Eigen::Vector3d& sigma_v_r,
    std::vector<uint>& inlier_idx_best,
    std::vector<uint>& outlier_idx_best,
    Eigen::Vector3d& w_r)
{
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
            bool rtn = solve3DFull_holonomic(radar_data_iter, pitch, roll, v_r);
            
            if (rtn) {
                Eigen::VectorXd err(radar_data.rows());
                for (int i = 0; i < radar_data.rows(); ++i) {
                    // Calculation of expected_vr using the holonomic model:
                    double expected_vr = H_all(i, 0) * std::cos(pitch) + 
                                         H_all(i, 1) * std::cos(roll) +  
                                         H_all(i, 2) * (std::sin(pitch) * std::sin(roll));
                    err(i) = std::abs(y_all(i) - expected_vr);
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
            return false;
        }
        Eigen::MatrixXd radar_data_inlier(inlier_idx_best.size(), 4);
        for (uint i = 0; i < inlier_idx_best.size(); ++i)
            radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));
        
        // Choose between 3D holonomic or 2D planar with rotation
        bool use_planar_model = true;  // Flag to enable planar model with rotation
        
        bool rtn;
        if (use_planar_model) {
            // Use 2D planar model to get rotation
            rtn = solve2DPlanar(radar_data_inlier, v_r, w_r);
            
            // Apply coordinate transformation (same as before)
            double temp = v_r.x();
            v_r.x() = v_r.y();  // Y becomes X
            v_r.y() = -temp;    // X becomes -Y (for 90° clockwise rotation)
            
            // No need to swap z component as it's zero
            
            // Also swap angular velocities appropriately
            temp = w_r.x();
            w_r.x() = w_r.y();
            w_r.y() = -temp;
            // w_r.z remains the same (rotation around vertical axis)
        } else {
            // Use original 3D holonomic model without rotation
            rtn = solve3DFull_holonomic(radar_data_inlier, pitch, roll, v_r);
            
            // Apply coordinate transformation
            double temp = v_r.x();
            v_r.x() = v_r.y();
            v_r.y() = -temp;
            
            // No rotation estimation
            w_r = Eigen::Vector3d::Zero();
        }
        
        return rtn;
    }

    return false;
}

// Solve for non-holonomic vehicles
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

// Solve for holonomic vehicles
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

// 2D planar model for estimating [vx, vy, ωz]
bool RadarEgoVel::solve2DPlanar(const Eigen::MatrixXd& radar_data, Eigen::Vector3d& v_r, Eigen::Vector3d& w_r) {
    int N = radar_data.rows();
    if (N < 3) return false;

    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b = radar_data.col(3); // Doppler measurements

    // Extract normalized direction vectors
    Eigen::VectorXd nx = radar_data.col(0);
    Eigen::VectorXd ny = radar_data.col(1);
    
    // Use a balanced perpendicular scale
    double perp_scale = 8.0;
    
    for (int i = 0; i < N; ++i) {
        // Fill linear velocity components
        A(i, 0) = nx(i);
        A(i, 1) = ny(i);
        
        // Create orthogonal lever arm (perpendicular to direction)
        double x_perp = -ny(i) * perp_scale;  // Perpendicular x (90° rotated)
        double y_perp = nx(i) * perp_scale;   // Perpendicular y (90° rotated)
        
        // Rotation term: v_tangential = ω × r
        A(i, 2) = -y_perp*nx(i) + x_perp*ny(i);
    }

    // Use regularization to stabilize the solution
    Eigen::MatrixXd reg = Eigen::MatrixXd::Zero(3, 3);
    reg(2, 2) = 0.2;  // Keep regularization for stability
    
    // Solve the system
    Eigen::Vector3d result = (A.transpose() * A + reg).ldlt().solve(A.transpose() * b);
    
    // Extract results
    v_r.x() = result(0);
    v_r.y() = result(1);
    v_r.z() = 0.0;
    
    w_r.x() = 0.0;
    w_r.y() = 0.0;
    
    // Calculate linear speed (for speed-dependent scaling)
    double speed = std::sqrt(v_r.x()*v_r.x() + v_r.y()*v_r.y());
    
    // Inverse speed factor: higher at low speeds, lower at high speeds
    double speed_factor = 1.0;
    if (speed > 0.01) {  // Only apply scaling above 0.5 m/s
        // Base scale factor of 0.3
        // At 1 m/s: factor = 0.3 * (1.5/1.0) = 0.45
        // At 5 m/s: factor = 0.3 * (1.5/5.0) = 0.09
        double reference_speed = 1.5;  // Reference speed for normalization
        speed_factor = 0.3 * (reference_speed / speed);
        
        // Clamp to reasonable range
        speed_factor = std::min(speed_factor, 2.0);  // Cap max scale
        speed_factor = std::max(speed_factor, 0.1);  // Ensure minimum scale
    } else {
        // At very low speeds, use the default scaling factor
        speed_factor = 0.3;
    }
    
    // Apply speed-dependent scaling to angular velocity
    w_r.z() = result(2) * speed_factor;
    
    return true;
}

}
