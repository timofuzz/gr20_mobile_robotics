#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Eigen/Dense"
#include <opencv2/core/core.hpp>
#include "rio_utils/radar_point_cloud.hpp"
#include "RadarEgoVel.hpp"
#include <chrono>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// RadarProcessor class handles point cloud processing, transformation between sensor frames,
// and ego-velocity estimation using radar data.
class RadarProcessor : public rclcpp::Node {
public:
    RadarProcessor() : Node("radar_pcl_processor") {
        // Declare all parameters with sensible defaults
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("radar_topic", "/hugin_raf_1/radar_data");
        this->declare_parameter<bool>("enable_dynamic_object_removal", true);
        this->declare_parameter<bool>("holonomic_vehicle", true);
        this->declare_parameter<int>("intensity_channel", 2);
        this->declare_parameter<int>("doppler_channel", 0);
        this->declare_parameter<double>("distance_near_thresh", 0.1);
        this->declare_parameter<double>("distance_far_thresh", 80.0);
        this->declare_parameter<double>("z_low_thresh", -40.0);
        this->declare_parameter<double>("z_high_thresh", 100.0);
        this->declare_parameter<double>("min_dist", 0.5);
        this->declare_parameter<double>("max_dist", 400.0);
        this->declare_parameter<double>("min_db", 5.0);
        this->declare_parameter<double>("elevation_thresh_deg", 50.0);
        this->declare_parameter<double>("azimuth_thresh_deg", 56.5);
        this->declare_parameter<double>("doppler_velocity_correction_factor", 1.0);
        this->declare_parameter<double>("thresh_zero_velocity", 0.05);
        this->declare_parameter<double>("allowed_outlier_percentage", 0.30);
        this->declare_parameter<double>("sigma_zero_velocity_x", 1.0e-03);
        this->declare_parameter<double>("sigma_zero_velocity_y", 3.2e-03);
        this->declare_parameter<double>("sigma_zero_velocity_z", 1.0e-02);
        this->declare_parameter<double>("sigma_offset_radar_x", 0.0);
        this->declare_parameter<double>("sigma_offset_radar_y", 0.0);
        this->declare_parameter<double>("sigma_offset_radar_z", 0.0);
        this->declare_parameter<double>("max_sigma_x", 0.2);
        this->declare_parameter<double>("max_sigma_y", 0.2);
        this->declare_parameter<double>("max_sigma_z", 0.2);
        this->declare_parameter<double>("max_r_cond", 0.2);
        this->declare_parameter<double>("outlier_prob", 0.05);
        this->declare_parameter<double>("success_prob", 0.995);
        this->declare_parameter<double>("N_ransac_points", 5.0);
        this->declare_parameter<double>("inlier_thresh", 0.5);
        // ...declare any other parameters you use...

        getParams();
        setupSubscribersAndPublishers();
        initializeTransformation();
    }

private:
    // Function to load parameters from the ROS2 parameter server.
    void getParams() {
        rio::RadarEgoVelocityEstimatorConfig config;

        // Retrieve ROS parameters
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("radar_topic", radar_topic_);
        this->get_parameter("enable_dynamic_object_removal", enable_dynamic_object_removal_);
        this->get_parameter("holonomic_vehicle", holonomic_vehicle_);
        this->get_parameter("distance_near_thresh", distance_near_thresh_);
        this->get_parameter("distance_far_thresh", distance_far_thresh_);
        this->get_parameter("z_low_thresh", z_low_thresh_);
        this->get_parameter("z_high_thresh", z_high_thresh_);

        // Retrieve estimator configuration parameters
        this->get_parameter("min_dist", config.min_dist);
        this->get_parameter("max_dist", config.max_dist);
        this->get_parameter("min_db", config.min_db);
        this->get_parameter("elevation_thresh_deg", config.elevation_thresh_deg);
        this->get_parameter("azimuth_thresh_deg", config.azimuth_thresh_deg);
        this->get_parameter("doppler_velocity_correction_factor", config.doppler_velocity_correction_factor);
        this->get_parameter("thresh_zero_velocity", config.thresh_zero_velocity);
        this->get_parameter("allowed_outlier_percentage", config.allowed_outlier_percentage);
        this->get_parameter("sigma_zero_velocity_x", config.sigma_zero_velocity_x);
        this->get_parameter("sigma_zero_velocity_y", config.sigma_zero_velocity_y);
        this->get_parameter("sigma_zero_velocity_z", config.sigma_zero_velocity_z);
        this->get_parameter("sigma_offset_radar_x", config.sigma_offset_radar_x);
        this->get_parameter("sigma_offset_radar_y", config.sigma_offset_radar_y);
        this->get_parameter("sigma_offset_radar_z", config.sigma_offset_radar_z);
        this->get_parameter("max_sigma_x", config.max_sigma_x);
        this->get_parameter("max_sigma_y", config.max_sigma_y);
        this->get_parameter("max_sigma_z", config.max_sigma_z);
        this->get_parameter("max_r_cond", config.max_r_cond);
        this->get_parameter("use_cholesky_instead_of_bdcsvd", config.use_cholesky_instead_of_bdcsvd);
        this->get_parameter("use_ransac", config.use_ransac);
        this->get_parameter("outlier_prob", config.outlier_prob);
        this->get_parameter("success_prob", config.success_prob);
        this->get_parameter("N_ransac_points", config.N_ransac_points);
        this->get_parameter("inlier_thresh", config.inlier_thresh);

        // Initialize the radar ego-velocity estimator with the retrieved configuration
        estimator_ = std::make_shared<rio::RadarEgoVel>(config);
    }

    // Set up ROS2 subscribers and publishers for IMU and radar point clouds.
    void setupSubscribersAndPublishers() {
        // Subscribe to IMU data
        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                imuCallback(msg);
            });

        // Subscribe to radar point cloud data
        radar_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            radar_topic_, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                cloudCallback(msg);
            });

        // Publishers for processed data
        twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/ego_vel_twist", 5);
        radar_filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
        inlier_pc2_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/inlier_pointcloud", 5);
        outlier_pc2_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_pointcloud", 5);
        raw_pc2_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/raw_pointcloud", 10);
    }

    // Initialize transformation matrices between different sensor frames
    void initializeTransformation() {
        // Transformation from Livox to RGB frame (change if needed)
        livox_to_rgb_ = (cv::Mat_<double>(4,4) <<
            -0.006878330000, -0.999969000000, 0.003857230000, 0.029164500000,
            -7.737180000000E-05, -0.003856790000, -0.999993000000, 0.045695200000,
            0.999976000000, -0.006878580000, -5.084110000000E-05, -0.19018000000,
            0, 0, 0, 1);
        rgb_to_livox_ = livox_to_rgb_.inv();

        thermal_to_rgb_ = (cv::Mat_<double>(4,4) <<
            0.9999526089706319, 0.008963747151337641, -0.003798822163962599, 0.18106962419014,
            -0.008945181135788245, 0.9999481006917174, 0.004876439015823288, -0.04546324090016857,
            0.00384233617405678, -0.004842226763999368, 0.999980894463835, 0.08046453079998771,
            0, 0, 0, 1);

        radar_to_thermal_ = (cv::Mat_<double>(4,4) <<
            0.999665, 0.00925436, -0.0241851, -0.0248342,
            -0.00826999, 0.999146, 0.0404891, 0.0958317,
            0.0245392, -0.0402755, 0.998887, 0.0268037,
            0, 0, 0, 1);

        change_radar_frame_ = (cv::Mat_<double>(4,4) <<
            0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1);

        // Compute the transformation from radar to Livox frame
        radar_to_livox_ = rgb_to_livox_ * thermal_to_rgb_ * radar_to_thermal_ * change_radar_frame_;
    }

    // IMU callback for processing orientation and updating quaternions (change if needed)
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        // Convert IMU orientation to Eigen quaternion
        Eigen::Quaterniond q_ahrs(
            imu_msg->orientation.w,
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z);

        // Apply rotation adjustments
        Eigen::Quaterniond q_r =
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_rr =
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q_out = q_r * q_ahrs * q_rr;

        // Update the current orientation quaternion
        q_current_ = tf2::Quaternion(q_out.x(), q_out.y(), q_out.z(), q_out.w());
    }

    // Radar point cloud callback for filtering and processing radar data
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
        // Convert PointCloud2 to pcl::PointCloud, then process as before
        sensor_msgs::msg::PointCloud2 msg_fixed = *pcl_msg;
        for (auto &field : msg_fixed.fields) {
            if (field.name == "power") {
                field.name = "intensity";
            }
        }

        pcl::PointCloud<RadarPointCloudType>::Ptr radar_cloud_raw(new pcl::PointCloud<RadarPointCloudType>);
        pcl::fromROSMsg(msg_fixed, *radar_cloud_raw);

        // Publish the raw radar point cloud data
        sensor_msgs::msg::PointCloud2 pc2_raw_msg;
        pcl::toROSMsg(*radar_cloud_raw, pc2_raw_msg);
        pc2_raw_msg.header.stamp = pcl_msg->header.stamp;
        pc2_raw_msg.header.frame_id = "base_link";
        raw_pc2_publisher_->publish(pc2_raw_msg);

        // Estimate ego velocity based on the radar data
        Eigen::Vector3d v_radar, sigma_v_radar, w_radar; // <-- Add w_radar
        sensor_msgs::msg::PointCloud2 inlier_radar_msg, outlier_radar_msg;

        if (initialization_) {
            q_previous_ = q_current_;
            initialization_ = false;
        }

        // Calculate rotation between previous and current orientations
        q_rotation_ = q_previous_.inverse() * q_current_;
        tf2::Matrix3x3 q_rotation_matrix(q_rotation_.normalize());
        double roll, pitch, yaw;
        q_rotation_matrix.getRPY(roll, pitch, yaw);

        // Estimate ego velocity using the radar ego-velocity estimator
        //RCLCPP_INFO(this->get_logger(), "Calling estimator...");
        // Update this call to pass w_radar if your estimator supports it
        bool ok = estimator_->estimate(pc2_raw_msg, pitch, roll, yaw, holonomic_vehicle_, v_radar, sigma_v_radar, inlier_radar_msg, outlier_radar_msg, w_radar);
        // If your estimate() does not yet take w_radar, update its signature and implementation accordingly

        //RCLCPP_INFO(this->get_logger(), "Estimator returned: %d", ok);
        if (ok) {
            // Publish the estimated twist
            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = pc2_raw_msg.header.stamp;
            twist_msg.twist.linear.x = v_radar.x();
            twist_msg.twist.linear.y = v_radar.y();
            twist_msg.twist.linear.z = v_radar.z();
            twist_msg.twist.angular.x = w_radar.x();
            twist_msg.twist.angular.y = w_radar.y();
            twist_msg.twist.angular.z = w_radar.z();
            twist_publisher_->publish(twist_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Velocity estimation failed.");
        }

        // Process inlier radar point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_cloud_inlier(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_cloud_raw_(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(inlier_radar_msg, *radar_cloud_inlier);
        pcl::fromROSMsg(pc2_raw_msg, *radar_cloud_raw_);

        // Choose source cloud based on dynamic object removal flag
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr source_cloud;
        if (enable_dynamic_object_removal_) {
            source_cloud = radar_cloud_inlier;
        } else {
            source_cloud = radar_cloud_raw_;
        }

        if (source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Source cloud is empty. Skipping.");
            return;
        }

        // Apply distance and height filtering to the source cloud
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr filtered_cloud = distanceFilter(source_cloud);
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header.stamp = pc2_raw_msg.header.stamp;
        filtered_cloud_msg.header.frame_id = "base_link";
        radar_filtered_publisher_->publish(filtered_cloud_msg);

        // Update previous orientation
        q_previous_ = q_current_;
    }

    // Distance and height filter for point clouds
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        filtered->reserve(cloud->size());

        std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const pcl::PointXYZI& p) {
            double distance = p.getVector3fMap().norm();
            double z = p.z;
            return distance > distance_near_thresh_ && distance < distance_far_thresh_ && z < z_high_thresh_ && z > z_low_thresh_;
        });

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;
        filtered->header = cloud->header;

        return filtered;
    }

    // Member variables
    std::string imu_topic_;
    std::string radar_topic_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_filtered_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_pc2_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr outlier_pc2_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc2_publisher_;

    cv::Mat radar_to_livox_, thermal_to_rgb_, radar_to_thermal_, rgb_to_livox_, livox_to_rgb_, change_radar_frame_;
    std::shared_ptr<rio::RadarEgoVel> estimator_;

    tf2::Quaternion q_previous_;
    tf2::Quaternion q_current_;
    tf2::Quaternion q_rotation_;

    bool initialization_ = true;
    bool enable_dynamic_object_removal_;
    bool holonomic_vehicle_;
    double distance_near_thresh_;
    double distance_far_thresh_;
    double z_low_thresh_;
    double z_high_thresh_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}