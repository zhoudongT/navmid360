#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class ICPNode : public rclcpp::Node
{
public:
    ICPNode()
        : Node("icp_node")
    {
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("fitness_score_thre", 0.0);
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type","livox");
        this->declare_parameter("icp_planar_constraint", true);
        this->declare_parameter("gravity_align_enable", true);
        this->declare_parameter("imu_topic", "/livox/imu");
        this->declare_parameter("scan_azimuth_offset_deg", 0.0);
        this->declare_parameter("icp_crop_min_z", -0.8);
        this->declare_parameter("icp_crop_max_z", 3.0);
        this->declare_parameter("icp_max_horizontal_range", 60.0);
        this->declare_parameter("icp_coarse_yaw_search_enable", true);
        this->declare_parameter("icp_coarse_yaw_half_range_deg", 30.0);
        this->declare_parameter("icp_coarse_yaw_step_deg", 3.0);
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("publish_map_to_odom_tf", false);
        this->declare_parameter("continuous_tracking", false);
        this->declare_parameter("icp_result_publish_interval", 15);
        this->declare_parameter("use_ndt_prefit", true);
        this->declare_parameter("ndt_resolution", 1.0);
        this->declare_parameter("ndt_step_size", 0.1);
        this->declare_parameter("ndt_max_iter", 35);

        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("RANSAC_outlier_rejection_threshold", RANSAC_outlier_rejection_threshold);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("fitness_score_thre", fitness_score_thre);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("icp_planar_constraint", icp_planar_constraint_);
        this->get_parameter("gravity_align_enable", gravity_align_enable_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("scan_azimuth_offset_deg", scan_azimuth_offset_deg_);
        this->get_parameter("icp_crop_min_z", icp_crop_min_z_);
        this->get_parameter("icp_crop_max_z", icp_crop_max_z_);
        this->get_parameter("icp_max_horizontal_range", icp_max_horizontal_range_);
        this->get_parameter("icp_coarse_yaw_search_enable", icp_coarse_yaw_search_enable_);
        this->get_parameter("icp_coarse_yaw_half_range_deg", icp_coarse_yaw_half_range_deg_);
        this->get_parameter("icp_coarse_yaw_step_deg", icp_coarse_yaw_step_deg_);
        this->get_parameter("odom_frame_id", odom_frame_id_);
        this->get_parameter("publish_map_to_odom_tf", publish_map_to_odom_tf_);
        this->get_parameter("continuous_tracking", continuous_tracking_);
        this->get_parameter("icp_result_publish_interval", icp_result_publish_interval_);
        this->get_parameter("use_ndt_prefit", use_ndt_prefit_);
        this->get_parameter("ndt_resolution", ndt_resolution_);
        this->get_parameter("ndt_step_size", ndt_step_size_);
        this->get_parameter("ndt_max_iter", ndt_max_iter_);

        if (publish_map_to_odom_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            RCLCPP_INFO(this->get_logger(), "icp_node will publish map->odom TF");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "icp_node: map->odom TF disabled (use tf_manager /relocalization/pose)");
        }

        if (gravity_align_enable_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                std::bind(&ICPNode::imu_callback, this, std::placeholders::_1));
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
#ifdef USE_LIVOX
        if(pcl_type == "livox")
        {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&ICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        }
        else
        {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);
        
        // init guess
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_a);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", initial_x, initial_y, initial_z, initial_a);
        // Load the target point cloud from a PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *target_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file target.pcd");
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from target.pcd", target_cloud_->width * target_cloud_->height);

        // downsample the target cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_map;
        sor_map.setInputCloud(target_cloud_);
        sor_map.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
        sor_map.filter(*target_cloud_);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud to %d data points", target_cloud_->width * target_cloud_->height);
        // Publish the downsampled target cloud

        pcl::toROSMsg(*target_cloud_, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        map_pub_->publish(target_cloud_msg);
        if (publish_map_to_odom_tf_) {
            publish_map_to_odom_tf(initGuess);
        }
    }

private:
    /** 地面车 + 水平先验图：只保留 x/y/z/yaw，去掉 ICP 对朝天雷达误估的 roll/pitch */
    Eigen::Matrix4f constrain_planar(const Eigen::Matrix4f & t) const
    {
        if (!icp_planar_constraint_) {
            return t;
        }
        Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
        const float yaw = std::atan2(t(1, 0), t(0, 0));
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        out(0, 0) = c;
        out(0, 1) = -s;
        out(1, 0) = s;
        out(1, 1) = c;
        out(0, 3) = t(0, 3);
        out(1, 3) = t(1, 3);
        out(2, 3) = t(2, 3);
        return out;
    }

    static Eigen::Matrix4f rotation_yaw(float yaw)
    {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        m(0, 0) = c;
        m(0, 1) = -s;
        m(1, 0) = s;
        m(1, 1) = c;
        return m;
    }

    void publish_map_to_odom_tf(const Eigen::Matrix4f & t)
    {
        if (!publish_map_to_odom_tf_ || !tf_broadcaster_) {
            return;
        }
        const Eigen::Matrix4f planar = constrain_planar(t);
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame;
        msg.child_frame_id = odom_frame_id_;
        msg.transform.translation.x = planar(0, 3);
        msg.transform.translation.y = planar(1, 3);
        msg.transform.translation.z = planar(2, 3);
        const Eigen::Matrix3f rot = planar.block<3, 3>(0, 0);
        const Eigen::Quaternionf q(rot);
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_quat_ = Eigen::Quaternionf(
            static_cast<float>(msg->orientation.w),
            static_cast<float>(msg->orientation.x),
            static_cast<float>(msg->orientation.y),
            static_cast<float>(msg->orientation.z));
        if (imu_quat_.norm() > 1e-6f) {
            imu_quat_.normalize();
            have_imu_ = true;
        }
    }

    Eigen::Matrix3f leveling_rotation() const
    {
        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
        if (gravity_align_enable_ && have_imu_) {
            const Eigen::Matrix3f r_imu = imu_quat_.toRotationMatrix();
            const float yaw = std::atan2(r_imu(1, 0), r_imu(0, 0));
            r = (Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()) * imu_quat_).toRotationMatrix().transpose();
        }
        if (std::abs(scan_azimuth_offset_deg_) > 1e-4) {
            const float a = static_cast<float>(scan_azimuth_offset_deg_ * M_PI / 180.0);
            r = Eigen::AngleAxisf(a, Eigen::Vector3f::UnitZ()).toRotationMatrix() * r;
        }
        return r;
    }

    void preprocess_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
    {
        const Eigen::Matrix3f r_level = leveling_rotation();
        pcl::PointCloud<pcl::PointXYZ> filtered;
        filtered.reserve(cloud->size());
        for (const auto & p : cloud->points) {
            const Eigen::Vector3f v =
                r_level * Eigen::Vector3f(p.x, p.y, p.z);
            if (v.z() < static_cast<float>(icp_crop_min_z_) ||
                v.z() > static_cast<float>(icp_crop_max_z_)) {
                continue;
            }
            if (v.head<2>().norm() > static_cast<float>(icp_max_horizontal_range_)) {
                continue;
            }
            pcl::PointXYZ out;
            out.x = v.x();
            out.y = v.y();
            out.z = v.z();
            filtered.push_back(out);
        }
        *cloud = filtered;
        cloud->width = cloud->size();
        cloud->height = 1;
    }

    struct IcpAlignResult
    {
        double fitness = 1e9;
        bool converged = false;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    };

    IcpAlignResult run_ndt_once(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
        const Eigen::Matrix4f & guess) const
    {
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setInputSource(source);
        ndt.setInputTarget(target_cloud_);
        ndt.setResolution(static_cast<float>(ndt_resolution_));
        ndt.setStepSize(static_cast<float>(ndt_step_size_));
        ndt.setMaximumIterations(ndt_max_iter_);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        ndt.align(aligned, guess);
        IcpAlignResult out;
        out.fitness = ndt.getFitnessScore();
        out.converged = ndt.hasConverged();
        if (out.converged) {
            out.transform = constrain_planar(ndt.getFinalTransformation());
        }
        return out;
    }

    IcpAlignResult run_icp_once(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
        const Eigen::Matrix4f & guess) const
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned, guess);
        IcpAlignResult out;
        out.fitness = icp.getFitnessScore();
        out.converged = icp.hasConverged();
        if (out.converged) {
            out.transform = constrain_planar(icp.getFinalTransformation());
        }
        return out;
    }

    IcpAlignResult run_icp_with_yaw_search(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
        const Eigen::Matrix4f & guess,
        bool allow_yaw_search)
    {
        Eigen::Matrix4f refined_guess = guess;
        if (use_ndt_prefit_ && manual_pose_set_) {
            const IcpAlignResult ndt_out = run_ndt_once(source, guess);
            if (ndt_out.converged) {
                refined_guess = ndt_out.transform;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000,
                    "NDT prefit fitness=%.4f", ndt_out.fitness);
            }
        }
        IcpAlignResult best = run_icp_once(source, refined_guess);
        if (!allow_yaw_search || !icp_coarse_yaw_search_enable_ ||
            best.fitness < fitness_score_thre) {
            return best;
        }
        const float base_yaw = std::atan2(guess(1, 0), guess(0, 0));
        const float half = static_cast<float>(icp_coarse_yaw_half_range_deg_ * M_PI / 180.0);
        const float step_deg = static_cast<float>(icp_coarse_yaw_step_deg_);
        const float step = std::max(step_deg, 0.5f) * static_cast<float>(M_PI / 180.0);
        for (float dy = -half; dy <= half + 1e-6f; dy += step) {
            Eigen::Matrix4f g = guess;
            g.block<3, 3>(0, 0) = rotation_yaw(base_yaw + dy).block<3, 3>(0, 0);
            const IcpAlignResult trial = run_icp_once(source, g);
            if (trial.converged && trial.fitness < best.fitness) {
                best = trial;
            }
        }
        if (best.converged) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "ICP yaw search best fitness=%.4f yaw(deg)=%.2f",
                best.fitness,
                std::atan2(best.transform(1, 0), best.transform(0, 0)) * 180.0 / M_PI);
        }
        return best;
    }

    void handle_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        preprocess_scan(input_cloud);
        if (input_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "ICP scan empty after gravity crop (imu=%s)", have_imu_ ? "ok" : "missing");
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Downsampled input cloud to %zu data points", input_cloud->size());

        if (pose_published_ && !continuous_tracking_) {
            return;
        }

        scan_frame_count_++;
        const bool allow_yaw_search =
            (scan_frame_count_ % 3 == 0) || !have_imu_;
        const IcpAlignResult icp_result =
            run_icp_with_yaw_search(input_cloud, initGuess, allow_yaw_search);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "ICP fitness score: %.4f (yaw_search=%s)", icp_result.fitness,
            allow_yaw_search ? "on" : "off");

        if (icp_result.converged && icp_result.fitness < fitness_score_thre) {
            converged_count++;
            initGuess = icp_result.transform;
            if (converged_count < converged_count_thre) {
                publish_transformed_scan(input_cloud, initGuess);
                target_cloud_msg.header.stamp = this->now();
                map_pub_->publish(target_cloud_msg);
                return;
            }
            if (!manual_pose_set_) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "ICP aligned (fitness=%.3f). Set 2D Pose in RViz to start localization.",
                    icp_result.fitness);
                publish_transformed_scan(input_cloud, initGuess);
                return;
            }
            if (continuous_tracking_) {
                const int interval = std::max(1, icp_result_publish_interval_);
                if (!pose_published_ || (scan_frame_count_ % interval == 0)) {
                    publish_icp_pose(initGuess, input_cloud, !pose_published_);
                }
                publish_transformed_scan(input_cloud, initGuess);
                return;
            }
            RCLCPP_INFO(
                this->get_logger(), "ICP converged!!! fitness=%f count=%d",
                icp_result.fitness, converged_count);
            publish_icp_pose(initGuess, input_cloud, true);
            return;
        }
        converged_count = 0;
        initGuess = icp_result.converged ? icp_result.transform : initGuess;
        publish_transformed_scan(input_cloud, constrain_planar(initGuess));
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        handle_scan(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        input_cloud->reserve(msg->point_num);
        for (size_t i = 0; i < msg->point_num; i++) {
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;
        handle_scan(input_cloud);
    }
#endif

    void publish_transformed_scan(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
        const Eigen::Matrix4f & transform)
    {
        const Eigen::Matrix4f t = constrain_planar(transform);
        pcl::PointCloud<pcl::PointXYZ> vis = *input_cloud;
        pcl::transformPointCloud(vis, vis, t);
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(vis, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = this->now();
        transformed_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(transformed_cloud_msg);
    }

    void publish_icp_pose(const Eigen::Matrix4f & transformation_result,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
        bool log_once)
    {
        const Eigen::Matrix4f t = constrain_planar(transformation_result);
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame;
        pose_msg.pose.pose.position.x = t(0, 3);
        pose_msg.pose.pose.position.y = t(1, 3);
        pose_msg.pose.pose.position.z = t(2, 3);
        Eigen::Matrix3f rotation = t.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        publisher_->publish(pose_msg);
        pose_published_ = true;
        if (publish_map_to_odom_tf_) {
            publish_map_to_odom_tf(t);
        }
        publish_transformed_scan(input_cloud, t);
        if (log_once) {
            RCLCPP_INFO(this->get_logger(), "Published /icp_result (ScanContext+NDT/ICP lock)");
        } else {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Published /icp_result (continuous tracking)");
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convert the incoming pose to an Eigen matrix
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        double r, p, yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
        converged_count = 0;
        pose_published_ = false;
        manual_pose_set_ = true;
        // map->odom 由 go2 tf_manager 根据 /icp_result 发布，此处只更新 initGuess
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};

    Eigen::Matrix4f initGuess;
    double initial_x, initial_y, initial_z, initial_a;
    int solver_max_iter;
    double max_correspondence_distance, RANSAC_outlier_rejection_threshold;
    std::string map_path, map_frame;
    double fitness_score_thre;
    double map_voxel_leaf_size, cloud_voxel_leaf_size;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int converged_count = 0;
    int converged_count_thre;
    std::string pcl_type;
    bool pose_published_ = false;
  /**  RViz 2D Pose 置位后才允许 ICP 发布 /icp_result，避免 (0,0,0) 假收敛触发 6700 万点重建  */
    bool manual_pose_set_ = false;
    bool icp_planar_constraint_ = true;
    bool gravity_align_enable_ = true;
    std::string imu_topic_{"/livox/imu"};
    double scan_azimuth_offset_deg_ = 0.0;
    double icp_crop_min_z_ = -0.8;
    double icp_crop_max_z_ = 3.0;
    double icp_max_horizontal_range_ = 60.0;
    bool icp_coarse_yaw_search_enable_ = true;
    double icp_coarse_yaw_half_range_deg_ = 30.0;
    double icp_coarse_yaw_step_deg_ = 3.0;
    bool publish_map_to_odom_tf_ = true;
    bool continuous_tracking_ = false;
    int icp_result_publish_interval_ = 15;
    bool use_ndt_prefit_ = true;
    double ndt_resolution_ = 1.0;
    double ndt_step_size_ = 0.1;
    int ndt_max_iter_ = 35;
    std::string odom_frame_id_{"odom"};
    bool have_imu_ = false;
    Eigen::Quaternionf imu_quat_{Eigen::Quaternionf::Identity()};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int scan_frame_count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
