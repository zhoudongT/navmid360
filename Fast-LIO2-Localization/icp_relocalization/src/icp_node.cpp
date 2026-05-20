#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // Rotate pcl alone x axis for 180 degree
        // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        // rotation(1, 1) = -1;
        // rotation(2, 2) = -1;
        // pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        // icp.setTransformationEpsilon(1e-8);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        // icp.setRANSACIterations(100);
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore();
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", fitness_score);

        if (pose_published_)
        {
            return;
        }

        if (fitness_score < fitness_score_thre && icp.hasConverged())
        {
            converged_count++;
            if (converged_count < converged_count_thre)
            {
                initGuess = icp.getFinalTransformation();
                pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
                sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
                pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
                transformed_cloud_msg.header.stamp = this->now();
                transformed_cloud_msg.header.frame_id = map_frame;
                transformed_cloud_pub_->publish(transformed_cloud_msg);
                target_cloud_msg.header.stamp = this->now();
                map_pub_->publish(target_cloud_msg);
                return;
            }
            if (!manual_pose_set_)
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "ICP aligned (fitness=%.3f). Set 2D Pose in RViz to start localization.", fitness_score);
                return;
            }
            publish_icp_pose(icp.getFinalTransformation(), input_cloud);
        }
        else
        {
            converged_count = 0;
        }
        publish_transformed_scan(input_cloud, initGuess);
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < msg->point_num; i++)
        {
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // Rotate pcl alone x axis for 180 degree
        // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        // rotation(1, 1) = -1;
        // rotation(2, 2) = -1;
        // pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        // icp.setTransformationEpsilon(1e-8);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        // icp.setRANSACIterations(100);
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore();
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", fitness_score);

        if (pose_published_)
        {
            return;
        }

        if (icp.hasConverged() && fitness_score < fitness_score_thre)
        {
            converged_count++;
            initGuess = icp.getFinalTransformation();
            if (converged_count >= converged_count_thre)
            {
                if (!manual_pose_set_)
                {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "ICP aligned (fitness=%.3f). Set 2D Pose in RViz to start localization.", fitness_score);
                    publish_transformed_scan(input_cloud, initGuess);
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "ICP converged!!! fitness=%f count=%d",
                    fitness_score, converged_count);
                publish_icp_pose(icp.getFinalTransformation(), input_cloud);
                return;
            }
            RCLCPP_INFO(this->get_logger(),
                "ICP ok (fitness=%f), waiting more frames %d/%d",
                fitness_score, converged_count, converged_count_thre);
        }
        else
        {
            converged_count = 0;
        }

        publish_transformed_scan(input_cloud, initGuess);
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }
#endif

    void publish_transformed_scan(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
        const Eigen::Matrix4f & transform)
    {
        pcl::PointCloud<pcl::PointXYZ> vis = *input_cloud;
        pcl::transformPointCloud(vis, vis, transform);
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(vis, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = this->now();
        transformed_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(transformed_cloud_msg);
    }

    void publish_icp_pose(const Eigen::Matrix4f & transformation_result,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame;
        pose_msg.pose.pose.position.x = transformation_result(0, 3);
        pose_msg.pose.pose.position.y = transformation_result(1, 3);
        pose_msg.pose.pose.position.z = transformation_result(2, 3);
        Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        publisher_->publish(pose_msg);
        pose_published_ = true;
        publish_transformed_scan(input_cloud, transformation_result);
        RCLCPP_INFO(this->get_logger(), "Published /icp_result (one-shot)");
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

        geometry_msgs::msg::PoseWithCovarianceStamped pose_out = *msg;
        pose_out.header.stamp = this->now();
        pose_out.header.frame_id = map_frame;
        publisher_->publish(pose_out);
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
