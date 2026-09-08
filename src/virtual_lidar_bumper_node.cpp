#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/transforms.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <memory>
#include <vector>

using PointType = pcl::PointXYZ;

class VirtualLidarBumperNode : public rclcpp::Node {
public:
    VirtualLidarBumperNode() : Node("virtual_lidar_bumper") {
        robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
        transform_tolerance_ = this->declare_parameter<double>("transform_tolerance", 0.1);

        min_x_ = this->declare_parameter<double>("min_x", 0.0);
        max_x_ = this->declare_parameter<double>("max_x", 1.0);
        min_y_ = this->declare_parameter<double>("min_y", -0.5);
        max_y_ = this->declare_parameter<double>("max_y", 0.5);
        min_z_ = this->declare_parameter<double>("min_z", -0.5);
        max_z_ = this->declare_parameter<double>("max_z", 0.5);

        min_points_ = this->declare_parameter<int>("min_points", 5);

        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        publisher_ = this->create_publisher<std_msgs::msg::Bool>("bumper_triggered", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", rclcpp::SensorDataQoS(),
            std::bind(&VirtualLidarBumperNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    std::string robot_frame_;
    double transform_tolerance_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    int min_points_;

    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        pcl::PointCloud<PointType> input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        pcl::PointCloud<PointType>::Ptr cloud_in_robot_frame;

        if (msg->header.frame_id == robot_frame_) {
            cloud_in_robot_frame = std::make_shared<pcl::PointCloud<PointType>>(input_cloud);
        } else {
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = buffer_->lookupTransform(
                    robot_frame_, msg->header.frame_id, msg->header.stamp,
                    tf2::durationFromSec(transform_tolerance_));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Could not transform pointcloud from '%s' to '%s': %s",
                    msg->header.frame_id.c_str(), robot_frame_.c_str(), ex.what());
                return;
            }

            Eigen::Affine3f transform_eigen = tf2::transformToEigen(tf.transform).cast<float>();
            pcl::PointCloud<PointType> transformed_cloud;
            pcl::transformPointCloud(input_cloud, transformed_cloud, transform_eigen);
            cloud_in_robot_frame = std::make_shared<pcl::PointCloud<PointType>>(transformed_cloud);
        }

        pcl::CropBox<PointType> crop_box;
        crop_box.setInputCloud(cloud_in_robot_frame);
        crop_box.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
        crop_box.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));

        std::vector<int> indices;
        crop_box.filter(indices);

        std_msgs::msg::Bool bumper_msg;
        bumper_msg.data = static_cast<int>(indices.size()) >= min_points_;
        publisher_->publish(bumper_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLidarBumperNode>());
    rclcpp::shutdown();
    return 0;
}
