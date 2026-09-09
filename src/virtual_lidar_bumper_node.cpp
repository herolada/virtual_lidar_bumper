#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/transforms.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <algorithm>
#include <cstdio>
#include <deque>
#include <memory>
#include <numeric>
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
        window_size_ = this->declare_parameter<int>("window_size", 1);
        if (window_size_ < 1) {
            window_size_ = 1;
        }

        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        publisher_ = this->create_publisher<std_msgs::msg::Bool>("bumper_triggered", 10);
        box_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/virtual_bumper/box", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", rclcpp::SensorDataQoS(),
            std::bind(&VirtualLidarBumperNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    std::string robot_frame_;
    double transform_tolerance_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    int min_points_;
    int window_size_;
    std::deque<int> point_count_history_;

    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_marker_publisher_;

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

        point_count_history_.push_back(static_cast<int>(indices.size()));
        while (static_cast<int>(point_count_history_.size()) > window_size_) {
            point_count_history_.pop_front();
        }
        double avg_points = std::accumulate(point_count_history_.begin(), point_count_history_.end(), 0.0) /
                             static_cast<double>(point_count_history_.size());

        bool triggered = avg_points >= min_points_;

        std_msgs::msg::Bool bumper_msg;
        bumper_msg.data = triggered;
        publisher_->publish(bumper_msg);

        publishBoxMarker(msg->header.stamp, indices.size(), avg_points, triggered);
    }

    void publishBoxMarker(
        const rclcpp::Time &stamp, size_t num_points, double avg_points, bool triggered) {
        double center_x = (min_x_ + max_x_) / 2.0;
        double center_y = (min_y_ + max_y_) / 2.0;
        double center_z = (min_z_ + max_z_) / 2.0;

        visualization_msgs::msg::Marker box_marker;
        box_marker.header.frame_id = robot_frame_;
        box_marker.header.stamp = stamp;
        box_marker.ns = "virtual_bumper";
        box_marker.id = 0;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;
        box_marker.pose.position.x = center_x;
        box_marker.pose.position.y = center_y;
        box_marker.pose.position.z = center_z;
        box_marker.pose.orientation.w = 1.0;
        box_marker.scale.x = std::max(max_x_ - min_x_, 1e-3);
        box_marker.scale.y = std::max(max_y_ - min_y_, 1e-3);
        box_marker.scale.z = std::max(max_z_ - min_z_, 1e-3);
        box_marker.color.a = 0.3;
        if (triggered) {
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
        } else {
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 0.0;
        }
        box_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = robot_frame_;
        text_marker.header.stamp = stamp;
        text_marker.ns = "virtual_bumper";
        text_marker.id = 1;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = center_x;
        text_marker.pose.position.y = center_y;
        text_marker.pose.position.z = max_z_ + 0.1;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.2;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        char avg_buf[16];
        std::snprintf(avg_buf, sizeof(avg_buf), "%.1f", avg_points);
        text_marker.text = std::to_string(num_points) + " pts (avg " + avg_buf + ")";
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(box_marker);
        marker_array.markers.push_back(text_marker);
        box_marker_publisher_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLidarBumperNode>());
    rclcpp::shutdown();
    return 0;
}
