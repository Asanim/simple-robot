#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <libfreenect.hpp>
#include <libfreenect_sync.h>

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kinect/point_cloud", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&KinectNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        publish_point_cloud();
        publish_image();
    }

    void publish_point_cloud() {
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        void *depth_data;
        uint32_t timestamp;

        if (freenect_sync_get_depth(&depth_data, &timestamp, 0, FREENECT_DEPTH_11BIT) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get depth data");
            return;
        }

        // Populate point_cloud_msg with depth_data
        point_cloud_msg.header.stamp = this->now();
        point_cloud_msg.header.frame_id = "kinect_depth_frame";
        point_cloud_msg.height = 480;
        point_cloud_msg.width = 640;
        point_cloud_msg.is_dense = false;
        point_cloud_msg.is_bigendian = false;
        point_cloud_msg.point_step = 3 * sizeof(float);
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

        sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

        uint16_t *depth = static_cast<uint16_t *>(depth_data);
        for (size_t i = 0; i < point_cloud_msg.height; ++i) {
            for (size_t j = 0; j < point_cloud_msg.width; ++j, ++iter_x, ++iter_y, ++iter_z) {
                *iter_x = static_cast<float>(j);
                *iter_y = static_cast<float>(i);
                *iter_z = static_cast<float>(depth[i * point_cloud_msg.width + j]);
            }
        }

        point_cloud_publisher_->publish(point_cloud_msg);
    }

    void publish_image() {
        sensor_msgs::msg::Image image_msg;
        void *rgb_data;
        uint32_t timestamp;

        if (freenect_sync_get_video(&rgb_data, &timestamp, 0, FREENECT_VIDEO_RGB) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get RGB data");
            return;
        }

        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "kinect_rgb_frame";
        image_msg.height = 480;
        image_msg.width = 640;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = 3 * image_msg.width;
        image_msg.data.assign(static_cast<uint8_t *>(rgb_data), static_cast<uint8_t *>(rgb_data) + (image_msg.step * image_msg.height));

        image_publisher_->publish(image_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
    return 0;
}
