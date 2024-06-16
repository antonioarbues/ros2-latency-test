// ROS2 node that publishes an Image on topic image_raw at a frequency of 100 Hz, timestamped with the current time.
// The image is a 640x480 pixel red image

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class Speaker : public rclcpp::Node
{
public:
    Speaker() : Node("speaker")
    {
        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        // set publisher to QoS profile sensor data
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Speaker::publish_image, this));

        // cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 255));  // ~40ms
        // cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));  // ~120ms
        // cv::Mat image(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 255));  // almost every image dropped
        // cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(0, 0, 255));  // almost every image dropped

        // black and white image
        // cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));

        cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 255));
        msg.height = image.rows;
        msg.width = image.cols;
        msg.encoding = "bgr8";
        msg.is_bigendian = 0;
        msg.step = image.cols * image.elemSize();
        size_t size = image.cols * image.rows * image.elemSize();
        msg.data.resize(size);
        memcpy(msg.data.data(), image.data, size);
    }

private:
    void publish_image()
    {
        msg.header.stamp = this->now();
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Speaker>());
    rclcpp::shutdown();
    return 0;
}