// ROS2 node subscribing to the image_raw topic and printing the timestamp and the delay of the received images.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        // qos_profile_sensor_data
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&Listener::image_callback, this, std::placeholders::_1));
        // subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 1, std::bind(&Listener::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image with timestamp %f and delay %f", msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec, 1000 * (this->now() - msg->header.stamp).seconds());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}