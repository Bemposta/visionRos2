#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node {

public:

    CompressedImageSubscriber() : Node("compressed_image_subscriber"){
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            rclcpp::SensorDataQoS(),
            std::bind(&CompressedImageSubscriber::callback, this, std::placeholders::_1));
    }

private:

    void callback(const sensor_msgs::msg::Image::SharedPtr msg){
        if (msg->encoding != "bgr8") {
            RCLCPP_ERROR(get_logger(), "Solo bgr8 soportado");
            return;
        }

        rclcpp::Time msg_time(msg->header.stamp);
        rclcpp::Time now_time = this->get_clock()->now();
        double delay = (now_time - msg_time).nanoseconds() / 1e9;
        RCLCPP_INFO(this->get_logger(), "Delay: %.4f seg.", delay);

        cv::Mat frame(
            msg->height,
            msg->width,
            CV_8UC3,
            const_cast<uint8_t*>(msg->data.data()),
            msg->step
        );

        cv::imshow("RAW image", frame);
        cv::waitKey(1);
        }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}