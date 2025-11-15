#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node {

public:

    CompressedImageSubscriber() : Node("compressed_image_subscriber"){
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/image/compressed",
            10,
            std::bind(&CompressedImageSubscriber::callback, this, std::placeholders::_1));
    }

private:

    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
        std::vector<uchar> data(msg->data.begin(), msg->data.end());
        cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);

        if (!frame.empty()){
            cv::imshow("Compressed Image", frame);
            cv::waitKey(1);
        }
        else{
            RCLCPP_WARN(this->get_logger(), "No se pudo decodificar la imagen");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
