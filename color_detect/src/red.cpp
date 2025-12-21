#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

/******************************************************************************************/
class ColorRedDetect : public rclcpp::Node {

/******************************************************************************************/
public:

    /*____________________________________________________________________________________*/
    ColorRedDetect() : Node("color_red_detect"){
        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<std::string>("topicCam", "/camera/image");
        this->declare_parameter<std::string>("topicDetect", "/detect/red");
        this->declare_parameter<bool>("showProcess", true);

        std::string topicCam = this->get_parameter("topicCam").as_string();
        std::string topicDetect = this->get_parameter("topicDetect").as_string();
		debug_ = this->get_parameter("debug").as_bool();    // Se puede hacer de las dos maneras...
		this->get_parameter("showProcess", showProcess_);   // Esta es mas compacta...
        topicCam += "/compressed";

        auto miQoS = rclcpp::SensorDataQoS();
        miQoS.keep_last(1);

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topicCam,
            miQoS,
            std::bind(&ColorRedDetect::callback, this, std::placeholders::_1)
        );
		publisher_ = this->create_publisher<std_msgs::msg::String>(topicDetect, miQoS);

        RCLCPP_INFO(this->get_logger(), "Inicializado el Nodo.");
        RCLCPP_INFO(this->get_logger(), "Parametro \"debug\": %s.", debug_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Parametro \"showProcess\": %s.", showProcess_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Parametro \"topicCam\": %s (se añade /compressed automaticamente).", topicCam.c_str());
        RCLCPP_INFO(this->get_logger(), "Parametro \"topicDetect\": %s.", topicDetect.c_str());
    }

/******************************************************************************************/
private:

    /*____________________________________________________________________________________*/
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
        std::vector<uchar> data(msg->data.begin(), msg->data.end());
        cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);

        if (frame.empty())
            return;

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, redMask;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 250, 250), mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 250, 250), mask2);
        redMask = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
        cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Guardar resultados en YAML (en memoria, no en fichero)
        cv::FileStorage fs(cv::String(), cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
        fs << "imgs" << "[" << frame.cols << frame.rows << "]";
        fs << "objs" << "[";
        int id = 0;
        for (const auto &contour : contours)
        {
            int area = static_cast<int>(cv::contourArea(contour));
            if (area < 600)
                continue;
            cv::Rect bbox = cv::boundingRect(contour);
            fs << "{";
            fs << "id" << id++;
            fs << "area" << area;
            fs << "xywh" << "[" << bbox.x << bbox.y << bbox.width << bbox.height << "]";  // bounding_box
            fs << "}";  // object

            cv::rectangle(frame, bbox, {0, 255, 0}, 2);
        }
        fs << "]";
        std::string json_string = fs.releaseAndGetString();
        if(debug_)
            RCLCPP_INFO(this->get_logger(), "Grupo: \n%s", json_string.c_str());

        std::erase(json_string, ' ');       //Esto elimina los [espacios en blanco] de la cadena
        std::erase(json_string, '\n');      //Esto elimina los [Enter] de la cadena
        auto message = std_msgs::msg::String();
        message.data = json_string;
        publisher_->publish(message);

        if(showProcess_){
            cv::imshow("Compressed Image", frame);
            cv::waitKey(1);
        }

    }

/******************************************************************************************/
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    bool debug_;
    bool showProcess_;
};

/******************************************************************************************/
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorRedDetect>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}