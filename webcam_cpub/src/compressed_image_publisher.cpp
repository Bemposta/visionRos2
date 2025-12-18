#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class CompressedImagePublisher : public rclcpp::Node{

public:

    CompressedImagePublisher() : Node("compressed_image_publisher"){

        declare_parameter<int>("camera_id", 0);
        declare_parameter<std::string>("topic", "/camera/image");
		declare_parameter<std::string>("video", "/home/mixi/ros2_ws/Tokyo_640.mp4");
        declare_parameter<double>("fps", 2);

        camera_id_ = get_parameter("camera_id").as_int();
        topic_ = get_parameter("topic").as_string();
		video_demo_ = get_parameter("video").as_string();
        timer_delay_ = int(1000.0 / get_parameter("fps").as_double());

        RCLCPP_INFO(get_logger(), "Camera ID: %d", camera_id_);
        RCLCPP_INFO(get_logger(), "Publishing on topic: %s", topic_.c_str());
        RCLCPP_INFO(get_logger(), "frame interval: %d milisecons", timer_delay_);
		if(camera_id_ < 0)
			 RCLCPP_INFO(get_logger(), "video: %s", video_demo_.c_str());
	      
		auto qos = rclcpp::QoS(1).best_effort();
        publisher_compress = create_publisher<sensor_msgs::msg::CompressedImage>(topic_+"/compressed", rclcpp::SensorDataQoS());
		publisher_raw = create_publisher<sensor_msgs::msg::Image>(topic_, qos);

        timer_ = create_wall_timer(
		std::chrono::milliseconds(timer_delay_),
		std::bind(&CompressedImagePublisher::timer_callback, this));

		if(camera_id_ >= 0)
			cap_.open(camera_id_);  // Abrir webcam por defecto
		else{
			cap_.open(video_demo_);  // Abrir webcam por defecto
			RCLCPP_ERROR(get_logger(), (std::string("video: ")+video_demo_).c_str());
		}
		if (!cap_.isOpened()){
			if(camera_id_ >= 0)
			  RCLCPP_ERROR(get_logger(), "No se pudo abrir la webcam");
			else
			  RCLCPP_ERROR(get_logger(), "No se pudo abrir video: ");
        }
    }

private:

    void timer_callback(){
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
            return;
            
        // IMPORTANTE: Reducir el buffer de la cámara
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // Opcional: Configurar resolución más baja
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	//Envio de imagen comprimida....
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};  // Calidad 70%
        cv::imencode(".jpg", frame, buf, params);  // Comprimir a JPEG
        auto msg_compressed = sensor_msgs::msg::CompressedImage();
	    msg_compressed.header.stamp = get_clock()->now();  // ← marca de tiempo ROS2
        msg_compressed.header.frame_id = "camera_frame";         // ← identificador del frame
        msg_compressed.format = "jpeg";
        msg_compressed.data.assign(buf.begin(), buf.end());
        publisher_compress->publish(msg_compressed);
		
		//Envio de imagen RAW
		sensor_msgs::msg::Image msg_raw;
        msg_raw.header.stamp = get_clock()->now();  // ← marca de tiempo ROS2
        msg_raw.header.frame_id = "camera_frame";         // ← identificador del frame
        msg_raw.height = frame.rows;
        msg_raw.width = frame.cols;
        msg_raw.encoding = "bgr8"; // formato típico en OpenCV
        msg_raw.is_bigendian = false;
        msg_raw.step = frame.cols * frame.elemSize();
        msg_raw.data.assign(frame.data, frame.data + (frame.rows * frame.cols * frame.elemSize()));
        publisher_raw->publish(msg_raw);
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compress;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_raw;
    
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
	
    int camera_id_;
    std::string topic_;
    int timer_delay_;
    std::string video_demo_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}