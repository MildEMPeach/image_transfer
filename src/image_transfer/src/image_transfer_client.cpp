#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <chrono>
#include <vector>

using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;
using namespace std::chrono_literals;

class image_transfer_client_node : public rclcpp::Node
{
private:
    rclcpp::Client<image_transfer_srv>::SharedPtr client_;

    sensor_msgs::msg::Image cv_mat_to_image(const cv::Mat& cv_image)
    {
        sensor_msgs::msg::Image image;
        image.height = cv_image.rows;
        image.width = cv_image.cols;
        image.encoding = "bgr8";
        image.step = cv_image.cols * cv_image.elemSize();
        image.data.assign(cv_image.data, cv_image.data + cv_image.total() * cv_image.elemSize());
        return image;
    };

    std::vector<uint8_t> compress_image(
        const cv::Mat& cv_image,
        const std::string& compression_type,
        int quality,
        int32_t& origin_width,
        int32_t& origin_height,
        std::string& encoding
    )
    {
        origin_width = cv_image.cols;
        origin_height = cv_image.rows;
        encoding = "bgr8";

        std::vector<uint8_t> compressed_data;
        std::vector<int> compression_params;
        
        if (compression_type == "jpeg")
        {
            compression_params = {cv::IMWRITE_JPEG_QUALITY, quality};
            cv::imencode(".jpg", cv_image, compressed_data, compression_params);
        }

        return compressed_data;
    };

public:
    image_transfer_client_node(): Node("image_transfer_client")
    {
        this->client_ = this->create_client<image_transfer_srv>("image_transfer");
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting For Server");
        }
    };

    void send(const std::string& image_path, const std::string& compression_type="jpeg", int quality=95)
    {
        // Read image
        cv::Mat cv_image = cv::imread(image_path);
        
        // Compress the image
        int32_t origin_width, origin_height;
        std::string encoding;
        std::vector<uint8_t> compressed_data;
        compressed_data = this->compress_image(
            cv_image,
            compression_type,
            quality,
            origin_width,
            origin_height,
            encoding
        );

        // Build a request
        auto request = std::make_shared<image_transfer_srv::Request>();
        request->compressed_data = compressed_data;
        request->compression_type = compression_type;
        request->origin_width = origin_width;
        request->origin_height = origin_height;
        request->encoding = encoding;

        // Send the request
        auto result = client_->async_send_request(
            request,
            [&](rclcpp::Client<image_transfer_srv>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Send Failure");
                }
            }
        );
    }

};

int main(int argc, char* argv[])
{   
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << "<image_path>" << std::endl;
        return 1;
    }
    std::string image_path = argv[1];
    rclcpp::init(argc, argv);
    auto client = std::make_shared<image_transfer_client_node>();
    // send request
    client->send(image_path);
    rclcpp::spin(client);
    rclcpp::shutdown();

}
