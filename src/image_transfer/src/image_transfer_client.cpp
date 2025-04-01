#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <chrono>

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

public:
    image_transfer_client_node(): Node("image_transfer_client")
    {
        this->client_ = this->create_client<image_transfer_srv>("image_transfer");
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting For Server");
        }
    };

    void send(const std::string& image_path)
    {
        cv::Mat cv_image = cv::imread(image_path);
        auto request = std::make_shared<image_transfer_srv::Request>();
        auto image = this->cv_mat_to_image(cv_image);
        request->image = image;
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
    std::cout << "argc= " << argc << std::endl;
    for (int i = 0; i < argc; i++)
    {
        std::cout << i << "th argument is" << *argv[i] << std::endl;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<image_transfer_client_node>();
    // send request
    client->send("/home/mildempeach/sunyi.jpg");
    
    rclcpp::spin(client);
    rclcpp::shutdown();

}
