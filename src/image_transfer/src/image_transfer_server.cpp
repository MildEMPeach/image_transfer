#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;

class image_transfer_server_node: public rclcpp::Node
{
private:
    rclcpp::Service<image_transfer_srv>::SharedPtr service_;
public:
    image_transfer_server_node(): Node("image_transfer_server")
    {
        // Initialize a service
       this->service_ = this->create_service<image_transfer_srv>(
           "image_transfer",
           std::bind(&image_transfer_server_node::server_callback, this, std::placeholders::_1, std::placeholders::_2)
       );
    };

    void server_callback(const image_transfer_srv::Request::SharedPtr request, 
        image_transfer_srv::Response::SharedPtr response) 
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(request->image);
        cv::Mat cv_image = cv_ptr->image;
        cv::imshow("RECEIVED image", cv_image);
        cv::waitKey(0);
        response->success = true;        
    };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<image_transfer_server_node>();
    rclcpp::spin(server_node);
    rclcpp::shutdown();
    return 0;
}
