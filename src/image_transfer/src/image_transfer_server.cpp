#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>



using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;

class image_transfer_server_node: public rclcpp::Node
{
private:
    rclcpp::Service<image_transfer_srv>::SharedPtr service_;

    // cv::Mat image_to_cv_mat(const sensor_msgs::msg::Image& image)
    // {
    //     cv::Mat mat(image.height, image.width, CV_8UC3, (void*)image.data.data());
    //     return mat.clone();
    // };

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
        cv::Mat decoded_cv_image;
        // Deal with compressed data
        if (request->compression_type == "jpeg")
        {
            const std::vector<uint8_t> buffer(
                request->compressed_data.begin(),
                request->compressed_data.end()
            );
            decoded_cv_image = cv::imdecode(buffer, cv::IMREAD_COLOR);

            // Resize the image
            if (decoded_cv_image.cols != request->origin_width ||
                decoded_cv_image.rows != request->origin_height)
            {
                cv::resize(decoded_cv_image, decoded_cv_image, 
                    cv::Size(request->origin_width, request->origin_height));  
            }
        }

        // show the image
        cv::imshow("RECEIVED Image", decoded_cv_image);
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
