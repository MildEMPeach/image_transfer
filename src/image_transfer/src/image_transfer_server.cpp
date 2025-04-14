#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>



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
        cv::Mat decoded_cv_image;
        
        // Deal with data
        if (request->compression_type == "none") {
            // Read raw data directly
            decoded_cv_image = cv::Mat(request->origin_height, request->origin_width, CV_8UC3, (void*) request->data.data()).clone();
        } else if (request->compression_type == "jpeg") {
            const std::vector<uint8_t> buffer(
                request->data.begin(),
                request->data.end()
            );
            
            // Maybe useful after later development
            // std::string file_extension;
            // if (request->compression_type == "jpeg")
            // {
            //     file_extension = ".jpg";
            // }

            decoded_cv_image = cv::imdecode(buffer, cv::IMREAD_COLOR);

            // Resize the image if needed
            if (decoded_cv_image.cols != request->origin_width ||
                decoded_cv_image.rows != request->origin_height)
            {
                cv::resize(decoded_cv_image, decoded_cv_image, 
                    cv::Size(request->origin_width, request->origin_height));  
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported compression type: %s", request->compression_type.c_str());
        }

        // show the image
        // cv::imshow("RECEIVED Image", decoded_cv_image);
        // cv::waitKey(0);
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
