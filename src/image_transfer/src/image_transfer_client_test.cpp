#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>

using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;
using namespace std::chrono_literals;

class image_transfer_client_node : public rclcpp::Node
{
private:
    rclcpp::Client<image_transfer_srv>::SharedPtr client_;

    std::vector<uint8_t> jpeg_compress(const cv::Mat& cv_image, const int quality)
    {
        std::vector<uint8_t> jpeg_compressed_data;
        std::vector<int> compression_params;

        compression_params = {cv::IMWRITE_JPEG_QUALITY, quality};
        cv::imencode(".jpeg", cv_image, jpeg_compressed_data, compression_params);

        return jpeg_compressed_data;
    }

    std::vector<uint8_t> no_compress(const cv::Mat& cv_image)
    {
        std::vector<uint8_t> row_data(cv_image.data, cv_image.data + cv_image.total() * cv_image.elemSize());
        return row_data;
    }

    // Add or self-designed more compression algorithems here.

    /**
     *  The following members are used for Test
     */
    std::ofstream data_logger;
    std::string log_filename = "transfer_metrics.csv";

public:
    image_transfer_client_node(): Node("image_transfer_client")
    {
        // Open log file
        data_logger.open(log_filename);
        if (!data_logger.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Can't open %s", log_filename.c_str());   
        } else {
            data_logger << "timestamp, image_size, compressed_size, compression_time_us, transfer_time_us\n";
        }
                
        // Declare parameters
        this->declare_parameter<std::string>("image_path", "");
        this->declare_parameter<bool>("use_compression", true);
        this->declare_parameter<std::string>("compression_type", "jpeg");
        this->declare_parameter<int>("quality", 95);
        
        // Create the client  
        this->client_ = this->create_client<image_transfer_srv>("image_transfer");
        
        // Wait for the server
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting For Server");
        }
    }

    ~image_transfer_client_node()
    {
        if (data_logger.is_open())
        {
            data_logger.close();
        }
    }

    struct Config {
        std::string image_path;
        bool use_compression = true;
        std::string compression_type = "jpeg";
        int quality = 95;
    };

    Config get_config()
    {
        Config config;
        config.image_path = this->get_parameter("image_path").as_string();
        config.use_compression = this->get_parameter("use_compression").as_bool();
        config.compression_type = this->get_parameter("compression_type").as_string();
        config.quality = this->get_parameter("quality").as_int();
        return config;
    }

    void print_usage()
    {
        RCLCPP_INFO(this->get_logger(), "Usage:");
        RCLCPP_INFO(this->get_logger(), "ros2 run pkg_name node_name \\");
        RCLCPP_INFO(this->get_logger(), " --ros-args -p image_path:=/path/to/image \\");
        RCLCPP_INFO(this->get_logger(), " -p use_compression:=false \\");
        RCLCPP_INFO(this->get_logger(), " -p compression_type:=jpeg \\");
        RCLCPP_INFO(this->get_logger(), " -p quality:=95");
    }

    void send()
    {   
        // Get config
        auto config = this->get_config();

        // Read image
        cv::Mat cv_image = cv::imread(config.image_path);
        // Read image error
        if (cv_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Can't read image: %s", config.image_path.c_str());
            return;
        }

        
        // build a request
        auto request = std::make_shared<image_transfer_srv::Request>();
        request->use_compression = config.use_compression;
        request->encoding = "bgr8";
        request->compression_type = config.compression_type;
        request->origin_width = cv_image.cols;
        request->origin_height = cv_image.rows;

        auto compression_start = std::chrono::high_resolution_clock::now();
        // Check if the image need compression, which is needed defaulty.
        // And fill up the 'data' part of the request
        if (config.use_compression) {
            // Can self-define more compression algorithms here
            if (config.compression_type == "jpeg")
            {
                // Use JPEG default
                request->data = this->jpeg_compress(cv_image, config.quality);
            } else if (config.compression_type == "png") {
                // Implement png method here
                RCLCPP_ERROR(this->get_logger(), "PNG compression not implemented yet");
                return;
            } else {
                // Other unsupported compression algorithm
                RCLCPP_ERROR(this->get_logger(), "Unsupported compression type: %s", config.compression_type.c_str()); 
                return;
            }
        } else {
            // Don't use any compression methods
            request->data = this->no_compress(cv_image);
        }
        auto compression_end = std::chrono::high_resolution_clock::now();
        
        auto transfer_start = std::chrono::high_resolution_clock::now();
        // Send the request
        auto future = client_->async_send_request(request);
        auto response = future.get();
        auto transfer_end = std::chrono::high_resolution_clock::now();
        auto compression_time = std::chrono::duration_cast<std::chrono::microseconds>(compression_end - compression_start);
        auto transfer_time = std::chrono::duration_cast<std::chrono::microseconds>(transfer_end - transfer_start);
        if (data_logger.is_open())
        {
            data_logger << std::chrono::system_clock::now().time_since_epoch().count() << ","
                        << cv_image.total() * cv_image.elemSize() << ","
                        << request->data.size() << ","
                        << compression_time.count() << ","
                        << transfer_time.count() << "\n";
        }
    }

};

int main(int argc, char* argv[])
{   
    // Initial ros2 environment
    rclcpp::init(argc, argv);

    // Create ros2 client node
    auto client = std::make_shared<image_transfer_client_node>();
    
    // CHeck the image path
    if (client->get_parameter("image_path").as_string().empty())
    {
        client->print_usage();
        rclcpp::shutdown();
        return 1;
    }

    // Send request
    client->send();

    // Run ros2 node and close
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
