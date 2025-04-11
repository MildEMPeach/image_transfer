#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <iomanip>

using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;
using namespace std::chrono_literals;

class image_transfer_client_node : public rclcpp::Node
{
private:
    rclcpp::Client<image_transfer_srv>::SharedPtr client_;

    // Save test result to a csv file.
    void save_to_csv(const TestResult& result, const std::string& filename = "test_results.csv")
    {
        std::ofstream file(filename, std::ios::app);

        file << std::fixed << std::setprecision(2);
        file << result.image_path << ","
             << result.original_size << ","
             << result.compressed_size << ","
             << result.compression_ratio << ","
             << result.compression_time << ","
             << result.transfer_time << ","
             << result.total_time << ","
             << this->get_parameter("compression_type").as_string() << ","
             << this->get_parameter("use_compression").as_bool() << "\n";
    }

    // define compression algorithms
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

public:
    image_transfer_client_node(): Node("image_transfer_client")
    {
        // Declare parameters
        this->declare_parameter<std::string>("image_path", "");
        this->declare_parameter<bool>("use_compression", true);
        this->declare_parameter<std::string>("compression_type", "jpeg");
        this->declare_parameter<int>("quality", 95);
        this->declare_parameter<bool>("test_mode", false);
        
        // Create the client  
        this->client_ = this->create_client<image_transfer_srv>("image_transfer");
        
        // Wait for the server
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting For Server");
        }
    }

    struct Config {
        std::string image_path;
        bool use_compression = true;
        std::string compression_type = "jpeg";
        int quality = 95;
        bool test_mode = false;
    };

    struct TestResult {
        std::string image_path;
        size_t original_size;
        size_t compressed_size;
        double compression_ratio;
        double compression_time;
        double transfer_time;
        double total_time;
    };

    Config get_config()
    {
        Config config;
        config.image_path = this->get_parameter("image_path").as_string();
        config.use_compression = this->get_parameter("use_compression").as_bool();
        config.compression_type = this->get_parameter("compression_type").as_string();
        config.quality = this->get_parameter("quality").as_int();
        config.test_mode = this->get_parameter("test_mode").as_bool();
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
        RCLCPP_INFO(this->get_logger(), " -p test_mode:=false");
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

        // Send the request
        // auto result = client_->async_send_request(
        //     request,
        //     [&](rclcpp::Client<image_transfer_srv>::SharedFuture future)
        //     {
        //         auto response = future.get();
        //         if (response->success)
        //         {
        //             RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
        //         } else {
        //             RCLCPP_INFO(this->get_logger(), "Send Failed!");
        //         }
        //     }
        // );

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Send Failed!");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send request");
        }

    }

    void send_test()
    {   
        TestResult test_result;
        test_result.image_path = this->get_parameter("image_path").as_string();


        // Get config
        auto config = this->get_config();

        auto total_start = std::chrono::high_resolution_clock::now();

        // Read image
        cv::Mat cv_image = cv::imread(config.image_path);
        test_result.original_size = cv_image.total() * cv_image.elemSize();

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

        auto compress_start = std::chrono::high_resolution_clock::now();
        std::vector<uint8_t> compressed_data;

        // Check if the image need compression, which is needed defaulty.
        // And fill up the 'data' part of the request
        if (config.use_compression) {
            // Can self-define more compression algorithms here
            if (config.compression_type == "jpeg")
            {
                // Use JPEG default
                compressed_data = this->jpeg_compress(cv_image, config.quality);
                request->data = compressed_data;
                test_result.compressed_size = compressed_data.size();
                test_result.compression_ratio = (1.0 - (double)test_result.compressed_size / test_result.original_size) * 100;
            } else if (config.compression_type == "png") {
                // Implement png method here
                RCLCPP_ERROR(this->get_logger(), "PNG compression not implemented yet");
                return;
            } else {
                // Other unsupported compression algorithm
                RCLCPP_ERROR(this->get_logger(), "Unsupported compression type: %s", config.compression_type.c_str()); 
                return;
            }
            test_result.compression_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - compress_start).count();

        } else {
            // Don't use any compression methods
            request->data = this->no_compress(cv_image);

            test_result.compressed_size = test_result.original_size;
            test_result.compression_ratio = 0.0;
            test_result.compression_time = 0.0;
        }


        auto transfer_start = std::chrono::high_resolution_clock::now();
        // Send the request
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Send Failed!");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send request");
        }

        test_result.transfer_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - transfer_start).count();
        test_result.total_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - total_start).count();

        this->print_test_result(test_result);
        this->save_to_csv(test_result);
    }

    void print_test_result(TestResult test_result)
    {
        RCLCPP_INFO(this->get_logger(), "Test Result:");
        RCLCPP_INFO(this->get_logger(), "Image Path: %s", test_result.image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Original Size: %zu bytes", test_result.original_size);
        RCLCPP_INFO(this->get_logger(), "Compressed Size: %zu bytes", test_result.compressed_size);
        RCLCPP_INFO(this->get_logger(), "Compression Ratio: %.2f%%", test_result.compression_ratio);
        RCLCPP_INFO(this->get_logger(), "Compression Time: %.2f seconds", test_result.compression_time);
        RCLCPP_INFO(this->get_logger(), "Transfer Time: %.2f seconds", test_result.transfer_time);
        RCLCPP_INFO(this->get_logger(), "Total Time: %.2f seconds", test_result.total_time);
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

    // Check if test mode is enabled
    if (client->get_parameter("test_mode").as_bool())
    {
        client->send_test();
    } else {
        client->send();
    }

    // Run ros2 node and close
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
