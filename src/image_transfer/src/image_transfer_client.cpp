#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include<getopt.h>

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

public:
    image_transfer_client_node(): Node("image_transfer_client")
    {
        this->client_ = this->create_client<image_transfer_srv>("image_transfer");
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
    };

    static Config parse_args(int argc, char* argv[])
    {
        Config config;
        int opt;

        static struct option long_options[] = {
            {"image", required_argument, 0, 'i'},
            {"no-compress", no_argument, 0, 'n'},
            {"type", required_argument, 0, 't'},
            {"quality", required_argument, 0, 'q'},
            {"help", no_argument, 0, 'h'},
            {0, 0, 0, 0}
        };

        while ((opt = getopt_long(argc, argv, "i:nt:q:h", long_options, NULL)) != -1)
        {
            switch (opt)
            {
            case 'i':
                config.image_path = optarg;
                break;
            case 'n':
                config.use_compression = false;
                break;
            case 't':
                config.compression_type = optarg; 
                break;
            case 'q':
                config.quality = std::stoi(optarg);
                break;
            case 'h':
                print_help();
                exit(0);
            default:
                print_help();
                exit(1);
            }
        }

        if (config.image_path.empty())
        {
            std::cerr << "Error: Image path is REQUIRED" << std::endl;
            print_help();
            exit(1);
        }

        return config;
    }

    static void print_help()
    {
        std::cout << "Usage: image_transfer_client [OPTIONS]\n"
                  << "OPTIONS:\n"
                  << " -i, --image PATH     Path to the image file (REQUIRED)\n"
                  << " -n, --no-compress     Disable compression\n"
                  << " -t, --type TYPE     Compression type(default: jpeg)\n"
                  << " -q, --quality QUALITY     Compression quality(default:95\n)"
                  << " -h, --help     Show this help message\n";
    }

    void send(const Config& config)
    {
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
        auto result = client_->async_send_request(
            request,
            [&](rclcpp::Client<image_transfer_srv>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Send Failed!");
                }
            }
        );
    }

};

int main(int argc, char* argv[])
{   
    // Initial ros2 environment
    rclcpp::init(argc, argv);

    // Create ros2 client node
    auto client = std::make_shared<image_transfer_client_node>();
    
    // Parse command line arguments
    auto config = image_transfer_client_node::parse_args(argc, argv);
    
    // Send the request
    client->send(config);

    // Run ros2 node and close
    rclcpp::spin(client);
    rclcpp::shutdown();

}
