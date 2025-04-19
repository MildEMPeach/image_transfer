#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <image_transfer_interfaces/msg/image_transfer.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <iomanip>

using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;
using image_transfer_msg = image_transfer_interfaces::msg::ImageTransfer;
using namespace std::chrono_literals;

class image_transfer_client_node : public rclcpp::Node
{
private:
    rclcpp::Client<image_transfer_srv>::SharedPtr client_;
    rclcpp::Publisher<image_transfer_msg>::SharedPtr publisher_;

    // =========================== define compression algorithms =========================================
    std::vector<uint8_t> jpeg_compress(const cv::Mat &cv_image, const int quality)
    {
        std::vector<uint8_t> jpeg_compressed_data;
        std::vector<int> compression_params;

        compression_params = {cv::IMWRITE_JPEG_QUALITY, quality};
        cv::imencode(".jpeg", cv_image, jpeg_compressed_data, compression_params);

        return jpeg_compressed_data;
    }

    std::vector<uint8_t> no_compress(const cv::Mat &cv_image)
    {
        std::vector<uint8_t> row_data(cv_image.data, cv_image.data + cv_image.total() * cv_image.elemSize());
        return row_data;
    }
    // =========================== define compression algorithms =========================================

    // =========================== Basic usage ===========================================================
    struct Config
    {
        std::string image_path;
        std::string compression_type;
        int quality;
        bool test_mode;
        int chunk_size;
    };

    Config get_config()
    {
        Config config;
        config.image_path = this->get_parameter("image_path").as_string();
        config.compression_type = this->get_parameter("compression_type").as_string();
        config.quality = this->get_parameter("quality").as_int();
        config.test_mode = this->get_parameter("test_mode").as_bool();
        config.chunk_size = this->get_parameter("chunk_size").as_int();
        return config;
    }
    // =========================== Basic usage ===========================================================
    // =========================== split image and send parallel =========================================
    std::vector<std::vector<uint8_t>> split_image(std::vector<uint8_t> &data, int chunk_size)
    {
        std::vector<std::vector<uint8_t>> chunks;
        for (size_t i = 0; i < data.size(); i += chunk_size)
        {
            auto end = std::min(i + chunk_size, data.size());
            chunks.emplace_back(data.begin() + i, data.begin() + end);
        }

        return chunks;
    }

    void send_parallel(const std::vector<std::shared_ptr<image_transfer_srv::Request>>& requests)
    {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(this->shared_from_this());

        std::thread executor_thread([&executor]() {
            executor.spin();
        });

        std::vector<std::future<void>> futures;
        for (const auto& req : requests)
        {
            futures.emplace_back(
                std::async(std::launch::async, [this, &req]() {
                    auto future = this->client_->async_send_request(req);
                })
            );
        }

        for (auto& f : futures)
        {
            f.wait();
        }

        executor.cancel();
        executor_thread.join();
    }

    void publish_parallel(const std::vector<image_transfer_msg>& chunks)
    {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(this->shared_from_this());

        std::thread executor_thread([&executor]() {
            executor.spin();
        });

        std::vector<std::future<void>> futures;
        for (const auto& chunk : chunks)
        {
            futures.emplace_back(
                std::async(std::launch::async, [this, &chunk]() {
                    this->publisher_->publish(chunk);
                })
            );
        }

        for (auto& f : futures)
        {
            f.wait();
        }

        executor.cancel();
        executor_thread.join();
    }
    

    std::string generate_unique_id()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        return std::to_string(timestamp) + "_" + std::to_string(rand() % 1000);
    }

    // =========================== split image and send parallel =========================================

    // =========================== Test mode =============================================================
    struct TestResult
    {
        std::string image_path;
        size_t original_size;
        size_t compressed_size;
        double compression_ratio;
        double compression_time;
        double transfer_time;
        double split_time;
        double total_time;
    };

    void print_test_result(const TestResult test_result)
    {
        RCLCPP_INFO(this->get_logger(), "Test Result:");
        RCLCPP_INFO(this->get_logger(), "Image Path: %s", test_result.image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Original Size: %zu bytes", test_result.original_size);
        RCLCPP_INFO(this->get_logger(), "Compressed Size: %zu bytes", test_result.compressed_size);
        RCLCPP_INFO(this->get_logger(), "Compression Ratio: %.2f%%", test_result.compression_ratio);
        RCLCPP_INFO(this->get_logger(), "Compression Time: %.2f seconds", test_result.compression_time);
        RCLCPP_INFO(this->get_logger(), "Split Time: %.2f seconds", test_result.split_time);
        RCLCPP_INFO(this->get_logger(), "Transfer Time: %.2f seconds", test_result.transfer_time);
        RCLCPP_INFO(this->get_logger(), "Total Time: %.2f seconds", test_result.total_time);
    }

    // Save test result to a csv file.
    void save_to_csv(const TestResult result, const std::string &filename = "test_results.csv")
    {
        std::ifstream check(filename);
        bool file_exists = check.good();
        check.close();

        if (!file_exists)
        {
            std::ofstream file(filename);
            file << "image_path,original_size,compressed_size,compression_ratio,compression_time,split_time,transfer_time,total_time,compression_type\n";
        }

        std::ofstream file(filename, std::ios::app);

        file << std::fixed << std::setprecision(2);
        file << result.image_path << ","
             << result.original_size << ","
             << result.compressed_size << ","
             << result.compression_ratio << ","
             << result.compression_time << ","
             << result.split_time << ","
             << result.transfer_time << ","
             << result.total_time << ","
             << this->get_parameter("compression_type").as_string() << "\n";
    }
    // ========================== Test mode ==============================================================

public:
    image_transfer_client_node() : Node("image_transfer_client")
    {
        // Declare parameters
        this->declare_parameter<std::string>("image_path", "");
        this->declare_parameter<std::string>("compression_type", "none");
        this->declare_parameter<int>("quality", 95);
        this->declare_parameter<bool>("test_mode", false);
        this->declare_parameter<int>("chunk_size", 0);

        // Qos
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // Create the client
        this->client_ = this->create_client<image_transfer_srv>("image_transfer", qos.get_rmw_qos_profile());

        // Create the publisher
        this->publisher_ = this->create_publisher<image_transfer_msg>("image_transfer", qos);

        // Wait for the server
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting For Server");
        }
    }

    void print_usage()
    {
        // TODO: add default value
        RCLCPP_INFO(this->get_logger(), "Usage:");
        RCLCPP_INFO(this->get_logger(), "ros2 run pkg_name node_name");
        RCLCPP_INFO(this->get_logger(), " --ros-args");

        // Parse the args
        // necessary:
        RCLCPP_INFO(this->get_logger(), " -p image_path:=/path/to/image");
        // Default is none
        RCLCPP_INFO(this->get_logger(), " -p compression_type:=none(default)/jpeg");
        RCLCPP_INFO(this->get_logger(), " -p quality:=95");
        RCLCPP_INFO(this->get_logger(), " -p test_mode:=false");
        RCLCPP_INFO(this->get_logger(), " -p chunk_size:=0(default)/1000/2000/3000/4000/5000");
    }

    void send()
    {   
        // Initial the test result
        TestResult test_result;
        test_result.image_path = this->get_parameter("image_path").as_string();

        // Start time
        auto total_start = std::chrono::high_resolution_clock::now();

        // Generate unique id
        std::string image_id = this->generate_unique_id();

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

        test_result.original_size = cv_image.total() * cv_image.elemSize();

        // Start compress time
        auto compress_start = std::chrono::high_resolution_clock::now();

        // Compress the image and save the information to data
        std::vector<uint8_t> data;
        if (config.compression_type == "none")
        {
            // Don't use any compression methods
            data = this->no_compress(cv_image);
            test_result.compressed_size = test_result.original_size;
            test_result.compression_ratio = 0.0;
            test_result.compression_time = 0.0;
        }
        else if (config.compression_type == "jpeg")
        {
            // Use JPEG default
            data = this->jpeg_compress(cv_image, config.quality);
            test_result.compressed_size = data.size();
            test_result.compression_ratio = (1.0 - (double)test_result.compressed_size / test_result.original_size) * 100;
            test_result.compression_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - compress_start).count();
        }
        else
        {
            // Other unsupported compression algorithm
            RCLCPP_ERROR(this->get_logger(), "Unsupported compression type: %s", config.compression_type.c_str());
            return;
        }

        // Decide if need to split the image and send parallely
        if (config.chunk_size > 0)
        {   
            // Start split the image time
            auto split_start = std::chrono::high_resolution_clock::now();

            // Need to split the image and send parallely
            auto chunks = this->split_image(data, config.chunk_size);

            test_result.split_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - split_start).count();

            // std::vector<std::shared_ptr<image_transfer_srv::Request>> requests;
            // for (size_t i = 0; i < chunks.size(); i++)
            // {
            //     auto request = std::make_shared<image_transfer_srv::Request>();
            //     request->encoding = "bgr8";
            //     request->compression_type = config.compression_type;
            //     request->origin_width = cv_image.cols;
            //     request->origin_height = cv_image.rows;
            //     request->total_chunks = chunks.size();
            //     request->chunk_index = i;
            //     request->data = chunks[i];
            //     request->image_id = image_id;
            //     request->test = config.test_mode;
            //     requests.push_back(request);
            // }

            std::vector<image_transfer_msg> chunks_msg;
            for (size_t i = 0; i < chunks.size(); i++)
            {
                image_transfer_msg msg;
                msg.encoding = "bgr8";
                msg.compression_type = config.compression_type;
                msg.origin_width = cv_image.cols;
                msg.origin_height = cv_image.rows;
                msg.total_chunks = chunks.size();
                msg.chunk_index = i;
                msg.data = chunks[i];
                msg.image_id = image_id;
                msg.test = config.test_mode;
                chunks_msg.push_back(msg);
            }

            auto transfer_start = std::chrono::high_resolution_clock::now();
            // Send the requests in parallel
            // this->send_parallel(requests);
            // this->send_parallel_test(requests);
            // this->send_parallel(requests);
            this->publish_parallel(chunks_msg);
            test_result.transfer_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - transfer_start).count();
            test_result.total_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - total_start).count();
            
            RCLCPP_INFO(this->get_logger(), "Send %zu chunks in parallel", chunks.size());
        }
        else
        {   
            test_result.split_time = 0.0;

            // Build the basic request
            auto request = std::make_shared<image_transfer_srv::Request>();
            request->encoding = "bgr8";
            request->compression_type = config.compression_type;
            request->origin_width = cv_image.cols;
            request->origin_height = cv_image.rows;
            request->data = data;
            request->test = config.test_mode;
            auto transfer_start = std::chrono::high_resolution_clock::now();
            // Send the request without splitting
            auto future = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Send sccessfully!");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Send Failed!");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send request");
            }
            test_result.transfer_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - transfer_start).count();
            test_result.total_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - total_start).count();
        }

        if (config.test_mode)
        {
            this->print_test_result(test_result);
            this->save_to_csv(test_result);
            exit(0);
        }

    }
};

    

int main(int argc, char *argv[])
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
    client->send();
    // Run ros2 node and close
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
