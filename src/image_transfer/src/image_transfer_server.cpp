#include <rclcpp/rclcpp.hpp>
#include <image_transfer_interfaces/srv/image_transfer.hpp>
#include <opencv2/opencv.hpp>



using image_transfer_srv = image_transfer_interfaces::srv::ImageTransfer;
using namespace std::chrono_literals;

class image_transfer_server_node: public rclcpp::Node
{
private:
    rclcpp::Service<image_transfer_srv>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;

    std::map<std::string, std::map<int, std::vector<uint8_t>>> image_buffer_;

    struct ImageStatus {
        int total_chunks;
        rclcpp::Time last_received_time;
        int origin_width;
        int origin_height;
        std::string encoding;
        std::string compression_type;
        bool test;
    };
    std::map<std::string, ImageStatus> image_status_;

    bool check_all_chunks_received(const std::string& image_id)
    {
        if (image_status_.find(image_id) == image_status_.end()) {
            return false; // Image ID not found
        }

        const int expected_chunks = image_status_[image_id].total_chunks;
        const auto& received_chunks = image_buffer_[image_id];

        return (static_cast<int>(received_chunks.size()) == expected_chunks);
    }

    void process_image(const std::string& image_id)
    {
        const auto& received_chunks = image_buffer_[image_id];
        const auto& status = image_status_[image_id];
        const bool test = status.test;

        std::vector<uint8_t> full_image_data;
        for (int i = 0; i < status.total_chunks; ++i)
        {
            if (received_chunks.find(i) == received_chunks.end()) {
                RCLCPP_ERROR(this->get_logger(), "Missing chunk %d for image ID %s", i, image_id.c_str());
                return; // Missing chunk, cannot process the image
            }
            const auto& chunk_data = received_chunks.at(i);
            full_image_data.insert(full_image_data.end(), chunk_data.begin(), chunk_data.end());
        }

        RCLCPP_INFO(this->get_logger(), "Full Image data loaded");

        // Decode the image data
        cv::Mat decoded_cv_image;

        const std::string& compression_type = status.compression_type;
        const int origin_width = status.origin_width;
        const int origin_height = status.origin_height;

        // Deal with data
        if (compression_type == "none") {
            // Read raw data directly
            decoded_cv_image = cv::Mat(origin_height, origin_width, CV_8UC3, (void*) full_image_data.data()).clone();
        } else if (compression_type == "jpeg") {
            const std::vector<uint8_t> buffer(
                full_image_data.begin(),
                full_image_data.end()
            );
            
            // Maybe useful after later development
            // std::string file_extension;
            // if (request->compression_type == "jpeg")
            // {
            //     file_extension = ".jpg";
            // }

            decoded_cv_image = cv::imdecode(buffer, cv::IMREAD_COLOR);

            // Resize the image if needed
            if (decoded_cv_image.cols != origin_width ||
                decoded_cv_image.rows != origin_height)
            {
                cv::resize(decoded_cv_image, decoded_cv_image, 
                    cv::Size(origin_width, origin_height));  
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported compression type: %s", compression_type.c_str());
        }

        if (!test) 
        {
            // show the image
            cv::imshow("RECEIVED Image", decoded_cv_image);
            cv::waitKey(0);
        }      
    }

    void cleanup_timeout_transfers() {
        const auto now = this->now();
        const double timeout_sec = 10.0;

        for (auto it = image_status_.begin(); it != image_status_.end();) {
            const auto& image_id = it->first;
            const auto& status = it->second;

            if ((now - status.last_received_time).seconds() > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "Image ID %s timed out. Cleaning up.", image_id.c_str());
                image_buffer_.erase(image_id);
                it = image_status_.erase(it); // Erase and move to the next element
            } else {
                ++it; // Move to the next element
            }
        }
    }

    

public:
    image_transfer_server_node(): Node("image_transfer_server")
    {   
        // qos
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        // Initialize a service
       this->service_ = this->create_service<image_transfer_srv>(
           "image_transfer",
           std::bind(&image_transfer_server_node::server_callback, this, std::placeholders::_1, std::placeholders::_2),
           qos.get_rmw_qos_profile()
        );

       this->cleanup_timer_ = this->create_wall_timer(
            5s, std::bind(&image_transfer_server_node::cleanup_timeout_transfers, this)
       );
    };

    void server_callback(const image_transfer_srv::Request::SharedPtr request, 
        image_transfer_srv::Response::SharedPtr response) 
    {           

        try {
            const std::string& image_id = request->image_id;
            const int chunk_index = request->chunk_index;
            const int total_chunks = request->total_chunks;

            image_buffer_[image_id][chunk_index] = request->data;

            image_status_[image_id] = {
                total_chunks,
                this->now(),
                request->origin_width,
                request->origin_height,
                request->encoding,
                request->compression_type,
                request->test
            };

            RCLCPP_INFO(this->get_logger(), "Received chunk %d of %d for image ID %s", 
                chunk_index, total_chunks, image_id.c_str());
            response->success = true;
            if (check_all_chunks_received(image_id))
            {
                // All chunks received
                RCLCPP_INFO(this->get_logger(), "All chunks received for image ID %s", image_id.c_str());
                this->process_image(image_id);
                image_buffer_.erase(image_id);
                image_status_.erase(image_id);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            response->success = false;
        }
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
