#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"    // ROS2 image message header
#include "sensor_msgs/msg/imu.hpp"      // ROS2 IMU message header
#include "cv_bridge/cv_bridge.h"        // Conversion between ROS and OpenCV images
#include "opencv2/opencv.hpp"           // Core OpenCV functionality
#include "opencv2/video.hpp"            // OpenCV video (for optical flow)
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>

// Constants for the topic names
const std::string IMAGE_TOPIC_NAME = "/zed/zed_node/left_raw/image_raw_color";
const std::string IMU_TOPIC_NAME = "/zed/zed_node/imu/data";

// A ROS2 node that subscribes to image and IMU topics, computes optical flow on images,
// displays the result with motion tracking, updates a motion graph, and writes data to files.
class OpticalFlowNode : public rclcpp::Node
{
public:
    OpticalFlowNode()
        : Node("image_subscriber_node"),
          max_history_size_(100),         // Store up to 100 frames of history for the graph
          graph_width_(600),              // Width of the motion graph window
          graph_height_(400),             // Height of the motion graph window
          last_image_timestamp_(0.0)
    {
        // Subscribe to the ZED camera's left raw color image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC_NAME,
            10, // queue size
            std::bind(&OpticalFlowNode::image_callback, this, std::placeholders::_1)
        );

        // Prepare a blank graph image with white background
        graph_image_ = cv::Mat(graph_height_, graph_width_, CV_8UC3, cv::Scalar(255, 255, 255));

        // Subscribe to the ZED camera's IMU data topic
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC_NAME,
            10, // queue size
            std::bind(&OpticalFlowNode::imu_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Node initialized. Subscribed to: %s and %s",
                    IMAGE_TOPIC_NAME.c_str(), IMU_TOPIC_NAME.c_str());
    }

    ~OpticalFlowNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down node and closing windows.");
        cv::destroyAllWindows();
    }

private:
    // Callback function for incoming image messages
    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try {
            // Convert ROS image message to an OpenCV BGR image
            cv::Mat current_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

            // If we have a previous image, compute optical flow
            if (!previous_image_.empty()) {
                // Compute optical flow between current and previous images
                cv::Mat optical_flow_image = compute_optical_flow(current_image, previous_image_);

                // Update and draw the motion graph based on the latest average motion
                update_and_draw_graph(avg_motion_);
                
                // Display the optical flow result and the motion graph
                cv::imshow("ZED Camera - Optical Flow", optical_flow_image);
                cv::imshow("Average Motion Graph", graph_image_);
            }

            // Write the average motion (from optical flow) to the CSV file
            write_image_data_to_csv(avg_motion_, img_msg);

            // Save the current image for the next callback
            previous_image_ = current_image.clone();

            // Allow OpenCV to refresh its GUI windows
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
        catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    // Callback function for incoming IMU messages
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // Write angular velocity data to CSV file
        write_imu_data_to_csv(imu_msg);
    }

    // Compute optical flow between two images and draw motion vectors
    cv::Mat compute_optical_flow(const cv::Mat &current, const cv::Mat &previous)
    {
        cv::Mat prev_gray, curr_gray;
        // Convert images to grayscale
        cv::cvtColor(previous, prev_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(current, curr_gray, cv::COLOR_BGR2GRAY);

        // Detect good features (corner points) to track in the previous image
        std::vector<cv::Point2f> points_prev, points_curr;
        cv::goodFeaturesToTrack(prev_gray, points_prev, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

        // Calculate optical flow (Lucas-Kanade) to track these points in the current image
        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03);
        cv::calcOpticalFlowPyrLK(prev_gray, curr_gray, points_prev, points_curr, status, err,
                                 cv::Size(15, 15), 2, termcrit);

        // Create a mask to draw motion lines
        cv::Mat mask = cv::Mat::zeros(current.size(), current.type());
        cv::Point2f total_motion(0.0f, 0.0f);
        int valid_count = 0;

        // Draw lines and circles for each valid tracked point
        for (size_t i = 0; i < points_prev.size(); i++)
        {
            if (status[i]) {
                // Draw the track line (from old to new position)
                cv::line(mask, points_curr[i], points_prev[i], cv::Scalar(0, 255, 0), 2);
                // Mark the current position with a circle
                cv::circle(current, points_curr[i], 5, cv::Scalar(0, 0, 255), -1);

                // Calculate motion vector and accumulate
                cv::Point2f motion_vec = points_curr[i] - points_prev[i];
                total_motion += motion_vec;
                valid_count++;
            }
        }

        // Calculate average motion if we have valid points
        if (valid_count > 0) {
            avg_motion_.x = total_motion.x / valid_count;
            avg_motion_.y = total_motion.y / valid_count;
            RCLCPP_INFO(this->get_logger(),
                        "Average Motion: x=%.2f, y=%.2f",
                        avg_motion_.x, avg_motion_.y);
        }

        // Overlay the motion lines on the current frame
        cv::Mat output;
        cv::add(current, mask, output);
        return output;
    }

    // Write image timestamp and average motion to a CSV file
    void write_image_data_to_csv(const cv::Point2f &avg_motion, 
                                 const sensor_msgs::msg::Image::SharedPtr &message)
    {
        // Extract timestamp from the ROS image message (in seconds)
        double current_time = message->header.stamp.sec + message->header.stamp.nanosec * 1e-9;
        double timestamp;

        // On the first image, use its timestamp. Otherwise, average with last timestamp
        if (last_image_timestamp_ != 0.0) {
            timestamp = last_image_timestamp_ + (current_time - last_image_timestamp_) / 2.0;
        } else {
            timestamp = current_time;
        }

        RCLCPP_INFO(this->get_logger(), "Image Timestamp: %f", timestamp);

        // Open the CSV file in append mode
        std::ofstream data_file(image_file_name_, std::ios::app);
        if (data_file.is_open()) {
            // Write timestamp and average motion values
            data_file << timestamp << " | "
                      << avg_motion.x << " | "
                      << avg_motion.y << "\n";
            data_file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not open image data file: %s",
                         image_file_name_.c_str());
        }

        // Update last image timestamp
        last_image_timestamp_ = current_time;
    }

    // Write IMU angular velocity data to a CSV file
    void write_imu_data_to_csv(const sensor_msgs::msg::Imu::SharedPtr &message)
    {
        // Extract timestamp from the ROS IMU message
        double timestamp = message->header.stamp.sec + message->header.stamp.nanosec * 1e-9;
        RCLCPP_INFO(this->get_logger(), "IMU Timestamp: %f", timestamp);

        // Open the CSV file in append mode
        std::ofstream data_file(imu_file_name_, std::ios::app);
        if (data_file.is_open()) {
            // Write timestamp and scaled angular velocity values (x10 for demonstration)
            data_file << timestamp << " | "
                      << (message->angular_velocity.x * 10.0) << " | "
                      << (message->angular_velocity.y * 10.0) << " | "
                      << (message->angular_velocity.z * 10.0) << "\n";
            data_file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not open IMU data file: %s",
                         imu_file_name_.c_str());
        }
    }

    // Update the motion history and draw the motion graph
    void update_and_draw_graph(const cv::Point2f &avg_motion)
{
    // Add current average motion to history vectors
    motion_x_history_.push_back(static_cast<double>(avg_motion.x));
    motion_y_history_.push_back(static_cast<double>(avg_motion.y));

    // Maintain history size limit
    while (motion_x_history_.size() > max_history_size_) {
        motion_x_history_.erase(motion_x_history_.begin());
        motion_y_history_.erase(motion_y_history_.begin());
    }

    // If not enough data, display a placeholder message
    if (motion_x_history_.size() < 2) {
        graph_image_ = cv::Scalar(255, 255, 255);  // Clear graph (white background)
        cv::putText(graph_image_, "Collecting data...",
                    cv::Point(10, graph_height_ / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        return;
    }

    // Compute min/max for scaling
    auto x_minmax = std::minmax_element(motion_x_history_.begin(), motion_x_history_.end());
    auto y_minmax = std::minmax_element(motion_y_history_.begin(), motion_y_history_.end());

    double min_val = std::min(*x_minmax.first, *y_minmax.first);
    double max_val = std::max(*x_minmax.second, *y_minmax.second);
    double range = max_val - min_val;

    // Prepare a blank graph canvas (white background)
    graph_image_ = cv::Scalar(255, 255, 255);

    // Graph area dimensions (leave padding for labels)
    int padding = 30;
    int plot_width = graph_width_ - 2 * padding;
    int plot_height = graph_height_ - 2 * padding;

    // Draw Y-axis
    cv::line(graph_image_,
             cv::Point(padding, padding),
             cv::Point(padding, padding + plot_height),
             cv::Scalar(0, 0, 0), 1);

    // Compute Y-position of X-axis (where Y=0)
    int zero_y_pos = padding + plot_height - static_cast<int>((0.0 - min_val) / range * plot_height);
    // Manual clamp to keep zero_y_pos within graph bounds
    if (zero_y_pos < padding) zero_y_pos = padding;
    if (zero_y_pos > padding + plot_height) zero_y_pos = padding + plot_height;

    // Draw X-axis (zero line)
    cv::line(graph_image_,
             cv::Point(padding, zero_y_pos),
             cv::Point(padding + plot_width, zero_y_pos),
             cv::Scalar(0, 0, 0), 1);

    // Label the zero line
    cv::putText(graph_image_, "0",
                cv::Point(padding - 10, zero_y_pos + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // Labels for oldest/newest time
    cv::putText(graph_image_, cv::format("-%zu frames", max_history_size_),
                cv::Point(padding, graph_height_ - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(graph_image_, "Now",
                cv::Point(padding + plot_width - 20, graph_height_ - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    // Colors for plotting X and Y motion
    cv::Scalar color_x(255, 0, 0); // Blue
    cv::Scalar color_y(0, 0, 255); // Red

    // Draw motion data lines
    for (size_t i = 1; i < motion_x_history_.size(); ++i)
    {
        // Calculate plot coordinates for point i-1
        int x1 = padding + static_cast<int>(((double)(i - 1) / (max_history_size_ - 1)) * plot_width);
        int y1_x = padding + plot_height - static_cast<int>(((motion_x_history_[i - 1] - min_val) / range) * plot_height);
        int y1_y = padding + plot_height - static_cast<int>(((motion_y_history_[i - 1] - min_val) / range) * plot_height);

        // Calculate plot coordinates for point i
        int x2 = padding + static_cast<int>(((double)i / (max_history_size_ - 1)) * plot_width);
        int y2_x = padding + plot_height - static_cast<int>(((motion_x_history_[i] - min_val) / range) * plot_height);
        int y2_y = padding + plot_height - static_cast<int>(((motion_y_history_[i] - min_val) / range) * plot_height);

        // Manual clamping for Y-coordinates
        y1_x = std::max(padding, std::min(padding + plot_height, y1_x));
        y2_x = std::max(padding, std::min(padding + plot_height, y2_x));
        y1_y = std::max(padding, std::min(padding + plot_height, y1_y));
        y2_y = std::max(padding, std::min(padding + plot_height, y2_y));

        // Draw connecting lines for X and Y motion data
        cv::line(graph_image_, cv::Point(x1, y1_x), cv::Point(x2, y2_x), color_x, 2);
        cv::line(graph_image_, cv::Point(x1, y1_y), cv::Point(x2, y2_y), color_y, 2);
    }

    // Legend and axis labels
    cv::putText(graph_image_, "X Motion (Blue)", cv::Point(graph_width_ - 150, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color_x, 1);
    cv::putText(graph_image_, "Y Motion (Red)", cv::Point(graph_width_ - 150, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color_y, 1);
    cv::putText(graph_image_, "Frames -->", cv::Point(graph_width_ / 2 - 30, graph_height_ - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    // Y-axis min and max labels
    cv::putText(graph_image_, cv::format("%.1f", max_val),
                cv::Point(5, padding + 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(graph_image_, cv::format("%.1f", min_val),
                cv::Point(5, padding + plot_height - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
}


    // Member variables
    cv::Mat previous_image_;                          // Last received image
    cv::Mat graph_image_;                             // Image for drawing the motion graph
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    double last_image_timestamp_;                     // Timestamp of last image for averaging
    cv::Point2f avg_motion_;                          // Most recent average motion vector
    std::string image_file_name_{"src/zed_sub/data/IMG_output.txt"};  // Output CSV for image data
    std::string imu_file_name_{"src/zed_sub/data/IMU_output.txt"};    // Output CSV for IMU data
    std::vector<double> motion_x_history_;            // History of X motion
    std::vector<double> motion_y_history_;            // History of Y motion
    size_t max_history_size_;                         // Maximum number of points in history
    int graph_width_;                                 // Width of graph image
    int graph_height_;                                // Height of graph image
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Create and spin the node
    rclcpp::spin(std::make_shared<OpticalFlowNode>());
    rclcpp::shutdown();
    return 0;
}
