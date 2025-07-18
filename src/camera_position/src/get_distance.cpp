#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ImagePosition : public rclcpp::Node {
public:

  explicit ImagePosition(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("image_distance", node_options){

    callback_publisher_position_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_point_cloud_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_point_cloud;
    options_point_cloud.callback_group = callback_subscriber_point_cloud_group_;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Pose>("image_position", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&ImagePosition::timer_callback, this),
        callback_publisher_position_group_); 

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/aligned_depth_to_color/image_raw", 10, 
        std::bind(&ImagePosition::point_cloud_callback, this, std::placeholders::_1),
        options_point_cloud);
  }

private:
  void timer_callback() {
    get_real_position();
    auto message = geometry_msgs::msg::Pose();
    message.position.x = x_position;
    message.position.y = y_position;
    message.position.z = z_position;
    publisher_->publish(message);
  }

  void point_cloud_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::this_thread::sleep_for(50ms);
    // --- Direct Lookup Calculation ---
    // Get a pointer to the start of the data for the target row 'v'
    const uint16_t* row_ptr = reinterpret_cast<const uint16_t*>(msg->data.data() + v * msg->step);
    // Get the depth value at the column 'u'
    uint16_t depth_mm = row_ptr[u];
    // --------------------------------

    // A value of 0 often means the depth sensor had no reading
    if (depth_mm == 0) {
      RCLCPP_WARN(this->get_logger(), "No depth data at pixel (%d, %d)", u, v);
      return;
    }

    // Convert depth from millimeters (uint16_t) to meters (float)
    float z_position = static_cast<float>(depth_mm) / 1000.0f;

    if (std::isnan(z_position) || std::isinf(z_position)) {
      RCLCPP_WARN(this->get_logger(), "Invalid depth value (NaN or Inf) at pixel (%d, %d)", u, v);
      return;
    }
    std::cout << "Z: " << z_position << std::endl;
  }

  void get_real_position()
   {
    x_position = (u - cx)*z_position/fx;
    y_position = (v - cy)*z_position/fy;
  }

  int index_position = 0;
  int WIDTH = 640;
  int HEIGHT = 480;
  int fx = 607.4315;
  int fy = 607.5863;
  int cy = 234.2057;
  int cx = 324.3362;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr point_cloud_subscription_;


  float z_position;
  float x_position;
  float y_position;
  int u = 320;
  int v = 240;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_position_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_point_cloud_group_;
};

/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ImagePosition> get_image_pose_node =
      std::make_shared<ImagePosition>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(get_image_pose_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}