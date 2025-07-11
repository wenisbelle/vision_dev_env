#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "yolov8_obb_msgs/msg/yolov8_inference.hpp"

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
      : Node("image_position", node_options){

    // Initialize the MutuallyExclusive callback group object
    callback_publisher_postion_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_point_cloud_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_inference_camera_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_point_cloud;
    options_point_cloud.callback_group = callback_subscriber_point_cloud_group_;

    rclcpp::SubscriptionOptions options_inference;
    options_inference.callback_group = callback_subscriber_inference_camera_group_;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Pose>("grasp_position", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&ImagePosition::timer_callback, this),
        callback_publisher_postion_group_); 

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "", 10, //// colocar o tópico correto
        std::bind(&ImagePosition::point_cloud_callback, this, std::placeholders::_1),
        options_point_cloud);

    inference_subscription_ = this->create_subscription<yolov8_obb_msgs::msg::Yolov8Inference>(
        "", 10, //// colocar o tópico correto
        std::bind(&ImagePosition::inference_callback, this, std::placeholders::_1),
        options_inference);

  }


private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Pose();
    message.position.x = pos_x;
    message.position.y = pos_y;
    message.position.z = pos_z;
    publisher_->publish(message);
  }

  void point_cloud_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::this_thread::sleep_for(100ms);
    float pos_x = msg->ranges[360]; // Verificar os valores corretos
    float pos_y = msg->ranges[180]; // 180
  }

  void inference_callback(const yolov8_obb_msgs::msg::Yolov8Inference::SharedPtr msg) {
    std::this_thread::sleep_for(100ms);
    std::array<float, 6> bb_coordinates = msg->yolov8_inference.coordinates;
  }

  float get_depth(std::array<float, 6>& coordinates, float& x, float& y)
  {


    
  }

  

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_action_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp_action::Client<OdomRecord>::SharedPtr client_ptr_;
  bool goal_done_;
  float vx;
  float vz;
  rclcpp::CallbackGroup::SharedPtr callback_action_moverobot_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_moverobot_group_;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_moverobot_group_;
};

/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceClient>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  std::shared_ptr<MoveRobot> move_robot_service_action_node =
      std::make_shared<MoveRobot>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_robot_service_action_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}