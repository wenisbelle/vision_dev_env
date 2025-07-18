#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"

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
    callback_publisher_position_group_ = this->create_callback_group(
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
        callback_publisher_position_group_); 

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10, 
        std::bind(&ImagePosition::point_cloud_callback, this, std::placeholders::_1),
        options_point_cloud);

    inference_subscription_ = this->create_subscription<yolov8_msgs::msg::Yolov8Inference>(
        "/Yolov8_Inference", 10,
        std::bind(&ImagePosition::inference_callback, this, std::placeholders::_1),
        options_inference);
  }


private:
  void timer_callback() {
    get_real_position;
    auto message = geometry_msgs::msg::Pose();
    message.position.x = pos_x;
    message.position.y = pos_y;
    message.position.z = pos_z;
    publisher_->publish(message);
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::this_thread::sleep_for(50ms);
    z_position = msg.data[index_position]
  }

  void inference_callback(const yolov8_msgs::msg::Yolov8Inference::SharedPtr msg) {
    std::this_thread::sleep_for(50ms);
    u = msg->yolov8_inference[0].center[0];
    v = msg->yolov8_inference[0].center[1];
    index_position = (v * WIDTH) + u;
  }

  get_real_position()
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr inference_subscription_;


  float z_position;
  float x_posititon;
  float y_postion;
  int u;
  int v;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_position_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_point_cloud_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_inference_camera_group_;
};

/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<MoveRobot> get_image_pose_node =
      std::make_shared<ImagePosition>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(get_image_pose_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}