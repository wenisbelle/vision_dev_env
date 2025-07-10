#include "custom_messages/action/odom_record.hpp"
#include "custom_messages/srv/detail/find_wall__struct.hpp"
#include "custom_messages/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <cstdlib>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using FindWall = custom_messages::srv::FindWall;

/************************************
************FIRST CLASS *************
*************************************/

class MoveRobot : public rclcpp::Node {
public:
  using OdomRecord = custom_messages::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ClientGoalHandle<OdomRecord>;

  explicit MoveRobot(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("move_robot_service_action_node_v1", node_options),
        goal_done_(false) {

    // Initialize the MutuallyExclusive callback group object
    callback_publisher_moverobot_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_action_moverobot_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_moverobot_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_subscriber_moverobot_group_;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&MoveRobot::timer_callback, this),
        callback_publisher_moverobot_group_); // 500 antes

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MoveRobot::topic_callback, this, std::placeholders::_1),
        options1);

    this->client_ptr_ = rclcpp_action::create_client<OdomRecord>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record");

    this->timer_action_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&MoveRobot::send_goal, this),
        callback_action_moverobot_group_);
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_action_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = OdomRecord::Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<OdomRecord>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&MoveRobot::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&MoveRobot::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&MoveRobot::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vx;
    message.angular.z = vz;
    publisher_->publish(message);
  }

  void MoveRight() {
    vx = 0.1;
    vz = -0.2;
  }

  void MoveLeft() {
    vx = 0.1;
    vz = 0.2;
  }

  void MoveForward() {
    vx = 0.1;
    vz = 0.0;
  }

  void TurnLeft() {
    vx = 0.1;
    vz = 1.0;
  }

  void TurnRight() {
    vx = 0.1;
    vz = -1.0;
  }

  void HardTurn() {
    vx = 0.0;
    vz = 1.0;
  }

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::this_thread::sleep_for(100ms);
    float Foward1 = msg->ranges[360];
    float Right = msg->ranges[180]; // 180
    float Left = msg->ranges[520];  // 520

    if (Right > 0.3 && Foward1 > 0.5) {
      MoveRight();
    } else if (Right < 0.2 && Foward1 > 0.5) {
      MoveLeft();
    } else if (Right < 0.3 && Right > 0.2 && Foward1 > 0.5) {
      MoveForward();
    } else if (Foward1 < 0.5) {
      TurnLeft();
    }

    if (Left <= 0.15) {
      TurnRight();
    }

    if (Foward1 < 0.20) {
      HardTurn();
    }

    if (this->goal_done_ == true) {
      vx = 0;
      vz = 0;
    }
  }

  // Action client functions

  void goal_response_callback(
      std::shared_future<GoalHandleOdomRecord::SharedPtr> future) {
    // this is necessary because the system uses ros2 foxy e not humble
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleOdomRecord::SharedPtr,
      const std::shared_ptr<const OdomRecord::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Current distance: %f",
                feedback->current_total);
  }

  void result_callback(const GoalHandleOdomRecord::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    // possivelmente vou printar um a um

    for (int i = 0; i < result.result->list_of_odoms.size(); ++i) {
      std::cout << "Data sample: " << i + 1
                << ". X: " << result.result->list_of_odoms[i].x
                << ". Y: " << result.result->list_of_odoms[i].y
                << ". THETA: " << result.result->list_of_odoms[i].z
                << std::endl;
    }
    auto final_message = geometry_msgs::msg::Twist();
    final_message.linear.x = 0;
    final_message.angular.z = 0;
    publisher_->publish(final_message);
    std::cout << "Kill the node." << std::endl;
    rclcpp::shutdown();
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
************SECOND CLASS *************
*************************************/

class ServiceClient : public rclcpp::Node {
private:
  rclcpp::Client<FindWall>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_client_;
  bool service_done_ = false;

  void timer_client_callback() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<FindWall::Request>();

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_client_callback, this,
                           std::placeholders::_1));
  }

  void response_client_callback(rclcpp::Client<FindWall>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success");
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<custom_messages::srv::FindWall>("findwall");
    timer_client_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_client_callback, this));
  }

  bool is_service_done() const { return this->service_done_; }
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