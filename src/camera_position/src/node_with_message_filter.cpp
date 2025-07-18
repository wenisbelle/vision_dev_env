#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class ImagePosition : public rclcpp::Node {
public:
  explicit ImagePosition(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("image_position", node_options) {

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("grasp_position", 10);

    // Use message_filters::Subscriber to wrap the normal subscriptions
    inference_sub_.subscribe(this, "/Yolov8_Inference");
    depth_cam_sub_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw");

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<yolov8_msgs::msg::Yolov8Inference, sensor_msgs::msg::Image>;

    // Create the synchronizer instance
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), 
        inference_sub_, depth_cam_sub_);

    synchronizer_->registerCallback(
        std::bind(&ImagePosition::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void synchronized_callback(
      const yolov8_msgs::msg::Yolov8Inference::ConstSharedPtr &inference_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
    RCLCPP_INFO(this->get_logger(), "Received synchronized messages!");

    if (inference_msg->yolov8_inference.empty()) {
      RCLCPP_WARN(this->get_logger(), "Inference message is empty.");
      return;
    }
    int u = inference_msg->yolov8_inference[0].center[0];
    int v = inference_msg->yolov8_inference[0].center[1];

    const uint16_t* row_ptr = reinterpret_cast<const uint16_t*>(depth_msg->data.data() + v * depth_msg->step);
    uint16_t depth_mm = row_ptr[u];

    if (depth_mm == 0) {
      RCLCPP_WARN(this->get_logger(), "No depth data at pixel (%d, %d)", u, v);
      return;
    }

    float z_camera = static_cast<float>(depth_mm) / 1000.0f;

    auto message = geometry_msgs::msg::Pose();
    message.position.x = (u - cx)*z_camera/fx;
    message.position.y = (v - cy)*z_camera/fy;
    message.position.z = z_camera;
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

  message_filters::Subscriber<yolov8_msgs::msg::Yolov8Inference> inference_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_cam_sub_;
  
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      yolov8_msgs::msg::Yolov8Inference, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  int WIDTH = 640;
  int HEIGHT = 480;
  int fx = 607.4315;
  int fy = 607.5863;
  int cy = 234.2057;
  int cx = 324.3362;
};

/************************************
************** MAIN *****************
*************************************/
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto get_image_pose_node = std::make_shared<ImagePosition>();
  executor.add_node(get_image_pose_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}