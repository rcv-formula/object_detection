// visualization_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class ObstacleVisualizer : public rclcpp::Node {
public:
  ObstacleVisualizer() : Node("obstacle_visualizer") {
    tracked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/tracked_obstacles", 20,
      std::bind(&ObstacleVisualizer::trackedCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/obstacle_visualization_marker_array", 20);
  }
private:
  void trackedCallback(const geometry_msgs::msg::PointStamped::SharedPtr tracked) {
    visualization_msgs::msg::MarkerArray marker_array;
    // 기존 마커 삭제 (DELETEALL 액션)
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = this->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // 단일 장애물을 마커로 표시 (실제 애플리케이션에서는 여러 장애물 정보를 누적하여 표시)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "tracked_obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = tracked->point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
    // 예시: 단일 장애물이므로 상대 차량 후보 색상(노란색) 사용
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tracked_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
