// time_diff_publisher_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

class TimeDiffPublisher : public rclcpp::Node
{
public:
  TimeDiffPublisher() : Node("time_diff_publisher")
  {
    // /scan 토픽 구독 (sensor_msgs::msg::LaserScan)
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&TimeDiffPublisher::scanCallback, this, std::placeholders::_1));
    
    // /obstacle_candidates 토픽 구독 (geometry_msgs::msg::PointStamped)
    obstacle_candidates_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/obstacle_candidates", 10,
      std::bind(&TimeDiffPublisher::obstacleCandidatesCallback, this, std::placeholders::_1));
    
    // /opponent_odom 토픽 구독 (nav_msgs::msg::Odometry)
    opponent_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/opponent_odom", 10,
      std::bind(&TimeDiffPublisher::opponentOdomCallback, this, std::placeholders::_1));
    
    // 시간 차이를 나타내는 토픽 (geometry_msgs::msg::Vector3) 퍼블리셔 생성
    time_diff_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/time_differences", 10);
    
    // 1초 주기로 타이머 콜백을 통해 시간 차이 계산 및 publish
    timer_ = this->create_wall_timer(
      1s, std::bind(&TimeDiffPublisher::timerCallback, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_scan_time_ = rclcpp::Time(msg->header.stamp);
  }
  
  void obstacleCandidatesCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_obstacle_candidate_time_ = rclcpp::Time(msg->header.stamp);
  }
  
  void opponentOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_opponent_odom_time_ = rclcpp::Time(msg->header.stamp);
  }
  
  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 세 토픽 모두 한 번 이상 메시지를 받았는지 확인
    if (last_scan_time_.nanoseconds() == 0 ||
        last_obstacle_candidate_time_.nanoseconds() == 0 ||
        last_opponent_odom_time_.nanoseconds() == 0)
    {
      return;
    }
    
    // 각 토픽의 timestamp 차이 (절대값) 계산
    rclcpp::Duration diff_scan_obstacle = last_scan_time_ - last_obstacle_candidate_time_;
    rclcpp::Duration diff_scan_odom = last_scan_time_ - last_opponent_odom_time_;
    rclcpp::Duration diff_obstacle_odom = last_obstacle_candidate_time_ - last_opponent_odom_time_;
    
    double diff_scan_obstacle_sec = std::abs(diff_scan_obstacle.seconds());
    double diff_scan_odom_sec = std::abs(diff_scan_odom.seconds());
    double diff_obstacle_odom_sec = std::abs(diff_obstacle_odom.seconds());
    
    // geometry_msgs::msg::Vector3 메시지에 각 시간 차이 대입
    geometry_msgs::msg::Vector3 diff_msg;
    diff_msg.x = diff_scan_obstacle_sec;
    diff_msg.y = diff_scan_odom_sec;
    diff_msg.z = diff_obstacle_odom_sec;
    
    time_diff_pub_->publish(diff_msg);
  }
  
  // 구독자, 퍼블리셔, 타이머
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_candidates_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opponent_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr time_diff_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 마지막으로 수신한 timestamp 저장 (스레드 안전을 위해 mutex 사용)
  rclcpp::Time last_scan_time_{0, 0};
  rclcpp::Time last_obstacle_candidate_time_{0, 0};
  rclcpp::Time last_opponent_odom_time_{0, 0};
  
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeDiffPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
