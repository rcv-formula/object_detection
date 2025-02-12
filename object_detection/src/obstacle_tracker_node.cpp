// obstacle_tracker_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <memory>
#include <chrono>

// 상수 정의 (M_PI가 정의되어 있지 않은 경우)
constexpr double PI = 3.14159265358979323846;

// 각도를 -PI ~ PI 범위로 normalize 하는 함수
double normalizeAngle(double angle) {
  while (angle > PI) {
    angle -= 2.0 * PI;
  }
  while (angle < -PI) {
    angle += 2.0 * PI;
  }
  return angle;
}

struct TrackedObstacle {
  double x, y;      // 병합된 박스의 중심 위치
  double vx, vy;    // 상대 속도 (x, y 방향)
  int id;
  int missed_scans;
  rclcpp::Time last_update;  // 마지막 업데이트 시각
};

class ObstacleTracker : public rclcpp::Node {
public:
  ObstacleTracker() : Node("obstacle_tracker"), next_id_(0) {
    // 기존 파라미터: 마커 병합 및 스무딩
    this->declare_parameter<double>("merge_range_threshold", 0.3);
    this->declare_parameter<double>("smoothing_factor", 0.8);
    this->declare_parameter<int>("missed_scan_threshold", 3);
    // heading_smoothing_factor: (0~1) 기존 헤딩에 얼마나 무게를 줄 것인가 (값이 클수록 기존 헤딩을 더 유지)
    this->declare_parameter<double>("heading_smoothing_factor", 0.8);
    // heading_outlier_threshold: 새로 계산한 헤딩과 기존 헤딩의 차이가 이 값(라디안)보다 크면 업데이트를 제한
    this->declare_parameter<double>("heading_outlier_threshold", 0.5);
    // initial_heading_damping: 초기 헤딩의 영향력을 얼마나 약화시킬지 결정 (0~1; 작을수록 초기값 영향이 약함)
    this->declare_parameter<double>("initial_heading_damping", 0.5);
    // ★ 추가: 최종 odom 출력에 대한 저역 필터 파라미터 (0~1; 값이 클수록 이전 값에 무게를 더 줌)
    this->declare_parameter<double>("odom_output_smoothing_factor", 0.8);
    // ★ 추가: 헤딩 보정시 내 odom과의 blending 임계값 (라디안, 예: 0.3)
    this->declare_parameter<double>("heading_blend_threshold", 0.3);
    // ★ 추가: 이전 마커와 현재 마커 간 거리가 이 임계치(예: 1.0 m)를 초과하면 이전 값을 무시하도록 함
    this->declare_parameter<double>("heading_distance_threshold", 1.0);
    // ★ 추가: 내 odom의 헤딩 보정 강도를 결정하는 파라미터 (0~1; 값이 클수록 내 odom 헤딩에 가깝게 보정)
    this->declare_parameter<double>("my_odom_correction_weight", 0.5);

    this->get_parameter("merge_range_threshold", merge_range_threshold_);
    this->get_parameter("smoothing_factor", smoothing_factor_);
    this->get_parameter("missed_scan_threshold", missed_scan_threshold_);
    this->get_parameter("heading_smoothing_factor", heading_smoothing_factor_);
    this->get_parameter("heading_outlier_threshold", heading_outlier_threshold_);
    this->get_parameter("initial_heading_damping", initial_heading_damping_);
    this->get_parameter("odom_output_smoothing_factor", odom_output_smoothing_factor_);
    this->get_parameter("heading_blend_threshold", heading_blend_threshold_);
    this->get_parameter("heading_distance_threshold", heading_distance_threshold_);
    this->get_parameter("my_odom_correction_weight", my_odom_correction_weight_);

    // 초기 헤딩 연산을 위한 초기화
    has_smoothed_yaw_ = false;
    heading_update_count_ = 0;
    has_my_heading_ = false;
    new_obstacle_detected_ = false;
    has_smoothed_output_ = false;

    my_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&ObstacleTracker::myOdomCallback, this, std::placeholders::_1));

    // 메시지 구독 및 퍼블리시
    candidate_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/obstacle_candidates", 20, std::bind(&ObstacleTracker::candidateCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/opponent_odom", 20);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("Obst_tracker_visualization_marker", 20);
  
    // Timer for periodic /odom publishing
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ObstacleTracker::publish_last_odom, this));
  }
  
private:
  // 내 odom 메시지로부터 헤딩을 추출하는 콜백 (Quaternion을 yaw로 변환)
  void myOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                         msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                               msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    my_heading_ = std::atan2(siny, cosy);
    has_my_heading_ = true;
  }
  
  void candidateCallback(const geometry_msgs::msg::PointStamped::SharedPtr candidate) {
    auto now = this->now();
    bool matched = false;
    
    // 기존 장애물과의 매칭 및 스무딩 업데이트
    for (auto &obst : tracked_obstacles_) {
      double dx = candidate->point.x - obst.x;
      double dy = candidate->point.y - obst.y;
      if (std::sqrt(dx * dx + dy * dy) < merge_range_threshold_) {
        double old_x = obst.x;
        double old_y = obst.y;
        obst.x = smoothing_factor_ * obst.x + (1 - smoothing_factor_) * candidate->point.x;
        obst.y = smoothing_factor_ * obst.y + (1 - smoothing_factor_) * candidate->point.y;
        
        double dt = (now - obst.last_update).seconds();
        if (dt > 0.0) {
          obst.vx = (obst.x - old_x) / dt;
          obst.vy = (obst.y - old_y) / dt;
        }
        obst.last_update = now;
        obst.missed_scans = 0;
        matched = true;
        break;
      }
    }
    
    // 매칭되지 않은 경우 새로운 장애물 생성
    if (!matched) {
      TrackedObstacle new_obst;
      new_obst.x = candidate->point.x;
      new_obst.y = candidate->point.y;
      new_obst.vx = 0.0;
      new_obst.vy = 0.0;
      new_obst.id = next_id_++;
      new_obst.missed_scans = 0;
      new_obst.last_update = now;
      tracked_obstacles_.push_back(new_obst);
    }
    
    // 이번 스캔에서 업데이트되지 않은 장애물 제거
    for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end(); ) {
      it->missed_scans++;
      if (it->missed_scans > missed_scan_threshold_) {
        it = tracked_obstacles_.erase(it);
      } else {
        ++it;
      }
    }
    
    // 여러 장애물이 있을 경우 클러스터링하여 평균 장애물 정보 계산
    TrackedObstacle avg_obst;
    if (!tracked_obstacles_.empty()) {
      if (tracked_obstacles_.size() == 1) {
        avg_obst = tracked_obstacles_.front();
      } else {
        std::vector<bool> visited(tracked_obstacles_.size(), false);
        std::vector<std::vector<TrackedObstacle>> clusters;
        for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
          if (visited[i])
            continue;
          std::vector<TrackedObstacle> cluster;
          std::vector<size_t> stack;
          stack.push_back(i);
          visited[i] = true;
          while (!stack.empty()) {
            size_t idx = stack.back();
            stack.pop_back();
            cluster.push_back(tracked_obstacles_[idx]);
            for (size_t j = 0; j < tracked_obstacles_.size(); ++j) {
              if (!visited[j]) {
                double dx = tracked_obstacles_[j].x - tracked_obstacles_[idx].x;
                double dy = tracked_obstacles_[j].y - tracked_obstacles_[idx].y;
                if (std::sqrt(dx * dx + dy * dy) < merge_range_threshold_) {
                  visited[j] = true;
                  stack.push_back(j);
                }
              }
            }
          }
          clusters.push_back(cluster);
        }
        
        size_t best_cluster_index = 0;
        size_t best_cluster_size = 0;
        for (size_t i = 0; i < clusters.size(); ++i) {
          if (clusters[i].size() > best_cluster_size) {
            best_cluster_size = clusters[i].size();
            best_cluster_index = i;
          }
        }
        
        const auto &best_cluster = clusters[best_cluster_index];
        double sum_x = 0.0, sum_y = 0.0, sum_vx = 0.0, sum_vy = 0.0;
        rclcpp::Time latest_time = best_cluster[0].last_update;
        for (const auto &obst : best_cluster) {
          sum_x += obst.x;
          sum_y += obst.y;
          sum_vx += obst.vx;
          sum_vy += obst.vy;
          if (obst.last_update > latest_time)
            latest_time = obst.last_update;
        }
        size_t count = best_cluster.size();
        avg_obst.x = sum_x / count;
        avg_obst.y = sum_y / count;
        avg_obst.vx = sum_vx / count;
        avg_obst.vy = sum_vy / count;
        avg_obst.last_update = latest_time;
      }
    }
    
    // 새로 감지된 장애물 여부 플래그 설정:
    if (has_smoothed_output_) {
      double dx_heading = avg_obst.x - smoothed_output_x_;
      double dy_heading = avg_obst.y - smoothed_output_y_;
      double distance_heading = std::sqrt(dx_heading * dx_heading + dy_heading * dy_heading);
      new_obstacle_detected_ = (distance_heading > heading_distance_threshold_);
    } else {
      new_obstacle_detected_ = true; // 최초 감지 시
    }
    
    // 헤딩(방향) 스무딩 처리
    double new_yaw = std::atan2(avg_obst.vy, avg_obst.vx);
    if (new_obstacle_detected_) {
      smoothed_yaw_ = new_yaw;  // 초기화: 현재 측정된 헤딩을 사용
      heading_update_count_ = 0;
      has_smoothed_yaw_ = true;
      has_smoothed_output_ = false;
    } else {
      double diff = normalizeAngle(new_yaw - smoothed_yaw_);
      if (std::fabs(diff) > heading_outlier_threshold_) {
        diff = std::copysign(heading_outlier_threshold_, diff);
      }
      double effective_smoothing = heading_smoothing_factor_;
      if (heading_update_count_ < 10) {
        effective_smoothing *= initial_heading_damping_;
      }
      smoothed_yaw_ = normalizeAngle(smoothed_yaw_ + (1 - effective_smoothing) * diff);
      heading_update_count_++;
    }
    
    // 내 odom의 헤딩 보정 적용  
    if (has_my_heading_) {
      double diff_my = normalizeAngle(smoothed_yaw_ - my_heading_);
      if (std::fabs(diff_my) > (PI/2)) {
        diff_my = std::copysign(PI/2, diff_my);
        smoothed_yaw_ = normalizeAngle(my_heading_ + diff_my);
      }
      if (std::fabs(diff_my) < heading_blend_threshold_) {
        double blend_weight = my_odom_correction_weight_;
        double sin_blended = blend_weight * std::sin(my_heading_) + (1 - blend_weight) * std::sin(smoothed_yaw_);
        double cos_blended = blend_weight * std::cos(my_heading_) + (1 - blend_weight) * std::cos(smoothed_yaw_);
        smoothed_yaw_ = std::atan2(sin_blended, cos_blended);
      }
    }
    
    // rviz2 시각화 및 odom 메시지 생성을 위해 쿼터니언용 sin, cos 계산
    double sin_yaw = std::sin(smoothed_yaw_ * 0.5);
    double cos_yaw = std::cos(smoothed_yaw_ * 0.5);
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = candidate->header.frame_id;
    
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = now;
    marker_msg.header.frame_id = candidate->header.frame_id;
    marker_msg.ns = "obstacle_tracker";
    marker_msg.id = 0; // 단일 장애물 마커
    
    // ★ 최종 odom 출력을 부드럽게 하기 위해 추가적인 smoothing 적용 (위치 및 속도)
    if (!tracked_obstacles_.empty()) {
      if (!has_smoothed_output_) {
        smoothed_output_x_ = avg_obst.x;
        smoothed_output_y_ = avg_obst.y;
        smoothed_output_vx_ = avg_obst.vx;
        smoothed_output_vy_ = avg_obst.vy;
        has_smoothed_output_ = true;
      } else {
        smoothed_output_x_ = odom_output_smoothing_factor_ * smoothed_output_x_ + (1 - odom_output_smoothing_factor_) * avg_obst.x;
        smoothed_output_y_ = odom_output_smoothing_factor_ * smoothed_output_y_ + (1 - odom_output_smoothing_factor_) * avg_obst.y;
        smoothed_output_vx_ = odom_output_smoothing_factor_ * smoothed_output_vx_ + (1 - odom_output_smoothing_factor_) * avg_obst.vx;
        smoothed_output_vy_ = odom_output_smoothing_factor_ * smoothed_output_vy_ + (1 - odom_output_smoothing_factor_) * avg_obst.vy;
      }
      
      odom_msg.pose.pose.position.x = smoothed_output_x_;
      odom_msg.pose.pose.position.y = smoothed_output_y_;
      odom_msg.pose.pose.position.z = 0.0;
      
      odom_msg.twist.twist.linear.x = smoothed_output_vx_;
      odom_msg.twist.twist.linear.y = smoothed_output_vy_;
      odom_msg.twist.twist.linear.z = 0.0;
      
      odom_msg.pose.pose.orientation.x = 0.0;
      odom_msg.pose.pose.orientation.y = 0.0;
      odom_msg.pose.pose.orientation.z = sin_yaw;
      odom_msg.pose.pose.orientation.w = cos_yaw;
      
      odom_msg.twist.twist.angular.x = 0.0;
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
      
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.type = visualization_msgs::msg::Marker::CUBE;
      
      marker_msg.pose.position.x = smoothed_output_x_;
      marker_msg.pose.position.y = smoothed_output_y_;
      marker_msg.pose.position.z = 0.0;
      
      marker_msg.pose.orientation.x = 0.0;
      marker_msg.pose.orientation.y = 0.0;
      marker_msg.pose.orientation.z = sin_yaw;
      marker_msg.pose.orientation.w = cos_yaw;
      
      marker_msg.scale.x = 0.3;
      marker_msg.scale.y = 0.3;
      marker_msg.scale.z = 0.05;
      
      marker_msg.color.a = 1.0;
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 0.0;
      marker_msg.color.b = 0.0;
      
      RCLCPP_INFO(this->get_logger(),
        "Detected obstacle at (x: %.2f, y: %.2f) with velocity (vx: %.2f, vy: %.2f) and heading: %.2f rad",
        smoothed_output_x_, smoothed_output_y_, smoothed_output_vx_, smoothed_output_vy_, smoothed_yaw_);
    } else {
      has_smoothed_output_ = false;
      
      odom_msg.pose.pose.position.x = 0.0;
      odom_msg.pose.pose.position.y = 0.0;
      odom_msg.pose.pose.position.z = 0.0;
      
      odom_msg.pose.pose.orientation.x = 0.0;
      odom_msg.pose.pose.orientation.y = 0.0;
      odom_msg.pose.pose.orientation.z = 0.0;
      odom_msg.pose.pose.orientation.w = 1.0;
      
      odom_msg.twist.twist.linear.x = 0.0;
      odom_msg.twist.twist.linear.y = 0.0;
      odom_msg.twist.twist.linear.z = 0.0;
      odom_msg.twist.twist.angular.x = 0.0;
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
      
      marker_msg.action = visualization_msgs::msg::Marker::DELETE;
      
      RCLCPP_INFO(this->get_logger(), "No obstacles detected.");
    }
    
    // ★ 여기서 내 odom의 헤딩과 계산된 장애물 헤딩의 차이가 40도(약 0.698 rad) 이상이면 출력하지 않음
    if (has_my_heading_) {
      double heading_diff = std::fabs(normalizeAngle(smoothed_yaw_ - my_heading_));
      if (heading_diff >= (40.0 * PI / 180.0)) {
        RCLCPP_WARN(this->get_logger(), 
                    "Heading difference too large: %.2f rad (>= 40 deg). Not publishing obstacle odom.", 
                    heading_diff);
        // 마커 삭제 및 odom 기본값으로 설정
        marker_msg.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(marker_msg);
        last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
        return;
      }
    }
    
    // odom 메시지는 주기적으로 타이머 콜백에서 퍼블리시되므로, 마지막 메시지는 저장
    last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(odom_msg);
    marker_pub_->publish(marker_msg);
  }

  // 주기적으로 마지막 odom 메시지를 퍼블리시하는 타이머 콜백
  void publish_last_odom() {
    if (last_odom_msg_) {
      odom_pub_->publish(*last_odom_msg_);
    }
  }
  
  // 구독자 및 퍼블리셔
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr candidate_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  // 추적 대상 장애물 리스트
  std::vector<TrackedObstacle> tracked_obstacles_;
  int next_id_;
  
  double merge_range_threshold_;
  double smoothing_factor_;
  int missed_scan_threshold_;
  double heading_smoothing_factor_; 
  double heading_outlier_threshold_;
  double initial_heading_damping_;
  double smoothed_yaw_;
  bool has_smoothed_yaw_;
  int heading_update_count_;
  double my_heading_;
  bool has_my_heading_;
  double odom_output_smoothing_factor_;
  bool has_smoothed_output_;
  double smoothed_output_x_, smoothed_output_y_, smoothed_output_vx_, smoothed_output_vy_;
  double heading_blend_threshold_;
  double heading_distance_threshold_;  
  bool new_obstacle_detected_;
  double my_odom_correction_weight_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
