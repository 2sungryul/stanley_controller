#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class LaneChangeStanleyController : public rclcpp::Node
{
public:
    LaneChangeStanleyController() : Node("lane_change_stanley_controller")
    {
        // 파라미터 선언 (원래 Stanley Controller와 동일)
        this->declare_parameter("k_gain", 4.5);
        this->declare_parameter("k_soft", 0.5);
        this->declare_parameter("max_linear_vel", 1.5);
        this->declare_parameter("min_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("wheelbase", 0.1);
        
        // 파라미터 가져오기
        k_gain_ = this->get_parameter("k_gain").as_double();
        k_soft_ = this->get_parameter("k_soft").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        
        // Subscribers
        plan1_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan1/desired_path", 10,
            std::bind(&LaneChangeStanleyController::plan1_callback, this, std::placeholders::_1));
        
        plan2_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan2/desired_path", 10,
            std::bind(&LaneChangeStanleyController::plan2_callback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&LaneChangeStanleyController::pose_callback, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&LaneChangeStanleyController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Lane Change Stanley Controller Started");
        RCLCPP_INFO(this->get_logger(), "  K gain: %.2f", k_gain_);
        RCLCPP_INFO(this->get_logger(), "  K soft: %.2f", k_soft_);
        RCLCPP_INFO(this->get_logger(), "  Max linear vel: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "  Starting with PLAN1");
    }

private:
    void plan1_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        plan1_path_ = *msg;
        has_plan1_ = true;
    }
    
    void plan2_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        plan2_path_ = *msg;
        has_plan2_ = true;
    }
    
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        has_pose_ = true;
    }
    
    double distance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    int find_closest_point(const nav_msgs::msg::Path& path)
    {
        if (path.poses.empty()) return -1;
        
        int closest_idx = 0;
        double min_dist = distance(current_pose_.x, current_pose_.y,
                                   path.poses[0].pose.position.x,
                                   path.poses[0].pose.position.y);
        
        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            double dist = distance(current_pose_.x, current_pose_.y,
                                  path.poses[i].pose.position.x,
                                  path.poses[i].pose.position.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    double calculate_cross_track_error(const nav_msgs::msg::Path& path, int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(path.poses.size()))
            return 0.0;
        
        double path_x = path.poses[idx].pose.position.x;
        double path_y = path.poses[idx].pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(path.poses[idx].pose.orientation, q);
        double roll, pitch, path_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, path_yaw);
        
        //double dx = current_pose_.x - path_x;
        //double dy = current_pose_.y - path_y;
        double dx = path_x-current_pose_.x;
        double dy = path_y-current_pose_.y;
        
        double normal_x = -std::sin(path_yaw);
        double normal_y = std::cos(path_yaw);
        
        double cte = dx * normal_x + dy * normal_y;
        
        return cte;
    }
    
    double calculate_heading_error(const nav_msgs::msg::Path& path, int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(path.poses.size()))
            return 0.0;
        
        tf2::Quaternion q;
        tf2::fromMsg(path.poses[idx].pose.orientation, q);
        double roll, pitch, path_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, path_yaw);
        
        double heading_error = normalize_angle(path_yaw - current_pose_.theta);
        
        return heading_error;
    }
    
    // 1회전 완료 감지 함수 (핵심 추가 기능)
    bool is_lap_completed(const nav_msgs::msg::Path& path)
    {
        if (path.poses.empty()) return false;
        
        int closest_idx = find_closest_point(path);
        if (closest_idx < 0) return false;
        
        // 경로 진행률 계산 (0.0 ~ 1.0)
        double progress = static_cast<double>(closest_idx) / 
                         static_cast<double>(path.poses.size() - 1);
        
        // 상태 머신 기반 랩 완료 감지
        if (!lap_started_ && progress > 0.1 && progress <0.5)
        {
            // 경로의 10% 이상 진행하면 랩 시작으로 간주
            lap_started_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Lap started (progress: %.1f%%)", progress * 100.0);
        }
        else if (lap_started_ && progress > 0.95)
        {
            // 경로의 90% 이상 완주하면 1회전 완료
            RCLCPP_DEBUG(this->get_logger(), "Lap completion detected (progress: %.1f%%)", progress * 100.0);
            return true;
        }
        
        return false;
    }
    
    geometry_msgs::msg::Twist calculate_stanley_control(const nav_msgs::msg::Path& path)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        if (path.poses.empty())
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }
        
        int closest_idx = find_closest_point(path);
        
        if (closest_idx < 0)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }
        
        // Cross-track error
        double cte = calculate_cross_track_error(path, closest_idx);
        
        // Heading error
        double heading_error = calculate_heading_error(path, closest_idx);
        
        // 속도 계산
        double velocity = max_linear_vel_;
        double abs_cte = std::abs(cte);
        if (abs_cte > 0.5)
        {
            velocity = std::max(min_linear_vel_, 
                              max_linear_vel_ * (1.0 - abs_cte * 0.3));
        }
        
        // Stanley control law
        double cross_track_term = std::atan2(k_gain_ * cte, k_soft_ + velocity);
        double steering_angle = heading_error + cross_track_term;
        
        // 각속도 계산
        double angular_velocity = steering_angle * 2.0;
        
        // 제한 적용
        angular_velocity = std::clamp(angular_velocity, 
                                     -max_angular_vel_, 
                                      max_angular_vel_);
        
        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = angular_velocity;
        
        return cmd_vel;
    }
    
    void control_loop()
    {
        if (!has_pose_ || !has_plan1_ || !has_plan2_)
        {
            return;
        }
        
        // 현재 추종할 경로 선택
        
        const nav_msgs::msg::Path& current_path = following_plan1_ ? 
                                                   plan1_path_ : plan2_path_;
        
        // 1회전 완료 확인 및 경로 전환
        
        if (is_lap_completed(current_path))
        {
            // 경로 전환: plan1 <-> plan2
            following_plan1_ = !following_plan1_;
            lap_started_ = false;  // 새 경로를 위해 리셋
            lap_count_++;
            
            RCLCPP_INFO(this->get_logger(), 
                       "=== Lap %d Completed! Switching to %s ===",
                       lap_count_,
                       following_plan1_ ? "PLAN1" : "PLAN2");
        }
          
        
        //const nav_msgs::msg::Path& current_path = plan1_path_;
        //const nav_msgs::msg::Path& current_path = plan2_path_;

        // Stanley 제어 계산
        geometry_msgs::msg::Twist cmd_vel = calculate_stanley_control(current_path);
        
        // 제어 명령 발행
        cmd_vel_pub_->publish(cmd_vel);
        
        // 주기적 상태 출력 (2초마다)
        
        static int counter = 0;
        if (++counter % 100 == 0)
        {
            int closest_idx = find_closest_point(current_path);
            double progress = closest_idx >= 0 ? 
                (static_cast<double>(closest_idx) / 
                 static_cast<double>(current_path.poses.size() - 1) * 100.0) : 0.0;
            
            RCLCPP_INFO(this->get_logger(),
                       "Path: %s | Lap: %d | Progress: %.1f%% | CTE: %.3f m",
                       following_plan1_ ? "PLAN1" : "PLAN2",
                       lap_count_ + 1,
                       progress,
                       closest_idx >= 0 ? 
                           calculate_cross_track_error(current_path, closest_idx) : 0.0);
        }
    }
    
    // Subscribers & Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan1_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan2_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data
    nav_msgs::msg::Path plan1_path_;
    nav_msgs::msg::Path plan2_path_;
    turtlesim::msg::Pose current_pose_;
    
    bool has_plan1_ = false;
    bool has_plan2_ = false;
    bool has_pose_ = false;
    
    // Lane change state (추가된 부분)
    bool following_plan1_ = true;   // 시작: plan1
    bool lap_started_ = false;      // 현재 랩 시작 여부
    int lap_count_ = 0;             // 완료한 랩 수
    
    // Parameters (원래 Stanley Controller와 동일)
    double k_gain_;
    double k_soft_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double lookahead_distance_;
    double goal_tolerance_;
    double wheelbase_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneChangeStanleyController>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}