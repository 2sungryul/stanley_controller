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

class OvertakingStanleyController : public rclcpp::Node
{
public:
    OvertakingStanleyController() : Node("overtaking_stanley_controller")
    {
        // 파라미터 선언 (lane_change_stanley_controller와 동일)
        this->declare_parameter("k_gain", 2.5);
        this->declare_parameter("k_soft", 0.5);
        this->declare_parameter("max_linear_vel", 3.0);
        this->declare_parameter("min_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("wheelbase", 0.1);
        
        // 추월 관련 파라미터
        this->declare_parameter("safe_distance", 1.5);          // 추월 시작 거리
        this->declare_parameter("overtake_complete_distance", 2.5);  // 추월 완료 거리
        
        // 파라미터 가져오기
        k_gain_ = this->get_parameter("k_gain").as_double();
        k_soft_ = this->get_parameter("k_soft").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        overtake_complete_distance_ = this->get_parameter("overtake_complete_distance").as_double();
        
        // Subscribers
        plan1_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan1/desired_path", 10,
            std::bind(&OvertakingStanleyController::plan1_callback, this, std::placeholders::_1));
        
        plan2_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan2/desired_path", 10,
            std::bind(&OvertakingStanleyController::plan2_callback, this, std::placeholders::_1));
        
        turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&OvertakingStanleyController::turtle1_pose_callback, this, std::placeholders::_1));
        
        turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10,
            std::bind(&OvertakingStanleyController::turtle2_pose_callback, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&OvertakingStanleyController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Overtaking Stanley Controller Started");
        RCLCPP_INFO(this->get_logger(), "  K gain: %.2f", k_gain_);
        RCLCPP_INFO(this->get_logger(), "  Safe distance: %.2f m", safe_distance_);
        RCLCPP_INFO(this->get_logger(), "  Overtake complete distance: %.2f m", overtake_complete_distance_);
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
    
    void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pose_ = *msg;
        has_turtle1_pose_ = true;
    }
    
    void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle2_pose_ = *msg;
        has_turtle2_pose_ = true;
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
    
    int find_closest_point(const nav_msgs::msg::Path& path, const turtlesim::msg::Pose& pose)
    {
        if (path.poses.empty()) return -1;
        
        int closest_idx = 0;
        double min_dist = distance(pose.x, pose.y,
                                   path.poses[0].pose.position.x,
                                   path.poses[0].pose.position.y);
        
        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            double dist = distance(pose.x, pose.y,
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

    int find_lookahead_point(const nav_msgs::msg::Path& path, int current_idx)
    {
        // Lookahead distance만큼 앞선 포인트 찾기
        double accumulated_dist = 0.0;
        int lookahead_idx = current_idx;
        
        for (size_t i = current_idx; i < path.poses.size() - 1; ++i)
        {
            double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
            double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
            
            accumulated_dist += std::sqrt(dx * dx + dy * dy);
            
            if (accumulated_dist >= lookahead_distance_)
            {
                lookahead_idx = i + 1;
                break;
            }
        }
        
        // 끝에 도달하면 마지막 포인트 사용
        if (lookahead_idx == current_idx && 
            current_idx < static_cast<int>(path.poses.size()) - 1)
        {
            lookahead_idx = path.poses.size() - 1;
        }
        
        return lookahead_idx;
    }

    double calculate_cross_track_error(const nav_msgs::msg::Path& path, int idx, 
                                       const turtlesim::msg::Pose& pose)
    {
        if (idx < 0 || idx >= static_cast<int>(path.poses.size()))
            return 0.0;
        
        double path_x = path.poses[idx].pose.position.x;
        double path_y = path.poses[idx].pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(path.poses[idx].pose.orientation, q);
        double roll, pitch, path_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, path_yaw);
        
        //double dx = pose.x - path_x;
        //double dy = pose.y - path_y;

        double dx = path_x - pose.x;
        double dy = path_y - pose.y;
        
        double normal_x = -std::sin(path_yaw);
        double normal_y = std::cos(path_yaw);
        
        double cte = dx * normal_x + dy * normal_y;
        
        return cte;
    }
    
    double calculate_heading_error(const nav_msgs::msg::Path& path, int idx, 
                                   const turtlesim::msg::Pose& pose)
    {
        if (idx < 0 || idx >= static_cast<int>(path.poses.size()))
            return 0.0;
        
        tf2::Quaternion q;
        tf2::fromMsg(path.poses[idx].pose.orientation, q);
        double roll, pitch, path_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, path_yaw);
        
        double heading_error = normalize_angle(path_yaw - pose.theta);
        
        return heading_error;
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
        
        int closest_idx = find_closest_point(path, turtle1_pose_);

        closest_idx = find_lookahead_point(path, closest_idx);
        
        if (closest_idx < 0)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }
        
        // Cross-track error
        double cte = calculate_cross_track_error(path, closest_idx, turtle1_pose_);
        
        // Heading error
        double heading_error = calculate_heading_error(path, closest_idx, turtle1_pose_);
        
        // 속도 계산
        double velocity = max_linear_vel_;
        double abs_cte = std::abs(cte);
        
        /*if (abs_cte > 0.5)
        {
            velocity = std::max(min_linear_vel_, max_linear_vel_ * (1.0 - abs_cte * 0.3));            
        }*/

        
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
    
    // 추월 상태 머신
    void update_overtaking_state()
    {
        // Turtle1과 Turtle2 사이의 거리 계산
        double dist_to_turtle2 = distance(
            turtle1_pose_.x, turtle1_pose_.y,
            turtle2_pose_.x, turtle2_pose_.y);
        
        switch (overtaking_state_)
        {
            case OvertakingState::FOLLOWING_PLAN1:
            {
                // Plan1 추종 중 - Turtle2와의 거리 확인
                if (dist_to_turtle2 < safe_distance_)
                {
                    // 너무 가까우면 추월 시작
                    overtaking_state_ = OvertakingState::OVERTAKING;
                    RCLCPP_INFO(this->get_logger(), 
                               "=== OVERTAKING START === Distance: %.2f m", dist_to_turtle2);
                }
                break;
            }
            
            case OvertakingState::OVERTAKING:
            {
                // Plan2로 추월 중 - 충분히 멀어졌는지 확인
                if (dist_to_turtle2 > overtake_complete_distance_)
                {
                    // 충분히 멀어지면 복귀 시작
                    overtaking_state_ = OvertakingState::RETURNING;
                    RCLCPP_INFO(this->get_logger(), 
                               "=== RETURNING TO PLAN1 === Distance: %.2f m", dist_to_turtle2);
                }
                break;
            }
            
            case OvertakingState::RETURNING:
            {
                // Plan1으로 복귀 중 - Plan1에 가까워졌는지 확인
                int closest_idx = find_closest_point(plan1_path_, turtle1_pose_);
                if (closest_idx >= 0)
                {
                    double dist_to_plan1 = distance(
                        turtle1_pose_.x, turtle1_pose_.y,
                        plan1_path_.poses[closest_idx].pose.position.x,
                        plan1_path_.poses[closest_idx].pose.position.y);
                    
                    // Plan1에 충분히 가까워지면 복귀 완료
                    if (dist_to_plan1 < 0.5)
                    {
                        overtaking_state_ = OvertakingState::FOLLOWING_PLAN1;
                        RCLCPP_INFO(this->get_logger(), 
                                   "=== RETURN COMPLETE === Back to PLAN1");
                    }
                }
                break;
            }
        }
    }
    
    void control_loop()
    {
        if (!has_turtle1_pose_ || !has_turtle2_pose_ || !has_plan1_ || !has_plan2_)
        {
            return;
        }
        
        // 추월 상태 업데이트
        update_overtaking_state();
        
        // 현재 상태에 따라 추종할 경로 선택
        const nav_msgs::msg::Path* current_path;
        
        switch (overtaking_state_)
        {
            case OvertakingState::FOLLOWING_PLAN1:
                current_path = &plan1_path_;
                break;
                
            case OvertakingState::OVERTAKING:
                current_path = &plan2_path_;
                break;
                
            case OvertakingState::RETURNING:
                current_path = &plan1_path_;
                break;
                
            default:
                current_path = &plan1_path_;
                break;
        }
        
        // Stanley 제어 계산
        geometry_msgs::msg::Twist cmd_vel = calculate_stanley_control(*current_path);
        
        // 제어 명령 발행
        cmd_vel_pub_->publish(cmd_vel);
        
        // 주기적 상태 출력 (2초마다)
        static int counter = 0;
        if (++counter % 100 == 0)
        {
            double dist_to_turtle2 = distance(
                turtle1_pose_.x, turtle1_pose_.y,
                turtle2_pose_.x, turtle2_pose_.y);
            
            std::string state_str;
            switch (overtaking_state_)
            {
                case OvertakingState::FOLLOWING_PLAN1:
                    state_str = "FOLLOWING_PLAN1";
                    break;
                case OvertakingState::OVERTAKING:
                    state_str = "OVERTAKING (PLAN2)";
                    break;
                case OvertakingState::RETURNING:
                    state_str = "RETURNING (PLAN1)";
                    break;
            }
            
            int closest_idx = find_closest_point(*current_path, turtle1_pose_);
            double cte = closest_idx >= 0 ? 
                calculate_cross_track_error(*current_path, closest_idx, turtle1_pose_) : 0.0;
            
            RCLCPP_INFO(this->get_logger(),
                       "State: %s | Dist to T2: %.2f m | CTE: %.3f m",
                       state_str.c_str(), dist_to_turtle2, cte);
        }
    }
    
    // 추월 상태 열거형
    enum class OvertakingState
    {
        FOLLOWING_PLAN1,    // Plan1 추종 중
        OVERTAKING,         // Plan2로 추월 중
        RETURNING           // Plan1으로 복귀 중
    };
    
    // Subscribers & Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan1_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan2_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data
    nav_msgs::msg::Path plan1_path_;
    nav_msgs::msg::Path plan2_path_;
    turtlesim::msg::Pose turtle1_pose_;
    turtlesim::msg::Pose turtle2_pose_;
    
    bool has_plan1_ = false;
    bool has_plan2_ = false;
    bool has_turtle1_pose_ = false;
    bool has_turtle2_pose_ = false;
    
    // Overtaking state
    OvertakingState overtaking_state_ = OvertakingState::FOLLOWING_PLAN1;
    
    // Parameters
    double k_gain_;
    double k_soft_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double lookahead_distance_;
    double goal_tolerance_;
    double wheelbase_;
    double safe_distance_;
    double overtake_complete_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OvertakingStanleyController>();
    
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