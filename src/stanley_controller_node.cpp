#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class StanleyController : public rclcpp::Node
{
public:
    StanleyController() : Node("stanley_controller")
    {
        // 파라미터 선언
        this->declare_parameter("k_gain", 1.5);           // Cross-track error gain
        //this->declare_parameter("k_gain", 5.5);           // Cross-track error gain
        this->declare_parameter("k_soft", 0.5);           // Softening gain for steering
        this->declare_parameter("max_linear_vel", 2.0);   // Maximum linear velocity
        this->declare_parameter("min_linear_vel", 0.5);   // Minimum linear velocity
        this->declare_parameter("max_angular_vel", 2.0);  // Maximum angular velocity
        this->declare_parameter("lookahead_distance", 0.1); // Lookahead distance
        this->declare_parameter("goal_tolerance", 0.1);   // Goal reached tolerance
        this->declare_parameter("wheelbase", 0.1);        // Vehicle wheelbase (for turtlesim)
        
        // 파라미터 가져오기
        k_gain_ = this->get_parameter("k_gain").as_double();
        k_soft_ = this->get_parameter("k_soft").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        
        // Subscriber 생성
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&StanleyController::pose_callback, this, std::placeholders::_1));
        
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/desired_path",
            10,
            std::bind(&StanleyController::path_callback, this, std::placeholders::_1));
        
        // Publisher 생성
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        
        // 타이머 생성 (제어 루프, 10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&StanleyController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Stanley Controller started");
        RCLCPP_INFO(this->get_logger(), "  k_gain: %.2f", k_gain_);
        RCLCPP_INFO(this->get_logger(), "  k_soft: %.2f", k_soft_);
        RCLCPP_INFO(this->get_logger(), "  max_linear_vel: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "  goal_tolerance: %.2f m", goal_tolerance_);
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }
        
        desired_path_ = *msg;
        path_received_ = true;
        goal_reached_ = false;
        
        //RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", desired_path_.poses.size());
    }
    
    void control_loop()
    {
        if (!pose_received_ || !path_received_)
        {
            // 데이터가 없으면 대기
            return;
        }
        
        /*if (goal_reached_)
        {
            // 목표에 도달하면 정지
            publish_cmd_vel(0.0, 0.0);
            return;
        }*/
        
        // Stanley 제어 알고리즘 실행
        stanley_control();
    }
    
    void stanley_control()
    {
        // 1. 가장 가까운 경로 점 찾기
        int closest_idx = find_closest_point();
        
        if (closest_idx < 0)
        {
            RCLCPP_WARN(this->get_logger(), "No valid path point found");
            publish_cmd_vel(0.0, 0.0);
            return;
        }
        
        // 2. 목표 지점까지의 거리 확인
        double dist_to_goal = distance_to_point(
            desired_path_.poses.back().pose.position.x,
            desired_path_.poses.back().pose.position.y);
                
        /*if (dist_to_goal < goal_tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            goal_reached_ = true;
            publish_cmd_vel(0.0, 0.0);
            return;
        }*/
        
        // 3. Lookahead 포인트 찾기
        int target_idx = find_lookahead_point(closest_idx);
        
        // 4. Cross-track error 계산
        double cross_track_error = calculate_cross_track_error(target_idx);
        
        // 5. Heading error 계산
        double heading_error = calculate_heading_error(target_idx);
        
        // 6. Stanley steering angle 계산
        double velocity = calculate_velocity(dist_to_goal);
        double steering_angle = calculate_stanley_steering(
            heading_error, cross_track_error, velocity);
        
        // 7. 각속도 계산 (Ackermann-like for differential drive)
        double angular_velocity = calculate_angular_velocity(steering_angle, velocity);
        
        // 8. 제어 명령 발행
        publish_cmd_vel(velocity, angular_velocity);
        
        // 로그 출력 (5초마다)
        static int log_counter = 0;
        //if (++log_counter % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(), 
                       "CTE:%.3f,Heading error:%.3f°,Vel:%.2f,Ang_vel:%.2f",
                       cross_track_error, heading_error * 180.0 / M_PI, 
                       velocity, angular_velocity);
        }
    }
    
    int find_closest_point()
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_idx = -1;
        
        for (size_t i = 0; i < desired_path_.poses.size(); ++i)
        {
            double dist = distance_to_point(
                desired_path_.poses[i].pose.position.x,
                desired_path_.poses[i].pose.position.y);
            
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    int find_lookahead_point(int current_idx)
    {
        // Lookahead distance만큼 앞선 포인트 찾기
        double accumulated_dist = 0.0;
        int lookahead_idx = current_idx;
        
        for (size_t i = current_idx; i < desired_path_.poses.size() - 1; ++i)
        {
            double dx = desired_path_.poses[i + 1].pose.position.x - 
                       desired_path_.poses[i].pose.position.x;
            double dy = desired_path_.poses[i + 1].pose.position.y - 
                       desired_path_.poses[i].pose.position.y;
            
            accumulated_dist += std::sqrt(dx * dx + dy * dy);
            
            if (accumulated_dist >= lookahead_distance_)
            {
                lookahead_idx = i + 1;
                break;
            }
        }
        
        // 끝에 도달하면 마지막 포인트 사용
        if (lookahead_idx == current_idx && 
            current_idx < static_cast<int>(desired_path_.poses.size()) - 1)
        {
            lookahead_idx = desired_path_.poses.size() - 1;
        }
        
        return lookahead_idx;
    }
    
    double calculate_cross_track_error(int target_idx)
    {
        // 현재 위치에서 목표 경로까지의 수직 거리
        double path_x = desired_path_.poses[target_idx].pose.position.x;
        double path_y = desired_path_.poses[target_idx].pose.position.y;
        
        // 경로의 방향 벡터 계산
        double path_yaw = get_yaw_from_pose(desired_path_.poses[target_idx]);
        
        // 현재 위치에서 경로 점까지의 벡터
        //double dx = current_pose_.x - path_x;
        //double dy = current_pose_.y - path_y;
        
        double dx = path_x - current_pose_.x;
        double dy = path_y - current_pose_.y;        
        
        // Cross-track error (경로에 수직인 방향의 거리)
        // 경로의 법선 벡터와 내적
        // Cross-track error: 경로의 왼쪽 법선 방향으로의 투영
        // 법선 벡터 N = [-sin(θ), cos(θ)]
        // CTE = N · [dx, dy]
        double cross_track_error = -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;
                
        return cross_track_error;
    }
    
    double calculate_heading_error(int target_idx)
    {
        // 경로의 방향과 현재 방향의 차이
        double path_yaw = get_yaw_from_pose(desired_path_.poses[target_idx]);
        double heading_error = normalize_angle(path_yaw - current_pose_.theta);
               
        return heading_error;
    }
    
    double calculate_stanley_steering(double heading_error, 
                                      double cross_track_error, 
                                      double velocity)
    {
        // Stanley 조향 법칙:
        // δ = θ_e + arctan(k * e / (k_s + v))
        // θ_e: heading error
        // e: cross-track error
        // k: gain
        // k_s: softening constant
        // v: velocity
        
        double cross_track_term = std::atan(
            k_gain_ * cross_track_error / (k_soft_ + velocity));
        
        double steering_angle = heading_error + cross_track_term;
        
        return steering_angle;
    }
    
    double calculate_velocity(double dist_to_goal)
    {
        // 목표 지점에 가까워질수록 감속
        double velocity = max_linear_vel_;
        
        // 목표 지점 근처에서 감속
        /*if (dist_to_goal < 1.0)
        {
            velocity = std::max(
                min_linear_vel_,
                max_linear_vel_ * dist_to_goal);            
        }*/
        
        return velocity;
    }
    
    double calculate_angular_velocity(double steering_angle, double velocity)
    {
        // Ackermann 조향을 차동 구동으로 근사
        // ω = v * tan(δ) / L
        // L: wheelbase
        
        double angular_velocity = velocity * std::tan(steering_angle) / wheelbase_;
        
        // 각속도 제한
        angular_velocity = std::clamp(angular_velocity, 
                                     -max_angular_vel_, 
                                     max_angular_vel_);
        
        return angular_velocity;
    }
    
    void publish_cmd_vel(double linear, double angular)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;
        
        cmd_vel_publisher_->publish(cmd_vel);
    }
    
    double distance_to_point(double x, double y)
    {
        double dx = current_pose_.x - x;
        double dy = current_pose_.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double get_yaw_from_pose(const geometry_msgs::msg::PoseStamped& pose_stamped)
    {
        tf2::Quaternion q(
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        return yaw;
    }
    
    double normalize_angle(double angle)
    {
        // 각도를 -π ~ π 범위로 정규화
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // 멤버 변수
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    turtlesim::msg::Pose current_pose_;
    nav_msgs::msg::Path desired_path_;
    
    bool pose_received_ = false;
    bool path_received_ = false;
    bool goal_reached_ = false;
    
    // 제어 파라미터
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
    
    auto node = std::make_shared<StanleyController>();
    
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
