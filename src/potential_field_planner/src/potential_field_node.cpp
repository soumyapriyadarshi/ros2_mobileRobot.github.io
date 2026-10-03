#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cmath>
#include <algorithm>
#include <memory>

class PotentialFieldPlanner : public rclcpp::Node
{
public:

    PotentialFieldPlanner():Node("potential_field_node"),
          x_(0.0),
          y_(0.0),
          yaw_(0.0),
          odom_received_(false),
          scan_received_(false)
    {
        // =========================================================
        // PARAMETERS
        // =========================================================

        goal_x_ = -5.0;
        goal_y_ = 5.0;

        mission_goal_x_ = goal_x_;
        mission_goal_y_ = goal_y_;

        attractive_gain_ = 2.5;
        repulsive_gain_ = 3.5;

        obstacle_distance_ = 0.25;
        // emergency_stop_distance_ = 1.0; 

        max_linear_velocity_ = 2.0;
        max_angular_velocity_ = 2.5;

        goal_tolerance_ = 0.15;

        // =========================================================
        // PUBLISHER
        // =========================================================

        cmd_pub_ =
            this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/diff_drive_controller/cmd_vel",
                10);

        // =========================================================
        // ODOMETRY SUBSCRIBER
        // =========================================================

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/diff_drive_controller/odom",
                10,
                std::bind(
                    &PotentialFieldPlanner::odomCallback,
                    this,
                    std::placeholders::_1));

        // =========================================================
        // LIDAR SUBSCRIBER
        // =========================================================

        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &PotentialFieldPlanner::scanCallback,
                    this,
                    std::placeholders::_1));

        // =========================================================
        // CONTROL TIMER
        // =========================================================

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &PotentialFieldPlanner::controlLoop,
                    this));

        using Trigger = std_srvs::srv::Trigger;\

        estop_srv_ = this->create_service<Trigger>(
            "emergency_stop",
            [this](const std::shared_ptr<Trigger::Request>,
                std::shared_ptr<Trigger::Response> res){
                    state_ = State::ESTOPPED;
                    stopRobot();
                    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP triggered");
                    res->success = true;
                    res->message = "Emergency stop engaged";
        });

        reset_srv_ = this -> create_service<Trigger>(
            "rest_to_start",
            [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
                if(!odom_received_){
                    res->success = false;
                    res->message = "No odometry yet";
                    return;
                }
                goal_x_ = init_x_;
                goal_y_ = init_y_;
                state_ = State::RETURNING_HOME;

                RCLCPP_WARN(this->get_logger(),
                    "Returning to start (%.2f, %.2f)", init_x_, init_y_
                );
                res->success = true;
                res->message = "Returning to start position";
            }
        );
        resume_srv_ = this->create_service<Trigger>("resume",
            [this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> res){
                goal_x_ = mission_goal_x_;
                goal_y_ = mission_goal_y_;
                state_ = State::RUNNING;
                RCLCPP_INFO(this->get_logger(), "Resuming mission");
                res->success = true;
                res->message = "Resumed";
        });



        RCLCPP_INFO(
            this->get_logger(),
            "Potential Field Planner started");

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: (%.2f, %.2f)",
            goal_x_,
            goal_y_);

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for odometry on /diff_drive_controller/odom");
    }

private:

    // =============================================================
    // ODOMETRY CALLBACK
    // =============================================================

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        // Quaternion -> yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        yaw_ = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));

        if (!odom_received_)
        {
            odom_received_ = true;

            init_x_ = x_;
            init_y_ = y_;
            init_yaw_ = yaw_;

            RCLCPP_INFO(
                this->get_logger(),
                "Odometry received!");

            RCLCPP_INFO(
                this->get_logger(),
                "Robot position: x=%.2f y=%.2f yaw=%.2f",
                x_,
                y_,
                yaw_);
        }
    }

    // =============================================================
    // LIDAR CALLBACK
    // =============================================================

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = *msg;

        if (!scan_received_)
        {
            scan_received_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Lidar data received!");
        }
    }

    // =============================================================
    // ANGLE NORMALIZATION
    // =============================================================

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    // =============================================================
    // CONTROL LOOP
    // =============================================================

    void controlLoop()
    {
        // ---------------------------------------------------------
        // Check odometry
        // ---------------------------------------------------------

        if (!odom_received_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for odometry...");

            stopRobot();
            return;
        }

        if(state_ == State::ESTOPPED){
            stopRobot();
            return;
        }

        // ---------------------------------------------------------
        // Distance to goal
        // ---------------------------------------------------------

        double dx = goal_x_ - x_;
        double dy = goal_y_ - y_;

        double distance_to_goal =
            std::sqrt(dx * dx + dy * dy);

        // ---------------------------------------------------------
        // Goal reached
        // ---------------------------------------------------------

        if (distance_to_goal < goal_tolerance_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Goal reached! x=%.2f y=%.2f",
                x_,
                y_);

            if (state_ == State::RETURNING_HOME){
                state_ = State::ESTOPPED;

                stopRobot();
                return;
            }

            stopRobot();
            return;
        }

        // =========================================================
        // ATTRACTIVE FORCE
        // =========================================================

        double attractive_x =
            attractive_gain_ * dx;

        double attractive_y =
            attractive_gain_ * dy;

        // =========================================================
        // REPULSIVE FORCE
        // =========================================================

        double repulsive_x = 0.0;
        double repulsive_y = 0.0;

        double min_range = 1000.0;

        if (scan_received_)
        {
            for (size_t i = 0; i < scan_.ranges.size(); i++)
            {
                double range = scan_.ranges[i];

                if (!std::isfinite(range))
                    continue;

                if (range <= 0.05)
                    continue;

                if (range < min_range){
                    min_range = range;
                }

                if (range > obstacle_distance_)
                    continue;

                double angle =
                    scan_.angle_min +
                    static_cast<double>(i) * scan_.angle_increment;

                // -------------------------------------------------
                // Repulsive potential
                // -------------------------------------------------

                double difference =
                    (1.0 / range) -
                    (1.0 / obstacle_distance_);

                double magnitude =
                    repulsive_gain_ *
                    difference /
                    (range * range);

                // Lidar obstacle direction
                double obstacle_x =
                    std::cos(angle);

                double obstacle_y =
                    std::sin(angle);

                // Push robot AWAY from obstacle
                repulsive_x -= magnitude * obstacle_x;

                repulsive_y -= magnitude * obstacle_y;
            }
        }

        if(min_range < emergency_stop_distance_){
            state_ = State::ESTOPPED;
            stopRobot();
            RCLCPP_ERROR(this->get_logger(), "AUTO EMERGENCY STOP");
            return;
        }

        // =========================================================
        // TOTAL FORCE
        // =========================================================

        double force_x =
            attractive_x + repulsive_x;

        double force_y =
            attractive_y + repulsive_y;

        // =========================================================
        // DESIRED DIRECTION
        // =========================================================

        double desired_angle =
            std::atan2(force_y, force_x);

        double angle_error =
            normalizeAngle(desired_angle - yaw_);

        // =========================================================
        // VELOCITY CONTROL
        // =========================================================

        double linear_velocity =
            max_linear_velocity_ *
            std::cos(angle_error);

        // Don't move backward
        if (linear_velocity < 0.0)
            linear_velocity = 0.0;

        double angular_velocity =
            2.0 * angle_error;

        // Limit velocities
        linear_velocity =
            std::clamp(
                linear_velocity,
                0.0,
                max_linear_velocity_);

        angular_velocity =
            std::clamp(
                angular_velocity,
                -max_angular_velocity_,
                max_angular_velocity_);

        // If robot is facing away from force,
        // rotate first.
        if (std::abs(angle_error) > 1.0)
        {
            linear_velocity = 0.0;
        }

        // =========================================================
        // PUBLISH COMMAND
        // =========================================================

        geometry_msgs::msg::TwistStamped cmd;

        cmd.header.stamp = this->get_clock()->now();

        cmd.header.frame_id = "base_link";

        cmd.twist.linear.x =
            linear_velocity;

        cmd.twist.linear.y = 0.0;

        cmd.twist.linear.z = 0.0;

        cmd.twist.angular.x = 0.0;

        cmd.twist.angular.y = 0.0;

        cmd.twist.angular.z =
            angular_velocity;

        cmd_pub_->publish(cmd);

        // =========================================================
        // DEBUG
        // =========================================================

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Robot: (%.2f, %.2f) | Goal: (%.2f, %.2f) | "
            "Distance: %.2f | Cmd: %.2f m/s %.2f rad/s",
            x_,
            y_,
            goal_x_,
            goal_y_,
            distance_to_goal,
            linear_velocity,
            angular_velocity);
    }

    // =============================================================
    // STOP ROBOT
    // =============================================================

    void stopRobot()
    {
        geometry_msgs::msg::TwistStamped cmd;

        cmd.header.stamp = this->get_clock()->now();

        cmd.header.frame_id = "base_link";

        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.linear.z = 0.0;

        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0;

        cmd_pub_->publish(cmd);
    }

    // =============================================================
    // VARIABLES
    // =============================================================

    // Robot pose
    double x_;
    double y_;
    double yaw_;

    // Goal
    double goal_x_;
    double goal_y_;

    // Potential field parameters
    double attractive_gain_;
    double repulsive_gain_;
    double obstacle_distance_;

    // Velocity limits
    double max_linear_velocity_;
    double max_angular_velocity_;

    double goal_tolerance_;

    // State
    bool odom_received_;
    bool scan_received_;

    sensor_msgs::msg::LaserScan scan_;

    // Emergency stop / reset
    enum class State { RUNNING, ESTOPPED, RETURNING_HOME };
    State state_ = State::RUNNING;

    double init_x_ = 0.0, init_y_ = 0.0, init_yaw_ = 0.0;
    double mission_goal_x_ = 0.0, mission_goal_y_ = 0.0;
    double emergency_stop_distance_ = 0.25;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

    // ROS
    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(
    int argc,
    char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PotentialFieldPlanner>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}