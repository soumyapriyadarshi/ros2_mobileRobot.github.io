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

        goal_x_ = 9.0;
        goal_y_ = 8.0;

        mission_goal_x_ = goal_x_;
        mission_goal_y_ = goal_y_;

        attractive_gain_ = 5.0;
        repulsive_gain_ = 2.0;

        obstacle_distance_ = 1.5;
        // emergency_stop_distance_ = 1.0; 

        max_linear_velocity_ = 1.5;
        max_angular_velocity_ = 0.5;

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

        using Trigger = std_srvs::srv::Trigger;

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
        x_ = msg->pose.pose.position.x + spawn_x_;
        y_ = msg->pose.pose.position.y + spawn_y_;

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
        // REACTIVE AVOIDANCE (replaces the potential field forces)
        //  1. wall far   -> go forward
        //  2. wall near  -> stop, compare left / right
        //  3. a side is open -> turn that way
        // =========================================================

        if (!scan_received_)
        {
            stopRobot();
            return;
        }

        double goal_err = normalizeAngle(std::atan2(dy, dx) - yaw_);

        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        reactiveCommand(goal_err, linear_velocity, angular_velocity);

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
    // REACTIVE "GO FORWARD / CHECK SIDES / TURN TO OPEN SIDE" LOGIC
    // Paste these members inside class PotentialFieldPlanner (private:)
    // =============================================================

    enum class Mode { FORWARD, TURN_LEFT, TURN_RIGHT };
    Mode mode_ = Mode::FORWARD;
    int  turn_ticks_ = 0;                 // how long we have been turning (50 ms per tick)
    bool turn_back_ = false;
    double turn_start_yaw_ = 0.0;

    // ---- distances (metres, measured from the ROBOT BODY, not from the lidar) ----
    static constexpr double kFrontOffset = 0.54;  // lidar -> front edge of robot
    static constexpr double kSideOffset  = 0.49;  // lidar -> side edge of robot
    static constexpr double kFar   = 1.0;   // wall further than this  -> full speed;  also "front is clear again"
    static constexpr double kNear  = 0.5;   // wall nearer than this   -> stop and look at both sides
    static constexpr double kOpen  = 0.5;   // a side gap bigger than this counts as "open, can go that way"
    static constexpr double kCruise   = 0.5;  // m/s straight ahead
    static constexpr double kMinSpeed = 0.15; // m/s when approaching a wall
    static constexpr double kTurnRate = 0.5;  // rad/s when turning in place

    // Smallest valid range inside the angle window [a_from, a_to] (radians, 0 = straight ahead,
    // + = left). "inf" means the lidar saw nothing, so it counts as far away.
    double sectorMin(double a_from, double a_to) const
    {
        double best = scan_.range_max;
        for (size_t i = 0; i < scan_.ranges.size(); ++i)
        {
            double a = scan_.angle_min + static_cast<double>(i) * scan_.angle_increment;
            if (a < a_from || a > a_to) continue;
            double r = scan_.ranges[i];
            if (!std::isfinite(r) || r < scan_.range_min) continue;
            best = std::min(best, r);
        }
        return best;
    }

    // goal_err = (direction to goal) - yaw, only used as a tie-break when both sides are open.
    void reactiveCommand(double goal_err, double &lin, double &ang)
    {
        lin = 0.0;
        ang = 0.0;

        // free space between the robot's edge and the wall, in three directions
        const double front = sectorMin(-0.52,  0.52) - kFrontOffset;  // +-30 deg
        const double left  = sectorMin( 0.79,  2.36) - kSideOffset;   // 45..135 deg
        const double right = sectorMin(-2.36, -0.79) - kSideOffset;

        switch (mode_)
        {
        case Mode::FORWARD:
            if (front > kNear)
            {
                // RULE 1: wall far away -> go forward (slow down smoothly between kFar and kNear)
                double f = std::clamp((front - kNear) / (kFar - kNear), 0.0, 1.0);
                lin = kMinSpeed + (kCruise - kMinSpeed) * f;

                // gentle centring so the robot does not scrape a side wall
                double l = std::min(left, 1.5), r = std::min(right, 1.5);
                ang = std::clamp(0.4 * (l - r), -0.3, 0.3);
            }
            else
            {
                // RULE 2: wall is near -> stop and compare both sides
                const double left_run = std::min(sectorMin(1.05, 2.09) - kSideOffset, 6.0);
                const double right_run = std::min(sectorMin(-2.09, -1.05) - kSideOffset, 6.0);
                const double back_run = std::min(std::min(sectorMin(2.62, M_PI), 
                sectorMin(-M_PI, -2.62)) - kFrontOffset, 6.0);

                struct Cand { const char *name; double heading; double run; };
                const Cand cands[3] = {
                    {"LEFT",  +M_PI / 2, left_run},
                    {"RIGHT", -M_PI / 2, right_run},
                    {"BACK",   M_PI,     back_run}};

                // RULE 3: among the passable directions take the one that points most toward the goal.
                //   A small gap that leads toward the goal beats a big gap that leads away.
                //   score = how well it points at the goal (cos: +1 toward, -1 away)
                //           + a small bonus for extra free space (tie-break only)
                int best = -1;
                double best_score = -1e9;
                for (int i = 0; i < 3; ++i)
                {
                    if (cands[i].run <= kOpen) continue;                       // blocked
                    double align = std::cos(normalizeAngle(goal_err - cands[i].heading));
                    double score = align + 0.15 * std::min(cands[i].run, 3.0) / 3.0;
                    if (score > best_score) { best_score = score; best = i; }
                }

                // RULE 4: walls on every side -> go where the wall is farthest away
                if (best < 0)
                {
                    best = 0;
                    for (int i = 1; i < 3; ++i)
                        if (cands[i].run > cands[best].run) best = i;
                }

                turn_back_ = (best == 2);
                bool turn_left = (best == 0) || (best == 2 && left_run >= right_run);
                mode_ = turn_left ? Mode::TURN_LEFT : Mode::TURN_RIGHT;
                turn_ticks_ = 0;
                turn_start_yaw_ = yaw_;
                RCLCPP_INFO(this->get_logger(),
                    "Wall ahead (%.2f m). L=%.2f R=%.2f B=%.2f | goal %.0f deg -> choose %s",
                    front, left_run, right_run, back_run,
                    goal_err * 180.0 / M_PI, cands[best].name);
            }
            break;

        case Mode::TURN_LEFT:
        case Mode::TURN_RIGHT:
        {
            // commit to the chosen direction until the front is clear (kFar)
            ang = (mode_ == Mode::TURN_LEFT ? 1.0 : -1.0) * kTurnRate;
            double turned = std::abs(normalizeAngle(yaw_ - turn_start_yaw_));
            bool done = front > kFar && (!turn_back_ || turned > 2.6);   // BACK: must turn ~150 deg first
            if (done)
                mode_ = Mode::FORWARD;
            else if (++turn_ticks_ > 300)                                // 15 s stuck: try the other way
            {
                mode_ = (mode_ == Mode::TURN_LEFT) ? Mode::TURN_RIGHT : Mode::TURN_LEFT;
                turn_ticks_ = 0;
                turn_back_ = false;
            }
            break;
        }
        }
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

    // robot spawn pose in the world (must match gazebo.launch.py: -x -8 -y -8)
    double spawn_x_ = -9.0;
    double spawn_y_ = -8.0;

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