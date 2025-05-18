#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::vector;
using std::string;

/**
 * @brief A ROS 2 node that controls a Universal Robot (UR) arm.
 *
 * This node subscribes to the /joint_states topic for the current joint states of the arm.
 * It also subscribes to the /io_states topic for the current IO states of the arm.
 * Furthermore, it subscribes to the /target_pose topic to receive the target pose of the arm.
 * The node then publishes the target pose to the /ur/target_pose topic.
 * Additionally, it publishes a sinusoidal joint state to the /joint_states topic.
 */
class URControlNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the URControlNode class.
   *
   * Initializes the node, creates subscribers and publishers, and sets up the timer.
   */
  URControlNode() : Node("ur_control_node"), current_time_(0.0)
  {
    // Create subscribers
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&URControlNode::joint_states_callback, this, std::placeholders::_1));

    io_states_sub_ = this->create_subscription<ur_msgs::msg::IOStates>(
      "/io_states", 10,
      std::bind(&URControlNode::io_states_callback, this, std::placeholders::_1));

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/ur/target_pose", 10,
      std::bind(&URControlNode::target_pose_callback, this, std::placeholders::_1));

    // Create publishers
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ur/target_pose", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Create timer for continuous joint movement
    timer_ = this->create_wall_timer(
      100ms, std::bind(&URControlNode::timer_callback, this));

    // Initialize joint names
    joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    RCLCPP_INFO(this->get_logger(), "UR Control Node has been initialized");
  }

private:
  /**
   * @brief Callback function for the /joint_states topic.
   *
   * This function is called whenever a new message is received on the /joint_states topic.
   * It simply logs a debug message to indicate that a new joint state has been received.
   */
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received joint states");
  }

  /**
   * @brief Callback function for the /io_states topic.
   *
   * This function is called whenever a new message is received on the /io_states topic.
   * It simply logs a debug message to indicate that a new IO state has been received.
   */
  void io_states_callback(const ur_msgs::msg::IOStates::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received IO states");
  }

  /**
   * @brief Callback function for the /target_pose topic.
   *
   * This function is called whenever a new message is received on the /target_pose topic.
   * It logs an info message to indicate that a new target pose has been received, and stores
   * the target pose for visualization.
   */
  void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received target pose - Position: [%.3f, %.3f, %.3f]",
                msg->position.x, msg->position.y, msg->position.z);
    // Store target position for visualization
    target_x_ = msg->position.x;
    target_y_ = msg->position.y;
    target_z_ = msg->position.z;
  }

  /**
   * @brief Timer callback function.
   *
   * This function is called periodically by the timer. It creates a sinusoidal joint state
   * and publishes it to the /joint_states topic.
   */
   void timer_callback()
   {
     current_time_ += 0.1;
     double pos = 2.0 * sin(current_time_);
     RCLCPP_INFO(this->get_logger(), "Publishing simulated joint position: %.2f", pos);
   
     auto joint_state = sensor_msgs::msg::JointState();
     joint_state.header.stamp = this->now();
     joint_state.name = joint_names_;
    
    // Create a dramatic sinusoidal movement
    joint_state.position = {
      1.0 * sin(current_time_),  // Shoulder pan
      1.0/2 * cos(current_time_),  // Shoulder lift
      1.1* sin(current_time_),  // Elbow
      1.0 + current_time_,  // Wrist 1
      1.2 + current_time_,  // Wrist 2
      0.8 + current_time_   // Wrist 3
    };
   
     joint_state_pub_->publish(joint_state);
   }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  vector<string> joint_names_;
  double current_time_;
  double target_x_{0.0}, target_y_{0.0}, target_z_{0.0};
};

// UR Control Node - Main entry point for Universal Robots controller
// Manages ROS 2 node lifecycle and real-time control loop
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<URControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/**
 * @brief Main control loop iteration
 * @param time Current simulation time
 * @param period Time since last iteration
 * @returns true if update succeeded, false otherwise
 * 
 * 1. Reads current joint states from hardware interface
 * 2. Calculates desired motion using trajectory planner
 * 3. Applies safety checks and velocity limits
 * 4. Sends new target positions to robot controller
 */
// bool update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
//   // Core control logic
// }

// Configuration parameters (loaded from ROS params)
// struct Params {
//   double control_frequency = 100.0;  // Hz (loop rate)
//   double max_joint_velocity = 1.57;  // rad/s (≈90°/s)
//   std::string control_mode = "position";  // position/velocity/torque
// } params_;
