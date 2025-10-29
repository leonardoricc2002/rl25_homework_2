#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <sstream>
#include <vector>

using std::placeholders::_1;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    // Parametro per scegliere il tipo di controller: "position" o "trajectory"
    this->declare_parameter<std::string>("mode", "position");
    this->get_parameter("mode", mode_);

    RCLCPP_INFO(this->get_logger(), "Controller mode: %s", mode_.c_str());

    // Subscriber per leggere lo stato dei giunti
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&ArmControllerNode::jointStateCallback, this, _1));

    // Publisher in base al tipo di controller
    if (mode_ == "position") {
      joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_position_controller/commands", 10);
    } else if (mode_ == "trajectory") {
      traj_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_trajectory_controller/joint_trajectory", 10);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s", mode_.c_str());
      rclcpp::shutdown();
    }

    // Timer per inviare comandi periodici ogni 0.1s
    cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArmControllerNode::sendJointCommands, this));

    // Timer per stampare lo stato dei giunti ogni 1s
    print_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ArmControllerNode::printJointStates, this));

    RCLCPP_INFO(this->get_logger(), "ArmControllerNode started!");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
  }

  void printJointStates()
  {
    if (!last_joint_state_) return;

    std::stringstream ss;
    ss << "Current joint positions: [";
    for (size_t i = 0; i < last_joint_state_->position.size(); ++i) {
      ss << last_joint_state_->position[i];
      if (i != last_joint_state_->position.size() - 1) ss << ", ";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void sendJointCommands()
  {
    double t = this->now().seconds();
    std::vector<double> positions = {0.1*sin(t), 0.1*cos(t), 0.05*sin(t), 0.05*cos(t)};

    if (mode_ == "position" && joint_cmd_pub_) {
      std_msgs::msg::Float64MultiArray cmd_msg;
      cmd_msg.data = positions;
      joint_cmd_pub_->publish(cmd_msg);

    } else if (mode_ == "trajectory" && traj_cmd_pub_) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.joint_names = {"j0", "j1", "j2", "j3"};

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = rclcpp::Duration::from_seconds(0.1);

      traj_msg.points.push_back(point);
      traj_cmd_pub_->publish(traj_msg);
    }
  }

  std::string mode_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_cmd_pub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr print_timer_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
