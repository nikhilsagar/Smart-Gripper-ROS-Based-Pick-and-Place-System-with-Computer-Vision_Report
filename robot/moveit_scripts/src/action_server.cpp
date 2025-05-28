#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class IKActionServer : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  IKActionServer() : Node("ik_action_server")
  {
    // Creating IK service client
    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    while (!ik_client_->wait_for_service(5s))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for IK service to be available...");
    }

    // Creating action server
    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "follow_ik_trajectory",
      std::bind(&IKActionServer::handle_goal, this, _1, _2),
      std::bind(&IKActionServer::handle_cancel, this, _1),
      std::bind(&IKActionServer::handle_accepted, this, _1));

    // Creating joint_states subscriber
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IKActionServer::jointStateCallback, this, _1));

    // Setting up serial communication with Arduino
    serial_port_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port to Arduino.");
    } else {
      struct termios tty;
      if (tcgetattr(serial_port_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes.");
      }

      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
      tty.c_iflag &= ~IGNBRK;
      tty.c_lflag = 0;
      tty.c_oflag = 0;
      tty.c_cc[VMIN]  = 0;
      tty.c_cc[VTIME] = 5;
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_cflag &= ~(PARENB | PARODD);
      tty.c_cflag &= ~CSTOPB;
      tty.c_cflag &= ~CRTSCTS;

      if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Serial port to Arduino initialized.");
      }
    }

    RCLCPP_INFO(this->get_logger(), "IK Action Server has been started");
  }

private:
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  const std::string PLANNING_GROUP = "arm_2";
  std::promise<std::vector<double>> ik_promise_;
  int serial_port_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    
    // Checking if the trajectory message is not empty
    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty trajectory received");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    // Executing goal in a separate thread
    std::thread{std::bind(&IKActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    // Getting the goal message with the cartesian position and orientation (angle relative to horizontal)
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    // Extracting position and orientation from the goal
    // Format: [x, y, z, angle]
    if (goal->trajectory.points.size() == 0 || goal->trajectory.points[0].positions.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Invalid trajectory input: expecting at least 4 position values");
      goal_handle->abort(result);
      return;
    }

    double x = goal->trajectory.points[0].positions[0];
    double y = goal->trajectory.points[0].positions[1];
    double z = goal->trajectory.points[0].positions[2];
    double angle_ = goal->trajectory.points[0].positions[3];
    RCLCPP_INFO(this->get_logger(), "Target position: %f, %f, %f, angle: %f", x, y, z, angle_);

    // Calibrating for error
    double calibrated_x = (x > 0) ? x + (-0.01177 * angle_ + -0.00314) : x + (0.01177 * angle_ + -0.00314);
    double calibrated_y = (x > 0) ? y + (-0.01285 * angle_ + 0.00443) : y + (-0.01177 * angle_ + -0.00314);
    double calibrated_z = z + (-0.00961 * (angle_ * angle_) + 0.00203 * angle_ + 0.02276);
    RCLCPP_INFO(this->get_logger(), "Compensated coords: %f, %f, %f", calibrated_x, calibrated_y, calibrated_z);

    // Calculating adjusted position for link_4
    double xd = 0.028884, yd = 0.095638, zd = -0.045587;
    double l = sqrt(xd * xd + yd * yd + zd * zd);
    double theta_ = 1.57 - abs(angle_);
    double dl = sin(theta_) * l;
    double dz = cos(theta_) * l;
    dz = (angle_ < 0) ? -dz : dz;
    double r = sqrt(calibrated_x * calibrated_x + calibrated_y * calibrated_y);
    double Px = calibrated_x * (r - dl) / r;
    double Py = calibrated_y * (r - dl) / r;
    double Pz = calibrated_z - dz;

    // Creating IK request
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = Px;
    target_pose.pose.position.y = Py;
    target_pose.pose.position.z = Pz;
    target_pose.pose.orientation.w = 1.0;
    std::string group_ = "arm_2";

    // Computing IK using callback
    ik_promise_ = std::promise<std::vector<double>>();
    auto ik_future = ik_promise_.get_future();
    
    computeIK(target_pose, group_);
    
    // Waiting for the IK result
    auto joint_positions = ik_future.get();
    
    if (joint_positions.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid IK solution found. Exiting.");
      goal_handle->abort(result);
      return;
    }

    // Adjusting joint 4 angle
    double j2 = joint_positions[1];
    double j3 = joint_positions[2];
    double j4 = angle_ - (j3 - j2);
    joint_positions[3] = j4;

    // Adding gripper joint (joint_5) manually for logging (you can change value as needed)
    double gripper_position = 0.0;  // Or -0.03 if gripper is open
    joint_positions.push_back(gripper_position);

    // Print all joint waypoints in degrees
    RCLCPP_INFO(this->get_logger(), "Planned Joint Waypoints (Degrees):");
    for (size_t i = 0; i < joint_positions.size(); ++i) {
      double degrees = joint_positions[i] * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(), "joint_%zu: %.2f degrees", i + 1, degrees);
    }

    // Creating and configuring MoveGroupInterface
    auto move_group_node = std::make_shared<rclcpp::Node>(
        "ik_moveit2_move_group",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        // 🔥 Increase movement speed
    move_group.setMaxVelocityScalingFactor(1.0);     // 1.0 = 100% of max velocity
    move_group.setMaxAccelerationScalingFactor(1.0); // 1.0 = 100% of max acceleration

    move_group.setJointValueTarget(std::vector<double>(joint_positions.begin(), joint_positions.begin() + 4));  // Only arm joints for planning

    // Planning
    RCLCPP_INFO(this->get_logger(), "Planning motion...");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    auto planning_result = move_group.plan(my_plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      goal_handle->abort(result);
      return;
    }

    // Publishing action server feedback
    feedback->joint_names = goal->trajectory.joint_names;
    goal_handle->publish_feedback(feedback);

    // Executing plan
    RCLCPP_INFO(this->get_logger(), "Executing motion...");
    auto execution_result = move_group.execute(my_plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Execution failed.");
      goal_handle->abort(result);
      return;
    }

    // Success
    RCLCPP_INFO(this->get_logger(), "Motion execution succeeded!");
    result->error_code = 0;  // Success code
    goal_handle->succeed(result);
  }

  void computeIK(const geometry_msgs::msg::PoseStamped &target_pose, std::string group_)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = group_;
    request->ik_request.pose_stamped = target_pose;
    request->ik_request.timeout.sec = 1;
    request->ik_request.avoid_collisions = true;

    auto response_callback = 
      [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
        auto response = future.get();
        if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          if(response->solution.joint_state.position.size() < 4)
          {
            RCLCPP_ERROR(this->get_logger(), "IK solution does not have enough joint values.");
            ik_promise_.set_value({});
            return;
          }
          std::vector<double> joint_positions(response->solution.joint_state.position.begin(),
                                           response->solution.joint_state.position.begin() + 4);
          ik_promise_.set_value(joint_positions);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "IK service failed with error code: %d", response->error_code.val);
          ik_promise_.set_value({});
        }
      };

    // Sending the request with the callback
    ik_client_->async_send_request(request, response_callback);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Mapped Joint Positions (Degrees for Arduino):");
  
    std::ostringstream oss;
    double mapped_degrees[5] = {0};  // Fixed order: joint_1 to joint_5
  
    for (size_t i = 0; i < msg->name.size(); ++i) {
      double rad = msg->position[i];
      double mapped_deg = 0.0;
  
      if (msg->name[i] == "joint_1") {
        mapped_deg = (rad + 1.57) * (180.0 / (2 * 1.57));
        mapped_degrees[0] = mapped_deg;
      } else if (msg->name[i] == "joint_2") {
        mapped_deg = (1.57 - rad) * (180.0 / (2 * 1.57));
        mapped_degrees[1] = mapped_deg;
      } else if (msg->name[i] == "joint_3") {
        mapped_deg = (rad + 1.57) * (180.0 / (2 * 1.57));
        mapped_degrees[2] = mapped_deg;
      } else if (msg->name[i] == "joint_4") {
        mapped_deg = (rad + 1.57) * (180.0 / (2 * 1.57));
        mapped_degrees[3] = mapped_deg;
      } else if (msg->name[i] == "joint_5") {
        mapped_deg = (-rad) * (180.0 / 0.03);
        mapped_degrees[4] = mapped_deg;
      }
    }
  
    // Clamp and log each joint, send to Arduino in order joint_1 to joint_5
    for (int j = 0; j < 5; ++j) {
      if (mapped_degrees[j] < 0) mapped_degrees[j] = 0;
      if (mapped_degrees[j] > 180) mapped_degrees[j] = 180;
  
      RCLCPP_INFO(this->get_logger(), "joint_%d: %.2f degrees (mapped)", j + 1, mapped_degrees[j]);
      oss << mapped_degrees[j];
      if (j < 4) {
        oss << ",";
      }
    }
  
    oss << "\n";
  
    std::string data_to_send = oss.str();
    if (serial_port_ >= 0) {
      write(serial_port_, data_to_send.c_str(), data_to_send.size());
    }
  }
    
  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
