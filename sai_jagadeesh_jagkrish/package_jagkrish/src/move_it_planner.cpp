#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initializing ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_it_planner");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_it_planner");

  // Creating the MoveIt MoveGroup Interface for arm and gripper
  using moveit::planning_interface::MoveGroupInterface;

  // Creating the MoveGroupInterface for the arm 
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  // Creating the MoveGroupInterface for the gripper
  auto move_gripper = MoveGroupInterface(node, "panda_hand");

  // Intially openning the gripper for the robot
  RCLCPP_INFO(logger, "Opening the Gripper");
  move_gripper.setNamedTarget("gripper_open");
  bool success_gripper1 = static_cast<bool>(move_gripper.move());
  if (!success_gripper1) {
    RCLCPP_ERROR(logger, "Failed to open the gripper!");
  }
  // Wait for 2 seconds to move to next step for better visualization
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Setting the picking target pose for the arm
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -0.084592;
    msg.orientation.x = 0.92446;
    msg.orientation.y = -0.36195;
    msg.orientation.z = 0.084883;
    msg.position.x = 0.69865;
    msg.position.y = 0.029493;
    msg.position.z = 0.65734;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  // Plan to the first target pose
  auto [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Executing the picking plan and closing gripper after attaining the pose
  if (success) {
    move_group_interface.execute(plan);
    // Manipulate gripper after moving the arm
    RCLCPP_INFO(logger, "Closing the Gripper");
    move_gripper.setNamedTarget("gripper_close");
    bool success_gripper2 = static_cast<bool>(move_gripper.move());
    if (!success_gripper2) {
      RCLCPP_ERROR(logger, "Failed to close the gripper!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Wait for 2 seconds to move to next step for better visualization
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Setting the placing target pose for the arm
  auto const target_pose2 = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.74376;  
    msg.orientation.x = -0.50999;
    msg.orientation.y = -0.33999;
    msg.orientation.z = -0.26673;
    msg.position.x = -0.22063;  
    msg.position.y = 0.55178;    
    msg.position.z = 0.94884;    
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose2);

  // Planning for the second pose
  auto [success2, plan2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Executing the placing plan and opening gripper after attaining the pose
  if (success2) {
    move_group_interface.execute(plan2);
    // Manipulate gripper after moving the arm
    RCLCPP_INFO(logger, "Opening the Gripper");
    move_gripper.setNamedTarget("gripper_open");
    bool success_gripper3 = static_cast<bool>(move_gripper.move());
    if (!success_gripper3) {
      RCLCPP_ERROR(logger, "Failed to open the gripper!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for the second pose failed!");
  }

  // Setting back to the initial pose of the arm(home pose)
  auto const intial_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.0;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.position.x = 0.087;
    msg.position.y = 0.0;
    msg.position.z = 0.925;
    return msg;
  }();
  move_group_interface.setPoseTarget(intial_pose);
  // Plan to the first target pose
  auto [success_home, plan_home] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Executing the plan to move to the home pose
  if (success_home) {
    move_group_interface.execute(plan_home);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS and exit
  rclcpp::shutdown();
  return 0;
}
