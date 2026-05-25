#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);

  auto node =
      std::make_shared<rclcpp::Node>(
          "xarm_pick_place",
          rclcpp::NodeOptions()
              .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor](){
      executor.spin();
  });
  
  moveit::planning_interface::MoveGroupInterface arm(
      node,
      "xarm5");

  moveit::planning_interface::MoveGroupInterface gripper(
      node,
      "xarm_gripper");


  auto pose = arm.getCurrentPose().pose;

  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(node->get_logger(),"OPEN");

  gripper.setNamedTarget("open");
  gripper.move();

  geometry_msgs::msg::Pose target;
  target = pose;
  

  target.position.x += 0.;
  target.position.y = 0.25;
  target.position.z += 0;


  RCLCPP_INFO(node->get_logger(),"MOVE TO PICK");

  arm.setPoseTarget(target);
  arm.move();

  RCLCPP_INFO(node->get_logger(),"CLOSE");

  gripper.setNamedTarget("close");
  gripper.move();

  target.position.y = -0.25;

  RCLCPP_INFO(node->get_logger(),"MOVE TO PLACE");

  arm.setPoseTarget(target);
  arm.move();

  RCLCPP_INFO(node->get_logger(),"OPEN");

  gripper.setNamedTarget("open");
  gripper.move();

  RCLCPP_INFO(node->get_logger(),"HOLD");

  arm.setNamedTarget("hold");
  arm.move();

  rclcpp::shutdown();
  spinner.join();

  return 0;
}