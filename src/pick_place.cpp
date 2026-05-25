#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using moveit::planning_interface::MoveGroupInterface;

constexpr double APPROACH_HEIGHT = 0.15;
constexpr double LIFT_HEIGHT     = 0.15;

void moveToPose(
    MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& pose)
{
    arm.setApproximateJointValueTarget(pose);
    arm.move();
}

void openGripper(MoveGroupInterface& gripper)
{
    gripper.setNamedTarget("open");
    gripper.move();
}

void closeGripper(MoveGroupInterface& gripper)
{
    gripper.setNamedTarget("close");
    gripper.move();
}

void pickObject(
    MoveGroupInterface& arm,
    MoveGroupInterface& gripper,
    geometry_msgs::msg::Pose object_pose)
{
    auto approach = object_pose;
    approach.position.z += APPROACH_HEIGHT;

    // Approach
    moveToPose(arm, approach);

    // Descend
    moveToPose(arm, object_pose);

    // Grab
    closeGripper(gripper);

    // Lift
    moveToPose(arm, approach);
}

void placeObject(
    MoveGroupInterface& arm,
    MoveGroupInterface& gripper,
    geometry_msgs::msg::Pose basket_pose)
{
    auto approach = basket_pose;
    approach.position.z += APPROACH_HEIGHT;

    // Approach basket
    moveToPose(arm, approach);

    // Descend
    moveToPose(arm, basket_pose);

    // Release
    openGripper(gripper);

    // Lift away
    moveToPose(arm, approach);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "xarm_pick_place",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    MoveGroupInterface arm(node, "xarm5");
    MoveGroupInterface gripper(node, "xarm_gripper");

    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    openGripper(gripper);

    //------------------------------------------------------------------
    // Tool locations
    //------------------------------------------------------------------

    std::vector<geometry_msgs::msg::Pose> tools(3);

    auto current = arm.getCurrentPose().pose;

    tools[0] = current;
    tools[0].position.x = 0.40;
    tools[0].position.y = 0.25;

    tools[1] = current;
    tools[1].position.x = 0.40;
    tools[1].position.y = 0.50;

    tools[2] = current;
    tools[2].position.x = 0.40;
    tools[2].position.y = -0.50;

    //------------------------------------------------------------------
    // Basket location
    //------------------------------------------------------------------

    geometry_msgs::msg::Pose basket = current;
    basket.position.x = 0.40;
    basket.position.y = -0.25;

    //------------------------------------------------------------------
    // Pick all tools and place into basket
    //------------------------------------------------------------------

    for (size_t i = 0; i < tools.size(); ++i)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Processing tool %ld",
            static_cast<long>(i + 1));

        pickObject(arm, gripper, tools[i]);
        placeObject(arm, gripper, basket);
    }

    //------------------------------------------------------------------
    // Return home
    //------------------------------------------------------------------
    
    arm.setNamedTarget("hold");
    arm.move();

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
