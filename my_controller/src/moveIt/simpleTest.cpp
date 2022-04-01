
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Getting Basic Information

    //  print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    //  print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    //关节空间规划
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.0;  // radians
    joint_group_positions[1] = 0.0;  // radians
    joint_group_positions[2] = 0.0;  // radians
    joint_group_positions[3] = 0.0;  // radians
    joint_group_positions[4] = 0.0;  // radians
    joint_group_positions[5] = 0.0;  // radians

    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        move_group.execute(my_plan);
    sleep(5);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1;
    target_pose1.orientation.x = 0;
    target_pose1.orientation.y = 0;
    target_pose1.orientation.z = 0;

    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        move_group.execute(my_plan);
    sleep(5);

    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");


    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list.
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
    target_pose3.position.x += 0.1;
    target_pose3.position.z += 0.1;
    waypoints.push_back(target_pose3);  // up and out

    target_pose3.position.y -= 0.1;
    waypoints.push_back(target_pose3);  // left

    target_pose3.position.z -= 0.1;
    target_pose3.position.y += 0.1;
    target_pose3.position.x -= 0.1;
    waypoints.push_back(target_pose3);  // down and right (back to start)

    // We want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively
    // disabling it.
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
    move_group.setPoseTargets(waypoints,"ee_link");
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
        move_group.execute(my_plan);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);

    ros::waitForShutdown();
}
