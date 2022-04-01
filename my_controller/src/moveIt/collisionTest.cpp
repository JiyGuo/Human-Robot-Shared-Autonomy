//
// Created by jiyguo on 2020/12/13.
//
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    bool visualizing_plan = true;
    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup move_group("manipulator");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup("manipulator");
    std::vector<double> joint_group_positions;
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
    move_group.setMaxVelocityScalingFactor(0.2);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        move_group.execute(my_plan);
    sleep(5);

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.


    // Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // First, we will define the collision object message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "box1";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.2;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.3;
    box_pose.position.y = 0.05;
    box_pose.position.z =  0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    /* Sleep so we have time to see the object in RViz */
    sleep(2.0);

    // Planning with collision detection can be slow.  Lets set the planning time
    // to be sure the planner has enough time to plan around the box.  10 seconds
    // should be plenty.
    move_group.setPlanningTime(10.0);

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
    // Now when we plan a trajectory it will avoid the obstacle
    move_group.setStartState(*move_group.getCurrentState());
    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
             success?"":"FAILED");

    if(success)
        move_group.execute(my_plan);
    sleep(5);


    // Now, let's remove the collision object from the world.
    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    /* Sleep to give Rviz time to show the object is no longer there. */
    sleep(4.0);

// END_TUTORIAL

    ros::shutdown();
    return 0;
}
