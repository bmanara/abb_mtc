#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("abb_mtc");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
  public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

  private:
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_-> get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("abb_mtc", options) }
{

}

void MTCTaskNode::setupPlanningScene()
{
  // TODO: Hopefully this will be flexible enough in the future, but not required for now
  RCLCPP_INFO_STREAM(LOGGER, "Setting up planning scene...");
  moveit::planning_interface::PlanningSceneInterface psi;

  // Adding wall to planning scene
  moveit_msgs::msg::CollisionObject wall;
  wall.id = "wall";
  wall.header.frame_id = "map";
  wall.primitives.resize(1);
  wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  wall.primitives[0].dimensions = { 8.0, 0.1, 2.0 }; // Length, Width, Height

  geometry_msgs::msg::Pose wall_pose;
  wall_pose.position.x = 2.0;
  wall_pose.position.y = 3.0; // Position the wall behind the robot
  wall_pose.position.z = 1.0; // Position the wall at half the height
  wall_pose.orientation.w = 1.0; // No rotation
  wall.pose = wall_pose;

  psi.applyCollisionObject(wall);

  // Adding underbelly to planning scene
  moveit_msgs::msg::CollisionObject underbelly;
  underbelly.id = "underbelly";
  underbelly.header.frame_id = "map";
  underbelly.primitives.resize(1);
  underbelly.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  underbelly.primitives[0].dimensions = { 3.0, 2.4, 0.05 };

  geometry_msgs::msg::Pose underbelly_pose;
  underbelly_pose.position.x = 0.0;
  underbelly_pose.position.y = 0.6;
  underbelly_pose.position.z = 2.8; // Position the underbelly above the ground

  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI * 0.225, Eigen::Vector3d::UnitX()) * 
                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  underbelly_pose.orientation = tf2::toMsg(q);
  underbelly.pose = underbelly_pose;

  psi.applyCollisionObject(underbelly);
  RCLCPP_INFO_STREAM(LOGGER, "Planning scene setup complete");
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed: " << e);
    return ;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed.");
    return ;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    return ;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Task executed successfully");
  return ;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "abb_arm";
  const auto& hand_group_name = "sander";
  const auto& hand_frame = "sander_pad_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.2);
  sampling_planner->setMaxAccelerationScalingFactor(0.2);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.2);
  interpolation_planner->setMaxAccelerationScalingFactor(0.2);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(.001);

  auto stage_home_pose= std::make_unique<mtc::stages::MoveTo>("move to home pose", interpolation_planner);
  stage_home_pose->setGroup(arm_group_name);
  stage_home_pose->setGoal("home_configuration");
  task.add(std::move(stage_home_pose));

  auto stage_ready_pose = std::make_unique<mtc::stages::MoveTo>("move to ready pose", interpolation_planner);
  stage_ready_pose->setGroup(arm_group_name);
  stage_ready_pose->setGoal("ready_configuration");
  task.add(std::move(stage_ready_pose));

  auto stage_move_to_sand = std::make_unique<mtc::stages::Connect>(
    "move to sand",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } }
  );
  stage_move_to_sand->setTimeout(5.0);
  stage_move_to_sand->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_sand));

  mtc::Stage* allow_collision_stage = nullptr;

  {
    auto container = std::make_unique<mtc::SerialContainer>("sanding container");
    task.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
    container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" } );

    {
      auto stage_move_towards = std::make_unique<mtc::stages::MoveRelative>("move towards underbelly", cartesian_planner);
      stage_move_towards->properties().set("marker_ns", "move_towards_wall");
      stage_move_towards->properties().set("link", hand_frame);
      stage_move_towards->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage_move_towards->setMinMaxDistance(0.1, 0.3);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "underbelly";
      vec.vector.y = -0.5; // Move towards the underbelly
      vec.vector.z = 0.5; // Move towards the underbelly
      stage_move_towards->setDirection(vec);
      container->insert(std::move(stage_move_towards));
    }
    
    {
      auto stage = std::make_unique<mtc::stages::GeneratePose>("generate target pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "target_pose");
      stage->setMonitoredStage(current_state_ptr);

      // Define the target pose for the end effector
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "underbelly";
      target_pose_msg.pose.position.x = 0.5; // Adjust this based on your
      target_pose_msg.pose.position.y = -0.1; // Centered in the y-axis
      target_pose_msg.pose.position.z = -0.4; // Adjust this based on your
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * 
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
      target_pose_msg.pose.orientation = tf2::toMsg(q);
      stage->setPose(target_pose_msg);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      container->insert(std::move(wrapper));
    }

    {
      auto stage_allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions (eef, underbelly)");
      stage_allow->allowCollisions("underbelly",
                                    task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                                    true);
      allow_collision_stage = stage_allow.get();
      container->insert(std::move(stage_allow));
    }

    {
      // Now that arm and eef are in position, use MoveRelative to move the end effector towards the wall
      auto stage = std::make_unique<mtc::stages::MoveRelative>("contact underbelly", cartesian_planner);
      stage->properties().set("marker_ns", "wall_contact");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.3, 0.35);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0; // Move towards the wall
      stage->setDirection(vec);
      container->insert(std::move(stage));
    }

    task.add(std::move(container));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
