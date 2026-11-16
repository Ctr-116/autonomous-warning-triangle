// ROS2 Humble port of the ROS1 arm_moveit_demo C++ demos.
//
// A single binary with a --mode parameter covering all six behaviors:
//   random_move, set_pose_plan, set_joint_plan,
//   cartesian, attached_object, multi_track_motion
//
// Original executable aliases (01_random_move ... 06_multi_track_motion)
// are compiled with ARM_MOVEIT_DEMO_DEFAULT_MODE set to their default mode.
//
// Uses actual MoveGroupInterface / PlanningSceneInterface API.
// A dedicated executor thread runs the callback group; shutdown is bounded.
// Plan only by default; execution is opt-in via --execute.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/object_color.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace {

using namespace std::chrono_literals;

constexpr const char* ARM_GROUP = "arm_group";
constexpr const char* EE_LINK = "arm_link5";

struct ModeOptions {
  std::string mode;
  bool execute = false;
  int iterations = 0;
  double planning_time = 5.0;
};

class DemoRunner : public rclcpp::Node {
 public:
  DemoRunner(const std::string& name, const ModeOptions& opts)
      : rclcpp::Node(name), opts_(opts) {
    // Must be a non-mutually-exclusive callback group since we call services
    // from this thread and the executor must dispatch their replies.
    auto cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_group_ = cbg;
  }

  bool initialize() {
    RCLCPP_INFO(get_logger(), "Waiting for MoveIt move_group ...");
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), ARM_GROUP);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface init failed: %s", ex.what());
      return false;
    }
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setPlanningTime(opts_.planning_time);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);
    move_group_->setGoalJointTolerance(0.001);
    scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(
        node_for_scene_->get_namespace(), false);
    return true;
  }

  bool planAndMaybeExecute(
      const moveit::planning_interface::MoveGroupInterface::Plan& plan,
      const std::string& label) {
    if (plan.trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: empty trajectory", label.c_str());
      return false;
    }
    if (!opts_.execute) {
      RCLCPP_INFO(get_logger(), "%s: plan_only (use --execute)",
                  label.c_str());
      return true;
    }
    auto code = move_group_->execute(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: execution failed (%d)", label.c_str(),
                   code.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s: executed", label.c_str());
    return true;
  }

  bool moveNamed(const std::string& name) {
    if (!move_group_->setNamedTarget(name)) {
      RCLCPP_ERROR(get_logger(), "Named target '%s' not found", name.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Named target plan failed: %s", name.c_str());
      return false;
    }
    return planAndMaybeExecute(plan, std::string("named:") + name);
  }

  bool moveJoints(const std::vector<double>& joints,
                  const std::string& label) {
    if (!move_group_->setJointValueTarget(joints)) {
      RCLCPP_ERROR(get_logger(), "setJointValueTarget failed for %s",
                   label.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: plan failed", label.c_str());
      return false;
    }
    return planAndMaybeExecute(plan, label);
  }

  // ---------- modes ----------
  int runRandomMove() {
    if (!moveNamed("down")) return 1;
    const int n = opts_.iterations > 0 ? opts_.iterations : 1;
    int rc = 0;
    for (int i = 0; i < n; ++i) {
      move_group_->setRandomTarget();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(get_logger(), "random target %d plan failed", i + 1);
        rc = 1;
        continue;
      }
      if (!planAndMaybeExecute(plan, "random_target")) rc = 1;
    }
    return rc;
  }

  int runSetPosePlan() {
    if (!moveNamed("up")) return 1;
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.18721125717113798;
    pose.position.y = -0.008718652395814977;
    pose.position.z = 0.4787351295417709;
    pose.orientation.x = 0.08637313729188083;
    pose.orientation.y = 0.40819551903689105;
    pose.orientation.z = 0.10050372779920543;
    pose.orientation.w = 0.9032248336328139;
    if (!move_group_->setPoseTarget(pose)) {
      RCLCPP_ERROR(get_logger(), "setPoseTarget failed");
      return 1;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "pose plan failed");
      return 1;
    }
    return planAndMaybeExecute(plan, "pose") ? 0 : 1;
  }

  int runSetJointPlan() {
    std::vector<double> joints{0.0, 0.79, -1.57, -1.57, 0.0};
    if (!moveNamed("up")) return 1;
    return moveJoints(joints, "joint_target") ? 0 : 1;
  }

  int runCartesian() {
    std::vector<double> init{0.0, -0.69, -0.17, 0.86, 0.0};
    if (!moveJoints(init, "cartesian_init")) return 1;

    geometry_msgs::msg::Pose base;
    base.position.x = 0.2;
    base.position.y = 0.0;
    base.position.z = 0.3;
    base.orientation.w = 1.0;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto cur = base;
    waypoints.push_back(cur);
    cur.position.x -= 0.04;
    waypoints.push_back(cur);
    cur.position.z -= 0.02;
    waypoints.push_back(cur);
    cur.position.x += 0.04;
    waypoints.push_back(cur);
    cur.position.z -= 0.02;
    waypoints.push_back(cur);
    cur.position.x += 0.03;
    waypoints.push_back(cur);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.1;
    const double jump_threshold = 0.0;
    const int maxtries = 100;
    int attempts = 0;
    double fraction = 0.0;
    while (fraction < 1.0 && attempts < maxtries) {
      fraction = move_group_->computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory, true);
      ++attempts;
      if (attempts % 10 == 0) {
        RCLCPP_INFO(get_logger(), "cartesian attempt %d fraction=%.3f",
                    attempts, fraction);
      }
    }
    if (fraction < 0.99 || trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "cartesian path failed fraction=%.3f after %d attempts",
                   fraction, attempts);
      return 1;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return planAndMaybeExecute(plan, "cartesian") ? 0 : 1;
  }

  int runAttachedObject() {
    if (!moveNamed("up")) return 1;

    moveit_msgs::msg::CollisionObject obj;
    obj.id = "obj";
    obj.header.frame_id = move_group_->getPlanningFrame();
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {0.2, 0.1, 0.02};
    geometry_msgs::msg::Pose opos;
    opos.position.x = 0.2;
    opos.position.y = 0.0;
    opos.position.z = 0.26;
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    opos.orientation.x = q.x();
    opos.orientation.y = q.y();
    opos.orientation.z = q.z();
    opos.orientation.w = q.w();
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(opos);

    std::vector<moveit_msgs::msg::CollisionObject> objects{obj};
    std::vector<moveit_msgs::msg::ObjectColor> colors;
    moveit_msgs::msg::ObjectColor color;
    color.id = "obj";
    color.color.r = 1.0;
    color.color.g = 0.0;
    color.color.b = 0.0;
    color.color.a = 0.5;
    colors.push_back(color);
    if (!scene_->applyCollisionObjects(objects, colors)) {
      RCLCPP_ERROR(get_logger(), "applyCollisionObjects failed");
      return 1;
    }
    if (!moveNamed("down")) return 1;

    const int n = opts_.iterations > 0 ? opts_.iterations : 3;
    int rc = 0;
    for (int i = 0; i < n; ++i) {
      move_group_->setRandomTarget();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(get_logger(), "random iteration %d plan failed", i + 1);
        rc = 1;
        continue;
      }
      if (!planAndMaybeExecute(plan, "attached_random")) rc = 1;
    }
    moveit_msgs::msg::CollisionObject removed;
    removed.id = "obj";
    removed.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene_->applyCollisionObject(removed)) {
      RCLCPP_ERROR(get_logger(), "applyCollisionObject REMOVE failed");
      rc = 1;
    }
    return rc;
  }

  int runMultiTrackMotion() {
    RCLCPP_INFO(get_logger(), "Move to down");
    if (!moveNamed("down")) return 1;

    std::vector<std::vector<double>> poses{
        {1.34, -1.0, -0.61, 0.2, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0},
        {-1.16, -0.97, -0.81, -0.79, 3.14}};

    moveit_msgs::msg::RobotTrajectory joined;
    moveit::core::RobotStatePtr state = move_group_->getCurrentState();
    for (const auto& p : poses) {
      moveit::planning_interface::MoveGroupInterface::Plan part;
      if (!move_group_->setJointValueTarget(p)) {
        RCLCPP_ERROR(get_logger(), "multi_trajectory setJointValueTarget failed");
        return 1;
      }
      if (move_group_->plan(part) != moveit::core::MoveItErrorCode::SUCCESS ||
          part.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(get_logger(), "multi_trajectory plan failed");
        return 1;
      }
      joined.joint_trajectory.joint_names =
          part.trajectory_.joint_trajectory.joint_names;
      for (const auto& pt : part.trajectory_.joint_trajectory.points) {
        joined.joint_trajectory.points.push_back(pt);
      }
      state->setJointGroupPositions(ARM_GROUP, p);
      move_group_->setStartState(*state);
    }

    try {
      robot_trajectory::RobotTrajectory rt(
          move_group_->getCurrentState()->getRobotModel(), ARM_GROUP);
      rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), joined);
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, 1.0, 1.0);
      rt.getRobotTrajectoryMsg(joined);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Time parameterization failed: %s", ex.what());
      return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = joined;
    return planAndMaybeExecute(plan, "multi_track_motion") ? 0 : 1;
  }

  int dispatch() {
    if (opts_.mode == "random_move") return runRandomMove();
    if (opts_.mode == "set_pose_plan") return runSetPosePlan();
    if (opts_.mode == "set_joint_plan") return runSetJointPlan();
    if (opts_.mode == "cartesian") return runCartesian();
    if (opts_.mode == "attached_object") return runAttachedObject();
    if (opts_.mode == "multi_track_motion") return runMultiTrackMotion();
    RCLCPP_ERROR(get_logger(), "Unknown mode '%s'", opts_.mode.c_str());
    return 2;
  }

  rclcpp::CallbackGroup::SharedPtr callback_group() const { return callback_group_; }
  const ModeOptions& options() const { return opts_; }

 private:
  // Dummy node passed to PlanningSceneInterface; both share the same ROS context.
  rclcpp::Node::SharedPtr node_for_scene_ = this->shared_from_this();
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  ModeOptions opts_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

ModeOptions parseArgs(int argc, char** argv) {
  ModeOptions opts;
#ifdef ARM_MOVEIT_DEMO_DEFAULT_MODE
  opts.mode = ARM_MOVEIT_DEMO_DEFAULT_MODE;
#else
  opts.mode = "set_joint_plan";
#endif
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--execute") {
      opts.execute = true;
    } else if (a.rfind("--mode=", 0) == 0) {
      opts.mode = a.substr(7);
    } else if (a == "--mode" && i + 1 < argc) {
      opts.mode = argv[++i];
    } else if (a.rfind("--iterations=", 0) == 0) {
      opts.iterations = std::stoi(a.substr(13));
    } else if (a == "--iterations" && i + 1 < argc) {
      opts.iterations = std::stoi(argv[++i]);
    } else if (a.rfind("--planning-time=", 0) == 0) {
      opts.planning_time = std::stod(a.substr(16));
    } else if (a == "--planning-time" && i + 1 < argc) {
      opts.planning_time = std::stod(argv[++i]);
    }
  }
  return opts;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ModeOptions opts = parseArgs(argc, argv);

  auto node = std::make_shared<DemoRunner>("arm_moveit_demo_cpp", opts);
  RCLCPP_INFO(node->get_logger(), "mode=%s execute=%s",
              opts.mode.c_str(), opts.execute ? "true" : "false");

  // Dedicated executor thread guarantees service replies can be processed
  // while the main thread awaits its own client futures.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3, false);
  executor.add_node(node);
  std::atomic<bool> executor_running{true};
  std::thread executor_thread([&executor, &executor_running]() {
    while (rclcpp::ok() && executor_running.load()) {
      try {
        executor.spin_once(50ms);
      } catch (const std::exception&) {
        break;
      }
    }
  });

  int rc = 1;
  try {
    if (node->initialize()) {
      rc = node->dispatch();
    } else {
      RCLCPP_ERROR(node->get_logger(), "initialization failed");
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled exception: %s", ex.what());
    rc = 1;
  }

  // Bounded shutdown.
  rclcpp::shutdown();
  executor_running.store(false);
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  executor.remove_node(node);
  return rc;
}
