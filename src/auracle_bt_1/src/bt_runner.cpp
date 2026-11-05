// auracle_bt mission runner.
//
// Sits above Nav2: it sends goals to the NavigateThroughPoses action, it does not drive.
// Default tree (behavior_trees/goal_tree.xml): wait for Nav2 -> wait for the 2D Pose Estimate you
// give in RViz -> then, for every 2D Goal Pose you click, drive there. No coordinates to collect.
// Other trees (pass tree_file, a bare name is looked up in behavior_trees/):
//   region_tree.xml  start pose + route_N from config/route.yaml (coordinates needed)
//   sorting_tree.xml old vision-driven tree (lives in src/auracle_bt, not installed here: pass its full path)
//
// Run with the same namespace as Nav2 (launch/mission.launch.py, namespace:=auracle), so the
// relative names below become /auracle/navigate_to_pose, /auracle/initialpose, /auracle/mission_goal.
// RViz's 2D Goal Pose must publish to mission_goal (laptop: goal_topic:=mission_goal), NOT goal_pose:
// Nav2's own bt_navigator listens on goal_pose and would drive there by itself, ignoring this tree.
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using Poses = std::vector<geometry_msgs::msg::PoseStamped>;
using Nav = nav2_msgs::action::NavigateThroughPoses;
using Nav1 = nav2_msgs::action::NavigateToPose;

// Things the ROS callbacks in main() tell the tree nodes.
struct Shared {
  std::string label;        // vision label (sorting tree only)
  bool have_amcl{false};    // AMCL has published a pose: it is up
  bool have_pose_estimate{false};   // a 2D Pose Estimate arrived on initialpose
  bool accepting_goal{false};       // WaitForGoal is running: a clicked goal is taken
  bool goal_ready{false};
  geometry_msgs::msg::PoseStamped goal;
};

static std::string share_dir(const std::string & pkg) {
  return ament_index_cpp::get_package_share_directory(pkg);
}

// Wait for the vision detector, store the label (sorting tree only)
class GetDetectedClass : public BT::StatefulActionNode {
public:
  GetDetectedClass(const std::string & n, const BT::NodeConfig & c, std::shared_ptr<Shared> s)
  : BT::StatefulActionNode(n, c), state_(s) {}
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<std::string>("detected_class")};
  }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    if (state_->label.empty()) return BT::NodeStatus::RUNNING;
    std::string l = state_->label;
    std::transform(l.begin(), l.end(), l.begin(), ::toupper);
    setOutput("detected_class", l);
    return BT::NodeStatus::SUCCESS;
  }
  std::shared_ptr<Shared> state_;
};

// Wait until Nav2's action server is up AND AMCL has published (so the map frame exists).
class WaitForNav2 : public BT::StatefulActionNode {
public:
  WaitForNav2(const std::string & n, const BT::NodeConfig & c,
              rclcpp::Node::SharedPtr node, std::shared_ptr<Shared> s)
  : BT::StatefulActionNode(n, c), node_(node), state_(s) {
    client_ = rclcpp_action::create_client<Nav>(node_, "navigate_through_poses");
  }
  static BT::PortsList providedPorts() {
    // false = only wait for the action server (AMCL has no pose yet when you still have to give one)
    return {BT::InputPort<bool>("need_amcl", true, "also wait for AMCL's first pose")};
  }
  BT::NodeStatus onStart() override {
    need_amcl_ = true;
    getInput("need_amcl", need_amcl_);
    t0_ = node_->now();
    last_log_ = t0_;
    return check();
  }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    const bool server = client_->action_server_is_ready();
    if (server && (state_->have_amcl || !need_amcl_)) {
      RCLCPP_INFO(node_->get_logger(), need_amcl_ ? "Nav2 and AMCL are up" : "Nav2 is up");
      return BT::NodeStatus::SUCCESS;
    }
    if ((node_->now() - last_log_).seconds() > 5.0) {
      last_log_ = node_->now();
      RCLCPP_INFO(node_->get_logger(), "waiting for Nav2 (%s) and AMCL (%s) ...",
                  server ? "up" : "not yet", state_->have_amcl ? "up" : "not yet");
    }
    return BT::NodeStatus::RUNNING;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Shared> state_;
  rclcpp_action::Client<Nav>::SharedPtr client_;
  rclcpp::Time t0_, last_log_;
  bool need_amcl_{true};
};

// Ask for a 2D Pose Estimate and wait until one arrives (RViz publishes it on initialpose, AMCL
// reads the same message). Holds 2.5 s afterwards so AMCL has applied it before any goal is sent.
class WaitForInitialPose : public BT::StatefulActionNode {
public:
  WaitForInitialPose(const std::string & n, const BT::NodeConfig & c,
                     rclcpp::Node::SharedPtr node, std::shared_ptr<Shared> s)
  : BT::StatefulActionNode(n, c), node_(node), state_(s) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    state_->have_pose_estimate = false;
    got_ = false;
    last_log_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
      "Set the robot's position: RViz 2D Pose Estimate, click where it is and drag the way it faces");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!got_) {
      if (state_->have_pose_estimate) {
        got_ = true;
        t0_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "pose estimate received");
      } else {
        if ((node_->now() - last_log_).seconds() > 10.0) {
          last_log_ = node_->now();
          RCLCPP_INFO(node_->get_logger(), "still waiting for the 2D Pose Estimate in RViz ...");
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    return (node_->now() - t0_).seconds() > 2.5 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Shared> state_;
  rclcpp::Time t0_, last_log_;
  bool got_{false};
};

// Wait for a 2D Goal Pose click (topic mission_goal) and put it on the blackboard.
class WaitForGoal : public BT::StatefulActionNode {
public:
  WaitForGoal(const std::string & n, const BT::NodeConfig & c,
              rclcpp::Node::SharedPtr node, std::shared_ptr<Shared> s)
  : BT::StatefulActionNode(n, c), node_(node), state_(s) {}
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }
  BT::NodeStatus onStart() override {
    state_->goal_ready = false;
    state_->accepting_goal = true;
    RCLCPP_INFO(node_->get_logger(), "Ready: click a 2D Goal Pose in RViz (the target region)");
    return check();
  }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override { state_->accepting_goal = false; }
private:
  BT::NodeStatus check() {
    if (!state_->goal_ready) return BT::NodeStatus::RUNNING;
    state_->accepting_goal = false;
    state_->goal_ready = false;
    setOutput("goal", state_->goal);
    return BT::NodeStatus::SUCCESS;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Shared> state_;
};

// Drive to one goal with Nav2 (NavigateToPose).
class NavigateToGoal : public BT::StatefulActionNode {
  using GH = rclcpp_action::ClientGoalHandle<Nav1>;
public:
  NavigateToGoal(const std::string & n, const BT::NodeConfig & c, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(n, c), node_(node) {
    client_ = rclcpp_action::create_client<Nav1>(node_, "navigate_to_pose");
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }
  BT::NodeStatus onStart() override {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose is not available");
      return BT::NodeStatus::FAILURE;
    }
    Nav1::Goal g;
    g.pose = goal;
    g.pose.header.frame_id = "map";
    g.pose.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());   // latest tf
    done_ = false; ok_ = false; handle_.reset();
    rclcpp_action::Client<Nav1>::SendGoalOptions o;
    o.goal_response_callback = [this](GH::SharedPtr h) {
      if (!h) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 rejected the goal");
        done_ = true; ok_ = false;
      } else { handle_ = h; } };
    o.result_callback = [this](const GH::WrappedResult & r) {
      done_ = true;
      ok_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
      RCLCPP_INFO(node_->get_logger(), "goal finished: %s", ok_ ? "reached" : "not reached"); };
    client_->async_send_goal(g, o);
    RCLCPP_INFO(node_->get_logger(), "driving to (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!done_) return BT::NodeStatus::RUNNING;
    return ok_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override { if (handle_) client_->async_cancel_goal(handle_); }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Nav1>::SharedPtr client_;
  GH::SharedPtr handle_;
  bool done_{false}, ok_{false};
};

// Tell AMCL where the robot is parked ("start" in the route file), like 2D Pose Estimate in RViz.
// Holds for a couple of seconds so AMCL has applied it before the first goal.
class SetInitialPose : public BT::StatefulActionNode {
public:
  SetInitialPose(const std::string & n, const BT::NodeConfig & c, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(n, c), node_(node) {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  }
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("file")}; }

  BT::NodeStatus onStart() override {
    std::string file;
    if (!getInput("file", file) || file.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "SetInitialPose: no route file");
      return BT::NodeStatus::FAILURE;
    }
    try {
      YAML::Node root = YAML::LoadFile(file);
      if (!root["start"]) {
        RCLCPP_ERROR(node_->get_logger(), "SetInitialPose: no 'start' in %s", file.c_str());
        return BT::NodeStatus::FAILURE;
      }
      x_ = root["start"]["x"].as<double>();
      y_ = root["start"]["y"].as<double>();
      yaw_ = root["start"]["yaw"].as<double>();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "SetInitialPose: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
    t0_ = node_->now();
    sent_second_ = false;
    publish();
    RCLCPP_INFO(node_->get_logger(), "initial pose set to (%.2f, %.2f, yaw %.2f)", x_, y_, yaw_);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const double t = (node_->now() - t0_).seconds();
    if (!sent_second_ && t > 1.0) { publish(); sent_second_ = true; }   // in case the first was lost
    return t > 2.5 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  void publish() {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.pose.pose.position.x = x_;
    m.pose.pose.position.y = y_;
    m.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    m.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    m.pose.covariance[0] = 0.04;    // x   (0.2 m)
    m.pose.covariance[7] = 0.04;    // y
    m.pose.covariance[35] = 0.03;   // yaw (~10 deg)
    pub_->publish(m);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Time t0_;
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool sent_second_{false};
};

// Read a list of {x, y, yaw} from a yaml file (key = path_name)
class LoadWaypoints : public BT::SyncActionNode {
public:
  LoadWaypoints(const std::string & n, const BT::NodeConfig & c)
  : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("path_name"),
            BT::InputPort<std::string>("file"),
            BT::OutputPort<Poses>("goals")};
  }
  BT::NodeStatus tick() override {
    auto log = rclcpp::get_logger("LoadWaypoints");
    std::string key, file;
    if (!getInput("path_name", key)) return BT::NodeStatus::FAILURE;
    if (!getInput("file", file) || file.empty())
      file = share_dir("auracle_bringup") + "/config/waypoints.yaml";   // old behaviour
    Poses goals;
    try {
      YAML::Node root = YAML::LoadFile(file);
      if (!root[key] || !root[key].IsSequence() || root[key].size() == 0) {
        RCLCPP_ERROR(log, "'%s' in %s is missing or empty: fill in the route first",
                     key.c_str(), file.c_str());
        return BT::NodeStatus::FAILURE;
      }
      for (const auto & p : root[key]) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.pose.position.x = p["x"].as<double>();
        ps.pose.position.y = p["y"].as<double>();
        const double yaw = p["yaw"].as<double>();
        ps.pose.orientation.z = std::sin(yaw / 2.0);
        ps.pose.orientation.w = std::cos(yaw / 2.0);
        goals.push_back(ps);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(log, "cannot read %s: %s", file.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(log, "loaded %zu points from %s", goals.size(), file.c_str());
    setOutput("goals", goals);
    return BT::NodeStatus::SUCCESS;
  }
};

// Drive the route with Nav2
class FollowWaypoints : public BT::StatefulActionNode {
  using GH = rclcpp_action::ClientGoalHandle<Nav>;
public:
  FollowWaypoints(const std::string & n, const BT::NodeConfig & c, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(n, c), node_(node) {
    client_ = rclcpp_action::create_client<Nav>(node_, "navigate_through_poses");
  }
  static BT::PortsList providedPorts() { return {BT::InputPort<Poses>("goals")}; }
  BT::NodeStatus onStart() override {
    Poses goals;
    if (!getInput("goals", goals) || goals.empty()) return BT::NodeStatus::FAILURE;
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "navigate_through_poses is not available");
      return BT::NodeStatus::FAILURE;
    }
    Nav::Goal g;
    g.poses = goals;
    done_ = false; ok_ = false; handle_.reset();
    rclcpp_action::Client<Nav>::SendGoalOptions o;
    o.goal_response_callback = [this](GH::SharedPtr h) {
      if (!h) { done_ = true; ok_ = false; } else { handle_ = h; } };
    o.result_callback = [this](const GH::WrappedResult & r) {
      done_ = true;
      ok_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
      RCLCPP_INFO(node_->get_logger(), "route finished: %s", ok_ ? "succeeded" : "failed"); };
    client_->async_send_goal(g, o);
    RCLCPP_INFO(node_->get_logger(), "driving %zu points", goals.size());
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!done_) return BT::NodeStatus::RUNNING;
    return ok_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override { if (handle_) client_->async_cancel_goal(handle_); }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Nav>::SharedPtr client_;
  GH::SharedPtr handle_;
  bool done_{false}, ok_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_runner");   // use_sim_time comes from the launch file
  node->declare_parameter<std::string>("tree_file", "");
  node->declare_parameter<std::string>("route_file", "");
  node->declare_parameter<int>("route", 1);   // 1, 2, 3: which route in route.yaml (route_1, ...)

  std::string tree_file = node->get_parameter("tree_file").as_string();
  std::string route_file = node->get_parameter("route_file").as_string();
  const std::string share = share_dir("auracle_bt");
  if (tree_file.empty()) tree_file = "goal_tree.xml";
  if (tree_file.find('/') == std::string::npos) tree_file = share + "/behavior_trees/" + tree_file;
  if (route_file.empty()) route_file = share + "/config/route.yaml";

  auto state = std::make_shared<Shared>();
  auto label_sub = node->create_subscription<std_msgs::msg::String>(
    "/detected_class", 10,
    [state](std_msgs::msg::String::SharedPtr m) { state->label = m->data; });
  // AMCL latches its last pose, so a late subscriber still gets it (transient local).
  auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(1).transient_local().reliable(),
    [state](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) { state->have_amcl = true; });

  // RViz "2D Pose Estimate" publishes here (AMCL reads the same topic). Volatile: only seen while running.
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    [state](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) { state->have_pose_estimate = true; });
  // RViz "2D Goal Pose" (launched with goal_topic:=mission_goal). Taken only while WaitForGoal runs.
  auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "mission_goal", 10,
    [state, node](geometry_msgs::msg::PoseStamped::SharedPtr m) {
      if (!state->accepting_goal) {
        RCLCPP_WARN(node->get_logger(), "goal ignored: not ready for one (set the pose estimate first, or the robot is still driving)");
        return;
      }
      state->goal = *m;
      state->goal_ready = true; });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetDetectedClass>("GetDetectedClass", state);
  factory.registerNodeType<WaitForNav2>("WaitForNav2", node, state);
  factory.registerNodeType<WaitForInitialPose>("WaitForInitialPose", node, state);
  factory.registerNodeType<WaitForGoal>("WaitForGoal", node, state);
  factory.registerNodeType<NavigateToGoal>("NavigateToGoal", node);
  factory.registerNodeType<SetInitialPose>("SetInitialPose", node);
  factory.registerNodeType<LoadWaypoints>("LoadWaypoints");
  factory.registerNodeType<FollowWaypoints>("FollowWaypoints", node);

  auto tree = factory.createTreeFromFile(tree_file);
  tree.rootBlackboard()->set("route_file", route_file);   // the trees read it as {route_file}
  const std::string route_key = "route_" + std::to_string(node->get_parameter("route").as_int());
  tree.rootBlackboard()->set("route_key", route_key);       // {route_key} in region_tree.xml
  RCLCPP_INFO(node->get_logger(), "tree: %s | file: %s | %s", tree_file.c_str(), route_file.c_str(), route_key.c_str());

  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }
  tree.haltTree();   // cancels the Nav2 goal if we were interrupted
  RCLCPP_INFO(node->get_logger(), "Tree finished: %s", BT::toStr(status).c_str());
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}