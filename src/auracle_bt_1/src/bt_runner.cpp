// auracle_bt mission runner.
//
// Sits above Nav2: it sends goals to the NavigateThroughPoses action, it does not drive.
// Default tree (behavior_trees/region_tree.xml): wait for Nav2 + AMCL -> give AMCL the parked start
// pose -> load the route from config/route.yaml -> drive it. Nothing has to be clicked.
// The old vision-driven tree (sorting_tree.xml) still loads: pass tree_file to select it.
//
// Run with the same namespace as Nav2 (launch/mission.launch.py, namespace:=auracle), so the
// relative names below become /auracle/navigate_through_poses, /auracle/amcl_pose, /auracle/initialpose.
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
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

// Things the ROS callbacks in main() tell the tree nodes.
struct Shared {
  std::string label;        // vision label (sorting tree only)
  bool have_amcl{false};    // AMCL has published a pose: it is up
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
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    t0_ = node_->now();
    last_log_ = t0_;
    return check();
  }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    const bool server = client_->action_server_is_ready();
    if (server && state_->have_amcl) {
      RCLCPP_INFO(node_->get_logger(), "Nav2 and AMCL are up");
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

  std::string tree_file = node->get_parameter("tree_file").as_string();
  std::string route_file = node->get_parameter("route_file").as_string();
  const std::string share = share_dir("auracle_bt");
  if (tree_file.empty()) tree_file = share + "/behavior_trees/region_tree.xml";
  if (route_file.empty()) route_file = share + "/config/route.yaml";

  auto state = std::make_shared<Shared>();
  auto label_sub = node->create_subscription<std_msgs::msg::String>(
    "/detected_class", 10,
    [state](std_msgs::msg::String::SharedPtr m) { state->label = m->data; });
  // AMCL latches its last pose, so a late subscriber still gets it (transient local).
  auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(1).transient_local().reliable(),
    [state](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) { state->have_amcl = true; });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetDetectedClass>("GetDetectedClass", state);
  factory.registerNodeType<WaitForNav2>("WaitForNav2", node, state);
  factory.registerNodeType<SetInitialPose>("SetInitialPose", node);
  factory.registerNodeType<LoadWaypoints>("LoadWaypoints");
  factory.registerNodeType<FollowWaypoints>("FollowWaypoints", node);

  auto tree = factory.createTreeFromFile(tree_file);
  tree.rootBlackboard()->set("route_file", route_file);   // the trees read it as {route_file}
  RCLCPP_INFO(node->get_logger(), "tree: %s | route: %s", tree_file.c_str(), route_file.c_str());

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