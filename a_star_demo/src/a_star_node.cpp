#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
struct Cell
{
  int x{0};
  int y{0};
  double g{0.0};
  double h{0.0};
  std::shared_ptr<Cell> parent{nullptr};

  double f() const { return g + h; }
};

using CellPtr = std::shared_ptr<Cell>;

std::vector<std::pair<int, int>> reconstructPath(const CellPtr & goal)
{
  std::vector<std::pair<int, int>> path;
  for (auto cur = goal; cur != nullptr; cur = cur->parent) {
    path.emplace_back(cur->x, cur->y);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

int manhattan(const CellPtr & a, const CellPtr & b)
{
  return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}
}

class AStarNode : public rclcpp::Node
{
public:
  AStarNode() : Node("a_star_node")
  {
    declare_parameter<int>("start_x", 0);
    declare_parameter<int>("start_y", 0);
    declare_parameter<int>("goal_x", 9);
    declare_parameter<int>("goal_y", 9);
    declare_parameter<std::string>("frame_id", "map");

    loadGrid();
    computePath();

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local();
    latched_qos.reliable();

    path_pub_ = create_publisher<nav_msgs::msg::Path>("a_star/path", latched_qos);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishPath(); });
  }

private:
  void loadGrid()
  {
    grid_ = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 1, 0, 0, 0},
      {0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
      {0, 0, 1, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
  }

  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(grid_.size()) &&
           y < static_cast<int>(grid_.front().size());
  }

  bool isFree(int x, int y) const
  {
    return inBounds(x, y) && grid_[x][y] == 0;
  }

  std::vector<std::pair<int, int>> neighbors(int x, int y) const
  {
    static const std::vector<std::pair<int, int>> dirs{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    std::vector<std::pair<int, int>> out;
    out.reserve(4);
    for (const auto & d : dirs) {
      int nx = x + d.first;
      int ny = y + d.second;
      if (isFree(nx, ny)) {
        out.emplace_back(nx, ny);
      }
    }
    return out;
  }

  bool closed(int x, int y) const
  {
    return closed_[x][y];
  }

  void markClosed(int x, int y)
  {
    closed_[x][y] = true;
  }

  std::vector<std::pair<int, int>> search(int sx, int sy, int gx, int gy)
  {
    closed_.assign(grid_.size(), std::vector<bool>(grid_.front().size(), false));

    auto start = std::make_shared<Cell>();
    start->x = sx;
    start->y = sy;

    auto goal = std::make_shared<Cell>();
    goal->x = gx;
    goal->y = gy;

    auto cmp = [](const CellPtr & a, const CellPtr & b) { return a->f() > b->f(); };
    std::priority_queue<CellPtr, std::vector<CellPtr>, decltype(cmp)> open(cmp);
    open.push(start);

    while (!open.empty()) {
      auto cur = open.top();
      open.pop();

      if (!inBounds(cur->x, cur->y) || closed(cur->x, cur->y)) {
        continue;
      }

      markClosed(cur->x, cur->y);

      if (cur->x == goal->x && cur->y == goal->y) {
        return reconstructPath(cur);
      }

      for (const auto & n : neighbors(cur->x, cur->y)) {
        auto next = std::make_shared<Cell>();
        next->x = n.first;
        next->y = n.second;
        next->g = cur->g + 1.0;
        next->h = static_cast<double>(manhattan(next, goal));
        next->parent = cur;
        open.push(next);
      }
    }

    return {};
  }

  void computePath()
  {
    int sx = get_parameter("start_x").as_int();
    int sy = get_parameter("start_y").as_int();
    int gx = get_parameter("goal_x").as_int();
    int gy = get_parameter("goal_y").as_int();
    auto frame_id = get_parameter("frame_id").as_string();

    if (!isFree(sx, sy) || !isFree(gx, gy)) {
      RCLCPP_WARN(get_logger(), "Start or goal is an obstacle or out of bounds");
      return;
    }

    auto cells = search(sx, sy, gx, gy);
    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No path found");
      return;
    }

    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = now();
    msg.poses.reserve(cells.size());

    for (const auto & c : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = static_cast<double>(c.first);
      pose.pose.position.y = static_cast<double>(c.second);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_msg_ = msg;
    RCLCPP_INFO(get_logger(), "Planned path with %zu steps", path_msg_.poses.size());
  }

  void publishPath()
  {
    if (path_msg_.poses.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Path empty; nothing to publish");
      return;
    }
    path_msg_.header.stamp = now();
    path_pub_->publish(path_msg_);
  }

  std::vector<std::vector<int>> grid_{};
  std::vector<std::vector<bool>> closed_{};
  nav_msgs::msg::Path path_msg_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarNode>());
  rclcpp::shutdown();
  return 0;
}
