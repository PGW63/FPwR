#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "find_start_bar/action/find_bar_plane.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::duration<double, std::milli>;

double elapsed_ms(const Clock::time_point & from)
{
  return std::chrono::duration_cast<Ms>(Clock::now() - from).count();
}

struct Point3
{
  float x{};
  float y{};
  float z{};
};

struct Plane
{
  float a{};
  float b{};
  float c{};
  float d{};
};

Point3 transform_point(const Point3 & point, const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q = transform.transform.rotation;
  const auto & t = transform.transform.translation;

  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  return {
    static_cast<float>(
      (1.0 - 2.0 * (yy + zz)) * point.x +
      (2.0 * (xy - wz)) * point.y +
      (2.0 * (xz + wy)) * point.z + t.x),
    static_cast<float>(
      (2.0 * (xy + wz)) * point.x +
      (1.0 - 2.0 * (xx + zz)) * point.y +
      (2.0 * (yz - wx)) * point.z + t.y),
    static_cast<float>(
      (2.0 * (xz - wy)) * point.x +
      (2.0 * (yz + wx)) * point.y +
      (1.0 - 2.0 * (xx + yy)) * point.z + t.z)};
}

Point3 transform_origin(const geometry_msgs::msg::TransformStamped & transform)
{
  return {
    static_cast<float>(transform.transform.translation.x),
    static_cast<float>(transform.transform.translation.y),
    static_cast<float>(transform.transform.translation.z)};
}

bool read_float32_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const size_t byte_index,
  float & value)
{
  if (byte_index + sizeof(float) > cloud.data.size()) {
    return false;
  }
  std::memcpy(&value, cloud.data.data() + byte_index, sizeof(float));
  return std::isfinite(value);
}

int field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name &&
             field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    });
  return it == cloud.fields.end() ? -1 : static_cast<int>(it->offset);
}

std::vector<Point3> extract_points(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const geometry_msgs::msg::TransformStamped & transform,
  const Point3 & robot_position,
  const float max_range_m)
{
  std::vector<Point3> points;
  const int x_offset = field_offset(cloud, "x");
  const int y_offset = field_offset(cloud, "y");
  const int z_offset = field_offset(cloud, "z");
  if (x_offset < 0 || y_offset < 0 || z_offset < 0 || cloud.point_step == 0) {
    return points;
  }

  const size_t count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  points.reserve(count);
  const float max_range_sq = max_range_m * max_range_m;

  for (size_t i = 0; i < count; ++i) {
    const size_t point_start = i * cloud.point_step;
    float x{};
    float y{};
    float z{};
    if (!read_float32_field(cloud, point_start + x_offset, x) ||
      !read_float32_field(cloud, point_start + y_offset, y) ||
      !read_float32_field(cloud, point_start + z_offset, z))
    {
      continue;
    }

    const Point3 map_point = transform_point({x, y, z}, transform);
    const float dx = map_point.x - robot_position.x;
    const float dy = map_point.y - robot_position.y;
    const float range_sq = dx * dx + dy * dy;
    if (range_sq <= max_range_sq) {
      points.push_back(map_point);
    }
  }

  return points;
}

bool plane_from_points(const Point3 & p1, const Point3 & p2, const Point3 & p3, Plane & plane)
{
  const float ux = p2.x - p1.x;
  const float uy = p2.y - p1.y;
  const float uz = p2.z - p1.z;
  const float vx = p3.x - p1.x;
  const float vy = p3.y - p1.y;
  const float vz = p3.z - p1.z;

  const float a = uy * vz - uz * vy;
  const float b = uz * vx - ux * vz;
  const float c = ux * vy - uy * vx;
  const float norm = std::sqrt(a * a + b * b + c * c);
  if (norm < 1.0e-6F) {
    return false;
  }

  plane.a = a / norm;
  plane.b = b / norm;
  plane.c = c / norm;
  plane.d = -(plane.a * p1.x + plane.b * p1.y + plane.c * p1.z);
  return true;
}

float point_plane_distance(const Point3 & point, const Plane & plane)
{
  return std::fabs(plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d);
}

}  // namespace

class FindStartBarNode : public rclcpp::Node
{
public:
  using FindBarPlane = find_start_bar::action::FindBarPlane;
  using GoalHandleFindBarPlane = rclcpp_action::ServerGoalHandle<FindBarPlane>;

  FindStartBarNode()
  : Node("find_start_bar_node")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    default_max_range_m_ = declare_parameter<double>("max_range_m", 2.0);
    default_distance_threshold_m_ = declare_parameter<double>("distance_threshold_m", 0.05);
    default_min_inliers_ = declare_parameter<int>("min_inliers", 200);
    default_max_iterations_ = declare_parameter<int>("max_iterations", 300);
    default_approach_offset_m_ = declare_parameter<double>("approach_offset_m", 0.5);
    default_accumulation_frames_ = declare_parameter<int>("accumulation_frames", 10);
    max_stored_frames_ = declare_parameter<int>("max_stored_frames", 10);
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        recent_clouds_.push_back(msg);
        while (static_cast<int>(recent_clouds_.size()) > max_stored_frames_) {
          recent_clouds_.pop_front();
        }
      });

    action_server_ = rclcpp_action::create_server<FindBarPlane>(
      this,
      "find_bar_plane",
      std::bind(&FindStartBarNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FindStartBarNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FindStartBarNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "FindStartBar action server ready. cloud_topic=%s action=find_bar_plane target_frame=%s robot_frame=%s",
      cloud_topic_.c_str(), target_frame_.c_str(), robot_frame_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FindBarPlane::Goal> goal)
  {
    if (!goal || !goal->start) {
      RCLCPP_WARN(get_logger(), "Rejecting find_bar_plane goal: start=false");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Received find_bar_plane goal: start=true");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindBarPlane>)
  {
    RCLCPP_INFO(get_logger(), "Received find_bar_plane cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindBarPlane> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "find_bar_plane goal accepted");
    std::thread{std::bind(&FindStartBarNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindBarPlane> goal_handle)
  {
    const auto start_time = Clock::now();
    auto result = std::make_shared<FindBarPlane::Result>();
    publish_feedback(goal_handle, "started", 0);

    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> clouds;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      clouds.assign(recent_clouds_.begin(), recent_clouds_.end());
    }

    if (clouds.empty()) {
      result->success = false;
      RCLCPP_WARN(get_logger(), "find_bar_plane failed: No PointCloud2 has been received yet.");
      goal_handle->succeed(result);
      return;
    }

    const float max_range_m = static_cast<float>(default_max_range_m_);
    const float distance_threshold_m = static_cast<float>(default_distance_threshold_m_);
    const int min_inliers = default_min_inliers_;
    const int max_iterations = default_max_iterations_;
    const float approach_offset_m = static_cast<float>(default_approach_offset_m_);
    const int accumulation_frames = default_accumulation_frames_;

    if (static_cast<int>(clouds.size()) > accumulation_frames) {
      clouds.erase(clouds.begin(), clouds.end() - accumulation_frames);
    }

    RCLCPP_INFO(
      get_logger(),
      "Starting plane extraction: buffered_frames=%zu requested_frames=%d used_candidates=%zu "
      "max_range=%.3f threshold=%.3f min_inliers=%d iterations=%d offset=%.3f",
      recent_cloud_count(), accumulation_frames, clouds.size(), max_range_m,
      distance_threshold_m, min_inliers, max_iterations, approach_offset_m);

    Point3 robot_position;
    try {
      publish_feedback(goal_handle, "looking_up_robot_tf", 10);
      robot_position = transform_origin(
        tf_buffer_->lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException & ex) {
      result->success = false;
      RCLCPP_WARN(get_logger(), "find_bar_plane failed: Failed to lookup robot transform: %s", ex.what());
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Robot pose for extraction: frame=%s robot_frame=%s xy=(%.3f, %.3f)",
      target_frame_.c_str(), robot_frame_.c_str(), robot_position.x, robot_position.y);

    std::vector<Point3> points;
    int used_frames = 0;
    publish_feedback(goal_handle, "accumulating_clouds", 20);
    for (const auto & cloud : clouds) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          target_frame_, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
        const auto frame_points = extract_points(*cloud, transform, robot_position, max_range_m);
        RCLCPP_INFO(
          get_logger(), "Accumulated cloud frame: source_frame=%s raw_points=%u kept_points=%zu",
          cloud->header.frame_id.c_str(), cloud->width * cloud->height, frame_points.size());
        points.insert(points.end(), frame_points.begin(), frame_points.end());
        ++used_frames;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping cloud frame because TF lookup failed: %s", ex.what());
      }
    }

    if (points.size() < 3) {
      result->success = false;
      RCLCPP_WARN(
        get_logger(),
        "find_bar_plane failed: Not enough transformed valid points within max_range_m. "
        "used_frames=%d points=%zu",
        used_frames, points.size());
      goal_handle->succeed(result);
      return;
    }

    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<size_t> sample_index(0, points.size() - 1);

    Plane best_plane;
    int best_inliers = 0;
    publish_feedback(goal_handle, "running_ransac", 40);

    for (int iteration = 1; iteration <= max_iterations; ++iteration) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        RCLCPP_INFO(get_logger(), "find_bar_plane canceled after %.2f ms", elapsed_ms(start_time));
        goal_handle->canceled(result);
        return;
      }

      const size_t i1 = sample_index(rng);
      const size_t i2 = sample_index(rng);
      const size_t i3 = sample_index(rng);
      if (i1 == i2 || i1 == i3 || i2 == i3) {
        continue;
      }

      Plane candidate;
      if (!plane_from_points(points[i1], points[i2], points[i3], candidate)) {
        continue;
      }

      int inliers = 0;
      for (const auto & point : points) {
        if (point_plane_distance(point, candidate) <= distance_threshold_m) {
          ++inliers;
        }
      }

      if (inliers > best_inliers) {
        best_inliers = inliers;
        best_plane = candidate;
      }

      if (iteration == 1 || iteration % 20 == 0 || iteration == max_iterations) {
        const int progress =
          40 + static_cast<int>((50.0 * static_cast<double>(iteration)) / max_iterations);
        publish_feedback(goal_handle, "running_ransac", std::min(progress, 90));
      }
    }

    result->success = best_inliers >= min_inliers;
    if (result->success) {
      publish_feedback(goal_handle, "computing_target", 90);
      std::vector<size_t> inlier_indices;
      inlier_indices.reserve(static_cast<size_t>(best_inliers));
      Point3 inlier_centroid;
      for (size_t i = 0; i < points.size(); ++i) {
        if (point_plane_distance(points[i], best_plane) > distance_threshold_m) {
          continue;
        }

        inlier_indices.push_back(i);
        inlier_centroid.x += points[i].x;
        inlier_centroid.y += points[i].y;
        inlier_centroid.z += points[i].z;
      }

      if (inlier_indices.empty()) {
        result->success = false;
        RCLCPP_WARN(
          get_logger(), "find_bar_plane failed: Plane found, but could not determine a usable edge point.");
        goal_handle->succeed(result);
        return;
      }

      const float inv_inliers = 1.0F / static_cast<float>(inlier_indices.size());
      inlier_centroid.x *= inv_inliers;
      inlier_centroid.y *= inv_inliers;
      inlier_centroid.z *= inv_inliers;

      float plane_to_robot_x = robot_position.x - inlier_centroid.x;
      float plane_to_robot_y = robot_position.y - inlier_centroid.y;
      const float plane_to_robot_norm =
        std::sqrt(plane_to_robot_x * plane_to_robot_x + plane_to_robot_y * plane_to_robot_y);
      if (plane_to_robot_norm < 1.0e-6F) {
        result->success = false;
        RCLCPP_WARN(
          get_logger(),
          "find_bar_plane failed: Plane found, but robot is too close to the plane centroid direction.");
        goal_handle->succeed(result);
        return;
      }

      plane_to_robot_x /= plane_to_robot_norm;
      plane_to_robot_y /= plane_to_robot_norm;

      size_t edge_index = inlier_indices.front();
      float best_edge_projection = -std::numeric_limits<float>::max();
      for (const size_t index : inlier_indices) {
        const float projection =
          (points[index].x - inlier_centroid.x) * plane_to_robot_x +
          (points[index].y - inlier_centroid.y) * plane_to_robot_y;
        if (projection > best_edge_projection) {
          best_edge_projection = projection;
          edge_index = index;
        }
      }

      const Point3 edge_point = points[edge_index];
      const float dx = robot_position.x - edge_point.x;
      const float dy = robot_position.y - edge_point.y;
      const float distance_to_edge = std::sqrt(dx * dx + dy * dy);
      if (distance_to_edge < 1.0e-6F) {
        result->success = false;
        RCLCPP_WARN(
          get_logger(), "find_bar_plane failed: Plane found, but robot is already on the selected edge point.");
        goal_handle->succeed(result);
        return;
      }

      const float direction_x = (robot_position.x - edge_point.x) / distance_to_edge;
      const float direction_y = (robot_position.y - edge_point.y) / distance_to_edge;

      result->target_x_m = edge_point.x + direction_x * approach_offset_m;
      result->target_y_m = edge_point.y + direction_y * approach_offset_m;
      RCLCPP_INFO(
        get_logger(),
        "Plane edge selected: edge=(%.3f, %.3f) target=(%.3f, %.3f) "
        "distance_to_edge=%.3f offset=%.3f plane=[%.3f %.3f %.3f %.3f]",
        edge_point.x, edge_point.y, result->target_x_m, result->target_y_m,
        distance_to_edge, approach_offset_m,
        best_plane.a, best_plane.b, best_plane.c, best_plane.d);
      publish_feedback(goal_handle, "succeeded", 100);
    } else {
      RCLCPP_WARN(
        get_logger(), "find_bar_plane failed: No plane reached min_inliers. best_inliers=%d min_inliers=%d",
        best_inliers, min_inliers);
    }

    RCLCPP_INFO(
      get_logger(),
      "RANSAC result: success=%s frames=%d points=%zu inliers=%d threshold=%.3f "
      "max_range=%.2f target=(%.3f, %.3f) elapsed=%.2f ms",
      result->success ? "true" : "false", used_frames, points.size(), best_inliers,
      distance_threshold_m, max_range_m, result->target_x_m, result->target_y_m,
      elapsed_ms(start_time));

    goal_handle->succeed(result);
  }

  size_t recent_cloud_count()
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return recent_clouds_.size();
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleFindBarPlane> & goal_handle,
    const std::string & state,
    const int progress_percent)
  {
    auto feedback = std::make_shared<FindBarPlane::Feedback>();
    feedback->state = state;
    feedback->progress_percent = progress_percent;
    goal_handle->publish_feedback(feedback);
  }

  std::string cloud_topic_;
  double default_max_range_m_{};
  double default_distance_threshold_m_{};
  double default_approach_offset_m_{};
  int default_min_inliers_{};
  int default_max_iterations_{};
  int default_accumulation_frames_{};
  int max_stored_frames_{};
  std::string target_frame_;
  std::string robot_frame_;

  std::mutex cloud_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> recent_clouds_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_action::Server<FindBarPlane>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FindStartBarNode>());
  rclcpp::shutdown();
  return 0;
}
