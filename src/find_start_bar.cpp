#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "inha_interfaces/action/find_bar_plane.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::duration<double, std::milli>;
constexpr double kPi = 3.14159265358979323846;

double elapsed_ms(const Clock::time_point & from)
{
  return std::chrono::duration_cast<Ms>(Clock::now() - from).count();
}

bool directory_exists(const std::string & path)
{
  struct stat info {};
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool ensure_directory(const std::string & path)
{
  if (path.empty()) {
    return false;
  }

  if (directory_exists(path)) {
    return true;
  }

  std::string current;
  current.reserve(path.size());
  for (const char ch : path) {
    current.push_back(ch);
    if (ch != '/' || current.size() == 1) {
      continue;
    }

    if (!directory_exists(current) && mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }

  return directory_exists(path) || mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

std::string timestamp_string(const char * format)
{
  const std::time_t now = std::time(nullptr);
  std::tm local_time {};
  localtime_r(&now, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, format);
  return stream.str();
}

template<typename... Args>
std::string format_message(const char * format, Args... args)
{
  const int size = std::snprintf(nullptr, 0, format, args...);
  if (size <= 0) {
    return format;
  }

  std::vector<char> buffer(static_cast<size_t>(size) + 1);
  std::snprintf(buffer.data(), buffer.size(), format, args...);
  return std::string(buffer.data(), static_cast<size_t>(size));
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
  const geometry_msgs::msg::TransformStamped & cloud_to_output_transform,
  const geometry_msgs::msg::TransformStamped & cloud_to_robot_transform,
  const float roi_x_min_m,
  const float roi_x_max_m,
  const float roi_y_min_m,
  const float roi_y_max_m,
  const float roi_z_min_m,
  const float roi_z_max_m,
  const float robot_clearance_m)
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
  const float robot_clearance_sq = robot_clearance_m * robot_clearance_m;

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

    const Point3 robot_point = transform_point({x, y, z}, cloud_to_robot_transform);
    if (robot_point.x < roi_x_min_m || robot_point.x > roi_x_max_m ||
      robot_point.y < roi_y_min_m || robot_point.y > roi_y_max_m)
    {
      continue;
    }

    if (robot_point.z < roi_z_min_m || robot_point.z > roi_z_max_m) {
      continue;
    }

    const float robot_distance_sq = robot_point.x * robot_point.x + robot_point.y * robot_point.y;
    if (robot_distance_sq <= robot_clearance_sq) {
      continue;
    }

    const Point3 output_point = transform_point({x, y, z}, cloud_to_output_transform);
    points.push_back(output_point);
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

Point3 centroid_from_indices(
  const std::vector<Point3> & points,
  const std::vector<size_t> & indices)
{
  Point3 centroid;
  if (indices.empty()) {
    return centroid;
  }

  for (const size_t index : indices) {
    centroid.x += points[index].x;
    centroid.y += points[index].y;
    centroid.z += points[index].z;
  }

  const float inv_count = 1.0F / static_cast<float>(indices.size());
  centroid.x *= inv_count;
  centroid.y *= inv_count;
  centroid.z *= inv_count;
  return centroid;
}

float median_value(std::vector<float> values)
{
  if (values.empty()) {
    return 0.0F;
  }

  const size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const float upper = values[middle];
  if ((values.size() % 2U) != 0U) {
    return upper;
  }

  const float lower = *std::max_element(values.begin(), values.begin() + middle);
  return 0.5F * (lower + upper);
}

Point3 median_from_indices(
  const std::vector<Point3> & points,
  const std::vector<size_t> & indices)
{
  std::vector<float> x_values;
  std::vector<float> y_values;
  std::vector<float> z_values;
  x_values.reserve(indices.size());
  y_values.reserve(indices.size());
  z_values.reserve(indices.size());

  for (const size_t index : indices) {
    x_values.push_back(points[index].x);
    y_values.push_back(points[index].y);
    z_values.push_back(points[index].z);
  }

  return {
    median_value(std::move(x_values)),
    median_value(std::move(y_values)),
    median_value(std::move(z_values))};
}

bool select_robot_side_edge_point(
  const std::vector<Point3> & points,
  const std::vector<size_t> & indices,
  const Point3 & inlier_centroid,
  const Point3 & robot_position,
  Point3 & edge_point,
  float & distance_to_edge)
{
  if (indices.empty()) {
    return false;
  }

  float plane_to_robot_x = robot_position.x - inlier_centroid.x;
  float plane_to_robot_y = robot_position.y - inlier_centroid.y;
  const float plane_to_robot_norm =
    std::sqrt(plane_to_robot_x * plane_to_robot_x + plane_to_robot_y * plane_to_robot_y);
  if (plane_to_robot_norm < 1.0e-6F) {
    return false;
  }

  plane_to_robot_x /= plane_to_robot_norm;
  plane_to_robot_y /= plane_to_robot_norm;

  size_t edge_index = indices.front();
  float best_edge_projection = -std::numeric_limits<float>::max();
  for (const size_t index : indices) {
    const float projection =
      (points[index].x - inlier_centroid.x) * plane_to_robot_x +
      (points[index].y - inlier_centroid.y) * plane_to_robot_y;
    if (projection > best_edge_projection) {
      best_edge_projection = projection;
      edge_index = index;
    }
  }

  edge_point = points[edge_index];
  const float dx = robot_position.x - edge_point.x;
  const float dy = robot_position.y - edge_point.y;
  distance_to_edge = std::sqrt(dx * dx + dy * dy);
  return true;
}

int remove_vertical_planes(
  std::vector<Point3> & points,
  const int max_planes,
  const int max_iterations,
  const float distance_threshold_m,
  const int min_inliers,
  const float max_abs_normal_z,
  std::mt19937 & rng)
{
  int total_removed = 0;

  for (int plane_index = 0; plane_index < max_planes; ++plane_index) {
    if (points.size() < 3) {
      break;
    }

    std::uniform_int_distribution<size_t> sample_index(0, points.size() - 1);
    Plane best_plane;
    std::vector<size_t> best_inliers;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
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
      if (std::fabs(candidate.c) > max_abs_normal_z) {
        continue;
      }

      std::vector<size_t> inliers;
      inliers.reserve(points.size());
      for (size_t i = 0; i < points.size(); ++i) {
        if (point_plane_distance(points[i], candidate) <= distance_threshold_m) {
          inliers.push_back(i);
        }
      }

      if (inliers.size() > best_inliers.size()) {
        best_inliers = std::move(inliers);
        best_plane = candidate;
      }
    }

    if (static_cast<int>(best_inliers.size()) < min_inliers) {
      break;
    }

    (void)best_plane;
    std::vector<bool> remove_mask(points.size(), false);
    for (const size_t index : best_inliers) {
      remove_mask[index] = true;
    }

    std::vector<Point3> kept_points;
    kept_points.reserve(points.size() - best_inliers.size());
    for (size_t i = 0; i < points.size(); ++i) {
      if (!remove_mask[i]) {
        kept_points.push_back(points[i]);
      }
    }

    total_removed += static_cast<int>(best_inliers.size());
    points.swap(kept_points);
  }

  return total_removed;
}

sensor_msgs::msg::PointCloud2 make_point_cloud_msg(
  const std::vector<Point3> & points,
  const std::vector<size_t> & indices,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(indices.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = sizeof(float);
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 2 * sizeof(float);
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.data.resize(static_cast<size_t>(cloud.row_step));
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto & point = points[indices[i]];
    const size_t offset = i * cloud.point_step;
    std::memcpy(cloud.data.data() + offset, &point.x, sizeof(float));
    std::memcpy(cloud.data.data() + offset + sizeof(float), &point.y, sizeof(float));
    std::memcpy(cloud.data.data() + offset + 2 * sizeof(float), &point.z, sizeof(float));
  }

  return cloud;
}

}  // namespace

class FindStartBarNode : public rclcpp::Node
{
public:
  using FindBarPlane = inha_interfaces::action::FindBarPlane;
  using GoalHandleFindBarPlane = rclcpp_action::ServerGoalHandle<FindBarPlane>;

  FindStartBarNode()
  : Node("find_start_bar_node")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    default_roi_x_min_m_ = declare_parameter<double>("roi_x_min_m", 0.0);
    default_roi_x_max_m_ = declare_parameter<double>("roi_x_max_m", 2.3);
    default_roi_y_min_m_ = declare_parameter<double>("roi_y_min_m", -0.5);
    default_roi_y_max_m_ = declare_parameter<double>("roi_y_max_m", 0.5);
    default_roi_z_min_m_ = declare_parameter<double>("roi_z_min_m", 0.5);
    default_roi_z_max_m_ = declare_parameter<double>("roi_z_max_m", 1.6);
    default_robot_clearance_m_ = declare_parameter<double>("robot_clearance_m", 0.45);
    default_distance_threshold_m_ = declare_parameter<double>("distance_threshold_m", 0.02);
    default_max_plane_tilt_deg_ = declare_parameter<double>("max_plane_tilt_deg", 10.0);
    remove_walls_ = declare_parameter<bool>("remove_walls", true);
    wall_max_planes_ = declare_parameter<int>("wall_max_planes", 2);
    wall_distance_threshold_m_ = declare_parameter<double>("wall_distance_threshold_m", 0.03);
    wall_min_inliers_ = declare_parameter<int>("wall_min_inliers", 300);
    wall_max_abs_normal_z_ = declare_parameter<double>("wall_max_abs_normal_z", 0.25);
    default_min_inliers_ = declare_parameter<int>("min_inliers", 300);
    default_max_iterations_ = declare_parameter<int>("max_iterations", 300);
    map_z_offset_m_ = declare_parameter<double>("map_z_offset_m", -0.226);
    default_accumulation_frames_ = declare_parameter<int>("accumulation_frames", 10);
    max_stored_frames_ = declare_parameter<int>("max_stored_frames", 10);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_nav");
    debug_plane_topic_ = declare_parameter<std::string>(
      "debug_plane_topic", "/find_start_bar/plane_inliers");
    target_z_topic_ = declare_parameter<std::string>(
      "target_z_topic", "/find_start_bar/target_z_m");
    log_directory_ = declare_parameter<std::string>(
      "log_directory", "/home/nvidia/inha_log/module/find_start_bar");

    initialize_file_logger();

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
    debug_plane_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      debug_plane_topic_, rclcpp::QoS(1).reliable());
    target_z_pub_ = create_publisher<std_msgs::msg::Float32>(
      target_z_topic_, rclcpp::QoS(1).reliable());
    target_z_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&FindStartBarNode::publish_target_z, this));

    action_server_ = rclcpp_action::create_server<FindBarPlane>(
      this,
      "find_bar_plane",
      std::bind(&FindStartBarNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FindStartBarNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FindStartBarNode::handle_accepted, this, std::placeholders::_1));

    log_info(
      "FindStartBar action server ready. cloud_topic=%s action=find_bar_plane map_frame=%s "
      "odom_frame=%s robot_frame=%s debug_plane_topic=%s target_z_topic=%s roi_x=[%.2f, %.2f] "
      "roi_y=[%.2f, %.2f] roi_z=[%.2f, %.2f] "
      "robot_clearance=%.2f max_plane_tilt=%.1f deg map_z_offset=%.3f remove_walls=%s log_file=%s",
      cloud_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), robot_frame_.c_str(),
      debug_plane_topic_.c_str(), target_z_topic_.c_str(),
      default_roi_x_min_m_, default_roi_x_max_m_, default_roi_y_min_m_, default_roi_y_max_m_,
      default_roi_z_min_m_, default_roi_z_max_m_, default_robot_clearance_m_,
      default_max_plane_tilt_deg_, map_z_offset_m_,
      remove_walls_ ? "true" : "false", log_file_path_.empty() ? "disabled" : log_file_path_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FindBarPlane::Goal> goal)
  {
    if (!goal) {
      log_warn("Rejecting find_bar_plane goal: goal is null");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->frame != map_frame_ && goal->frame != odom_frame_) {
      log_warn(
        "Rejecting find_bar_plane goal: unsupported frame='%s' (allowed: '%s', '%s')",
        goal->frame.c_str(), map_frame_.c_str(), odom_frame_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    log_info("Received find_bar_plane goal: frame=%s", goal->frame.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindBarPlane>)
  {
    log_info("Received find_bar_plane cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindBarPlane> goal_handle)
  {
    const size_t target_z_goal_sequence = pause_target_z_for_new_goal();
    log_info("find_bar_plane goal accepted; target z topic publishing paused");
    std::thread{
      std::bind(&FindStartBarNode::execute, this, goal_handle, target_z_goal_sequence)}.detach();
  }

  void execute(
    const std::shared_ptr<GoalHandleFindBarPlane> goal_handle,
    const size_t target_z_goal_sequence)
  {
    const auto start_time = Clock::now();
    auto result = std::make_shared<FindBarPlane::Result>();
    const std::string output_frame = goal_handle->get_goal()->frame;
    publish_feedback(goal_handle, "started", 0);

    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> clouds;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      clouds.assign(recent_clouds_.begin(), recent_clouds_.end());
    }

    if (clouds.empty()) {
      result->success = false;
      log_warn("find_bar_plane failed: No PointCloud2 has been received yet.");
      goal_handle->succeed(result);
      return;
    }

    const float roi_x_min_m = static_cast<float>(default_roi_x_min_m_);
    const float roi_x_max_m = static_cast<float>(default_roi_x_max_m_);
    const float roi_y_min_m = static_cast<float>(default_roi_y_min_m_);
    const float roi_y_max_m = static_cast<float>(default_roi_y_max_m_);
    const float roi_z_min_m = static_cast<float>(default_roi_z_min_m_);
    const float roi_z_max_m = static_cast<float>(default_roi_z_max_m_);
    const float robot_clearance_m = static_cast<float>(default_robot_clearance_m_);
    const float distance_threshold_m = static_cast<float>(default_distance_threshold_m_);
    const float min_abs_normal_z = static_cast<float>(
      std::cos(default_max_plane_tilt_deg_ * kPi / 180.0));
    const float wall_distance_threshold_m = static_cast<float>(wall_distance_threshold_m_);
    const float wall_max_abs_normal_z = static_cast<float>(wall_max_abs_normal_z_);
    const int min_inliers = std::max(1, default_min_inliers_);
    const int max_iterations = default_max_iterations_;
    const float map_z_offset_m = static_cast<float>(map_z_offset_m_);
    const int accumulation_frames = default_accumulation_frames_;

    if (static_cast<int>(clouds.size()) > accumulation_frames) {
      clouds.erase(clouds.begin(), clouds.end() - accumulation_frames);
    }

    log_info(
      "Starting plane extraction: buffered_frames=%zu requested_frames=%d used_candidates=%zu "
      "roi_x=[%.3f, %.3f] roi_y=[%.3f, %.3f] roi_z=[%.3f, %.3f] "
      "robot_clearance=%.3f threshold=%.3f "
      "max_tilt=%.1fdeg min_abs_normal_z=%.3f remove_walls=%s wall_threshold=%.3f "
      "wall_min_inliers=%d wall_max_abs_normal_z=%.3f min_inliers=%d iterations=%d "
      "map_z_offset=%.3f output_frame=%s",
      recent_cloud_count(), accumulation_frames, clouds.size(), roi_x_min_m, roi_x_max_m,
      roi_y_min_m, roi_y_max_m, roi_z_min_m, roi_z_max_m, robot_clearance_m, distance_threshold_m,
      default_max_plane_tilt_deg_, min_abs_normal_z, remove_walls_ ? "true" : "false",
      wall_distance_threshold_m, wall_min_inliers_, wall_max_abs_normal_z, min_inliers,
      max_iterations, map_z_offset_m, output_frame.c_str());

    Point3 robot_position;
    try {
      publish_feedback(goal_handle, "looking_up_robot_tf", 10);
      robot_position = transform_origin(
        tf_buffer_->lookupTransform(output_frame, robot_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException & ex) {
      result->success = false;
      log_warn("find_bar_plane failed: Failed to lookup robot transform: %s", ex.what());
      goal_handle->succeed(result);
      return;
    }

    log_info(
      "Robot pose for extraction: frame=%s robot_frame=%s xy=(%.3f, %.3f)",
      output_frame.c_str(), robot_frame_.c_str(), robot_position.x, robot_position.y);

    std::vector<Point3> points;
    int used_frames = 0;
    publish_feedback(goal_handle, "accumulating_clouds", 20);
    for (const auto & cloud : clouds) {
      try {
        const auto cloud_to_output_transform = tf_buffer_->lookupTransform(
          output_frame, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
        const auto cloud_to_robot_transform = tf_buffer_->lookupTransform(
          robot_frame_, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
        const auto frame_points = extract_points(
          *cloud, cloud_to_output_transform, cloud_to_robot_transform, roi_x_min_m, roi_x_max_m,
          roi_y_min_m, roi_y_max_m, roi_z_min_m, roi_z_max_m, robot_clearance_m);
        log_info(
          "Accumulated cloud frame: source_frame=%s raw_points=%u kept_points=%zu",
          cloud->header.frame_id.c_str(), cloud->width * cloud->height, frame_points.size());
        points.insert(points.end(), frame_points.begin(), frame_points.end());
        ++used_frames;
      } catch (const tf2::TransformException & ex) {
        log_warn_throttle(
          2000,
          "Skipping cloud frame because TF lookup failed: %s", ex.what());
      }
    }

    std::mt19937 rng{std::random_device{}()};
    if (remove_walls_ && points.size() >= 3) {
      publish_feedback(goal_handle, "removing_walls", 30);
      const size_t before_wall_removal = points.size();
      const int removed_wall_points = remove_vertical_planes(
        points, wall_max_planes_, max_iterations, wall_distance_threshold_m, wall_min_inliers_,
        wall_max_abs_normal_z, rng);
      log_info(
        "Wall removal complete: before=%zu removed=%d remaining=%zu max_planes=%d threshold=%.3f "
        "min_inliers=%d max_abs_normal_z=%.3f",
        before_wall_removal, removed_wall_points, points.size(), wall_max_planes_,
        wall_distance_threshold_m, wall_min_inliers_, wall_max_abs_normal_z);
    }

    if (points.size() < 3) {
      result->success = false;
      log_warn(
        "find_bar_plane failed: Not enough transformed valid points within ROI. "
        "roi_x=[%.2f, %.2f] roi_y=[%.2f, %.2f] roi_z=[%.2f, %.2f] "
        "robot_clearance=%.2f used_frames=%d points=%zu",
        roi_x_min_m, roi_x_max_m, roi_y_min_m, roi_y_max_m, roi_z_min_m, roi_z_max_m,
        robot_clearance_m, used_frames, points.size());
      goal_handle->succeed(result);
      return;
    }

    std::vector<Point3> remaining_plane_points = points;
    Plane best_plane;
    int best_inliers = 0;
    int max_inliers = 0;
    int candidate_planes = 0;
    float best_plane_height = -std::numeric_limits<float>::max();
    float best_edge_distance = std::numeric_limits<float>::max();
    int search_round = 0;
    publish_feedback(goal_handle, "running_ransac", 40);

    while (remaining_plane_points.size() >= 3) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        log_info("find_bar_plane canceled after %.2f ms", elapsed_ms(start_time));
        goal_handle->canceled(result);
        return;
      }

      std::uniform_int_distribution<size_t> sample_index(0, remaining_plane_points.size() - 1);
      Plane extracted_plane;
      std::vector<size_t> extracted_inliers;

      for (int iteration = 1; iteration <= max_iterations; ++iteration) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          log_info("find_bar_plane canceled after %.2f ms", elapsed_ms(start_time));
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
        if (!plane_from_points(
            remaining_plane_points[i1], remaining_plane_points[i2], remaining_plane_points[i3],
            candidate))
        {
          continue;
        }
        if (std::fabs(candidate.c) < min_abs_normal_z) {
          continue;
        }

        std::vector<size_t> inliers;
        inliers.reserve(remaining_plane_points.size());
        for (size_t i = 0; i < remaining_plane_points.size(); ++i) {
          if (point_plane_distance(remaining_plane_points[i], candidate) <= distance_threshold_m) {
            inliers.push_back(i);
          }
        }

        if (inliers.size() > extracted_inliers.size()) {
          extracted_inliers = std::move(inliers);
          extracted_plane = candidate;
        }

        if (iteration == 1 || iteration % 20 == 0 || iteration == max_iterations) {
          const int progress =
            40 + std::min(
              50, search_round * 10 +
              static_cast<int>((10.0 * static_cast<double>(iteration)) / max_iterations));
          publish_feedback(goal_handle, "running_ransac", std::min(progress, 90));
        }
      }

      const int extracted_inlier_count = static_cast<int>(extracted_inliers.size());
      max_inliers = std::max(max_inliers, extracted_inlier_count);
      if (extracted_inlier_count < min_inliers) {
        break;
      }

      ++candidate_planes;
      const Point3 candidate_centroid =
        centroid_from_indices(remaining_plane_points, extracted_inliers);
      Point3 candidate_edge_point;
      float candidate_edge_distance = std::numeric_limits<float>::max();
      select_robot_side_edge_point(
        remaining_plane_points, extracted_inliers, candidate_centroid, robot_position,
        candidate_edge_point, candidate_edge_distance);
      if (candidate_centroid.z > best_plane_height + 1.0e-6F ||
        (std::fabs(candidate_centroid.z - best_plane_height) <= 1.0e-6F &&
        (candidate_edge_distance + 1.0e-6F < best_edge_distance ||
        (std::fabs(candidate_edge_distance - best_edge_distance) <= 1.0e-6F &&
        extracted_inlier_count > best_inliers))))
      {
        best_plane_height = candidate_centroid.z;
        best_edge_distance = candidate_edge_distance;
        best_inliers = extracted_inlier_count;
        best_plane = extracted_plane;
      }

      std::vector<bool> remove_mask(remaining_plane_points.size(), false);
      for (const size_t index : extracted_inliers) {
        remove_mask[index] = true;
      }

      std::vector<Point3> kept_points;
      kept_points.reserve(remaining_plane_points.size() - extracted_inliers.size());
      for (size_t i = 0; i < remaining_plane_points.size(); ++i) {
        if (!remove_mask[i]) {
          kept_points.push_back(remaining_plane_points[i]);
        }
      }

      remaining_plane_points.swap(kept_points);
      ++search_round;
    }

    result->success = best_inliers >= min_inliers;
    if (result->success) {
      publish_feedback(goal_handle, "computing_target", 90);
      std::vector<size_t> inlier_indices;
      inlier_indices.reserve(static_cast<size_t>(best_inliers));
      for (size_t i = 0; i < points.size(); ++i) {
        if (point_plane_distance(points[i], best_plane) > distance_threshold_m) {
          continue;
        }

        inlier_indices.push_back(i);
      }

      if (inlier_indices.empty()) {
        result->success = false;
        log_warn(
          "find_bar_plane failed: Plane found, but could not determine a usable edge point.");
        goal_handle->succeed(result);
        return;
      }

      debug_plane_pub_->publish(
        make_point_cloud_msg(points, inlier_indices, output_frame, now()));
      log_info(
        "Published debug plane cloud: topic=%s points=%zu frame=%s",
        debug_plane_topic_.c_str(), inlier_indices.size(), output_frame.c_str());

      const Point3 plane_median = median_from_indices(points, inlier_indices);
      result->target_x_m = plane_median.x;
      result->target_y_m = plane_median.y;
      result->target_z_m = plane_median.z - map_z_offset_m;
      set_target_z(result->target_z_m, target_z_goal_sequence);
      log_info(
        "Plane median selected: frame=%s median=(%.3f, %.3f, %.3f) "
        "target=(%.3f, %.3f, %.3f) map_z_offset=%.3f plane=[%.3f %.3f %.3f %.3f]",
        output_frame.c_str(),
        plane_median.x, plane_median.y, plane_median.z,
        result->target_x_m, result->target_y_m, result->target_z_m,
        map_z_offset_m,
        best_plane.a, best_plane.b, best_plane.c, best_plane.d);
      publish_feedback(goal_handle, "succeeded", 100);
    } else {
      log_warn(
        "find_bar_plane failed: No horizontal plane candidate selected. "
        "max_inliers=%d min_inliers=%d candidates=%d min_abs_normal_z=%.3f",
        max_inliers, min_inliers, candidate_planes, min_abs_normal_z);
    }

    log_info(
      "RANSAC result: success=%s frames=%d points=%zu selected_inliers=%d max_inliers=%d "
      "candidates=%d selected_height=%.3f selected_edge_distance=%.3f threshold=%.3f "
      "roi_x=[%.2f, %.2f] roi_y=[%.2f, %.2f] roi_z=[%.2f, %.2f] "
      "target=(%.3f, %.3f, %.3f) elapsed=%.2f ms",
      result->success ? "true" : "false", used_frames, points.size(), best_inliers,
      max_inliers, candidate_planes,
      best_plane_height == -std::numeric_limits<float>::max() ? -1.0F : best_plane_height,
      best_edge_distance == std::numeric_limits<float>::max() ? -1.0F : best_edge_distance,
      distance_threshold_m, roi_x_min_m, roi_x_max_m, roi_y_min_m, roi_y_max_m,
      roi_z_min_m, roi_z_max_m,
      result->target_x_m, result->target_y_m, result->target_z_m,
      elapsed_ms(start_time));

    goal_handle->succeed(result);
  }

  void initialize_file_logger()
  {
    if (!ensure_directory(log_directory_)) {
      RCLCPP_WARN(
        get_logger(), "Failed to create log directory: %s error=%s",
        log_directory_.c_str(), std::strerror(errno));
      return;
    }

    log_file_path_ =
      log_directory_ + "/find_start_bar_" + timestamp_string("%Y%m%d_%H%M%S") + ".log";
    log_file_.open(log_file_path_, std::ios::out | std::ios::app);
    if (!log_file_.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
      log_file_path_.clear();
      return;
    }

    append_log("INFO", "File logging started: path=" + log_file_path_);
  }

  void append_log(const std::string & level, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!log_file_.is_open()) {
      return;
    }

    log_file_ << timestamp_string("%Y-%m-%d %H:%M:%S") << " [" << level << "] "
              << message << '\n';
    log_file_.flush();
  }

  template<typename... Args>
  void log_info(const char * format, Args... args)
  {
    const std::string message = format_message(format, args...);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    append_log("INFO", message);
  }

  template<typename... Args>
  void log_warn(const char * format, Args... args)
  {
    const std::string message = format_message(format, args...);
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    append_log("WARN", message);
  }

  template<typename... Args>
  void log_warn_throttle(const int duration_ms, const char * format, Args... args)
  {
    const std::string message = format_message(format, args...);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), duration_ms, "%s", message.c_str());

    const auto now_time = Clock::now();
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      if (last_throttled_file_log_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
          now_time - last_throttled_file_log_time_).count() < duration_ms)
      {
        return;
      }
      last_throttled_file_log_time_ = now_time;
    }

    append_log("WARN", message);
  }

  size_t recent_cloud_count()
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return recent_clouds_.size();
  }

  size_t pause_target_z_for_new_goal()
  {
    std::lock_guard<std::mutex> lock(target_z_mutex_);
    has_target_z_ = false;
    return ++target_z_goal_sequence_;
  }

  void set_target_z(const float target_z_m, const size_t goal_sequence)
  {
    {
      std::lock_guard<std::mutex> lock(target_z_mutex_);
      if (goal_sequence != target_z_goal_sequence_) {
        return;
      }
      latest_target_z_m_ = target_z_m;
      has_target_z_ = true;
    }

    log_info(
      "Target z topic publishing resumed: topic=%s z=%.3f period=1.0s",
      target_z_topic_.c_str(), target_z_m);
  }

  void publish_target_z()
  {
    std_msgs::msg::Float32 msg;
    {
      std::lock_guard<std::mutex> lock(target_z_mutex_);
      if (!has_target_z_) {
        return;
      }
      msg.data = latest_target_z_m_;
    }

    target_z_pub_->publish(msg);
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
  double default_roi_x_min_m_{};
  double default_roi_x_max_m_{};
  double default_roi_y_min_m_{};
  double default_roi_y_max_m_{};
  double default_roi_z_min_m_{};
  double default_roi_z_max_m_{};
  double default_robot_clearance_m_{};
  double default_distance_threshold_m_{};
  double default_max_plane_tilt_deg_{};
  bool remove_walls_{};
  int wall_max_planes_{};
  double wall_distance_threshold_m_{};
  int wall_min_inliers_{};
  double wall_max_abs_normal_z_{};
  double map_z_offset_m_{};
  int default_min_inliers_{};
  int default_max_iterations_{};
  int default_accumulation_frames_{};
  int max_stored_frames_{};
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  std::string debug_plane_topic_;
  std::string target_z_topic_;
  std::string log_directory_;
  std::string log_file_path_;

  std::mutex cloud_mutex_;
  std::mutex log_mutex_;
  std::mutex target_z_mutex_;
  bool has_target_z_{false};
  float latest_target_z_m_{};
  size_t target_z_goal_sequence_{0};
  Clock::time_point last_throttled_file_log_time_{};
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> recent_clouds_;
  std::ofstream log_file_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_plane_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_z_pub_;
  rclcpp::TimerBase::SharedPtr target_z_timer_;
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
