#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "pcd_io_minimal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PcdToOctomapNode : public rclcpp::Node
{
public:
  PcdToOctomapNode()
  : Node("pcd_to_octomap")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("pcd_file_cmd_topic", "/pcd_file_cmd");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("voxel_downsample_m", 0.0);
    declare_parameter<int>("min_points_per_voxel", 3);
    declare_parameter<int>("min_cluster_voxels", 4);
    declare_parameter<double>("min_z", -1.0e9);
    declare_parameter<double>("max_z", 1.0e9);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    pcd_file_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("pcd_file_cmd_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&PcdToOctomapNode::onPcdFileCmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PcdToOctomapNode::publishMap, this));

    const auto pcd_file = get_parameter("pcd_file").as_string();
    if (!pcd_file.empty()) {
      loadPcd(pcd_file);
    } else {
      RCLCPP_INFO(get_logger(), "No initial pcd_file set. Waiting for /pcd_file_cmd.");
    }
  }

private:
  struct Key
  {
    unsigned int k[3];

    bool operator==(const Key & other) const
    {
      return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const
    {
      std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
      seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  void onPcdFileCmd(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    loadPcd(msg->data);
  }

  struct VoxelCell
  {
    int64_t a;
    int64_t b;
    int64_t c;

    bool operator==(const VoxelCell & o) const noexcept
    {
      return a == o.a && b == o.b && c == o.c;
    }
  };

  struct VoxelCellHash
  {
    std::size_t operator()(const VoxelCell & c) const noexcept
    {
      std::size_t s = std::hash<int64_t>{}(c.a);
      s ^= std::hash<int64_t>{}(c.b) + 0x9e3779b9 + (s << 6) + (s >> 2);
      s ^= std::hash<int64_t>{}(c.c) + 0x9e3779b9 + (s << 6) + (s >> 2);
      return s;
    }
  };

  static std::vector<std::array<double, 3>> voxelDownsample(
    const std::vector<std::array<double, 3>> & pts, double voxel_m)
  {
    if (voxel_m <= 0.0 || pts.empty()) {
      return pts;
    }
    const double inv = 1.0 / voxel_m;
    std::unordered_map<VoxelCell, std::array<double, 3>, VoxelCellHash> sum;
    std::unordered_map<VoxelCell, std::size_t, VoxelCellHash> cnt;
    for (const auto & p : pts) {
      const VoxelCell key{
        static_cast<int64_t>(std::floor(p[0] * inv)),
        static_cast<int64_t>(std::floor(p[1] * inv)),
        static_cast<int64_t>(std::floor(p[2] * inv)),
      };
      auto it = sum.find(key);
      if (it == sum.end()) {
        sum.emplace(key, p);
        cnt[key] = 1;
      } else {
        it->second[0] += p[0];
        it->second[1] += p[1];
        it->second[2] += p[2];
        ++cnt[key];
      }
    }
    std::vector<std::array<double, 3>> out;
    out.reserve(sum.size());
    for (const auto & kv : sum) {
      const std::size_t n = cnt.at(kv.first);
      out.push_back({{
        kv.second[0] / static_cast<double>(n),
        kv.second[1] / static_cast<double>(n),
        kv.second[2] / static_cast<double>(n),
      }});
    }
    return out;
  }

  void loadPcd(const std::string & pcd_file)
  {
    std::vector<std::array<double, 3>> points;
    std::string err;
    if (!dong_octomap_pcd::load_pcd_xyz(pcd_file, points, err)) {
      RCLCPP_ERROR(get_logger(), "Failed to read PCD file: %s (%s)", pcd_file.c_str(), err.c_str());
      return;
    }

    const double voxel_downsample = get_parameter("voxel_downsample_m").as_double();
    if (voxel_downsample > 0.0) {
      points = voxelDownsample(points, voxel_downsample);
    }

    tree_ = std::make_shared<octomap::OcTree>(get_parameter("resolution").as_double());
    const int min_points_per_voxel =
      std::max(1, static_cast<int>(get_parameter("min_points_per_voxel").as_int()));
    const int min_cluster_voxels =
      std::max(1, static_cast<int>(get_parameter("min_cluster_voxels").as_int()));

    std::unordered_map<Key, std::size_t, KeyHash> voxel_counts;
    voxel_counts.reserve(points.size());
    for (const auto & point : points) {
      octomap::OcTreeKey raw_key;
      if (!tree_->coordToKeyChecked(
          static_cast<float>(point[0]),
          static_cast<float>(point[1]),
          static_cast<float>(point[2]),
          raw_key))
      {
        continue;
      }
      const Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts[key];
    }

    std::unordered_set<Key, KeyHash> occupied_keys;
    occupied_keys.reserve(voxel_counts.size());
    for (const auto & entry : voxel_counts) {
      if (static_cast<int>(entry.second) >= min_points_per_voxel) {
        occupied_keys.insert(entry.first);
      }
    }

    std::size_t removed_cluster_voxels = 0;
    if (min_cluster_voxels > 1 && !occupied_keys.empty()) {
      std::unordered_set<Key, KeyHash> filtered_keys;
      filtered_keys.reserve(occupied_keys.size());
      std::unordered_set<Key, KeyHash> visited;
      visited.reserve(occupied_keys.size());

      for (const auto & seed : occupied_keys) {
        if (visited.find(seed) != visited.end()) {
          continue;
        }

        std::deque<Key> queue;
        std::vector<Key> cluster;
        queue.push_back(seed);
        visited.insert(seed);

        while (!queue.empty()) {
          const Key current = queue.front();
          queue.pop_front();
          cluster.push_back(current);

          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                  continue;
                }

                const auto nx = static_cast<int64_t>(current.k[0]) + dx;
                const auto ny = static_cast<int64_t>(current.k[1]) + dy;
                const auto nz = static_cast<int64_t>(current.k[2]) + dz;
                if (nx < 0 || ny < 0 || nz < 0) {
                  continue;
                }

                const Key neighbor{{
                  static_cast<unsigned int>(nx),
                  static_cast<unsigned int>(ny),
                  static_cast<unsigned int>(nz)}};
                if (occupied_keys.find(neighbor) == occupied_keys.end() ||
                  visited.find(neighbor) != visited.end())
                {
                  continue;
                }

                visited.insert(neighbor);
                queue.push_back(neighbor);
              }
            }
          }
        }

        if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
          filtered_keys.insert(cluster.begin(), cluster.end());
        } else {
          removed_cluster_voxels += cluster.size();
        }
      }

      occupied_keys = std::move(filtered_keys);
    }

    std::size_t inserted_count = 0;
    for (const auto & key : occupied_keys) {
      octomap::OcTreeKey octo_key;
      octo_key.k[0] = key.k[0];
      octo_key.k[1] = key.k[1];
      octo_key.k[2] = key.k[2];
      tree_->updateNode(tree_->keyToCoord(octo_key), true);
      ++inserted_count;
    }

    tree_->updateInnerOccupancy();
    loaded_pcd_file_ = pcd_file;
    publishMap();
    RCLCPP_INFO(
      get_logger(),
      "Loaded PCD file: %s, source_points=%zu, counted_voxels=%zu, kept_voxels=%zu, "
      "removed_small_cluster_voxels=%zu, occupied_voxels=%zu, min_points_per_voxel=%d, "
      "min_cluster_voxels=%d",
      pcd_file.c_str(), points.size(), voxel_counts.size(), inserted_count,
      removed_cluster_voxels, tree_->size(), min_points_per_voxel, min_cluster_voxels);
  }

  void publishMap()
  {
    if (!tree_) {
      return;
    }

    octomap_msgs::msg::Octomap map_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OcTree to octomap message.");
      return;
    }

    map_msg.header.stamp = now();
    map_msg.header.frame_id = get_parameter("frame_id").as_string();
    octomap_pub_->publish(map_msg);
  }

  std::string loaded_pcd_file_;
  std::shared_ptr<octomap::OcTree> tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcd_file_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
