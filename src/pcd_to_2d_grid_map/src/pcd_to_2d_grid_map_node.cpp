#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

class PcdTo2DGridMap
{
public:
  PcdTo2DGridMap(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_saved_(false)
  {
    pnh_.param<std::string>("mode", mode_, std::string("pcd"));  // pcd or topic
    pnh_.param<std::string>("pcd_path", pcd_path_, std::string(""));
    pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, std::string("/Laser_map"));
    pnh_.param<std::string>("output_dir", output_dir_, std::string("/tmp"));
    pnh_.param<std::string>("map_name", map_name_, std::string("map"));

    pnh_.param<std::string>("frame_id", frame_id_, std::string("map"));

    pnh_.param("resolution", resolution_, 0.05);
    pnh_.param("z_min", z_min_, 0.05);
    pnh_.param("z_max", z_max_, 1.50);
    pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.10);
    pnh_.param("padding_radius", padding_radius_, 0.10);
    pnh_.param("min_points_per_cell", min_points_per_cell_, 2);
    pnh_.param("save_pgm_yaml", save_pgm_yaml_, true);
    pnh_.param("process_once", process_once_, true);

    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("generated_map", 1, true);

    if (mode_ == "pcd")
    {
      processPcdFile();
    }
    else if (mode_ == "topic")
    {
      sub_ = nh_.subscribe(pointcloud_topic_, 1, &PcdTo2DGridMap::cloudCallback, this);
      ROS_INFO("Waiting for point cloud topic: %s", pointcloud_topic_.c_str());
    }
    else
    {
      ROS_ERROR("Unknown mode: %s, should be 'pcd' or 'topic'", mode_.c_str());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher grid_pub_;

  std::string mode_;
  std::string pcd_path_;
  std::string pointcloud_topic_;
  std::string output_dir_;
  std::string map_name_;
  std::string frame_id_;

  double resolution_;
  double z_min_;
  double z_max_;
  double voxel_leaf_size_;
  double padding_radius_;
  int min_points_per_cell_;
  bool save_pgm_yaml_;
  bool process_once_;
  bool has_saved_;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (process_once_ && has_saved_) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Received cloud: %zu points", cloud->points.size());
    processCloud(cloud, msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id);

    if (process_once_)
      has_saved_ = true;
  }

  void processPcdFile()
  {
    if (pcd_path_.empty())
    {
      ROS_ERROR("mode=pcd but pcd_path is empty");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *cloud) != 0)
    {
      ROS_ERROR("Failed to load PCD: %s", pcd_path_.c_str());
      return;
    }

    ROS_INFO("Loaded PCD: %s, points=%zu", pcd_path_.c_str(), cloud->points.size());
    processCloud(cloud, frame_id_);
    has_saved_ = true;
  }

  void processCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    const std::string& input_frame)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *cloud = *input_cloud;

    if (voxel_leaf_size_ > 1e-6)
    {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
      vg.filter(*filtered);
      cloud = filtered;
      ROS_INFO("After voxel filter: %zu points", cloud->points.size());
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    std::vector<pcl::PointXYZI> selected_points;
    selected_points.reserve(cloud->points.size());

    for (const auto& pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      if (pt.z < z_min_ || pt.z > z_max_)
        continue;

      selected_points.push_back(pt);

      min_x = std::min(min_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    if (selected_points.empty())
    {
      ROS_ERROR("No valid points after z filtering. Check z_min/z_max.");
      return;
    }

    const int width  = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;

    ROS_INFO("Map bounds: x[%.3f, %.3f], y[%.3f, %.3f], size=%d x %d",
             min_x, max_x, min_y, max_y, width, height);

    std::vector<int> count_map(width * height, 0);
    std::vector<int8_t> occ_map(width * height, 0);  // 0 free, 100 occupied

    auto indexOf = [width](int x, int y) {
      return y * width + x;
    };

    for (const auto& pt : selected_points)
    {
      int gx = static_cast<int>(std::floor((pt.x - min_x) / resolution_));
      int gy = static_cast<int>(std::floor((pt.y - min_y) / resolution_));

      if (gx < 0 || gx >= width || gy < 0 || gy >= height)
        continue;

      count_map[indexOf(gx, gy)]++;
    }

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        if (count_map[indexOf(x, y)] >= min_points_per_cell_)
        {
          occ_map[indexOf(x, y)] = 100;
        }
      }
    }

    int padding_cells = static_cast<int>(std::round(padding_radius_ / resolution_));
    if (padding_cells > 0)
    {
      std::vector<int8_t> padded_map = occ_map;
      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          if (occ_map[indexOf(x, y)] != 100) continue;

          for (int dy = -padding_cells; dy <= padding_cells; ++dy)
          {
            for (int dx = -padding_cells; dx <= padding_cells; ++dx)
            {
              int nx = x + dx;
              int ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

              double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
              if (dist <= padding_radius_)
              {
                padded_map[indexOf(nx, ny)] = 100;
              }
            }
          }
        }
      }
      occ_map.swap(padded_map);
    }

    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = input_frame.empty() ? frame_id_ : input_frame;
    grid.info.resolution = resolution_;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width * height, 0);

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        grid.data[indexOf(x, y)] = occ_map[indexOf(x, y)] == 100 ? 100 : 0;
      }
    }

    grid_pub_.publish(grid);
    ROS_INFO("Published OccupancyGrid.");

    if (save_pgm_yaml_)
    {
      saveMapFiles(grid, occ_map, min_x, min_y, width, height);
    }
  }

  void saveMapFiles(const nav_msgs::OccupancyGrid& grid,
                    const std::vector<int8_t>& occ_map,
                    double origin_x, double origin_y,
                    int width, int height)
  {
    const std::string pgm_path = output_dir_ + "/" + map_name_ + ".pgm";
    const std::string yaml_path = output_dir_ + "/" + map_name_ + ".yaml";

    std::ofstream pgm(pgm_path, std::ios::out | std::ios::binary);
    if (!pgm.is_open())
    {
      ROS_ERROR("Failed to open PGM file for writing: %s", pgm_path.c_str());
      return;
    }

    pgm << "P5\n" << width << " " << height << "\n255\n";

    auto indexOf = [width](int x, int y) {
      return y * width + x;
    };

    for (int y = height - 1; y >= 0; --y)
    {
      for (int x = 0; x < width; ++x)
      {
        unsigned char value = 254;  // free
        if (occ_map[indexOf(x, y)] == 100)
          value = 0;                // occupied
        pgm.write(reinterpret_cast<char*>(&value), 1);
      }
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml.is_open())
    {
      ROS_ERROR("Failed to open YAML file for writing: %s", yaml_path.c_str());
      return;
    }

    yaml << "image: " << map_name_ << ".pgm\n";
    yaml << "resolution: " << resolution_ << "\n";
    yaml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    ROS_INFO("Saved map files:");
    ROS_INFO("  %s", pgm_path.c_str());
    ROS_INFO("  %s", yaml_path.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_to_2d_grid_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PcdTo2DGridMap node(nh, pnh);

  ros::spin();
  return 0;
}