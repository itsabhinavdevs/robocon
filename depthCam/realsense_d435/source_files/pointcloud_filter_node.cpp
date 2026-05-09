// create ~/robocon_ws/src/robocon_odometry/src/pointcloud_filter_node.cpp

// filters raw D435 point cloud for clean odometry input
// layers: range clip → voxel grid → SOR → RANSAC ground removal → normal estimation


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <chrono>

using PointT    = pcl::PointXYZ;
using CloudT    = pcl::PointCloud<PointT>;
using CloudNT   = pcl::PointCloud<pcl::PointNormal>;

class PointCloudFilterNode : public rclcpp::Node
{
public:
  PointCloudFilterNode() : Node("pointcloud_filter_node")
  {
    //declare parameters
    this->declare_parameter<double>("voxel_size",              0.05);
    this->declare_parameter<double>("min_depth",               0.3);
    this->declare_parameter<double>("max_depth",               4.0);
    this->declare_parameter<double>("min_height",              -0.5);
    this->declare_parameter<double>("max_height",              2.0);
    this->declare_parameter<int>   ("sor_k_neighbors",         20);
    this->declare_parameter<double>("sor_std_ratio",           2.0);
    this->declare_parameter<bool>  ("enable_sor",              true);
    this->declare_parameter<bool>  ("enable_radius_filter",    false);
    this->declare_parameter<int>   ("radius_min_neighbors",    10);
    this->declare_parameter<double>("radius_search",           0.05);
    this->declare_parameter<bool>  ("remove_ground",           true);
    this->declare_parameter<double>("ground_dist_thresh",      0.02);
    this->declare_parameter<int>   ("ground_ransac_iters",     500);
    this->declare_parameter<bool>  ("estimate_normals",        true);
    this->declare_parameter<double>("normal_radius",           0.10);

    //get parameters
    voxel_size_      = this->get_parameter("voxel_size").as_double();
    min_depth_       = this->get_parameter("min_depth").as_double();
    max_depth_       = this->get_parameter("max_depth").as_double();
    min_height_      = this->get_parameter("min_height").as_double();
    max_height_      = this->get_parameter("max_height").as_double();
    sor_k_           = this->get_parameter("sor_k_neighbors").as_int();
    sor_std_         = this->get_parameter("sor_std_ratio").as_double();
    do_sor_          = this->get_parameter("enable_sor").as_bool();
    do_radius_       = this->get_parameter("enable_radius_filter").as_bool();
    rad_min_n_       = this->get_parameter("radius_min_neighbors").as_int();
    rad_r_           = this->get_parameter("radius_search").as_double();
    remove_ground_   = this->get_parameter("remove_ground").as_bool();
    gnd_thresh_      = this->get_parameter("ground_dist_thresh").as_double();
    gnd_iters_       = this->get_parameter("ground_ransac_iters").as_int();
    est_normals_     = this->get_parameter("estimate_normals").as_bool();
    normal_radius_   = this->get_parameter("normal_radius").as_double();

    //subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points",
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1));

    //publisher
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud/filtered", rclcpp::SensorDataQoS());

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud/ground", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "PointCloudFilterNode started | voxel=%.2fm | SOR=%s | ground=%s | normals=%s",
      voxel_size_,
      do_sor_ ? "ON" : "OFF",
      remove_ground_ ? "ON" : "OFF",
      est_normals_ ? "ON" : "OFF");
  }

private:
  //parameters
  double voxel_size_, min_depth_, max_depth_, min_height_, max_height_;
  int    sor_k_;
  double sor_std_;
  bool   do_sor_, do_radius_;
  int    rad_min_n_;
  double rad_r_;
  bool   remove_ground_;
  double gnd_thresh_;
  int    gnd_iters_;
  bool   est_normals_;
  double normal_radius_;

  //ros handles
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    ground_pub_;


  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto t0 = std::chrono::steady_clock::now();

    //convert ROS2 pointcloud2 → PCL cloud
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    //step 1a: passthrough filter — depth range (Z axis in camera frame)
    //camera frame: Z = forward depth, Y = down, X = right
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(static_cast<float>(min_depth_),
                           static_cast<float>(max_depth_));
      pass.filter(*cloud);
    }

    //step 1b: passthrough filter — height (Y axis)
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(static_cast<float>(min_height_),
                           static_cast<float>(max_height_));
      pass.filter(*cloud);
    }

    if (cloud->size() < 50) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Too few points after range clip: %zu", cloud->size());
      return;
    }

    size_t pts_before_filter = cloud->size();

    //step 2: voxel grid downsampling
    //divide 3D space into voxels of voxel_size^3 cubes.
    //replace all points in each voxel with their centroid.
    //300K points → ~3K–15K points depending on scene.
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(voxel_size_),
                     static_cast<float>(voxel_size_),
                     static_cast<float>(voxel_size_));
      vg.filter(*cloud);
    }

    //step 3: statistical outlier removal
    //for each point: compute mean distance to K nearest neighbors.
    //points where mean_dist > global_mean + std_ratio * global_std are outliers.
    //removes isolated speckle noise points.
    if (do_sor_ && cloud->size() > 20) {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(sor_k_);
      sor.setStddevMulThresh(sor_std_);
      sor.filter(*cloud);
    }

    //step 4: radius outlier removal (optional, slower)
    //each point must have at least rad_min_n_ neighbors within radius rad_r_.
    if (do_radius_ && cloud->size() > 20) {
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(cloud);
      ror.setRadiusSearch(rad_r_);
      ror.setMinNeighborsInRadius(rad_min_n_);
      ror.filter(*cloud);
    }

    //step 5: ground plane segmentation (RANSAC)
    //RANSAC: randomly sample 3 points, fit a plane ax+by+cz+d=0,
    //count inliers (within gnd_thresh_ distance). Repeat gnd_iters_ times.
    //best plane = ground. Remove it — ground has no useful features for VO.
    CloudT::Ptr ground_cloud(new CloudT);
    if (remove_ground_ && cloud->size() > 50) {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(gnd_thresh_);
      seg.setMaxIterations(gnd_iters_);
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      if (!inliers->indices.empty()) {
        // sanity check: ground plane normal should be mostly vertical
        // in camera frame, Y is down, so ground normal b (coefficient[1]) ≈ ±1.
        float b = coefficients->values[1];
        if (std::abs(b) > 0.5f) {
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud(cloud);
          extract.setIndices(inliers);

          // extract ground separately (for debug visualization)
          extract.setNegative(false);
          extract.filter(*ground_cloud);

          // remove ground from main cloud
          extract.setNegative(true);
          extract.filter(*cloud);
        }
      }
    }

    //step 6: normal estimation
    //normals are required for point-to-plane ICP (faster convergence).
    //RTAB-map uses these when running ICP refinement.
    if (est_normals_ && !cloud->empty()) {
      pcl::NormalEstimation<PointT, pcl::Normal> ne;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

      tree->setInputCloud(cloud);
      ne.setInputCloud(cloud);
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(normal_radius_);
      ne.compute(*normals);
      // note: normals are computed but not concatenated here since
      // filtered_pub_ publishes XYZ only. For full PointNormal cloud, use pcl::concatenateFields — add if needed.
    }

    //publish filtered cloud
    publishCloud(cloud, msg->header, filtered_pub_);

    //publish ground cloud (for RViz debug)
    if (!ground_cloud->empty()) {
      publishCloud(ground_cloud, msg->header, ground_pub_);
    }

    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "PC filter: %zu → %zu pts (ground: %zu) | %.0f ms",
      pts_before_filter, cloud->size(), ground_cloud->size(), ms);
  }

  void publishCloud(
    const CloudT::Ptr & cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud, out_msg);
    out_msg.header = header;
    pub->publish(out_msg);
  }
};

//main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
