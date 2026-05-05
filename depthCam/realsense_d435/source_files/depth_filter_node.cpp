// create ~/robocon_ws/src/robocon_odometry/src/depth_filter_node.cpp



// filters raw D435 depth image before point cloud conversion
// layers: range clip → edge removal → temporal median → bilateral filter → inpainting


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>
#include <cmath>

class DepthFilterNode : public rclcpp::Node
{
public:
  DepthFilterNode() : Node("depth_filter_node")
  {
    // declare parameters
    this->declare_parameter<int>("min_depth_mm",          300);
    this->declare_parameter<int>("max_depth_mm",          4000);
    this->declare_parameter<int>("bilateral_d",           5);
    this->declare_parameter<double>("bilateral_sigma",    50.0);
    this->declare_parameter<int>("edge_kernel_size",      3);
    this->declare_parameter<int>("temporal_frames",       5);
    this->declare_parameter<bool>("enable_temporal",      true);
    this->declare_parameter<bool>("enable_bilateral",     true);
    this->declare_parameter<bool>("enable_edge_removal",  true);
    this->declare_parameter<bool>("enable_inpaint",       true);

    // get parameters
    min_depth_    = this->get_parameter("min_depth_mm").as_int();
    max_depth_    = this->get_parameter("max_depth_mm").as_int();
    bil_d_        = this->get_parameter("bilateral_d").as_int();
    bil_sigma_    = this->get_parameter("bilateral_sigma").as_double();
    edge_k_       = this->get_parameter("edge_kernel_size").as_int();
    temporal_n_   = this->get_parameter("temporal_frames").as_int();
    do_temporal_  = this->get_parameter("enable_temporal").as_bool();
    do_bilateral_ = this->get_parameter("enable_bilateral").as_bool();
    do_edge_      = this->get_parameter("enable_edge_removal").as_bool();
    do_inpaint_   = this->get_parameter("enable_inpaint").as_bool();

    // subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&DepthFilterNode::depthCallback, this, std::placeholders::_1));

    // publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/depth/filtered", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "DepthFilterNode started | range=[%d,%d]mm | temporal=%s | bilateral=%s",
      min_depth_, max_depth_,
      do_temporal_ ? "ON" : "OFF",
      do_bilateral_ ? "ON" : "OFF");
  }

private:
  // parameters
  int    min_depth_, max_depth_;
  int    bil_d_;
  double bil_sigma_;
  int    edge_k_;
  int    temporal_n_;
  bool   do_temporal_, do_bilateral_, do_edge_, do_inpaint_;

  // temporal buffer
  std::deque<cv::Mat> depth_buffer_;

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;


  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // convert ROS image → cv::Mat (16UC1, values in mm)
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat depth = cv_ptr->image.clone();  // uint16, values in mm

    // step 1: Range Clipping
    // zero out pixels outside [min_depth, max_depth] mm
    cv::Mat range_mask = (depth >= static_cast<uint16_t>(min_depth_)) &
                         (depth <= static_cast<uint16_t>(max_depth_));
    depth.setTo(0, ~range_mask);

    // step 2: Edge Noise Removal
    // flying pixels cluster near depth discontinuities
    // we detect where depth jumps sharply and zero a border around those edges
    if (do_edge_) {
      removeEdgeNoise(depth);
    }

    // step 3: Temporal Median Filter
    // keep a rolling buffer of the last N frames
    // for each pixel, take the median of valid (non-zero) values
    // this eliminates single-frame spikes / flicker without blurring edges
    if (do_temporal_) {
      depth = temporalMedian(depth);
    }

    // step 4: Bilateral Filter
    // edge-preserving spatial smoothing
    // smooths flat surfaces while keeping sharp depth boundaries
    if (do_bilateral_) {
      applyBilateral(depth);
    }

    // step 5: Inpainting
    // fill remaining zero (invalid) pixels using nearest-valid-neighbor propagation
    if (do_inpaint_) {
      inpaintHoles(depth);
    }

    // publish
    cv_bridge::CvImage out_img;
    out_img.header   = msg->header;
    out_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    out_img.image    = depth;
    pub_->publish(*out_img.toImageMsg());
  }

  // removeEdgeNoise
  void removeEdgeNoise(cv::Mat & depth)
  {
    // convert to 8-bit for Canny (Canny requires 8-bit input)
    cv::Mat depth_8u;
    depth.convertTo(depth_8u, CV_8U, 255.0 / max_depth_);

    // detect strong edges (depth discontinuities)
    cv::Mat edges;
    cv::Canny(depth_8u, edges, 10, 30);

    // dilate edges to cover flying pixels nearby
    cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(edge_k_, edge_k_));
    cv::Mat dilated_edges;
    cv::dilate(edges, dilated_edges, kernel, cv::Point(-1,-1), 1);

    // zero out pixels near depth edges
    depth.setTo(0, dilated_edges > 0);
  }


  cv::Mat temporalMedian(const cv::Mat & current)
  {
    // add new frame to buffer
    depth_buffer_.push_back(current.clone());
    if (static_cast<int>(depth_buffer_.size()) > temporal_n_) {
      depth_buffer_.pop_front();
    }

    if (static_cast<int>(depth_buffer_.size()) < 2) {
      return current;
    }

    int H = current.rows, W = current.cols;
    int N = static_cast<int>(depth_buffer_.size());

    cv::Mat result = cv::Mat::zeros(H, W, CV_16UC1);

    // per-pixel median over buffer (ignoring zeros)
    for (int r = 0; r < H; ++r) {
      for (int c = 0; c < W; ++c) {
        std::vector<uint16_t> vals;
        vals.reserve(N);
        for (const auto & frame : depth_buffer_) {
          uint16_t v = frame.at<uint16_t>(r, c);
          if (v > 0) vals.push_back(v);
        }
        if (!vals.empty()) {
          std::nth_element(vals.begin(),
                           vals.begin() + vals.size() / 2,
                           vals.end());
          result.at<uint16_t>(r, c) = vals[vals.size() / 2];
        }
      }
    }
    return result;
  }

  // apply bilateral filter to depth image
  void applyBilateral(cv::Mat & depth)
  {
    // openCV bilateralFilter requires 8-bit or 32-bit float.
    // convert uint16 → float32, filter, convert back.
    cv::Mat depth_f;
    depth.convertTo(depth_f, CV_32F);

    // mask of valid pixels before filtering
    cv::Mat valid_mask = (depth > 0);

    cv::Mat filtered;
    cv::bilateralFilter(depth_f, filtered, bil_d_,
                        bil_sigma_,   // sigmaColor: depth variation allowed
                        bil_sigma_);  // sigmaSpace: spatial kernel size

    // restore zeros where original depth was invalid
    filtered.setTo(0.0f, ~valid_mask);

    filtered.convertTo(depth, CV_16U);
  }

  // inpaint holes in depth image
  void inpaintHoles(cv::Mat & depth)
  {
    // build mask of invalid (zero) pixels
    cv::Mat invalid_mask;
    cv::compare(depth, 0, invalid_mask, cv::CMP_EQ);

    int hole_count = cv::countNonZero(invalid_mask);
    int total      = depth.rows * depth.cols;

    // only inpaint if holes are < 40% of image (more = camera problem)
    if (hole_count == 0 || hole_count > total * 0.40) {
      return;
    }

    // normalize uint16 → uint8 for inpainting
    double min_val, max_val;
    cv::minMaxLoc(depth, &min_val, &max_val, nullptr, nullptr,
                  depth > 0);  // ignore zeros in min/max

    if (max_val <= 0.0) return;

    cv::Mat depth_8u;
    depth.convertTo(depth_8u, CV_8U, 255.0 / max_val);

    // TELEA inpainting — fast, good for small holes (radius 3px)
    cv::Mat inpainted;
    cv::inpaint(depth_8u, invalid_mask, inpainted, 3, cv::INPAINT_TELEA);

    // scale back to uint16
    cv::Mat inpainted_16u;
    inpainted.convertTo(inpainted_16u, CV_16U, max_val / 255.0);

    // only use inpainted values where original was zero
    inpainted_16u.copyTo(depth, invalid_mask);
  }
};

// main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthFilterNode>());
  rclcpp::shutdown();
  return 0;
}
