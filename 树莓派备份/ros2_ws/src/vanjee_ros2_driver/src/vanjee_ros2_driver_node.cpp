#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#include <vanjee_driver/utility/sync_queue.hpp>
#include <thread>
#include <cmath>
#include <memory>
#include <string>

using namespace vanjee::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class VanJeeRos2DriverNode : public rclcpp::Node {
public:
  VanJeeRos2DriverNode() : Node("vanjee_ros2_driver_node"), to_exit_process_(false) {
    // Declare parameters
    this->declare_parameter<std::string>("lidar_type", "wlr719c");
    this->declare_parameter<std::string>("host_address", "192.168.0.93");
    this->declare_parameter<std::string>("lidar_address", "192.168.0.2");
    this->declare_parameter<int>("host_port", 58587);
    this->declare_parameter<int>("lidar_port", 6050);
    this->declare_parameter<std::string>("frame_id", "laser");
    this->declare_parameter<std::string>("topic", "/vanjee_pointcloud");

    // Get parameters
    std::string lidar_type_str = this->get_parameter("lidar_type").as_string();
    std::string host_address = this->get_parameter("host_address").as_string();
    std::string lidar_address = this->get_parameter("lidar_address").as_string();
    int host_port = this->get_parameter("host_port").as_int();
    int lidar_port = this->get_parameter("lidar_port").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    std::string topic_name = this->get_parameter("topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Initializing VanJee LiDAR ROS 2 DriverNode...");
    RCLCPP_INFO(this->get_logger(), "Lidar Type: %s", lidar_type_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Host Address: %s:%d", host_address.c_str(), host_port);
    RCLCPP_INFO(this->get_logger(), "Lidar Address: %s:%d", lidar_address.c_str(), lidar_port);

    // Create Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);

    // Initialize Driver Parameters
    WJDriverParam param;
    param.lidar_type = getLidarType(lidar_type_str);
    param.input_type = InputType::ONLINE_LIDAR;
    param.input_param.host_msop_port = host_port;
    param.input_param.lidar_msop_port = lidar_port;
    param.input_param.host_address = host_address;
    param.input_param.lidar_address = lidar_address;
    param.input_param.group_address = "0.0.0.0";
    param.decoder_param.config_from_file = false;
    param.decoder_param.wait_for_difop = false;
    param.decoder_param.point_cloud_enable = true;
    param.decoder_param.laser_scan_enable = false;
    param.decoder_param.imu_enable = -1; // disable IMU for now

    // Pre-allocate some point clouds for the free queue
    for (int i = 0; i < 4; ++i) {
      free_cloud_queue_.push(std::make_shared<PointCloudMsg>());
    }

    // Register Callbacks
    driver_.regPointCloudCallback(
      [this]() {
        std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
        if (msg != nullptr) {
          return msg;
        }
        return std::make_shared<PointCloudMsg>();
      },
      [this](std::shared_ptr<PointCloudMsg> msg) {
        stuffed_cloud_queue_.push(msg);
      }
    );

    driver_.regImuPacketCallback(
      []() { return std::make_shared<ImuPacket>(); },
      [](std::shared_ptr<ImuPacket>) {}
    );
    driver_.regScanDataCallback(
      []() { return std::make_shared<ScanData>(); },
      [](std::shared_ptr<ScanData>) {}
    );
    driver_.regDeviceCtrlCallback(
      []() { return std::make_shared<DeviceCtrl>(); },
      [](std::shared_ptr<DeviceCtrl>) {}
    );
    driver_.regLidarParameterInterfaceCallback(
      []() { return std::make_shared<LidarParameterInterface>(); },
      [](std::shared_ptr<LidarParameterInterface>) {}
    );

    driver_.regExceptionCallback([this](const Error& code) {
      RCLCPP_WARN(this->get_logger(), "LiDAR Driver Exception: %s", code.toString().c_str());
    });

    // Initialize Driver
    if (!driver_.init(param)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize VanJee LiDAR Driver!");
      throw std::runtime_error("Driver init failed");
    }

    // Start Driver threads
    if (!driver_.start()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start VanJee LiDAR Driver!");
      throw std::runtime_error("Driver start failed");
    }

    // Start background point cloud processing & publishing thread
    process_thread_ = std::thread(&VanJeeRos2DriverNode::processPointCloud, this);
  }

  ~VanJeeRos2DriverNode() {
    to_exit_process_ = true;
    stuffed_cloud_queue_.push(nullptr); // Wake up wait loop
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
    driver_.stop();
    RCLCPP_INFO(this->get_logger(), "VanJee LiDAR driver stopped.");
  }

private:
  LidarType getLidarType(const std::string& type_str) {
    if (type_str == "wlr716mini") return LidarType::vanjee_716mini;
    if (type_str == "wlr718h") return LidarType::vanjee_718h;
    if (type_str == "wlr719") return LidarType::vanjee_719;
    if (type_str == "wlr719c") return LidarType::vanjee_719c;
    if (type_str == "wlr719e") return LidarType::vanjee_719e;
    if (type_str == "wlr720" || type_str == "wlr720_16") return LidarType::vanjee_720_16;
    if (type_str == "wlr720_32") return LidarType::vanjee_720_32;
    if (type_str == "wlr721") return LidarType::vanjee_721;
    if (type_str == "wlr722") return LidarType::vanjee_722;
    if (type_str == "wlr722f") return LidarType::vanjee_722f;
    if (type_str == "wlr722h") return LidarType::vanjee_722h;
    if (type_str == "wlr722z") return LidarType::vanjee_722z;
    if (type_str == "wlr733") return LidarType::vanjee_733;
    if (type_str == "wlr750") return LidarType::vanjee_750;
    if (type_str == "wlr760") return LidarType::vanjee_760;

    RCLCPP_WARN(this->get_logger(), "Unknown LiDAR type: %s, defaulting to wlr719c", type_str.c_str());
    return LidarType::vanjee_719c;
  }

  void processPointCloud() {
    while (!to_exit_process_) {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
      if (msg == nullptr || to_exit_process_) {
        continue;
      }

      publishPointCloud(msg);
      free_cloud_queue_.push(msg);
    }
  }

  void publishPointCloud(const std::shared_ptr<PointCloudMsg>& msg) {
    if (msg->points.empty()) {
      return;
    }

    auto ros_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    ros_msg->header.frame_id = frame_id_;
    
    // Convert double seconds to builtin_interfaces::msg::Time
    double sec;
    double nsec = std::modf(msg->timestamp, &sec);
    ros_msg->header.stamp.sec = static_cast<int32_t>(sec);
    ros_msg->header.stamp.nanosec = static_cast<uint32_t>(nsec * 1e9);

    ros_msg->height = 1;
    ros_msg->width = msg->points.size();
    ros_msg->is_dense = msg->is_dense;
    ros_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*ros_msg);
    modifier.setPointCloud2Fields(6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64
    );

    sensor_msgs::PointCloud2Iterator<float> iter_x(*ros_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*ros_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*ros_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*ros_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(*ros_msg, "ring");
    sensor_msgs::PointCloud2Iterator<double> iter_timestamp(*ros_msg, "timestamp");

    for (const auto& pt : msg->points) {
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      *iter_intensity = pt.intensity;
      *iter_ring = pt.ring;
      *iter_timestamp = pt.timestamp;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_ring;
      ++iter_timestamp;
    }

    pub_->publish(std::move(ros_msg));
  }

  LidarDriver<PointCloudMsg> driver_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string frame_id_;

  SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;

  std::thread process_thread_;
  std::atomic<bool> to_exit_process_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VanJeeRos2DriverNode>());
  rclcpp::shutdown();
  return 0;
}
