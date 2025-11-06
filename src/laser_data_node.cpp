#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

// Configuration structure for laser scan parameters
struct LaserConfig {
    // Frame IDs
    std::string frame_id;          // Target frame (base_link)
    std::string source_frame;      // Source frame (camera_left_link)

    // Topic names
    std::string input_topic;       // PointCloud2 input
    std::string output_topic;      // LaserScan output

    // Conversion parameters
    double conversion_height;      // Height slice [m]
    double conversion_range;       // Height tolerance [m]

    // Laser scan parameters
    int laser_density;             // Number of beams
    double laser_min_range;        // Minimum range [m]
    double laser_max_range;        // Maximum range [m]
    double laser_angle_min;        // Minimum angle [rad]
    double laser_angle_max;        // Maximum angle [rad]
    double laser_angle_increment;  // Angular spacing [rad]

    // Transform parameters
    double transform_tolerance;    // Transform staleness [s]

    // QoS parameters
    std::string qos_reliability;   // "reliable" or "best_effort"
    std::string qos_durability;    // "volatile" or "transient_local"
    int qos_depth;                 // Queue depth

    // Filtering parameters
    double filter_max_distance;    // Max distance between adjacent points [m]
    int filter_group_amount;       // Outlier detection window size
};

class LaserDataNode : public rclcpp::Node {
private:
    // Core components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    // TF2 for coordinate transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Configuration
    LaserConfig config_;

public:
    LaserDataNode() : Node("laser_data_node") {
        RCLCPP_INFO(get_logger(), "Initializing Laser Data Node...");

        // Initialize components
        initialize_parameters();
        setup_tf2();
        setup_publishers();
        setup_subscribers();

        RCLCPP_INFO(get_logger(), "Laser Data Node initialized successfully");
        RCLCPP_INFO(get_logger(), "  Input: %s", config_.input_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Output: %s", config_.output_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Transform: %s -> %s",
                   config_.source_frame.c_str(), config_.frame_id.c_str());
    }

    ~LaserDataNode() {
        RCLCPP_INFO(get_logger(), "Shutting down Laser Data Node");
    }

private:
    void initialize_parameters() {
        // ========================================================================
        // FRAME CONFIGURATION
        // ========================================================================
        declare_parameter("frame_id", "base_link");
        declare_parameter("source_frame", "camera_left_link");

        // ========================================================================
        // TOPIC NAMES
        // ========================================================================
        declare_parameter("input_topic", "/stereo_camera/points");
        declare_parameter("output_topic", "/scan");

        // ========================================================================
        // CONVERSION PARAMETERS
        // ========================================================================
        declare_parameter("conversion.height", 0.05);
        declare_parameter("conversion.range", 0.005);

        // ========================================================================
        // LASER SCAN PARAMETERS
        // ========================================================================
        declare_parameter("laser.density", 640);
        declare_parameter("laser.min_range", 0.3);
        declare_parameter("laser.max_range", 1.5);
        declare_parameter("laser.angle_min", -0.6370451769);
        declare_parameter("laser.angle_max", 0.6370451769);
        declare_parameter("laser.angle_increment", 0.001993907112);

        // ========================================================================
        // TRANSFORM PARAMETERS
        // ========================================================================
        declare_parameter("transform_tolerance", 0.066);

        // ========================================================================
        // QOS PARAMETERS
        // ========================================================================
        declare_parameter("qos.reliability", "reliable");
        declare_parameter("qos.durability", "volatile");
        declare_parameter("qos.depth", 1);

        // ========================================================================
        // FILTERING PARAMETERS
        // ========================================================================
        declare_parameter("filter.max_distance", 0.05);
        declare_parameter("filter.group_amount", 4);

        // Load all parameters into config struct
        load_parameters();
    }

    void load_parameters() {
        // Frame configuration
        config_.frame_id = get_parameter("frame_id").as_string();
        config_.source_frame = get_parameter("source_frame").as_string();

        // Topic names
        config_.input_topic = get_parameter("input_topic").as_string();
        config_.output_topic = get_parameter("output_topic").as_string();

        // Conversion parameters
        config_.conversion_height = get_parameter("conversion.height").as_double();
        config_.conversion_range = get_parameter("conversion.range").as_double();

        // Laser scan parameters
        config_.laser_density = get_parameter("laser.density").as_int();
        config_.laser_min_range = get_parameter("laser.min_range").as_double();
        config_.laser_max_range = get_parameter("laser.max_range").as_double();
        config_.laser_angle_min = get_parameter("laser.angle_min").as_double();
        config_.laser_angle_max = get_parameter("laser.angle_max").as_double();
        config_.laser_angle_increment = get_parameter("laser.angle_increment").as_double();

        // Transform parameters
        config_.transform_tolerance = get_parameter("transform_tolerance").as_double();

        // QoS parameters
        config_.qos_reliability = get_parameter("qos.reliability").as_string();
        config_.qos_durability = get_parameter("qos.durability").as_string();
        config_.qos_depth = get_parameter("qos.depth").as_int();

        // Filtering parameters
        config_.filter_max_distance = get_parameter("filter.max_distance").as_double();
        config_.filter_group_amount = get_parameter("filter.group_amount").as_int();

        RCLCPP_INFO(get_logger(), "Parameters loaded successfully");
    }

    void setup_tf2() {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(), "TF2 initialized");
    }

    void setup_publishers() {
        // Create QoS profile based on configuration
        auto qos = rclcpp::QoS(config_.qos_depth);

        if (config_.qos_reliability == "reliable") {
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        } else {
            qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        }

        if (config_.qos_durability == "volatile") {
            qos.durability(rclcpp::DurabilityPolicy::Volatile);
        } else {
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        }

        // Create LaserScan publisher
        laser_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            config_.output_topic, qos);

        RCLCPP_INFO(get_logger(), "LaserScan publisher created: %s", config_.output_topic.c_str());
    }

    void setup_subscribers() {
        // Create QoS profile to match pointcloud publisher
        auto qos = rclcpp::QoS(config_.qos_depth);

        if (config_.qos_reliability == "reliable") {
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        } else {
            qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        }

        if (config_.qos_durability == "volatile") {
            qos.durability(rclcpp::DurabilityPolicy::Volatile);
        } else {
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        }

        // Create PointCloud2 subscriber
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.input_topic,
            qos,
            std::bind(&LaserDataNode::pointcloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "PointCloud2 subscriber created: %s", config_.input_topic.c_str());
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // TODO: Implement conversion logic
        // 1. Validate pointcloud data
        // 2. Check transform availability
        // 3. Extract points from PointCloud2
        // 4. Transform points to base_link
        // 5. Filter by height (conversion_height ± conversion_range)
        // 6. Project to 2D and bin by angle
        // 7. Keep minimum distance per angle bin
        // 8. Apply noise filtering
        // 9. Publish LaserScan message

        RCLCPP_DEBUG(get_logger(), "PointCloud2 received: %d points",
                    msg->width * msg->height);

        // For now, just log that we received data
        // Implementation will come in next phase
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
