#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

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
        // 1. Validate input cloud
        if (msg->width * msg->height == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Received empty point cloud");
            return;
        }

        // 2. Lookup transform
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                config_.frame_id,
                msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(config_.transform_tolerance)
            );
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Transform failed: %s", ex.what());
            return;
        }

        // 3. Initialize LaserScan message
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = msg->header.stamp;
        scan.header.frame_id = config_.frame_id;
        scan.angle_min = config_.laser_angle_min;
        scan.angle_max = config_.laser_angle_max;
        scan.angle_increment = config_.laser_angle_increment;
        scan.time_increment = 0.0;
        scan.scan_time = 0.0;
        scan.range_min = config_.laser_min_range;
        scan.range_max = config_.laser_max_range;

        // Calculate number of range bins
        int num_ranges = static_cast<int>(
            (config_.laser_angle_max - config_.laser_angle_min) /
            config_.laser_angle_increment
        ) + 1;

        // Initialize all ranges to max_range (no detection)
        scan.ranges.assign(num_ranges, config_.laser_max_range);

        // 4. Iterate through point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        int points_processed = 0;
        int points_in_range = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // Skip invalid points
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                continue;
            }

            // Transform point to base_link frame
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header = msg->header;
            point_in.point.x = *iter_x;
            point_in.point.y = *iter_y;
            point_in.point.z = *iter_z;

            tf2::doTransform(point_in, point_out, transform);

            points_processed++;

            // 5. Height filtering - keep points in horizontal slice
            double height_diff = std::abs(point_out.point.z - config_.conversion_height);
            if (height_diff > config_.conversion_range) {
                continue;
            }

            // 6. Calculate angle and distance (polar coordinates)
            double angle = std::atan2(point_out.point.y, point_out.point.x);
            double distance = std::sqrt(
                point_out.point.x * point_out.point.x +
                point_out.point.y * point_out.point.y
            );

            // Check if distance is in valid range
            if (distance < config_.laser_min_range || distance > config_.laser_max_range) {
                continue;
            }

            // Check if angle is within scan range
            if (angle < config_.laser_angle_min || angle > config_.laser_angle_max) {
                continue;
            }

            // 7. Calculate bin index and keep minimum distance
            int index = static_cast<int>(
                (angle - config_.laser_angle_min) / config_.laser_angle_increment
            );

            if (index >= 0 && index < num_ranges) {
                // Keep minimum distance (closest obstacle - safety-first)
                if (distance < scan.ranges[index]) {
                    scan.ranges[index] = static_cast<float>(distance);
                    points_in_range++;
                }
            }
        }

        // 8. Apply noise filtering
        for (int i = 1; i < num_ranges - 1; ++i) {
            // Only filter points that have a valid reading
            if (scan.ranges[i] < config_.laser_max_range) {
                // Check distance to neighbors
                float diff_prev = std::abs(scan.ranges[i] - scan.ranges[i-1]);
                float diff_next = std::abs(scan.ranges[i] - scan.ranges[i+1]);

                // If point is isolated (far from neighbors), it's likely noise
                if (diff_prev > config_.filter_max_distance &&
                    diff_next > config_.filter_max_distance) {
                    scan.ranges[i] = config_.laser_max_range;
                }
            }
        }

        // 9. Publish LaserScan message
        laser_scan_pub_->publish(scan);

        RCLCPP_DEBUG(get_logger(),
                    "Processed %d points, %d in range, published %d laser ranges",
                    points_processed, points_in_range, num_ranges);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
