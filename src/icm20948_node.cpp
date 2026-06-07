#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <string>
#include <stdexcept>

// ---------------------------------------------------------------------------
// ICM-20948 register map
// ---------------------------------------------------------------------------

// Shared (all banks)
static constexpr uint8_t REG_BANK_SEL = 0x7F;

// Bank 0
static constexpr uint8_t WHO_AM_I = 0x00;
static constexpr uint8_t USER_CTRL = 0x03;
static constexpr uint8_t PWR_MGMT_1 = 0x06;
static constexpr uint8_t PWR_MGMT_2 = 0x07;
static constexpr uint8_t INT_PIN_CFG = 0x0F;
static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
static constexpr uint8_t GYRO_XOUT_H = 0x33;
static constexpr uint8_t TEMP_OUT_H = 0x39;
static constexpr uint8_t EXT_SLV_SENS_DATA_00 = 0x3B;

// Bank 2
static constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
static constexpr uint8_t GYRO_CONFIG_1 = 0x01;
static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
static constexpr uint8_t ACCEL_CONFIG = 0x14;

// Bank 3
static constexpr uint8_t I2C_MST_ODR_CONFIG = 0x00;
static constexpr uint8_t I2C_SLV0_ADDR = 0x03;
static constexpr uint8_t I2C_SLV0_REG = 0x04;
static constexpr uint8_t I2C_SLV0_CTRL = 0x05;
static constexpr uint8_t I2C_SLV0_DO = 0x06;

// Magnetometer (AK09916)
static constexpr uint8_t MAG_ADDR = 0x0C;
static constexpr uint8_t MAG_CNTL2 = 0x31;
static constexpr uint8_t MAG_STATUS1 = 0x10;

// Scale factors
static constexpr double ACCEL_SCALE = 9.80665 / 16384.0;   // ±2g  → m/s²
static constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0;  // ±250°/s → rad/s
static constexpr double MAG_SCALE = 0.15e-6;               // T/LSB (AK09916)

// Covariances (diagonal, from datasheet noise specs)
static const double ACCEL_VAR = std::pow(400e-6 * 9.80665, 2);
static const double GYRO_VAR = std::pow(0.01 * M_PI / 180.0, 2);

// ---------------------------------------------------------------------------
// ICM20948Node
// ---------------------------------------------------------------------------

class ICM20948Node : public rclcpp::Node
{
public:
  ICM20948Node()
  : rclcpp::Node("icm20948_imu"), i2c_fd_(-1), publish_count_(0)
  {
    // --- Declare parameters ---
    declare_parameter<int>("i2c_bus", 7);
    declare_parameter<int>("i2c_address", 0x68);
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<double>("publish_rate", 100.0);

    // --- Read parameters ---
    int bus = get_parameter("i2c_bus").as_int();
    int address = get_parameter("i2c_address").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    double rate = get_parameter("publish_rate").as_double();
    i2c_addr_ = static_cast<uint8_t>(address);

    // --- Open I2C bus ---
    std::string dev = "/dev/i2c-" + std::to_string(bus);
    i2c_fd_ = open(dev.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      throw std::runtime_error("Cannot open I2C device " + dev + ": " + std::strerror(errno));
    }

    // --- Initialize the IMU ---
    init_icm20948();

    // --- Publishers ---
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/magnetic_field", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);

    // --- Timer ---
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ICM20948Node::publish_imu, this));

    RCLCPP_INFO(
      get_logger(),
      "ICM-20948 node started on %s addr=0x%02X rate=%.1f Hz frame=%s",
      dev.c_str(), i2c_addr_, rate, frame_id_.c_str());
  }

  ~ICM20948Node()
  {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

private:
  // -----------------------------------------------------------------------
  // Low-level I2C helpers
  // -----------------------------------------------------------------------

  // Write a single byte to a register.
  void write_reg(uint8_t reg, uint8_t val)
  {
    uint8_t buf[2] = {reg, val};
    struct i2c_msg msg;
    msg.addr = i2c_addr_;
    msg.flags = 0;           // write
    msg.len = 2;
    msg.buf = buf;

    struct i2c_rdwr_ioctl_data data;
    data.msgs = &msg;
    data.nmsgs = 1;

    if (ioctl(i2c_fd_, I2C_RDWR, &data) < 0) {
      throw std::runtime_error(
              std::string("write_reg(0x") + std::to_string(reg) + "): " + std::strerror(errno));
    }
  }

  // Read `len` bytes starting at `reg` using a repeated-start transaction.
  void read_bytes(uint8_t reg, uint8_t * buf, size_t len)
  {
    struct i2c_msg msgs[2];

    // Write phase: send register address
    msgs[0].addr = i2c_addr_;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    // Read phase: receive data
    msgs[1].addr = i2c_addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = static_cast<__u16>(len);
    msgs[1].buf = buf;

    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &data) < 0) {
      throw std::runtime_error(
              std::string("read_bytes(0x") + std::to_string(reg) + "): " + std::strerror(errno));
    }
  }

  // Read a single register.
  uint8_t read_reg(uint8_t reg)
  {
    uint8_t val = 0;
    read_bytes(reg, &val, 1);
    return val;
  }

  // Select register bank (0–3).
  void select_bank(uint8_t bank)
  {
    write_reg(REG_BANK_SEL, static_cast<uint8_t>(bank << 4));
  }

  // -----------------------------------------------------------------------
  // Magnetometer helper – write via ICM-20948 I2C master
  // -----------------------------------------------------------------------

  void mag_write(uint8_t reg, uint8_t val)
  {
    select_bank(3);
    write_reg(I2C_SLV0_ADDR, MAG_ADDR);       // write mode (bit7 = 0)
    write_reg(I2C_SLV0_REG, reg);
    write_reg(I2C_SLV0_DO, val);
    write_reg(I2C_SLV0_CTRL, 0x81);            // enable, 1 byte
    usleep(1000);                               // 1 ms
    select_bank(0);
  }

  // -----------------------------------------------------------------------
  // Initialisation sequence
  // -----------------------------------------------------------------------

  void init_icm20948()
  {
    // 1. Check WHO_AM_I
    select_bank(0);
    uint8_t who = read_reg(WHO_AM_I);
    if (who != 0xEA) {
      throw std::runtime_error(
              "ICM-20948 WHO_AM_I mismatch: expected 0xEA, got 0x" +
              std::to_string(who));
    }
    RCLCPP_INFO(get_logger(), "ICM-20948 WHO_AM_I OK (0x%02X)", who);

    // 2. Wake up, auto-select clock
    write_reg(PWR_MGMT_1, 0x01);
    usleep(10000);  // 10 ms

    // 3. Enable all axes
    write_reg(PWR_MGMT_2, 0x00);

    // 4. Gyro config – Bank 2
    select_bank(2);
    write_reg(GYRO_CONFIG_1, 0x01);       // ±250°/s, DLPF on
    write_reg(GYRO_SMPLRT_DIV, 0x04);     // ODR divider

    // 5. Accel config – Bank 2
    write_reg(ACCEL_CONFIG, 0x01);        // ±2g, DLPF on
    write_reg(ACCEL_SMPLRT_DIV_2, 0x04);  // ODR divider

    // 6. Enable I2C master – Bank 0
    select_bank(0);
    write_reg(USER_CTRL, 0x20);

    // 7. I2C master ODR – Bank 3
    select_bank(3);
    write_reg(I2C_MST_ODR_CONFIG, 0x04);

    // 8. Wake magnetometer, set 100 Hz continuous mode
    mag_write(MAG_CNTL2, 0x08);
    usleep(10000);  // 10 ms

    // 9. Configure SLV0 to continuously read 9 bytes from MAG_STATUS1
    select_bank(3);
    write_reg(I2C_SLV0_ADDR, MAG_ADDR | 0x80);  // read mode
    write_reg(I2C_SLV0_REG, MAG_STATUS1);
    write_reg(I2C_SLV0_CTRL, 0x89);              // enable, 9 bytes

    // 10. Back to bank 0 for normal operation
    select_bank(0);

    RCLCPP_INFO(get_logger(), "ICM-20948 initialisation complete.");
  }

  // -----------------------------------------------------------------------
  // Publish callback
  // -----------------------------------------------------------------------

  void publish_imu()
  {
    auto now = get_clock()->now();

    // --- Read 14 bytes: accel(6) + temp(2) + gyro(6) ---
    uint8_t raw[14] = {};
    try {
      read_bytes(ACCEL_XOUT_H, raw, 14);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to read IMU data: %s", e.what());
      return;
    }

    // Big-endian signed 16-bit parsing
    auto be16s = [](const uint8_t * p) -> int16_t {
      return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    int16_t ax_raw = be16s(&raw[0]);
    int16_t ay_raw = be16s(&raw[2]);
    int16_t az_raw = be16s(&raw[4]);
    int16_t t_raw = be16s(&raw[6]);
    int16_t gx_raw = be16s(&raw[8]);
    int16_t gy_raw = be16s(&raw[10]);
    int16_t gz_raw = be16s(&raw[12]);

    // --- Read 9 bytes from magnetometer shadow registers ---
    uint8_t mag_raw[9] = {};
    try {
      read_bytes(EXT_SLV_SENS_DATA_00, mag_raw, 9);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to read magnetometer data: %s", e.what());
    }

    // mag_raw layout from AK09916: [ST1, HXL, HXH, HYL, HYH, HZL, HZH, -, ST2]
    // Little-endian byte order
    int16_t mx_raw = static_cast<int16_t>(
      (static_cast<uint16_t>(mag_raw[2]) << 8) | mag_raw[1]);
    int16_t my_raw = static_cast<int16_t>(
      (static_cast<uint16_t>(mag_raw[4]) << 8) | mag_raw[3]);
    int16_t mz_raw = static_cast<int16_t>(
      (static_cast<uint16_t>(mag_raw[6]) << 8) | mag_raw[5]);

    // ----------------------------------------------------------------
    // Publish sensor_msgs/Imu
    // ----------------------------------------------------------------
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id_;

    // No orientation – signal with covariance[0] = -1
    imu_msg.orientation_covariance[0] = -1.0;

    imu_msg.linear_acceleration.x = ax_raw * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = ay_raw * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = az_raw * ACCEL_SCALE;

    imu_msg.angular_velocity.x = gx_raw * GYRO_SCALE;
    imu_msg.angular_velocity.y = gy_raw * GYRO_SCALE;
    imu_msg.angular_velocity.z = gz_raw * GYRO_SCALE;

    // Diagonal covariance matrices (off-diagonal = 0)
    imu_msg.linear_acceleration_covariance.fill(0.0);
    imu_msg.linear_acceleration_covariance[0] = ACCEL_VAR;
    imu_msg.linear_acceleration_covariance[4] = ACCEL_VAR;
    imu_msg.linear_acceleration_covariance[8] = ACCEL_VAR;

    imu_msg.angular_velocity_covariance.fill(0.0);
    imu_msg.angular_velocity_covariance[0] = GYRO_VAR;
    imu_msg.angular_velocity_covariance[4] = GYRO_VAR;
    imu_msg.angular_velocity_covariance[8] = GYRO_VAR;

    imu_pub_->publish(imu_msg);

    // ----------------------------------------------------------------
    // Publish sensor_msgs/MagneticField
    // ----------------------------------------------------------------
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = now;
    mag_msg.header.frame_id = frame_id_;

    mag_msg.magnetic_field.x = mx_raw * MAG_SCALE;
    mag_msg.magnetic_field.y = my_raw * MAG_SCALE;
    mag_msg.magnetic_field.z = mz_raw * MAG_SCALE;

    // Unknown covariance
    mag_msg.magnetic_field_covariance.fill(0.0);

    mag_pub_->publish(mag_msg);

    // ----------------------------------------------------------------
    // Publish sensor_msgs/Temperature (every 10th cycle)
    // ----------------------------------------------------------------
    if (++publish_count_ >= 10) {
      publish_count_ = 0;

      // ICM-20948 datasheet: Temp_degC = (TEMP_OUT / 333.87) + 21.0
      double temperature = (static_cast<double>(t_raw) / 333.87) + 21.0;

      sensor_msgs::msg::Temperature temp_msg;
      temp_msg.header.stamp = now;
      temp_msg.header.frame_id = frame_id_;
      temp_msg.temperature = temperature;
      temp_msg.variance = 0.0;         // unknown

      temp_pub_->publish(temp_msg);
    }
  }

  // -----------------------------------------------------------------------
  // Member variables
  // -----------------------------------------------------------------------
  int i2c_fd_;
  uint8_t i2c_addr_;
  std::string frame_id_;
  int publish_count_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ICM20948Node>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("icm20948_imu"),
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
