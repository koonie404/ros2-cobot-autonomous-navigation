#include <cmath>
#include <thread>
#include <atomic>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "analysis_ex.hpp"

static constexpr char   PORT[]          = "/dev/ttyUSB0";
static constexpr int    BAUD            = 230400;
static constexpr int    STREAM_BUF_SIZE = 1024;
static constexpr int    PACKETS_PER_REV = 56;    // 1회전당 패킷 수 (667 / 12 ≈ 56)
static constexpr float  SENSOR_HZ       = 6.0f;

// ── 시리얼 포트 초기화 ────────────────────────────────────────────────────────
static int open_serial(const char *port, int baud)
{
    int fd = open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return fd;

    termios tty{};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B230400);          // baud 고정 (필요시 파라미터화)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// ── LidarPublisher 노드 ───────────────────────────────────────────────────────
class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher()
    : Node("lidar_publisher"), running_(false)
    {
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        fd_ = open_serial(PORT, BAUD);
        if (fd_ < 0)
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", PORT);

        initialize_scan();
        RCLCPP_INFO(get_logger(), "LD14P LiDAR publisher started on %s", PORT);
    }

    ~LidarPublisher()
    {
        if (fd_ >= 0) close(fd_);
    }

    void start()
    {
        running_ = true;
        thread_  = std::thread(&LidarPublisher::serial_loop, this);
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    // ── 초기화 (Python _initialize_scan 대응) ─────────────────────────────────
    void initialize_scan()
    {
        const int buf_size = GetScanBufferSize();

        scan_msg_.header.frame_id = "laser_frame";

        scan_msg_.angle_min = -M_PI;

        scan_msg_.angle_increment =
            2.0f * M_PI / buf_size;

        scan_msg_.angle_max =
            scan_msg_.angle_min
            + scan_msg_.angle_increment
            * (buf_size - 1);

        scan_msg_.scan_time      = 1.0f / SENSOR_HZ;
        scan_msg_.time_increment = scan_msg_.scan_time / buf_size;

        scan_msg_.range_min = 0.15f;
        scan_msg_.range_max = 8.0f;

        scan_msg_.ranges.assign(buf_size, 0.0f);
        scan_msg_.intensities.assign(buf_size, 0.0f);
    }

    // ── 발행 (Python _publish_scan 대응) ──────────────────────────────────────
    void publish_scan()
    {
        const auto &ranges      = GetRanges();
        const auto &intensities = GetIntensities();

        scan_msg_.header.stamp = now();

        std::copy(ranges.begin(),      ranges.end(),      scan_msg_.ranges.begin());
        std::copy(intensities.begin(), intensities.end(), scan_msg_.intensities.begin());

        scan_pub_->publish(scan_msg_);

        std::fill(scan_msg_.ranges.begin(),      scan_msg_.ranges.end(),      0.0f);
        std::fill(scan_msg_.intensities.begin(), scan_msg_.intensities.end(), 0.0f);
    }

    // ── 시리얼 루프 (Python _serial_loop 대응) ────────────────────────────────
    void serial_loop()
    {
        uint8_t stream_buf[STREAM_BUF_SIZE];
        int     parse_cnt = 0;

        while (running_)
        {
            int avail = 0;
            ioctl(fd_, FIONREAD, &avail);

            while (avail > 0)
            {
                int to_read = std::min(avail, STREAM_BUF_SIZE);
                int n       = read(fd_, stream_buf, to_read);
                if (n > 0)
                {
                    for (int i = 0; i < n; i++)
                    {
                        if (AnalysisOne(stream_buf[i]))
                        {
                            Parse();
                            if (++parse_cnt >= PACKETS_PER_REV)
                            {
                                parse_cnt = 0;
                                publish_scan();
                            }
                        }
                    }
                }
                ioctl(fd_, FIONREAD, &avail);
            }

            usleep(1000);   // Python time.sleep(0.001) 대응
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    sensor_msgs::msg::LaserScan scan_msg_;

    int              fd_      = -1;
    std::atomic<bool> running_;
    std::thread      thread_;
};

// ── 엔트리포인트 ──────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPublisher>();
    node->start();
    try {
        rclcpp::spin(node);
    } catch (...) {}
    node->stop();
    rclcpp::shutdown();
    return 0;
}


// cd ld14p_cpp
// mkdir build
// cd build
// cmake ..
// make
// ./ld14p_cpp