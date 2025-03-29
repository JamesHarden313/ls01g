#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "uart_driver.h"
using namespace std;

bool is_scan_stop = false;
bool is_motor_stop = false;
bool zero_as_max = true;
bool min_as_zero = true;
bool inverted = true;
string laser_link = "laser_link";
double angle_disable_min = -1;
double angle_disable_max = -1;
io_driver driver;

void publish_scan(const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub, 
                 double *dist, double *intensities, int count, 
                 rclcpp::Time start, double scan_time)
{
    static int scan_count = 0;
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = laser_link;
    scan_count++;
    scan_msg->angle_min = 0.0;
    scan_msg->angle_max = 2 * M_PI;
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(count - 1);
    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_time / (double)(count - 1);

    scan_msg->range_min = 0.1;
    scan_msg->range_max = 10.0;

    scan_msg->intensities.resize(count);
    scan_msg->ranges.resize(count);

    if (!inverted) {
        for (int i = count - 1; i >= 0; i--) {
            if (dist[count - i - 1] == 0.0 && zero_as_max)
                scan_msg->ranges[i] = scan_msg->range_max - 0.2;
            else if (dist[count - i - 1] == 0.0)
                if (min_as_zero)
                    scan_msg->ranges[i] = 0.0;
                else
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[i] = dist[count - i - 1] / 1000.0;
            scan_msg->intensities[i] = floor(intensities[count - i - 1]);
        }
    }
    else {
        for (int i = 0; i <= 179; i++) {
            if(dist[179-i] == 0.0 && zero_as_max)
                scan_msg->ranges[i] = scan_msg->range_max - 0.2;
            else if(dist[179-i] == 0.0)
                if (min_as_zero)
                    scan_msg->ranges[i] = 0.0;
                else
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[i] = dist[179-i] / 1000.0;
            scan_msg->intensities[i] = floor(intensities[179-i]);
        }
        for (int i = 180; i < 360; i++) {
            if(dist[540-i] == 0.0 && zero_as_max)
                scan_msg->ranges[i] = scan_msg->range_max - 0.2;
            else if(dist[540-i] == 0.0)
                if (min_as_zero)
                    scan_msg->ranges[i] = 0.0;
                else
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[i] = dist[540-i] / 1000.0;
            scan_msg->intensities[i] = floor(intensities[540-i]);
        }
    }
    for (int i = 0; i < 360; i++) {
        if((i >= angle_disable_min) && (i < angle_disable_max)) {
            if (min_as_zero)
                scan_msg->ranges[i] = 0.0;
            else
                scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    pub->publish(*scan_msg);
}

class LaserNode : public rclcpp::Node
{
public:
    LaserNode() : Node("ls01g")
    {
        // Declare and get parameters
        this->declare_parameter("scan_topic", "scan");
        this->declare_parameter("laser_link", "laser_link");
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("angle_disable_min", -1.0);
        this->declare_parameter("angle_disable_max", -1.0);
        this->declare_parameter("zero_as_max", true);
        this->declare_parameter("min_as_zero", true);
        this->declare_parameter("inverted", true);

        scan_topic_ = this->get_parameter("scan_topic").as_string();
        laser_link = this->get_parameter("laser_link").as_string();
        string port = this->get_parameter("serial_port").as_string();
        angle_disable_min = this->get_parameter("angle_disable_min").as_double();
        angle_disable_max = this->get_parameter("angle_disable_max").as_double();
        zero_as_max = this->get_parameter("zero_as_max").as_bool();
        min_as_zero = this->get_parameter("min_as_zero").as_bool();
        inverted = this->get_parameter("inverted").as_bool();

        // Create publisher and subscriber
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 1000);
        stop_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "startOrStop", 10, 
            std::bind(&LaserNode::start_stop_callback, this, std::placeholders::_1));

        // Initialize serial
        int ret = driver.OpenSerial(port.c_str(), B230400);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open port: %s", port.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Open port: %s", port.c_str());

        if (inverted) {
            RCLCPP_INFO(this->get_logger(), "This laser is inverted, zero degree direction is align with line");
        }

        driver.StartScan();
        RCLCPP_INFO(this->get_logger(), "Send start command successfully");

        // Start processing loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LaserNode::timer_callback, this));
    }

    ~LaserNode() {
        driver.StopScan(STOP_DATA);
        driver.StopScan(STOP_MOTOR);
        driver.CloseSerial();
        RCLCPP_INFO(this->get_logger(), "Keyboard Interrupt, ls01g stop!");
    }

private:
    void start_stop_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        Command cmd = (Command)msg->data;
        switch (cmd) {
            case STOP_DATA:
                if (!is_scan_stop) {
                    driver.StopScan(STOP_DATA);
                    is_scan_stop = true;
                    RCLCPP_INFO(this->get_logger(), "stop scan");
                }
                break;
            case STOP_MOTOR:
                if (!is_scan_stop) {
                    driver.StopScan(STOP_DATA);
                    is_scan_stop = true;
                    RCLCPP_INFO(this->get_logger(), "stop scan");
                }
                if (!is_motor_stop) {
                    driver.StopScan(STOP_MOTOR);
                    is_motor_stop = true;
                    RCLCPP_INFO(this->get_logger(), "stop motor");
                }
                break;
            case START_MOTOR_AND_SCAN:
                if (is_scan_stop) {
                    RCLCPP_INFO(this->get_logger(), "start scan");
                    int res = driver.StartScan();
                    RCLCPP_INFO(this->get_logger(), "start: %d", res);
                    is_scan_stop = false;
                    is_motor_stop = false;
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: %d", cmd);
                break;
        }
    }

    void timer_callback()
    {
        if (is_scan_stop)
            return;

        double angle[PACKLEN + 10];
        double distance[PACKLEN + 10];
        double data[PACKLEN + 10];
        double data_intensity[PACKLEN + 10];
        double speed;
        
        memset(data, 0, sizeof(data));
        int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
        for (int i = 0; i < ret; i++) {
            data[i] = distance[i];
            data_intensity[i] = angle[i];
        }
        
        static rclcpp::Time starts = this->now();
        rclcpp::Time ends = this->now();
        float scan_duration = (ends - starts).seconds() * 1e-3;
        
        publish_scan(scan_pub_, data, data_intensity, ret, starts, scan_duration);
        starts = ends;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000, "ls01g works fine!");
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    string scan_topic_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
