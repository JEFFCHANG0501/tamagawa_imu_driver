#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

// uint16_t counter;
int16_t angular_velocity_x = 0;
int16_t angular_velocity_y = 0;
int16_t angular_velocity_z = 0;
int16_t acceleration_x = 0;
int16_t acceleration_y = 0;
int16_t acceleration_z = 0;
int16_t roll_raw = 0;
int16_t pitch_raw = 0;
int16_t azimuth_raw = 0;
float roll = 0;
float pitch = 0;
float azimuth = 0;

class TagCanPublisher : public rclcpp::Node
{
public:
    TagCanPublisher() : Node("tag_can_driver")
    {

        RCLCPP_INFO(this->get_logger(), "Start tag_can_driver.");
        declare_parameter("imu_topic", "/imu");
        get_parameter("imu_topic", imu_topic_);
        declare_parameter("can_topic", "/can");
        get_parameter("can_topic", can_topic_);
        RCLCPP_INFO(this->get_logger(), "Subscribe CAN Topic: %s", can_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish IMU Topic: %s", imu_topic_.c_str());

        sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
          can_topic_, 10, std::bind(&TagCanPublisher::can_callback, this, _1));
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 100);
    }

private:
    std::string imu_topic_;
    std::string can_topic_;
    sensor_msgs::msg::Imu imu_msg;

    void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if(msg->id == 0x319)
        {
            imu_msg.header.frame_id = "imu";
            imu_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

            // counter = msg->data[1] + (msg->data[0] << 8);
            angular_velocity_x = msg->data[3] + (msg->data[2] << 8);
            imu_msg.angular_velocity.x =
              angular_velocity_x * (200 / pow(2, 15)) * M_PI / 180;
            
            angular_velocity_y = msg->data[5] + (msg->data[4] << 8);
            imu_msg.angular_velocity.y =
              angular_velocity_y * (200 / pow(2, 15)) * M_PI / 180;
              
            angular_velocity_z = msg->data[7] + (msg->data[6] << 8);
            imu_msg.angular_velocity.z =
              angular_velocity_z * (200 / pow(2, 15)) * M_PI / 180; 
        }
        if(msg->id == 0x31A)
        {
            acceleration_x = msg->data[3] + (msg->data[2] << 8);
            imu_msg.linear_acceleration.x = acceleration_x * (100 / pow(2, 15));
            acceleration_y = msg->data[5] + (msg->data[4] << 8);
            imu_msg.linear_acceleration.y = acceleration_y * (100 / pow(2, 15));
            acceleration_z = msg->data[7] + (msg->data[6] << 8);
            imu_msg.linear_acceleration.z = acceleration_z * (100 / pow(2, 15));
        }
        if(msg->id == 0x31B)
        {
            roll_raw = msg->data[3] + (msg->data[2] << 8);
            roll = roll_raw * (180 / pow(2, 15)) * M_PI / 180;
            pitch_raw = msg->data[5] + (msg->data[4] << 8);
            pitch = pitch_raw * (180 / pow(2, 15)) * M_PI / 180;
            azimuth_raw = msg->data[7] + (msg->data[6] << 8);
            azimuth = azimuth_raw * (180 / pow(2, 15)) * M_PI / 180;
            
            tf2::Quaternion q;
            q.setRPY(roll, pitch, azimuth);
            imu_msg.orientation.x = q.getX();
            imu_msg.orientation.y = q.getY();
            imu_msg.orientation.z = q.getZ();
            imu_msg.orientation.w = q.getW();
            pub_imu_->publish(imu_msg);

        }
    }
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagCanPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}