#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <memory>
#include <vector>

namespace transform_imu 
{
    class Transform_imu: public rclcpp::Node
    {
        public:
            Transform_imu();

        private:
        void callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void updateQuaternion(double &w, double &x, double &y, double &z, double wx, double wy, double wz, double dt);

        private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publish;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
        double w_new;
        double x_new;
        double y_new;
        double z_new;
    };

}