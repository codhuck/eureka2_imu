#include <eureka2_imu/main.hpp>

namespace transform_imu 
{
  Transform_imu ::Transform_imu()
  : rclcpp::Node("imu_transformation"),
  w_new(1.0),
  x_new(0.0),
  y_new(0.0),
  z_new(0.0)
  {
    publish=this->create_publisher<sensor_msgs::msg::Imu>("transform_imu", 10);
    subscription=this->create_subscription<sensor_msgs::msg::Imu>("/camera/camera/imu", 10, std::bind(&Transform_imu::callback, this, std::placeholders::_1));
    }

    void Transform_imu::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu message;
        message.angular_velocity.x = msg->angular_velocity.z;
        message.angular_velocity.y = -msg->angular_velocity.x;
        message.angular_velocity.z = -msg->angular_velocity.y;
        message.linear_acceleration.x = msg->linear_acceleration.z;
        message.linear_acceleration.y = -msg->linear_acceleration.x;
        message.linear_acceleration.z = -msg->linear_acceleration.y;
        updateQuaternion(w_new, x_new, y_new, z_new, message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z, 0.05);
        message.orientation.w = w_new;
        message.orientation.x = x_new;
        message.orientation.y = y_new;
        message.orientation.z = z_new;
        publish->publish(message);
    }

    void Transform_imu::updateQuaternion(double &w, double &x, double &y, double &z, double wx, double wy, double wz, double dt) 
    {
        double half_dt = 0.5 * dt;
        double new_w = w + (-x * wx - y * wy - z * wz) * half_dt;
        double new_x = x + (w * wx + y * wz - z * wy) * half_dt;
        double new_y = y + (w * wy - x * wz + z * wx) * half_dt;
        double new_z = z + (w * wz + x * wy - y * wx) * half_dt;
        double norm = std::sqrt(new_w*new_w + new_x*new_x + new_y*new_y + new_z*new_z);
        w = new_w / norm;
        x = new_x / norm;
        y = new_y / norm;
        z = new_z / norm;
}

  }

  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<transform_imu::Transform_imu> node = std::make_shared<transform_imu::Transform_imu>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
  }