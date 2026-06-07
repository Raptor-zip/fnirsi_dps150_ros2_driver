#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "fnirsi_dps150_driver/dps150_client.hpp"
#include "fnirsi_dps150_driver/msg/dps150_state.hpp"
#include "fnirsi_dps150_driver/srv/select_preset.hpp"
#include "fnirsi_dps150_driver/srv/set_float32.hpp"
#include "fnirsi_dps150_driver/srv/set_preset.hpp"
#include "fnirsi_dps150_driver/srv/set_protection.hpp"
#include "fnirsi_dps150_driver/srv/set_u_int8.hpp"

namespace fnirsi_dps150_driver
{

class Dps150Node : public rclcpp::Node
{
public:
  explicit Dps150Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Dps150Node() override;

private:
  template<typename Callable>
  void run_command(const char * label, Callable && callable)
  {
    try {
      callable();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "%s failed: %s", label, error.what());
      last_error_ = error.what();
    }
  }

  template<typename Callable>
  void service_command(bool & success, std::string & message, Callable && callable)
  {
    try {
      message = callable();
      success = true;
    } catch (const std::exception & error) {
      success = false;
      message = error.what();
      last_error_ = error.what();
    }
  }

  void connect_device();
  void disconnect_device();
  void refresh_after_write(bool request_refresh);
  void reconnect_tick();
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

  void checked_set_voltage(float volts);
  void checked_set_current(float amps);
  void checked_set_output_enabled(bool enabled);
  void checked_set_preset(std::uint8_t index, float volts, float amps, bool apply);
  void checked_select_preset(std::uint8_t index);
  void validate_voltage(float volts, const DeviceState & state) const;
  void validate_current(float amps, const DeviceState & state) const;
  void validate_power(float volts, float amps, const DeviceState & state) const;

  void publish_state(const DeviceState & state);
  msg::Dps150State to_msg(const DeviceState & state);

  std::string port_;
  int baud_rate_{115200};
  bool connect_on_start_{true};
  bool auto_refresh_after_write_{true};
  bool auto_reconnect_{true};
  bool manual_disconnect_{false};
  bool enforce_setpoint_limits_{true};
  bool allow_output_enable_during_protection_{false};
  int reconnect_period_ms_{2000};

  Dps150Client client_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;

  rclcpp::Publisher<msg::Dps150State>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr output_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refresh_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_output_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_metering_srv_;
  rclcpp::Service<srv::SetFloat32>::SharedPtr set_voltage_srv_;
  rclcpp::Service<srv::SetFloat32>::SharedPtr set_current_srv_;
  rclcpp::Service<srv::SetUInt8>::SharedPtr set_brightness_srv_;
  rclcpp::Service<srv::SetUInt8>::SharedPtr set_volume_srv_;
  rclcpp::Service<srv::SetPreset>::SharedPtr set_preset_srv_;
  rclcpp::Service<srv::SelectPreset>::SharedPtr select_preset_srv_;
  rclcpp::Service<srv::SetProtection>::SharedPtr set_protection_srv_;

  std::string last_error_;
  std::uint64_t reconnect_attempts_{0};
  std::uint64_t successful_reconnects_{0};
  rclcpp::Time last_state_time_;
};

}  // namespace fnirsi_dps150_driver
