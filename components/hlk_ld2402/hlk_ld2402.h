#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace hlk_ld2402 {

// Dedicated distance sensor class
class HlkLd2402DistanceSensor : public sensor::Sensor {
 public:
  void publish_distance(float distance) { this->publish_state(distance); }
};

class HLKLD2402Component : public Component, public uart::UARTDevice {
 public:
  HLKLD2402Component() = default;

  // Set up the component
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Setters for configuration
  void set_uart(uart::UARTComponent *uart) { this->set_uart_parent(uart); }
  void set_max_distance(float max_distance) { max_distance_ = max_distance; }
  void set_timeout(uint32_t timeout) { timeout_ = timeout * 1000; } // Convert to milliseconds
  void set_distance_sensor(HlkLd2402DistanceSensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_distance_update_interval(uint32_t distance_update_interval) { distance_update_interval_ = distance_update_interval; }

 protected:
  // Buffer handling
  void process_buffer_();
  void clear_buffer_();

  // Component variables
  static const uint8_t MAX_BUFFER_SIZE = 64;
  char buffer_[MAX_BUFFER_SIZE];
  size_t buffer_pos_{0};
  uint32_t last_read_time_{0};
  
  // Configuration parameters
  float max_distance_{10.0}; // Default 10 meters
  uint32_t timeout_{5000};   // Default 5 seconds in ms
  uint32_t distance_update_interval_{1000}; // Default 1 second in ms
  
  // Last value buffering
  float last_distance_{0.0};
  bool has_new_data_{false};
  uint32_t last_publish_time_{0};
  
  // Sensors
  HlkLd2402DistanceSensor *distance_sensor_{nullptr};
};

} // namespace hlk_ld2402
} // namespace esphome
