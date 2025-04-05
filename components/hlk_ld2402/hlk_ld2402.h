#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"
#include <vector>

namespace esphome {
namespace hlk_ld2402 {

// Dedicated distance sensor class
class HlkLd2402DistanceSensor : public sensor::Sensor {
 public:
  void publish_distance(float distance) { this->publish_state(distance); }
};

// Dedicated presence sensor class
class HlkLd2402PresenceSensor : public binary_sensor::BinarySensor {
 public:
  void publish_presence(bool presence) { this->publish_state(presence); }
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
  void set_distance_sensor(HlkLd2402DistanceSensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_presence_sensor(HlkLd2402PresenceSensor *presence_sensor) { presence_sensor_ = presence_sensor; }
  void set_distance_update_interval(uint32_t distance_update_interval) { distance_update_interval_ = distance_update_interval; }
  void set_timeout(uint32_t timeout) { timeout_ = timeout; }

  int my_available();
  int my_read();

 protected:
  // Buffer handling
  void clear_buffer_();
  
  // Binary frame processing
  bool process_binary_frame_(uint8_t byte);
  void handle_binary_frame_();
  void reset_binary_frame_();
  void process_text_buffer_();
  
  // Engineering mode commands
  void send_command_(uint16_t command, const uint8_t *data = nullptr, size_t data_len = 0);
  bool enable_configuration_();
  bool set_engineering_mode_();
  bool exit_configuration_();
  bool read_ack_(uint16_t command); // Add this line

  // Delayed engineering mode setup
  void setup_engineering_mode_();
  
  // Debugging helper
  void debug_dump_bytes_(const char* label, const uint8_t* data, size_t len);

  // Component variables
  static const uint8_t MAX_BUFFER_SIZE = 64;
  char buffer_[MAX_BUFFER_SIZE];
  size_t buffer_pos_{0};
  uint32_t last_read_time_{0};
  
  // Binary frame variables
  static const uint16_t MAX_BINARY_BUFFER_SIZE = 256;  // Changed from uint8_t to uint16_t
  uint8_t binary_buffer_[MAX_BINARY_BUFFER_SIZE];
  size_t binary_buffer_pos_{0};
  uint8_t frame_header_pos_{0};
  uint16_t expected_frame_length_{0};
  bool in_binary_frame_{false};
  
  // Configuration parameters
  float max_distance_{10.0}; // Default 10 meters
  uint32_t timeout_{5000};   // Default 5 seconds in ms
  uint32_t distance_update_interval_{1000}; // Default 1 second in ms
  
  // Last value buffering
  float last_distance_{0.0};
  bool has_new_data_{false};
  uint32_t last_publish_time_{0};
  
  // Presence data
  bool presence_detected_{false};
  bool has_presence_update_{false};
  uint32_t last_frame_received_time_{0};
  
  // Sensors
  HlkLd2402DistanceSensor *distance_sensor_{nullptr};
  HlkLd2402PresenceSensor *presence_sensor_{nullptr};

  // Input buffer
  std::vector<uint8_t> input_buffer_;

  static const uint16_t ACK_COMMAND_OFFSET = 0x0100;

  // Engineering mode setup timing
  bool engineering_mode_setup_complete_{false};
  uint32_t engineering_mode_setup_start_time_{0};
  static const uint32_t ENGINEERING_MODE_SETUP_DELAY = 30000; // 30 seconds
};

} // namespace hlk_ld2402
} // namespace esphome
