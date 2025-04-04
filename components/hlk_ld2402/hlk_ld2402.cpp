#include "hlk_ld2402.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2402 {

static const char *const TAG = "hlk_ld2402";

void HLKLD2402Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2402 component...");
  this->last_publish_time_ = millis();
}

void HLKLD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2402:");
  ESP_LOGCONFIG(TAG, "  Max Distance: %.1fm", this->max_distance_);
  ESP_LOGCONFIG(TAG, "  Timeout: %dms", this->timeout_);
  ESP_LOGCONFIG(TAG, "  Distance Update Interval: %dms", this->distance_update_interval_);
  this->check_uart_settings(115200);
}

void HLKLD2402Component::loop() {
  // Check if there are new bytes available in the UART
  while (available()) {
    char c = read();
    
    // Check for buffer overflow
    if (buffer_pos_ >= MAX_BUFFER_SIZE - 1) {
      clear_buffer_();
    }
    
    // Only add printable characters or newline
    if (isprint(c) || c == '\n' || c == '\r') {
      buffer_[buffer_pos_++] = c;
      
      // If we see a newline, process the buffer
      if (c == '\n' || c == '\r') {
        buffer_[buffer_pos_] = '\0';  // Null-terminate
        process_buffer_();
        clear_buffer_();
      }
    }
  }
  
  // Clear buffer if it's been too long since the last read
  if (buffer_pos_ > 0 && millis() - last_read_time_ > 1000) {
    clear_buffer_();
  }
  
  // Check if it's time to publish an update
  uint32_t now = millis();
  if (now - this->last_publish_time_ >= this->distance_update_interval_) {
    // Only publish if we have new data since last publish
    if (this->has_new_data_ && this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_distance(this->last_distance_);
      this->has_new_data_ = false;
      ESP_LOGD(TAG, "Publishing buffered distance: %.2f cm", this->last_distance_);
    }
    this->last_publish_time_ = now;
  }
}

void HLKLD2402Component::process_buffer_() {
  // Convert buffer to string for easier processing
  std::string line(buffer_);
  
  // Trim whitespace
  if (!line.empty()) {
    size_t last = line.find_last_not_of(" \n\r\t");
    if (last != std::string::npos) {
      line.erase(last + 1);
    }
  }
  
  // Skip empty lines
  if (line.empty() || line.find_first_not_of(" \n\r\t") == std::string::npos) {
    return;
  }
  
  // Log the raw data we received
  ESP_LOGV(TAG, "Received data: '%s'", line.c_str());
  
  // Convert to lowercase for case-insensitive comparison
  std::string lowercase_line = line;
  for (char &c : lowercase_line) {
    c = tolower(c);
  }
  
  // Check if we have a valid distance reading
  if (lowercase_line.find("distance:") != std::string::npos) {
    // Extract distance value
    float distance = atof(line.c_str() + 9); // +9 to skip "distance:"
    
    // Check if distance is within max range
    if (distance > 0 && distance <= this->max_distance_ * 100) { 
      // Store the distance value (in cm) for later publishing
      this->last_distance_ = distance;
      this->has_new_data_ = true;
      ESP_LOGV(TAG, "Updated buffer with new distance: %.2f cm", distance);
    } else {
      ESP_LOGD(TAG, "Distance out of range: %.2f cm", distance);
    }
  } else if (lowercase_line == "off") {
    // No target detected
    ESP_LOGD(TAG, "No target detected");
  } else {
    // Unknown data format
    ESP_LOGD(TAG, "Unknown data format: '%s'", line.c_str());
  }
}

void HLKLD2402Component::clear_buffer_() {
  // Reset buffer
  buffer_pos_ = 0;
  last_read_time_ = millis();
}

} // namespace hlk_ld2402
} // namespace esphome
