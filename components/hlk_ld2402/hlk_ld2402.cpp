#include "hlk_ld2402.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2402 {

static const char *const TAG = "hlk_ld2402";

void HLKLD2402Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2402 component...");
  this->last_publish_time_ = millis();
  
  // Initialize presence update flag to ensure first reading gets published
  this->has_presence_update_ = true;
  
  // Put the module into engineering mode on startup
  ESP_LOGD(TAG, "Enabling engineering mode...");
  if (enable_configuration_()) {
    ESP_LOGD(TAG, "Configuration mode enabled");
    delay(100);  // Small delay between commands
    
    if (set_engineering_mode_()) {
      ESP_LOGD(TAG, "Engineering mode set successfully");
    } else {
      ESP_LOGE(TAG, "Failed to set engineering mode");
    }
    
    delay(100);  // Small delay between commands
    if (exit_configuration_()) {
      ESP_LOGD(TAG, "Exited configuration mode");
    } else {
      ESP_LOGE(TAG, "Failed to exit configuration mode");
    }
  } else {
    ESP_LOGE(TAG, "Failed to enable configuration mode");
  }
  ESP_LOGI(TAG, "Initialized presence detection - starting state: not present");
}

void HLKLD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2402:");
  ESP_LOGCONFIG(TAG, "  Max Distance: %.1fm", this->max_distance_);
  ESP_LOGCONFIG(TAG, "  Distance Update Interval: %dms", this->distance_update_interval_);
  this->check_uart_settings(115200);
}

void HLKLD2402Component::loop() {
  // Check if there are new bytes available in the UART
  while (available()) {
    uint8_t byte = read();
    
    // Try to process as binary frame first
    if (process_binary_frame_(byte)) {
      continue;  // Byte was consumed by binary frame processing
    }
    
    // If not part of binary frame, process as text
    // Check for buffer overflow
    if (buffer_pos_ >= MAX_BUFFER_SIZE - 1) {
      clear_buffer_();
    }
    
    // Only add printable characters or newline
    if (isprint(byte) || byte == '\n' || byte == '\r') {
      buffer_[buffer_pos_++] = byte;
      
      // If we see a newline, process the buffer
      if (byte == '\n' || byte == '\r') {
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
    // Publish distance update if available
    if (this->has_new_data_ && this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_distance(this->last_distance_);
      this->has_new_data_ = false;
      ESP_LOGD(TAG, "Publishing buffered distance: %.2f cm", this->last_distance_);
    }
    
    // Publish presence update if available
    if (this->has_presence_update_ && this->presence_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing presence state: %s", 
              this->presence_detected_ ? "PRESENT" : "NOT PRESENT");
      this->presence_sensor_->publish_presence(this->presence_detected_);
      this->has_presence_update_ = false;
      ESP_LOGD(TAG, "Publishing presence state: %s", this->presence_detected_ ? "present" : "not present");
    }
    
    this->last_publish_time_ = now;
  }
}

bool HLKLD2402Component::process_binary_frame_(uint8_t byte) {
  // Frame header is F4 F3 F2 F1
  static const uint8_t FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
  // Frame footer is F8 F7 F6 F5
  static const uint8_t FRAME_FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};
  static uint32_t frame_count = 0; // Add a frame counter
  
  // Looking for frame header
  if (!in_binary_frame_) {
    if (byte == FRAME_HEADER[frame_header_pos_]) {
      ESP_LOGV(TAG, "Header byte %d matched: 0x%02X", frame_header_pos_, byte);
      binary_buffer_[frame_header_pos_] = byte;
      frame_header_pos_++;
      if (frame_header_pos_ == 4) {
        // Header found, start collecting frame data
        ESP_LOGD(TAG, "Binary frame header detected - starting collection");
        in_binary_frame_ = true;
        binary_buffer_pos_ = 4;  // Start after header
        frame_header_pos_ = 0;  // Reset for future headers
      }
      return true;
    } else {
      // Reset header detection if sequence breaks
      if (frame_header_pos_ > 0) {
        ESP_LOGV(TAG, "Header sequence broken at pos %d with byte 0x%02X", 
                frame_header_pos_, byte);
      }
      frame_header_pos_ = 0;
      return false;
    }
  }
  
  // In binary frame, collect data
  if (in_binary_frame_) {
    // Check for buffer overflow
    if (binary_buffer_pos_ >= MAX_BINARY_BUFFER_SIZE - 1) {
      ESP_LOGW(TAG, "Binary buffer overflow, resetting");
      reset_binary_frame_();
      return false;
    }
    
    binary_buffer_[binary_buffer_pos_++] = byte;
    
    // After getting header + 2 bytes for length, check expected frame length
    if (binary_buffer_pos_ == 6) {
      expected_frame_length_ = binary_buffer_[4] | (binary_buffer_[5] << 8);
      ESP_LOGV(TAG, "Expected binary frame length: %d", expected_frame_length_);
      
      // Sanity check on frame length
      if (expected_frame_length_ > MAX_BINARY_BUFFER_SIZE - 8) {  // Header (4) + Length (2) + Footer (4) = 10
        ESP_LOGW(TAG, "Invalid frame length: %d", expected_frame_length_);
        reset_binary_frame_();
        return false;
      }
    }
    
    // Check for complete frame (header + length field + data + footer)
    if (binary_buffer_pos_ >= 6 + expected_frame_length_ + 4) {
      // Check footer
      bool valid_footer = true;
      for (int i = 0; i < 4; i++) {
        if (binary_buffer_[binary_buffer_pos_ - 4 + i] != FRAME_FOOTER[i]) {
          valid_footer = false;
          break;
        }
      }
      
      if (valid_footer) {
        frame_count++;
        ESP_LOGD(TAG, "Valid binary frame #%u received, length: %d", 
                  frame_count, binary_buffer_pos_);
        
        // Print hex dump of the first few bytes
        char hex_buffer[100] = {0};
        char* pos = hex_buffer;
        int bytes_to_print = std::min(16, (int)binary_buffer_pos_);
        for (int i = 0; i < bytes_to_print; i++) {
          pos += sprintf(pos, "%02X ", binary_buffer_[i]);
        }
        ESP_LOGD(TAG, "Frame header+data: %s...", hex_buffer);
        
        handle_binary_frame_();
      } else {
        ESP_LOGW(TAG, "Invalid footer in binary frame");
      }
      
      reset_binary_frame_();
    }
    
    return true;
  }
  
  return false;
}

void HLKLD2402Component::handle_binary_frame_() {
  // We need at least header(4) + length(2) + status(1) + distance(2) bytes
  if (binary_buffer_pos_ < 9) {
    ESP_LOGW(TAG, "Binary frame too short");
    return;
  }
  
  // Status byte is at position 6
  uint8_t status = binary_buffer_[6];
  ESP_LOGD(TAG, "Status byte: 0x%02X (%d)", status, status);
  
  // Distance is 2 bytes at position 7 (little-endian)
  uint16_t distance = binary_buffer_[7] | (binary_buffer_[8] << 8);
  
  // Update presence based on status (0 = no person, 1 or 2 = person present)
  bool presence = (status == 1 || status == 2);
  
  ESP_LOGD(TAG, "Raw presence detection: status=%d, presence=%s", 
           status, presence ? "TRUE" : "FALSE");
  
  // Always update presence status for every valid frame
  this->presence_detected_ = presence;
  this->has_presence_update_ = true;
  
  ESP_LOGI(TAG, "Presence update: %s, status byte: %d", 
          presence ? "PRESENT" : "NOT PRESENT", status);
  
  ESP_LOGD(TAG, "Presence status: %s, status byte: %d", 
          presence ? "present" : "not present", status);
  
  // Optionally update distance from binary frame too
  float distance_cm = static_cast<float>(distance);
  if (distance > 0 && distance_cm <= this->max_distance_ * 100) {
    ESP_LOGV(TAG, "Binary frame distance: %.1f cm", distance_cm);
    // We could update this->last_distance_ here, but we already get text updates
  }
}

void HLKLD2402Component::reset_binary_frame_() {
  in_binary_frame_ = false;
  binary_buffer_pos_ = 0;
  frame_header_pos_ = 0;
  expected_frame_length_ = 0;
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

void HLKLD2402Component::send_command_(uint16_t command, const uint8_t *data, size_t data_len) {
  // Frame header: 0xFDFCFBFA
  this->write_byte(0xFD);
  this->write_byte(0xFC);
  this->write_byte(0xFB);
  this->write_byte(0xFA);
  
  // Frame data length (2 bytes, little-endian)
  uint16_t data_length = 2 + data_len;  // Command is 2 bytes + any additional data
  this->write_byte(data_length & 0xFF);
  this->write_byte((data_length >> 8) & 0xFF);
  
  // Command (2 bytes, little-endian)
  this->write_byte(command & 0xFF);
  this->write_byte((command >> 8) & 0xFF);
  
  // Data (if any)
  if (data != nullptr && data_len > 0) {
    for (size_t i = 0; i < data_len; i++) {
      this->write_byte(data[i]);
    }
  }
  
  // Frame footer: 0x04030201
  this->write_byte(0x04);
  this->write_byte(0x03);
  this->write_byte(0x02);
  this->write_byte(0x01);
  
  // Ensure all data is sent
  this->flush();
}

bool HLKLD2402Component::enable_configuration_() {
  // Command: 0x00FF with parameter 0x0001
  uint8_t data[2] = {0x01, 0x00};  // Little-endian: 0x0001
  send_command_(0x00FF, data, 2);
  return true;  // Simplified implementation without checking ACK
}

bool HLKLD2402Component::set_engineering_mode_() {
  // Command: 0x0012 with parameter 0x00000004
  uint8_t data[6] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x00};  // Little-endian: 0x00000004
  send_command_(0x0012, data, 6);
  return true;  // Simplified implementation without checking ACK
}

bool HLKLD2402Component::exit_configuration_() {
  // Command: 0x00FE with no parameters
  send_command_(0x00FE);
  return true;  // Simplified implementation without checking ACK
}

} // namespace hlk_ld2402
} // namespace esphome
