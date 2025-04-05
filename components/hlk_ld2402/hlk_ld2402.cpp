#include "hlk_ld2402.h"
#include "esphome/core/log.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace hlk_ld2402 {

static const char *const TAG = "hlk_ld2402";

void HLKLD2402Component::setup() {
  ESP_LOGI(TAG, "Setting up HLK-LD2402 component...");
  this->last_publish_time_ = millis();
  
  // Initialize presence update flag to ensure first reading gets published
  this->has_presence_update_ = true;
  this->last_frame_received_time_ = millis();
  
  // Schedule the engineering mode setup after a delay
  ESP_LOGI(TAG, "Will enable engineering mode after 30 seconds delay...");
  
  // We'll use a delayed setup approach - using the global set_timeout, not the member function
  esphome::set_timeout("engineering_mode_setup", 30000, [this]() {
    this->setup_engineering_mode_();
  });
  
  ESP_LOGI(TAG, "HLK-LD2402 initial setup complete, waiting for engineering mode setup");
}

void HLKLD2402Component::setup_engineering_mode_() {
  ESP_LOGI(TAG, "Starting delayed engineering mode setup now");
  
  // Put the module into engineering mode
  ESP_LOGI(TAG, "Enabling engineering mode...");
  if (enable_configuration_()) {
    ESP_LOGI(TAG, "Configuration mode enabled");
    delay(500);  // Increased delay between commands
    
    if (set_engineering_mode_()) {
      ESP_LOGI(TAG, "Engineering mode set successfully");
    } else {
      ESP_LOGE(TAG, "Failed to set engineering mode");
    }
    
    delay(500);  // Increased delay between commands
    if (exit_configuration_()) {
      ESP_LOGI(TAG, "Exited configuration mode");
    } else {
      ESP_LOGE(TAG, "Failed to exit configuration mode");
    }
  } else {
    ESP_LOGE(TAG, "Failed to enable configuration mode");
  }
  
  ESP_LOGI(TAG, "Engineering mode setup complete");
}

void HLKLD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2402:");
  ESP_LOGCONFIG(TAG, "  Max Distance: %.1fm", this->max_distance_);
  ESP_LOGCONFIG(TAG, "  Distance Update Interval: %dms", this->distance_update_interval_);
  this->check_uart_settings(115200);
}

int HLKLD2402Component::my_available() {
  return this->input_buffer_.size();
}

int HLKLD2402Component::my_read() {
  if (this->input_buffer_.empty()) {
    return -1;
  }
  int value = this->input_buffer_.front();
  this->input_buffer_.erase(this->input_buffer_.begin());
  return value;
}

void HLKLD2402Component::loop() {
  static uint32_t last_log_time = 0;
  uint32_t now = millis();
  
  // Periodically log UART status
  if (now - last_log_time > 5000) { // Log every 5 seconds
    ESP_LOGD(TAG, "Loop running, UART available: %d, Buffer size: %d", 
             uart::UARTDevice::available(), this->input_buffer_.size());
    last_log_time = now;
  }
  
  // Read all available bytes from UART into the input buffer
  int bytes_read = 0;
  while (uart::UARTDevice::available()) {
    uint8_t byte = uart::UARTDevice::read();
    this->input_buffer_.push_back(byte);
    
    // Log every byte in hex format to see what we're receiving
    ESP_LOGV(TAG, "Read byte: 0x%02X", byte);
    
    bytes_read++;
  }
  
  if (bytes_read > 0) {
    ESP_LOGD(TAG, "Read %d bytes from UART", bytes_read);
  }
  
  // Process all bytes in the input buffer
  int bytes_processed = 0;
  while (my_available()) {
    uint8_t byte = my_read();
    bytes_processed++;
    
    // Try to process as binary frame first
    if (process_binary_frame_(byte)) {
      ESP_LOGV(TAG, "Byte 0x%02X processed as part of binary frame", byte);
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
      
      // If we see a newline, process the text buffer
      if (byte == '\n' || byte == '\r') {
        buffer_[buffer_pos_] = '\0';  // Null-terminate
        process_text_buffer_();
        clear_buffer_();
      }
    }
  }
  
  if (bytes_processed > 0) {
    ESP_LOGD(TAG, "Processed %d bytes from buffer", bytes_processed);
  }
  
  // Check if it's time to publish an update
  if (now - this->last_publish_time_ >= this->distance_update_interval_) {
    // Publish distance update if available
    if (this->has_new_data_ && this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_distance(this->last_distance_);
      this->has_new_data_ = false;
      ESP_LOGD(TAG, "Publishing buffered distance: %.2f cm", this->last_distance_);
    }
    
    // Publish presence update if available
    if (this->has_presence_update_ && this->presence_sensor_ != nullptr) {
      this->presence_sensor_->publish_presence(this->presence_detected_);
      this->has_presence_update_ = false;
      ESP_LOGD(TAG, "Publishing presence state: %s", this->presence_detected_ ? "present" : "not present");
    }
    
    this->last_publish_time_ = now;
  }

  // Check for stale presence
  if (this->presence_detected_ && (now - this->last_frame_received_time_ > 5000)) {
    this->presence_detected_ = false;
    this->has_presence_update_ = true;
    ESP_LOGD(TAG, "No binary frame received for 5 seconds, setting presence to not present");
  }
}

bool HLKLD2402Component::process_binary_frame_(uint8_t byte) {
  // Frame header is F4 F3 F2 F1
  static const uint8_t FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
  // Frame footer is F8 F7 F6 F5
  static const uint8_t FRAME_FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

  // Debug current state of binary frame processing
  ESP_LOGV(TAG, "Binary frame state: in_frame=%d, pos=%d, header_pos=%d, byte=0x%02X", 
          in_binary_frame_, binary_buffer_pos_, frame_header_pos_, byte);
  
  // Looking for frame header
  if (!in_binary_frame_) {
    if (byte == FRAME_HEADER[frame_header_pos_]) {
      binary_buffer_[frame_header_pos_] = byte;
      frame_header_pos_++;
      if (frame_header_pos_ == 1) {
        ESP_LOGV(TAG, "Potential binary frame start: 0x%02X", byte);
      }
      if (frame_header_pos_ == 4) {
        // Header found, start collecting frame data
        ESP_LOGI(TAG, "Binary frame header detected: F4 F3 F2 F1");
        in_binary_frame_ = true;
        binary_buffer_pos_ = 4;  // Start after header
        frame_header_pos_ = 0;  // Reset for future headers
      }
      return true;
    } else {
      // Reset header detection if sequence breaks
      if (frame_header_pos_ > 0) {
        ESP_LOGV(TAG, "Frame header sequence broken at position %d, expected 0x%02X but got 0x%02X", 
                frame_header_pos_, FRAME_HEADER[frame_header_pos_], byte);
        frame_header_pos_ = 0;
      }
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
      ESP_LOGD(TAG, "Expected binary frame length: %d", expected_frame_length_);
      
      // Sanity check on frame length
      if (expected_frame_length_ > MAX_BINARY_BUFFER_SIZE - 8) {  // Header (4) + Length (2) + Footer (4) = 10
        ESP_LOGW(TAG, "Invalid frame length: %d", expected_frame_length_);
        reset_binary_frame_();
        return false;
      }
    }

    // If we have status byte (position 6), log it
    if (binary_buffer_pos_ == 7) {
      ESP_LOGD(TAG, "Frame status byte: 0x%02X", binary_buffer_[6]);
    }
    
    // Check for complete frame (header + length field + data + footer)
    if (binary_buffer_pos_ >= 6 + expected_frame_length_) {
      // Just got the last data byte, look for footer now
      ESP_LOGD(TAG, "Received all data bytes, looking for footer");
    }
    
    if (binary_buffer_pos_ >= 6 + expected_frame_length_ + 4) {
      // Check footer
      bool valid_footer = true;
      for (int i = 0; i < 4; i++) {
        if (binary_buffer_[binary_buffer_pos_ - 4 + i] != FRAME_FOOTER[i]) {
          valid_footer = false;
          ESP_LOGW(TAG, "Footer byte %d mismatch: expected 0x%02X, got 0x%02X", 
                  i, FRAME_FOOTER[i], binary_buffer_[binary_buffer_pos_ - 4 + i]);
          break;
        }
      }
      
      if (valid_footer) {
        ESP_LOGI(TAG, "Valid binary frame received, length: %d", binary_buffer_pos_);
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
  
  // Distance is 2 bytes at position 7 (little-endian)
  uint16_t distance = binary_buffer_[7] | (binary_buffer_[8] << 8);
  
  // Update presence based on status (0 = no person, 1 or 2 = person present)
  bool presence = (status == 1 || status == 2);
  
  // Always update presence status for every valid frame
  this->presence_detected_ = presence;
  this->has_presence_update_ = true;
  this->last_frame_received_time_ = millis();
  
  ESP_LOGI(TAG, "Presence status: %s, status byte: %d, distance: %d cm", 
          presence ? "present" : "not present", status, distance);
  
  // Optionally update distance from binary frame too
  float distance_cm = static_cast<float>(distance);
  if (distance > 0 && distance_cm <= this->max_distance_ * 100) {
    ESP_LOGI(TAG, "Binary frame distance: %.1f cm", distance_cm);
    // Update the distance value - previously we weren't doing this
    this->last_distance_ = distance_cm;
    this->has_new_data_ = true;
  }
}

void HLKLD2402Component::reset_binary_frame_() {
  ESP_LOGV(TAG, "Resetting binary frame state");
  in_binary_frame_ = false;
  binary_buffer_pos_ = 0;
  frame_header_pos_ = 0;
  expected_frame_length_ = 0;
}

void HLKLD2402Component::process_text_buffer_() {
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

bool HLKLD2402Component::read_ack_(uint16_t command) {
  std::vector<uint8_t> ack_bytes;
  uint32_t start_time = millis();
  
  for (int i = 0; i < 8; i++) {
    while (this->input_buffer_.empty()) {
      if (millis() - start_time > this->timeout_) {
        ESP_LOGW(TAG, "ACK timeout waiting for byte %d", i + 1);
        return false;
      }
      delay(1);  // Small delay to prevent busy-waiting
      
      // Read any available bytes from UART to the input buffer
      while (uart::UARTDevice::available()) {
        uint8_t byte = uart::UARTDevice::read();
        this->input_buffer_.push_back(byte);
      }
    }
    ack_bytes.push_back(this->input_buffer_.front());
    this->input_buffer_.erase(this->input_buffer_.begin());
  }

  uint16_t received_command = (ack_bytes[5] << 8) | ack_bytes[4];
  uint16_t ack_status = (ack_bytes[7] << 8) | ack_bytes[6];

  if (ack_bytes[0] != 0xFD || ack_bytes[1] != 0xFC || ack_bytes[2] != 0xFB || ack_bytes[3] != 0xFA) {
    ESP_LOGW(TAG, "Invalid ACK header");
    return false;
  }

  if (received_command != (command | 0x0100)) {
    ESP_LOGW(TAG, "Invalid ACK command: expected 0x%04X, got 0x%04X", command | 0x0100, received_command);
    return false;
  }

  if (ack_status != 0x0000) {
    ESP_LOGW(TAG, "Command 0x%04X failed, ACK status: 0x%04X", command, ack_status);
    return false;
  }

  ESP_LOGD(TAG, "Command 0x%04X successful", command);
  return true;
}

bool HLKLD2402Component::enable_configuration_() {
  ESP_LOGD(TAG, "Sending enable_configuration command 0x00FF");
  // Command: 0x00FF with parameter 0x0001
  uint8_t data[2] = {0x01, 0x00};  // Little-endian: 0x0001
  send_command_(0x00FF, data, 2);
  bool result = read_ack_(0x00FF);
  ESP_LOGD(TAG, "enable_configuration command result: %s", result ? "success" : "failed");
  return result;
}

bool HLKLD2402Component::set_engineering_mode_() {
  ESP_LOGD(TAG, "Sending set_engineering_mode command 0x0012");
  
  // According to the documentation, engineering mode parameter is 0x00000004
  uint8_t data[6] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x00};  // Little-endian: 0x00000004
  
  // Log the actual bytes being sent
  ESP_LOGD(TAG, "Engineering mode data: [%02X %02X %02X %02X %02X %02X]",
           data[0], data[1], data[2], data[3], data[4], data[5]);
  
  send_command_(0x0012, data, 6);
  bool result = read_ack_(0x0012);
  ESP_LOGI(TAG, "set_engineering_mode command result: %s", result ? "success" : "failed");
  return result;
}

bool HLKLD2402Component::exit_configuration_() {
  ESP_LOGD(TAG, "Sending exit_configuration command 0x00FE");
  // Command: 0x00FE with no parameters
  send_command_(0x00FE);
  bool result = read_ack_(0x00FE);
  ESP_LOGD(TAG, "exit_configuration command result: %s", result ? "success" : "failed");
  return result;
}

} // namespace hlk_ld2402
} // namespace esphome
