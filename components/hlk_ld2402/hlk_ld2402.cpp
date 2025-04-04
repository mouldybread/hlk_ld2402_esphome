#include "hlk_ld2402.h"

using namespace esphome;

static const char *TAG = "hlk_ld2402";

void HLKLD2402::setup() {
  // Optional: Set initial state if needed
  ESP_LOGI(TAG, "HLK-LD2402 sensor initialized");
}

void HLKLD2402::update() {
  // This will be called every update_interval (default 1 second)
  // We don't need to do anything here as we're reading continuously in loop()
}

void HLKLD2402::loop() {
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
}

void HLKLD2402::process_buffer_() {
  // Convert buffer to string for easier processing
  std::string line(buffer_);
  
  // Trim whitespace
  line.erase(line.find_last_not_of(" \n\r\t") + 1);
  
  // Check if we have a valid distance reading
  if (line.find("Distance:") != std::string::npos) {
    // Extract distance value
    float distance = atof(line.c_str() + 9); // +9 to skip "Distance:"
    
    // Publish the distance value
    distance_sensor_.publish_state(distance);
    
    // Log the updated distance
    ESP_LOGI(TAG, "Distance updated, distance %.2f", distance);
  } else if (line == "OFF") {
    // No target detected
    ESP_LOGI(TAG, "No target detected");
  }
}

void HLKLD2402::clear_buffer_() {
  // Reset buffer
  buffer_pos_ = 0;
  last_read_time_ = millis();
}
