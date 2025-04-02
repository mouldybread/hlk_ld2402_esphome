#include "hlk_ld2402.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2402 {

void HLKLD2402Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2402...");
  
  // Configure UART - explicitly set again
  auto *parent = (uart::UARTComponent *) this->parent_;
  parent->set_baud_rate(115200);
  parent->set_stop_bits(1);
  parent->set_data_bits(8);
  parent->set_parity(esphome::uart::UART_CONFIG_PARITY_NONE);

  // Try sending some test data to verify TX functionality
  write_str("TEST\n");
  ESP_LOGI(TAG, "Sent test string to verify TX line");
  delay(500);
  
  // Flush any residual data
  ESP_LOGI(TAG, "Flushing UART");
  flush();
  delay(100);
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }

  // IMPORTANT ADDITION: Ensure the device starts in normal mode
  // This prevents issues with leftover engineering mode from previous sessions
  ESP_LOGI(TAG, "Setting device to normal mode on startup...");
  
  // Enter config mode - use multiple attempts as device may be in an inconsistent state
  bool config_success = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    ESP_LOGI(TAG, "Startup config mode attempt %d", attempt + 1);
    if (enter_config_mode_()) {
      config_success = true;
      break;
    }
    delay(500);
  }
  
  if (config_success) {
    // Set work mode to normal
    if (set_work_mode_(MODE_NORMAL)) {
      ESP_LOGI(TAG, "Successfully initialized device to normal mode");
    } else {
      ESP_LOGW(TAG, "Failed to set normal mode, but continuing with initialization");
    }
    
    // Always exit config mode
    exit_config_mode_();
    delay(200);
  } else {
    ESP_LOGW(TAG, "Failed to enter config mode to initialize normal mode, continuing anyway");
  }

  // Initialize but don't touch the device if we don't need to
  // The logs show the device is already sending data correctly
  // Skip configuration and just set up sensors
  ESP_LOGI(TAG, "LD2402 appears to be sending data already. Skipping additional configuration.");
  ESP_LOGI(TAG, "Use the Engineering Mode button if you need to modify settings.");
  
  // Don't check power interference immediately - use a delayed operation
  ESP_LOGI(TAG, "Will check power interference and firmware version after 60 seconds");
  
  // Remove the immediate firmware version check - it will happen after 60 seconds
  // get_firmware_version_();
  
  // Set a default version - this will be displayed until we can determine the actual version
  if (firmware_version_text_sensor_ != nullptr) {
    firmware_version_text_sensor_->publish_state("HLK-LD2402");
  }
  
  // Start passive version detection immediately
  begin_passive_version_detection_();
  
  // Set initial operating mode text
  operating_mode_ = "Normal";
  publish_operating_mode_();
  
  // Initialize the throttle timestamp to avoid updates right after boot
  last_distance_update_ = millis();
}

// New function to passively monitor output for version info
void HLKLD2402Component::begin_passive_version_detection_() {
  ESP_LOGI(TAG, "Starting passive version detection");
  firmware_version_ = "HLK-LD2402"; // Default fallback version
}

// Add method to publish operating mode
void HLKLD2402Component::publish_operating_mode_() {
  if (operating_mode_text_sensor_ != nullptr) {
    operating_mode_text_sensor_->publish_state(operating_mode_);
    ESP_LOGI(TAG, "Published operating mode: %s", operating_mode_.c_str());
  }
}

// Update get_firmware_version_ method to use correct command and parsing
void HLKLD2402Component::get_firmware_version_() {
  ESP_LOGI(TAG, "Retrieving firmware version...");
  
  // Clear any pending data
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  bool entered_config_mode = false;
  
  if (!config_mode_) {
    if (!enter_config_mode_()) {
      ESP_LOGW(TAG, "Failed to enter config mode for firmware version check");
      if (firmware_version_text_sensor_ != nullptr) {
        firmware_version_text_sensor_->publish_state("Unknown - Config Failed");
      }
      return;
    }
    entered_config_mode = true;
  }
  
  // Per protocol spec 5.2.1 - use command 0x0000 for firmware version
  if (send_command_(CMD_GET_VERSION)) {
    delay(300);
    
    std::vector<uint8_t> response;
    if (read_response_(response, 1000)) {
      // According to the protocol, response format: 
      // version_length (2 bytes) + version_string (N bytes)
      if (response.size() >= 2) {
        uint16_t version_length = response[0] | (response[1] << 8);
        
        if (response.size() >= 2 + version_length && version_length > 0) {
          std::string version;
          // Extract version string
          for (size_t i = 2; i < 2 + version_length; i++) {
            version += (char)response[i];
          }
          
          firmware_version_ = version;
          ESP_LOGI(TAG, "Got firmware version: %s", version.c_str());
          
          if (firmware_version_text_sensor_ != nullptr) {
            firmware_version_text_sensor_->publish_state(version);
            ESP_LOGI(TAG, "Published firmware version: %s", version.c_str());
          }
        } else {
          ESP_LOGW(TAG, "Invalid version string length in response");
          if (firmware_version_text_sensor_ != nullptr) {
            firmware_version_text_sensor_->publish_state("Invalid Response");
          }
        }
      } else {
        ESP_LOGW(TAG, "Response too short for version data");
        if (firmware_version_text_sensor_ != nullptr) {
          firmware_version_text_sensor_->publish_state("Invalid Response Format");
        }
      }
    } else {
      ESP_LOGW(TAG, "No response to version command");
      if (firmware_version_text_sensor_ != nullptr) {
        firmware_version_text_sensor_->publish_state("No Response");
      }
    }
  } else {
    ESP_LOGW(TAG, "Failed to send version command");
    if (firmware_version_text_sensor_ != nullptr) {
      firmware_version_text_sensor_->publish_state("Command Failed");
    }
  }
  
  // Exit config mode if we entered it
  if (entered_config_mode) {
    exit_config_mode_();
  }
}

void HLKLD2402Component::loop() {
  static uint32_t last_byte_time = 0;
  static uint32_t last_process_time = 0; // Add throttling timer
  static const uint32_t PROCESS_INTERVAL = 2000; // Only process lines every 2 seconds
  static const uint32_t TIMEOUT_MS = 100; // Reset buffer if no data for 100ms
  static uint32_t last_debug_time = 0;
  static uint8_t raw_buffer[16];
  static size_t raw_pos = 0;
  static uint32_t last_status_time = 0;
  static uint32_t byte_count = 0;
  static uint8_t last_bytes[16] = {0};
  static size_t last_byte_pos = 0;
  static bool firmware_check_done = false;
  static bool power_check_done = false;
  static uint32_t startup_time = millis();
  static uint32_t last_eng_debug_time = 0;
  static uint32_t eng_mode_start_time = 0;
  static uint32_t last_eng_retry_time = 0;
  static uint8_t eng_retry_count = 0;
  static uint32_t firmware_check_time = 0;  // New variable to track when firmware check completes
  
  // Firmware version check earlier at 20 seconds to avoid conflict with power check
  if (!firmware_check_done && (millis() - startup_time) > 20000) {
    ESP_LOGI(TAG, "Performing firmware version check...");
    get_firmware_version_();
    firmware_check_done = true;
    firmware_check_time = millis();  // Record when firmware check completes
  }
  
  // Power interference check 3 seconds after firmware check completes
  if (firmware_check_done && !power_check_done && 
      firmware_check_time > 0 && (millis() - firmware_check_time) > 3000) {
    ESP_LOGI(TAG, "Performing power interference check...");
    check_power_interference();
    power_check_done = true;
  }
  
  // Add periodic debug message - reduce frequency
  if (millis() - last_debug_time > 30000) {  // Every 30 seconds
    ESP_LOGD(TAG, "Waiting for data. Available bytes: %d", available());
    last_debug_time = millis();
  }
  
  // Every 10 seconds, report status
  if (millis() - last_status_time > 10000) {
    ESP_LOGI(TAG, "Status: received %u bytes in last 10 seconds", byte_count);
    if (byte_count > 0) {
      char hex_buf[50] = {0};
      char ascii_buf[20] = {0};
      for (int i = 0; i < 16 && i < byte_count; i++) {
        sprintf(hex_buf + (i*3), "%02X ", last_bytes[i]);
        sprintf(ascii_buf + i, "%c", (last_bytes[i] >= 32 && last_bytes[i] < 127) ? last_bytes[i] : '.');
      }
      ESP_LOGI(TAG, "Last bytes (hex): %s", hex_buf);
      ESP_LOGI(TAG, "Last bytes (ascii): %s", ascii_buf);
    }
    byte_count = 0;
    last_status_time = millis();
  }
  
  // Add this at the beginning of the loop method
  if (operating_mode_ == "Engineering" && (millis() - last_eng_debug_time) > 5000) {
    ESP_LOGI(TAG, "Currently in engineering mode, waiting for data frames. Data enabled: %s, Sensors configured: %d",
             engineering_data_enabled_ ? "YES" : "NO", energy_gate_sensors_.size());
    last_eng_debug_time = millis();
  }
  
  // If we're in engineering mode, monitor if we're receiving data
  if (operating_mode_ == "Engineering") {
    uint32_t now = millis();
    
    // Set the start time if not set
    if (eng_mode_start_time == 0) {
      eng_mode_start_time = now;
    }
    
    // Check if we're getting data in engineering mode
    if ((now - last_status_time > 10000) && byte_count == 0) {
      // No bytes received in 10 seconds while in engineering mode
      if (now - last_eng_retry_time > 15000 && eng_retry_count < 3) {
        // Try re-triggering engineering mode data every 15 seconds, up to 3 times
        ESP_LOGW(TAG, "No data received in engineering mode - attempting to re-trigger data flow (attempt %d/3)", 
                eng_retry_count + 1);
        
        // Send a parameter read command which might trigger data flow
        uint8_t param_data[2];
        param_data[0] = 0x01;  // Max distance parameter
        param_data[1] = 0x00;
        send_command_(CMD_GET_PARAMS, param_data, sizeof(param_data));
        
        // Increment retry counter and update timestamp
        eng_retry_count++;
        last_eng_retry_time = now;
      }
      else if (eng_retry_count >= 3 && now - eng_mode_start_time > 60000) {
        // If we've been in engineering mode for over a minute with no data after 3 retries
        ESP_LOGW(TAG, "Engineering mode not producing data frames after multiple retries. Try power cycling the device.");
      }
    }
    else if (byte_count > 0) {
      // Reset retry counter if we got data
      eng_retry_count = 0;
    }
  }
  else {
    // Reset engineering mode tracking variables when not in engineering mode
    eng_mode_start_time = 0;
    eng_retry_count = 0;
    last_eng_retry_time = 0;
  }
  
  // Add this at the beginning of the loop method for additional safety
  if (operating_mode_ != "Engineering" && engineering_data_enabled_) {
    ESP_LOGI(TAG, "Detected inconsistent state: engineering data enabled but not in engineering mode. Fixing...");
    engineering_data_enabled_ = false;
  }
  
  while (available()) {
    uint8_t c;
    read_byte(&c);
    last_byte_time = millis();
    byte_count++;
    
    // Record last bytes for diagnostics
    last_bytes[last_byte_pos] = c;
    last_byte_pos = (last_byte_pos + 1) % 16;
    
    // Debug: Collect raw data
    raw_buffer[raw_pos++] = c;
    if (raw_pos >= sizeof(raw_buffer)) {
      raw_pos = 0;
    }
    
    // FIRST CHECK: Check for data frame header (F4 F3 F2 F1)
    if (c == DATA_FRAME_HEADER[0]) {
      // Need to check if this is actually a data frame
      if (available() >= 4) { // Need at least 4 more bytes to verify the header and frame type
        // Peek at the next 4 bytes to check for frame header and type
        bool is_data_frame = true;
        uint8_t peek_bytes[4]; // Header + frame type
        
        // Read the next 4 bytes without removing them from buffer
        for (int i = 0; i < 4; i++) {
          if (!available()) {
            is_data_frame = false;
            break;
          }
          
          read_byte(&peek_bytes[i]);
          
          // First 3 bytes should match header, 4th byte is frame type
          if (i < 3 && peek_bytes[i] != DATA_FRAME_HEADER[i+1]) {
            is_data_frame = false;
            break;
          }
        }
        
        if (is_data_frame) {
          // We have a proper data frame header! Collect the whole frame
          // The 4th byte is the frame type (0x83 for distance data, 0x84 for engineering data)
          uint8_t frame_type = peek_bytes[3];
          
          // Add more verbose logging for engineering mode
          if (operating_mode_ == "Engineering") {
            ESP_LOGI(TAG, "In engineering mode, received frame type: 0x%02X", frame_type);
          }
          
          std::vector<uint8_t> frame_data;
          
          // Add first 5 bytes (4 header + frame type)
          frame_data.push_back(c);  // First byte (already read)
          for (int i = 0; i < 4; i++) {
            frame_data.push_back(peek_bytes[i]);
          }
          
          // Read up to 100 bytes to capture the entire frame
          // This limit is just a safety mechanism - we'll read until we have a complete frame
          size_t max_frame_size = 100;
          size_t bytes_read = 0;
          
          while (available() && bytes_read < max_frame_size) {
            uint8_t data_byte;
            read_byte(&data_byte);
            frame_data.push_back(data_byte);
            bytes_read++;
          }
          
          // Process the data frame based on frame_type with additional checks
          if (frame_type == DATA_FRAME_TYPE_DISTANCE) {
            // MODIFICATION: If in engineering mode, process 0x83 frames as engineering data
            if (operating_mode_ == "Engineering") {
              ESP_LOGI(TAG, "Processing distance frame (0x83) as engineering data in engineering mode");
              if (process_engineering_from_distance_frame_(frame_data)) {
                // Successfully processed as engineering data
                ESP_LOGD(TAG, "Successfully processed 0x83 frame as engineering data");
              } else {
                ESP_LOGW(TAG, "Failed to process 0x83 frame as engineering data");
              }
            } else if (process_distance_frame_(frame_data)) {
              // Successfully processed as normal distance frame
            } else {
              ESP_LOGW(TAG, "Failed to process distance data frame");
            }
          } else if (frame_type == DATA_FRAME_TYPE_ENGINEERING) {
            if (process_engineering_data_(frame_data)) {
              ESP_LOGD(TAG, "Successfully processed engineering data frame");
            } else {
              ESP_LOGV(TAG, "Failed to process engineering data frame");
            }
          } else {
            ESP_LOGD(TAG, "Unknown frame type: 0x%02X", frame_type);
          }
          
          continue; // Skip further processing for this byte
        } else {
          // Not a valid data frame, add all read bytes to line buffer
          line_buffer_ += (char)c;
          for (int i = 0; i < 4 && !is_data_frame; i++) {
            line_buffer_ += (char)peek_bytes[i];
          }
        }
      }
    }
    
    // SECOND CHECK: Check for command/response frame header (FD FC FB FA)
    else if (c == FRAME_HEADER[0]) {
      // Only consider it a frame header if we have enough bytes available
      // ...existing code for checking command frame...
    }
    
    // Check for text data - add to line buffer
    if (c == '\n') {
      // Process complete line
      if (!line_buffer_.empty()) {
        bool should_process = millis() - last_process_time >= PROCESS_INTERVAL;
        
        if (should_process) {
          last_process_time = millis();
          
          // Less restrictive binary check - only look for obviously non-text chars
          bool is_binary = false;
          for (char ch : line_buffer_) {
            // Only consider control chars below space as binary (except tab and CR)
            if (ch < 32 && ch != '\t' && ch != '\r' && ch != '\n') {
              // Count actual binary characters
              int binary_count = 0;
              for (char c2 : line_buffer_) {
                if (c2 < 32 && c2 != '\t' && c2 != '\r' && c2 != '\n') {
                  binary_count++;
                }
              }
              
              // Only mark as binary if we have several binary chars (>25%)
              if (binary_count > line_buffer_.length() / 4) {
                is_binary = true;
                break;
              }
            }
          }
          
          // Debug - show the line data regardless
          ESP_LOGI(TAG, "Received line [%d bytes]: '%s'", line_buffer_.length(), line_buffer_.c_str());
          if (!is_binary) {
            process_line_(line_buffer_);
          } else {
            ESP_LOGD(TAG, "Skipped binary data that looks like a protocol frame");
            
            // Debug: Show hex representation of binary data
            char hex_buf[128] = {0};
            for (size_t i = 0; i < std::min(line_buffer_.length(), size_t(32)); i++) {
              sprintf(hex_buf + (i*3), "%02X ", (uint8_t)line_buffer_[i]);
            }
            ESP_LOGD(TAG, "Binary data hex: %s", hex_buf);
          }
        }
        line_buffer_.clear();
      }
    } else if (c != '\r') {  // Skip \r
      if (line_buffer_.length() < 1024) {
        line_buffer_ += (char)c;
        
        // Added: Check for direct "distance:" line without proper termination
        if (line_buffer_.length() >= 12 && 
            line_buffer_.compare(line_buffer_.length() - 12, 9, "distance:") == 0) {
          // We found a distance prefix - process the previous data if any
          std::string prev_data = line_buffer_.substr(0, line_buffer_.length() - 12);
          if (!prev_data.empty()) {
            ESP_LOGI(TAG, "Found distance prefix, processing previous data: '%s'", prev_data.c_str());
            process_line_(prev_data);
          }
          // Keep only the distance part
          line_buffer_ = line_buffer_.substr(line_buffer_.length() - 12);
        }
      } else {
        ESP_LOGW(TAG, "Line buffer overflow, clearing");
        line_buffer_.clear();
      }
    }
    
    // Additional processing in loop to passively detect version info
    // from normal operation output, even after initial check
    if (!firmware_version_.empty() && firmware_version_ != "Unknown" && 
        firmware_version_.find("HLK-LD2402") == 0 && 
        firmware_version_.find("v") == std::string::npos) {
      
      // We only have model info, still looking for version number
      if (c == '\n' && !line_buffer_.empty()) {
        if (line_buffer_.find("v") != std::string::npos || 
            line_buffer_.find("V") != std::string::npos ||
            line_buffer_.find("version") != std::string::npos ||
            line_buffer_.find("Version") != std::string::npos) {
          
          ESP_LOGI(TAG, "Found potential version info: %s", line_buffer_.c_str());
          // Extract version information
          std::string version = "HLK-LD2402";
          
          // Try to find version number pattern
          for (size_t i = 0; i < line_buffer_.length(); i++) {
            if ((i+2 < line_buffer_.length() && 
                isdigit(line_buffer_[i]) && 
                line_buffer_[i+1] == '.' && 
                isdigit(line_buffer_[i+2])) ||
                (line_buffer_[i] == 'v' || line_buffer_[i] == 'V')) {
              
              size_t start_pos = line_buffer_[i] == 'v' || line_buffer_[i] == 'V' ? i+1 : i;
              size_t end_pos = line_buffer_.find_first_not_of("0123456789.", start_pos);
              if (end_pos == std::string::npos) end_pos = line_buffer_.length();
              
              version = "v" + line_buffer_.substr(start_pos, end_pos - start_pos);
              firmware_version_ = version;
              
              if (firmware_version_text_sensor_ != nullptr) {
                firmware_version_text_sensor_->publish_state(version);
                ESP_LOGI(TAG, "Updated firmware version from passive detection: %s", version.c_str());
              }
              break;
            }
          }
        }
      }
    }
  }
  
  // Reset buffer if no data received for a while
  if (!line_buffer_.empty() && (millis() - last_byte_time > TIMEOUT_MS)) {
    line_buffer_.clear();
  }

  // Check calibration progress if needed
  if (calibration_in_progress_ && calibration_progress_sensor_ != nullptr) {
    uint32_t now = millis();
    if (now - last_calibration_check_ >= 5000) { // Check every 5 seconds
      last_calibration_check_ = now;
      
      if (send_command_(CMD_GET_CALIBRATION_STATUS)) {
        std::vector<uint8_t> response;
        if (read_response_(response)) {
          // Log the complete response for debugging
          char hex_buf[64] = {0};
          for (size_t i = 0; i < response.size() && i < 16; i++) {
            sprintf(hex_buf + (i*3), "%02X ", response[i]);
          }
          ESP_LOGD(TAG, "Calibration status response: %s", hex_buf);
          
          bool handled = false; // Track if we've handled the response
          
          // Handle the actual device response format which differs from the documentation
          // Expected format: 06 00 0A 01 00 00 XX 00 - where XX is the progress value
          if (!handled && response.size() >= 8) {
            // Check for a response that matches the observed pattern
            if (response[0] == 0x06 && response[1] == 0x00 &&
                response[2] == 0x0A && response[3] == 0x01) {
              
              // Extract progress from position 6
              uint16_t progress = response[6]; // Use only the progress byte
              
              ESP_LOGD(TAG, "Raw progress value: 0x%02X (%u)", progress, progress);
              
              // Convert to percentage - appears to be counting up to 100 (0x64)
              uint16_t percentage = (progress * 100) / 0x64;
              
              // Cap to 100% 
              if (percentage > 100) {
                percentage = 100;
              }
              
              ESP_LOGI(TAG, "Calibration progress: %u%% (raw value: %u)", 
                      percentage, progress);
              calibration_progress_ = percentage;
              
              if (this->calibration_progress_sensor_ != nullptr) {
                this->calibration_progress_sensor_->publish_state(percentage);
              }
              
              // Check if calibration is complete (progress value reaches 0x64)
              if (progress >= 0x64) {
                ESP_LOGI(TAG, "Calibration complete");
                calibration_in_progress_ = false;
                exit_config_mode_();
              }
              
              // We successfully processed the response
              handled = true;  // Mark as handled instead of using continue
            }
          }
          
          // Handle the documented response format just in case
          if (!handled && response.size() >= 2 && response[0] == 0x00 && response[1] == 0x00) {
            // According to protocol section 5.2.10:
            // Response format includes 2 bytes ACK status (00 00) followed by 2 bytes percentage
            if (response.size() >= 4) {
              // Read percentage value - little endian (LSB first)
              uint16_t progress = response[2] | (response[3] << 8);
              
              ESP_LOGD(TAG, "Raw progress value (standard format): 0x%04X (%u)", progress, progress);
              
              // Sanity check: ensure progress is 0-100
              if (progress > 100) {
                ESP_LOGW(TAG, "Invalid calibration progress value: %u, capping to 100", progress);
                progress = 100;
              }
              
              calibration_progress_ = progress;
              ESP_LOGI(TAG, "Calibration progress: %u%% (standard format)", progress);
              this->calibration_progress_sensor_->publish_state(progress);
              
              // Check if calibration is complete
              if (progress >= 100) {
                ESP_LOGI(TAG, "Calibration complete");
                calibration_in_progress_ = false;
                exit_config_mode_();
              }
              
              handled = true;  // Mark as handled instead of using continue
            }
          }
          
          // If we reach here and haven't handled the response, show a warning
          if (!handled) {
            ESP_LOGW(TAG, "Unrecognized calibration status response format. Raw bytes:");
            for (size_t i = 0; i < response.size() && i < 16; i++) {
              ESP_LOGW(TAG, "  Byte[%d] = 0x%02X", i, response[i]);
            }
          }
        } else {
          ESP_LOGW(TAG, "No response to calibration status query");
        }
      } else {
        ESP_LOGW(TAG, "Failed to send calibration status query");
      }
    }
  }
}

// Add new method to parse distance data frames
bool HLKLD2402Component::process_distance_frame_(const std::vector<uint8_t> &frame_data) {
  // Regular distance frame processing for normal mode
  // Ensure the frame is at least the minimum expected length
  if (frame_data.size() < 10) {
    ESP_LOGW(TAG, "Distance frame too short: %d bytes", frame_data.size());
    return false;
  }
  
  // Additional verification: double-check that this is really a distance frame
  // by confirming the frame type byte (should be 0x83)
  if (frame_data.size() >= 5 && frame_data[4] != DATA_FRAME_TYPE_DISTANCE) {
    ESP_LOGW(TAG, "Not a distance frame type: 0x%02X", frame_data[4]);
    return false;
  }

  // Enhanced frame debugging with clear annotation of each byte's position
  ESP_LOGI(TAG, "===== RADAR FRAME ANALYSIS =====");
  ESP_LOGI(TAG, "Frame size: %d bytes", frame_data.size());
  ESP_LOGI(TAG, "Expected format: [F4 F3 F2 F1][83][LEN-L][LEN-H][STATUS][DATA...]");
  
  // Output bytes with indices for easier analysis
  char hex_buf[256] = {0};
  char index_buf[256] = {0};
  
  // Generate index markers
  for (size_t i = 0; i < std::min(frame_data.size(), size_t(30)); i++) {
    sprintf(index_buf + (i*3), "%2d ", (int)i);
  }
  ESP_LOGI(TAG, "Byte idx: %s", index_buf);
  
  // Generate hex values
  for (size_t i = 0; i < std::min(frame_data.size(), size_t(30)); i++) {
    sprintf(hex_buf + (i*3), "%02X ", frame_data[i]);
  }
  ESP_LOGI(TAG, "Hex data: %s", hex_buf);
  
  // Annotate specific bytes with their meanings
  ESP_LOGI(TAG, "Interpretation:");
  ESP_LOGI(TAG, "- Bytes 0-3: Header = %02X %02X %02X %02X (should be F4 F3 F2 F1)", 
           frame_data[0], frame_data[1], frame_data[2], frame_data[3]);
  ESP_LOGI(TAG, "- Byte  4: Type = %02X (83 = distance frame)", 
           frame_data[4]);
  ESP_LOGI(TAG, "- Bytes 5-6: Length = %02X %02X (%d bytes)", 
           frame_data[5], frame_data[6], frame_data[5] | (frame_data[6] << 8));
  
  // Let's check EVERY possible detection status byte position from 7-10
  for (int i = 7; i < std::min(size_t(11), frame_data.size()); i++) {
    ESP_LOGI(TAG, "- Byte %2d: Value = %02X (if status byte: %s)", 
             i, frame_data[i],
             frame_data[i] == 0 ? "no person" : 
             frame_data[i] == 1 ? "person moving" : 
             frame_data[i] == 2 ? "stationary person" : "unknown");
  }
  
  // Parse the data length from the frame
  uint16_t data_length = frame_data[5] | (frame_data[6] << 8);
  ESP_LOGI(TAG, "Frame data length: %u bytes", data_length);
  
  // Check if we have at least enough data for distance values
  if (frame_data.size() < 14) {
    ESP_LOGW(TAG, "Frame too short to contain distance data");
    return false;
  }
  
  // We'll use the first non-zero value as our distance
  float min_distance_cm = 0;
  
  // Start at byte 13 (index 12) and look for the first valid distance
  for (size_t i = 12; i + 3 < frame_data.size(); i += 4) {
    uint32_t value = frame_data[i] | 
                    (frame_data[i+1] << 8) | 
                    (frame_data[i+2] << 16) | 
                    (frame_data[i+3] << 24);
                    
    // If the value is non-zero, convert to distance
    if (value > 0) {
      float distance = value * 0.1f; // Convert to cm
      
      // If this is the first valid value, or it's closer than our current minimum
      if (min_distance_cm == 0 || distance < min_distance_cm) {
        min_distance_cm = distance;
      }
      
      ESP_LOGI(TAG, "Distance value at pos %d: %.1f cm (raw: %u)", i, distance, value);
      
      // Find just one valid value, then break
      if (min_distance_cm > 0) {
        break;
      }
    }
  }
  
  // If we found a valid distance
  if (min_distance_cm > 0) {
    // Position 6 contains the presence indicator - 0x00 means no presence,
    // non-zero values indicate presence detected
    uint8_t detection_status = (frame_data.size() > 6) ? frame_data[6] : 0;
    
    // Get byte 7 value for motion detection (if available)
    uint8_t motion_value = (frame_data.size() > 7) ? frame_data[7] : 0;
    
    // Define status_text based on detection_status
    // NOTE: Despite what the manual says about status values 0=none, 1=moving, 2=stationary,
    // observations show the device only ever returns 0x00 or 0x01 in practice
    const char* status_text;
    if (detection_status == 0) {
      status_text = "no presence";
    } else if (detection_status == 1) {
      status_text = "person moving";
    } else if (detection_status == 2) {
      status_text = "stationary person"; // Never seen in practice
    } else {
      status_text = "unknown status";
    }
    
    // Log all values for debugging
    if (frame_data.size() > 9) {
      ESP_LOGI(TAG, "Frame bytes 5-9: %02X %02X %02X %02X %02X (byte 6 = status: %s, byte 7 = %u)",
               frame_data[5], frame_data[6], frame_data[7], frame_data[8], frame_data[9],
               detection_status == 0 ? "no presence" : "presence", motion_value);
    }
    
    // Update presence binary sensor
    update_binary_sensors_(min_distance_cm, detection_status);
    
    // Update motion binary sensor based on byte 7 value
    if (this->motion_binary_sensor_ != nullptr) {
      bool is_motion = (motion_value > 100);
      this->motion_binary_sensor_->publish_state(is_motion);
      ESP_LOGI(TAG, "Motion detection: %s (byte 7 value: %u)", is_motion ? "ACTIVE" : "INACTIVE", motion_value);
    }
    
    // For distance sensor, check throttling
    uint32_t now = millis(); // Add this line to get current time
    bool throttled = (this->distance_sensor_ != nullptr && 
                     now - last_distance_update_ < distance_throttle_ms_);
    
    // Only update distance sensor if not throttled
    if (!throttled && this->distance_sensor_ != nullptr) {
      static float last_reported_distance = 0;
      bool significant_change = fabsf(min_distance_cm - last_reported_distance) > 10.0f;
      
      if (significant_change) {
        ESP_LOGI(TAG, "Detected %s at distance (binary): %.1f cm", status_text, min_distance_cm);
        last_reported_distance = min_distance_cm;
      } else {
        ESP_LOGV(TAG, "Detected %s at distance (binary): %.1f cm", status_text, min_distance_cm);
      }
      
      this->distance_sensor_->publish_state(min_distance_cm);
      last_distance_update_ = now;
      ESP_LOGD(TAG, "Updated distance sensor");
    }
    
    return true;
  }
  
  return false;  // No valid distance found
}

// Add new method to process engineering data from 0x83 frames
bool HLKLD2402Component::process_engineering_from_distance_frame_(const std::vector<uint8_t> &frame_data) {
  // Early exit if engineering data processing is not enabled
  if (!engineering_data_enabled_) {
    ESP_LOGD(TAG, "Engineering data processing disabled");
    return false;
  }
  
  if (energy_gate_sensors_.empty()) {
    ESP_LOGD(TAG, "No energy gate sensors configured");
    return false;
  }
  
  // Ensure the frame is at least the minimum expected length
  // Header (5) + Length (2) + Some data
  if (frame_data.size() < 10) {
    ESP_LOGW(TAG, "Engineering frame too short: %d bytes", frame_data.size());
    return false;
  }
  
  // Check throttling - only log and update sensors if enough time has passed
  uint32_t now = millis();
  bool throttled = (now - last_engineering_update_ < engineering_throttle_ms_);
  
  if (!throttled) {
    // Only log the frame when not throttled
    char hex_buf[256] = {0};
    for (size_t i = 0; i < std::min(frame_data.size(), size_t(50)); i++) {
      sprintf(hex_buf + (i*3), "%02X ", frame_data[i]);
    }
    ESP_LOGI(TAG, "Processing 0x83 frame as engineering data: %s", hex_buf);
  }
  
  // For 0x83 frames in engineering mode, the energy values start at byte 13
  // Energy values are the same values that would be "distance" in normal mode
  const size_t motion_energy_start = 13;  // Start offset for energy values
  const size_t motion_gate_count = DEFAULT_GATES;    // Should use DEFAULT_GATES for consistency
  
  // Ensure we have enough data
  if (frame_data.size() < motion_energy_start + 4) {
    ESP_LOGW(TAG, "Frame too short for energy data");
    return false;
  }
  
  // Process each gate's energy value
  for (uint8_t i = 0; i < motion_gate_count; i++) {
    size_t offset = motion_energy_start + (i * 4);
    
    // Make sure we have enough data for this gate
    if (offset + 3 >= frame_data.size()) {
      ESP_LOGW(TAG, "Engineering frame truncated at gate %d", i);
      break;
    }
    
    // Extract 32-bit energy value (little-endian)
    uint32_t raw_energy = frame_data[offset] | 
                        (frame_data[offset+1] << 8) | 
                        (frame_data[offset+2] << 16) | 
                        (frame_data[offset+3] << 24);
    
    // Convert raw energy to dB as per manual: dB = 10 * log10(raw_value)
    // Only calculate for non-zero values to avoid log(0)
    float db_energy = 0;
    if (raw_energy > 0) {
      db_energy = 10.0f * log10f(raw_energy);
    }
    
    // Calculate approximate distance for this gate
    float gate_start_distance = i * DISTANCE_GATE_SIZE;
    float gate_end_distance = gate_start_distance + DISTANCE_GATE_SIZE;
    
    // Update sensor if configured for this gate and not throttled
    if (!throttled && i < energy_gate_sensors_.size() && energy_gate_sensors_[i] != nullptr) {
      energy_gate_sensors_[i]->publish_state(db_energy);
      
      // Only log gate data when not throttled
      ESP_LOGI(TAG, "Gate %d (%.1f-%.1f m) energy: %.1f dB (raw: %u)", 
              i, gate_start_distance, gate_end_distance, db_energy, raw_energy);
    }
  }
  
  // Update the throttle timestamp if we weren't throttled
  if (!throttled) {
    last_engineering_update_ = now;
  }
  
  return true;
}

// Add new method to process engineering data
bool HLKLD2402Component::process_engineering_data_(const std::vector<uint8_t> &frame_data) {
  // Early exit if engineering data processing is not enabled
  if (!engineering_data_enabled_) {
    ESP_LOGD(TAG, "Engineering data processing disabled");
    return false;
  }
  
  if (energy_gate_sensors_.empty()) {
    ESP_LOGD(TAG, "No energy gate sensors configured");
    return false;
  }
  
  // Make sure we're in engineering mode
  // NOTE: Changed this to be more permissive - removed strict config_mode check
  if (operating_mode_ != "Engineering") {
    ESP_LOGW(TAG, "Received engineering data frame but not in engineering mode! Current mode: %s", 
            operating_mode_.c_str());
    return false;
  }

  // Ensure the frame is at least the minimum expected length
  // Header (5) + Length (2) + Some data
  if (frame_data.size() < 10) {
    ESP_LOGW(TAG, "Engineering frame too short: %d bytes", frame_data.size());
    return false;
  }
  
  // Check throttling - only log and update sensors if enough time has passed
  uint32_t now = millis();
  bool throttled = (now - last_engineering_update_ < engineering_throttle_ms_);
  
  if (!throttled) {
    // Only log the frame when not throttled
    char hex_buf[128] = {0};
    for (size_t i = 0; i < std::min(frame_data.size(), size_t(40)); i++) {
      sprintf(hex_buf + (i*3), "%02X ", frame_data[i]);
    }
    ESP_LOGI(TAG, "Engineering frame received: %s", hex_buf);
  }
  
  // Verify frame type is engineering data (0x84)
  if (frame_data.size() >= 5 && frame_data[4] != DATA_FRAME_TYPE_ENGINEERING) {
    ESP_LOGW(TAG, "Not an engineering data frame: 0x%02X", frame_data[4]);
    return false;
  }

  // Process each gate's energy value
  const size_t motion_energy_start = 10;
  const size_t motion_gate_count = DEFAULT_GATES; // Uses DEFAULT_GATES gates from the constant
  
  for (uint8_t i = 0; i < motion_gate_count; i++) {
    size_t offset = motion_energy_start + (i * 4);
    
    // Make sure we have enough data for this gate
    if (offset + 3 >= frame_data.size()) {
      ESP_LOGW(TAG, "Engineering frame truncated at gate %d", i);
      break;
    }
    
    // Extract 32-bit energy value (little-endian)
    uint32_t raw_energy = frame_data[offset] | 
                        (frame_data[offset+1] << 8) | 
                        (frame_data[offset+2] << 16) | 
                        (frame_data[offset+3] << 24);
    
    // Convert raw energy to dB as per manual: dB = 10 * log10(raw_value)
    float db_energy = 0;
    if (raw_energy > 0) { // Avoid log10(0)
      db_energy = 10.0f * log10f(raw_energy);
    }
    
    // Calculate approximate distance for this gate
    float gate_start_distance = i * DISTANCE_GATE_SIZE;
    float gate_end_distance = gate_start_distance + DISTANCE_GATE_SIZE;
    
    // Update sensor if configured for this gate and not throttled
    if (!throttled && i < energy_gate_sensors_.size() && energy_gate_sensors_[i] != nullptr) {
      energy_gate_sensors_[i]->publish_state(db_energy);
      
      // Only log gate data when not throttled
      ESP_LOGI(TAG, "Gate %d (%.1f-%.1f m) energy: %.1f dB (raw: %u)", 
              i, gate_start_distance, gate_end_distance, db_energy, raw_energy);
    }
  }
  
  // Update the throttle timestamp if we weren't throttled
  if (!throttled) {
    last_engineering_update_ = now;
  }
  
  return true;
}

// Create a separate method for updating binary sensors to avoid code duplication
void HLKLD2402Component::update_binary_sensors_(float distance_cm, uint8_t detection_status) {
  // Update presence based on detection status
  if (this->presence_binary_sensor_ != nullptr) {
    // The byte at position 6 in the frame indicates presence:
    // - Value 0x00: No presence detected
    // - Non-zero values: Presence detected
    bool is_presence = (detection_status > 0);
    
    // Log before updating to ensure we see what's happening
    ESP_LOGI(TAG, "Updating presence sensor to %s (status code 0x%02X) at distance %.1f cm", 
             is_presence ? "ON" : "OFF", detection_status, distance_cm);
    
    this->presence_binary_sensor_->publish_state(is_presence);
    
    if (is_presence) {
      ESP_LOGI(TAG, "Presence detected: status code 0x%02X at %.1f cm", detection_status, distance_cm);
    } else {
      ESP_LOGI(TAG, "No presence detected at %.1f cm", distance_cm);
    }
  } else {
    ESP_LOGW(TAG, "Presence binary sensor not configured!");
  }
}

// Replace the damaged process_line_ method
void HLKLD2402Component::process_line_(const std::string &line) {
  ESP_LOGD(TAG, "Processing line: '%s'", line.c_str());
  
  // Handle OFF status
  if (line == "OFF") {
    ESP_LOGI(TAG, "No target detected");
    if (this->presence_binary_sensor_ != nullptr) {
      this->presence_binary_sensor_->publish_state(false);
    }
        
    // Only update the distance sensor if not throttled
    uint32_t now = millis();
    bool throttled = (this->distance_sensor_ != nullptr && 
                     now - last_distance_update_ < distance_throttle_ms_);
    
    if (!throttled && this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_state(0);
      last_distance_update_ = now; // Update timestamp for throttling
    }
    return;
  }

  // Handle different formats of distance data
  float distance_cm = 0;
  bool valid_distance = false;
  
  // Check if line contains "distance:" anywhere in the string
  size_t dist_pos = line.find("distance:");
  if (dist_pos != std::string::npos) {
    // Extract everything after "distance:"
    std::string distance_str = line.substr(dist_pos + 9);
    ESP_LOGV(TAG, "Found distance data: '%s'", distance_str.c_str());
    
    // Remove any trailing non-numeric characters
    size_t pos = distance_str.find_first_not_of("0123456789.");
    if (pos != std::string::npos) {
      distance_str = distance_str.substr(0, pos);
    }
    
    char *end;
    float distance = strtof(distance_str.c_str(), &end);
    
    if (end != distance_str.c_str()) {
      distance_cm = distance;
      valid_distance = true;
    }
  } else {
    // Try parsing just a number (some devices output just the number)
    bool is_numeric = true;
    for (char ch : line) {
      if (!isdigit(ch) && ch != '.') {
        is_numeric = false;
        break;
      }
    }
    
    if (is_numeric && !line.empty()) {
      char *end;
      float distance = strtof(line.c_str(), &end);
      
      if (end != line.c_str()) {
        distance_cm = distance;
        valid_distance = true;
        
        ESP_LOGV(TAG, "Detected numeric distance: %.1f cm", distance_cm);
      }
    }
  }
  
  if (valid_distance) {
    // Check if the line contains presence information
    uint8_t detection_status = 0;
    
    // Look for presence indicators in the line
    if (line.find("presence") != std::string::npos || 
        line.find("detected") != std::string::npos || 
        line.find("person") != std::string::npos) {
      // Default to moving person if presence is indicated
      detection_status = 1;
      
      // Check for stationary indicators
      if (line.find("stationary") != std::string::npos || 
          line.find("static") != std::string::npos) {
        detection_status = 2;
      }
    }
    
    // Always update binary sensors with detected status
    update_binary_sensors_(distance_cm, detection_status);
    
    // For distance sensor, apply throttling
    uint32_t now = millis();
    bool throttled = (this->distance_sensor_ != nullptr && 
                     now - last_distance_update_ < distance_throttle_ms_);
    
    if (!throttled && this->distance_sensor_ != nullptr) {
      // Use verbose level for regular updates, INFO only for significant changes
      static float last_reported_distance = 0;
      bool significant_change = fabsf(distance_cm - last_reported_distance) > 10.0f;
      
      if (significant_change) {
        ESP_LOGI(TAG, "Detected distance (text): %.1f cm", distance_cm);
        last_reported_distance = distance_cm;
      } else {
        ESP_LOGV(TAG, "Detected distance (text): %.1f cm", distance_cm);
      }
      
      this->distance_sensor_->publish_state(distance_cm);
      last_distance_update_ = now;
    }
  }
}

void HLKLD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2402:");
  ESP_LOGCONFIG(TAG, "  Firmware Version: %s", firmware_version_.c_str());
  ESP_LOGCONFIG(TAG, "  Max Distance: %.1f m", max_distance_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u s", timeout_);
}

bool HLKLD2402Component::write_frame_(const std::vector<uint8_t> &frame) {
  size_t written = 0;
  size_t tries = 0;
  while (written < frame.size() && tries++ < 3) {
    size_t to_write = frame.size() - written;
    write_array(&frame[written], to_write);  // write_array returns void
    written += to_write;
    if (written < frame.size()) {
      delay(5);
    }
  }
  return written == frame.size();
}

bool HLKLD2402Component::send_command_(uint16_t command, const uint8_t *data, size_t len) {
  std::vector<uint8_t> frame;
  
  // Header
  frame.insert(frame.end(), FRAME_HEADER, FRAME_HEADER + 4);
  
  // Length (2 bytes) - command (2 bytes) + data length
  uint16_t total_len = 2 + len;  // 2 for command, plus any additional data
  frame.push_back(total_len & 0xFF);
  frame.push_back((total_len >> 8) & 0xFF);
  
  // Command (2 bytes, little endian)
  frame.push_back(command & 0xFF);
  frame.push_back((command >> 8) & 0xFF);
  
  // Data (if any)
  if (data != nullptr && len > 0) {
    frame.insert(frame.end(), data, data + len);
  }
  
  // Footer
  frame.insert(frame.end(), FRAME_FOOTER, FRAME_FOOTER + 4);
  
  // Log the frame we're sending for debugging
  char hex_buf[128] = {0};
  for (size_t i = 0; i < frame.size() && i < 40; i++) {
    sprintf(hex_buf + (i*3), "%02X ", frame[i]);
  }
  ESP_LOGI(TAG, "Sending command 0x%04X, frame: %s", command, hex_buf);
  
  return write_frame_(frame);
}

// Modify read_response_ to accept a custom timeout
bool HLKLD2402Component::read_response_(std::vector<uint8_t> &response, uint32_t timeout_ms) {
  uint32_t start = millis();
  std::vector<uint8_t> buffer;
  uint8_t header_match = 0;
  uint8_t footer_match = 0;
  
  while ((millis() - start) < timeout_ms) {  // Use parameterized timeout
    if (available()) {
      uint8_t c;
      read_byte(&c);
      
      // Look for header
      if (header_match < 4) {
        if (c == FRAME_HEADER[header_match]) {
          header_match++;
          buffer.push_back(c);
        } else {
          header_match = 0;
          buffer.clear();
        }
        continue;
      }
      
      buffer.push_back(c);
      
      // Look for footer
      if (c == FRAME_FOOTER[footer_match]) {
        footer_match++;
        if (footer_match == 4) {
          // Extract the actual response data (remove header and footer)
          response.assign(buffer.begin() + 4, buffer.end() - 4);
          return true;
        }
      } else {
        footer_match = 0;
      }
    }
    yield();
  }
  
  ESP_LOGW(TAG, "Response timeout after %u ms", timeout_ms);
  return false;
}

// Modify get_parameter_ to use a longer timeout for power interference parameter
bool HLKLD2402Component::get_parameter_(uint16_t param_id, uint32_t &value) {
  ESP_LOGD(TAG, "Getting parameter 0x%04X", param_id);
  
  uint8_t data[2];
  data[0] = param_id & 0xFF;
  data[1] = (param_id >> 8) & 0xFF;
  
  if (!send_command_(CMD_GET_PARAMS, data, sizeof(data))) {
    ESP_LOGE(TAG, "Failed to send get parameter command");
    return false;
  }
  
  // Add a bigger delay for parameter 0x0005 (power interference)
  if (param_id == PARAM_POWER_INTERFERENCE) {
    delay(500);  // Wait longer for power interference parameter
  } else {
    delay(100);  // Standard delay for other parameters
  }
  
  std::vector<uint8_t> response;
  // Use longer timeout for power interference parameter
  uint32_t timeout = (param_id == PARAM_POWER_INTERFERENCE) ? 3000 : 1000;
  
  if (!read_response_(response, timeout)) {
    ESP_LOGE(TAG, "No response to get parameter command");
    return false;
  }
  
  // Log the response for debugging
  char hex_buf[64] = {0};
  for (size_t i = 0; i < response.size() && i < 16; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGD(TAG, "Get parameter response: %s", hex_buf);
  
  // Handle response in a permissive way
  if (response.size() >= 6) {
    // Standard response format
    value = response[2] | (response[3] << 8) | (response[4] << 16) | (response[5] << 24);
    ESP_LOGD(TAG, "Parameter 0x%04X value: %u", param_id, value);
    return true;
  } else if (response.size() >= 2) {
    // Shorter response, but possibly valid - use first 2 bytes
    value = response[0] | (response[1] << 8);
    ESP_LOGW(TAG, "Short parameter response, using value: %u", value);
    return true;
  }
  
  ESP_LOGE(TAG, "Invalid parameter response format");
  return false;
}

bool HLKLD2402Component::set_work_mode_(uint32_t mode) {
  return set_work_mode_with_timeout_(mode, 1000);  // Use default 1000ms timeout
}

bool HLKLD2402Component::set_work_mode_with_timeout_(uint32_t mode, uint32_t timeout_ms) {
  ESP_LOGI(TAG, "Setting work mode to %u (0x%X) with %ums timeout", mode, mode, timeout_ms);
  
  // Use production mode from manual instead of MODE_NORMAL
  if (mode == MODE_NORMAL) {
    mode = MODE_PRODUCTION;
    ESP_LOGI(TAG, "Using production mode 0x%X", mode);
  }
  
  uint8_t mode_data[6];
  mode_data[0] = 0x00;
  mode_data[1] = 0x00;
  mode_data[2] = mode & 0xFF;
  mode_data[3] = (mode >> 8) & 0xFF;
  mode_data[4] = (mode >> 16) & 0xFF;
  mode_data[5] = (mode >> 24) & 0xFF;
  
  // Log the payload data for debugging
  ESP_LOGD(TAG, "Mode payload: %02X %02X %02X %02X %02X %02X", 
           mode_data[0], mode_data[1], mode_data[2], 
           mode_data[3], mode_data[4], mode_data[5]);
  
  if (!send_command_(CMD_SET_MODE, mode_data, sizeof(mode_data))) {
    ESP_LOGE(TAG, "Failed to send mode command");
    return false;
  }
  
  // Use the extended timeout for response
  std::vector<uint8_t> response;
  if (!read_response_(response, timeout_ms)) {
    ESP_LOGE(TAG, "No response to mode command (timeout: %ums)", timeout_ms);
    return false;
  }
  
  // Log the response in detail - improved hex format logging
  ESP_LOGI(TAG, "Mode response hex bytes: %02X %02X %02X %02X %02X %02X", 
           response.size() > 0 ? response[0] : 0, 
           response.size() > 1 ? response[1] : 0,
           response.size() > 2 ? response[2] : 0,
           response.size() > 3 ? response[3] : 0,
           response.size() > 4 ? response[4] : 0,
           response.size() > 5 ? response[5] : 0);

  // Standard success check - ACK is 0x00 0x00
  bool success = false;
  if (response.size() >= 2 && response[0] == 0x00 && response[1] == 0x00) {
    success = true;
    ESP_LOGI(TAG, "Work mode set successfully (standard ACK)");
  }
  // Engineering mode special case - first byte matches requested mode value
  // Documentation shows one format but actual device uses different format
  else if (mode == MODE_ENGINEERING && response.size() >= 3 && 
           response[0] == (mode & 0xFF) && response[2] == (CMD_SET_MODE & 0xFF)) {
    // The response format for engineering mode appears to be:
    // [mode_byte] [00] [cmd_echo] [01] [00] [00]
    success = true;
    ESP_LOGI(TAG, "Engineering mode set successfully (device-specific response format)");
  }
  // NEW: Additional format for exiting engineering mode
  else if (mode == MODE_PRODUCTION && response.size() >= 6 && 
           response[0] == 0x04 && response[2] == (CMD_SET_MODE & 0xFF) && 
           response[3] == 0x01 && response[4] == 0x00 && response[5] == 0x00) {
    // When exiting engineering mode, we get: 04 00 12 01 00 00
    // This appears to be [prev_mode] [00] [cmd_echo] [01] [00] [00]
    success = true;
    ESP_LOGI(TAG, "Normal mode set successfully (engineering exit response format)");
  }
  
  if (success) {
    // Update the operating mode text
    if (mode == MODE_NORMAL || mode == MODE_PRODUCTION) {
      operating_mode_ = "Normal";
    } else if (mode == MODE_ENGINEERING) {
      operating_mode_ = "Engineering";
    } else {
      operating_mode_ = "Unknown";
    }
    
    publish_operating_mode_();
    
    // Clear any pending data
    flush();
    while (available()) {
      uint8_t c;
      read_byte(&c);
    }
    return true;
  }
  
  ESP_LOGE(TAG, "Invalid response to set work mode - doesn't match expected patterns");
  return false;
}

// Keep the existing set_engineering_mode for backward compatibility (used as toggle)
void HLKLD2402Component::set_engineering_mode() {
  // Check if we're already in Engineering mode - if so, switch back to normal
  if (operating_mode_ == "Engineering") {
    ESP_LOGI(TAG, "Already in engineering mode, switching back to normal mode");
    set_normal_mode();
    return;
  }
  
  // If not in Engineering mode, call the new direct method
  set_engineering_mode_direct();
}

// New method that directly sets engineering mode without toggle behavior
void HLKLD2402Component::set_engineering_mode_direct() {
  // Check if already in engineering mode to avoid unnecessary actions
  if (operating_mode_ == "Engineering") {
    ESP_LOGI(TAG, "Already in engineering mode. No action needed.");
    return;
  }
  
  ESP_LOGI(TAG, "Switching to engineering mode...");
  
  // Disable data processing temporarily to ensure clean state
  engineering_data_enabled_ = false;

  // First ensure we're not in config mode already
  if (config_mode_) {
    exit_config_mode_();
    delay(200);
  }
  
  // Enter config mode
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for engineering mode");
    return;
  }
  
  // Based on the device's actual behavior seen in serial capture:
  // 1. Clear any pending data
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  // 2. Set engineering mode with command 0x0012, parameter 0x00000004
  ESP_LOGI(TAG, "Sending engineering mode command (0x0012)...");
  
  // Prepare the command data: 0x0000 followed by mode 0x00000004
  uint8_t mode_data[6];
  mode_data[0] = 0x00;  // First two bytes are 0x0000
  mode_data[1] = 0x00;
  mode_data[2] = 0x04;  // Engineering mode (0x00000004), little-endian
  mode_data[3] = 0x00;
  mode_data[4] = 0x00;
  mode_data[5] = 0x00;
  
  if (!send_command_(CMD_SET_MODE, mode_data, sizeof(mode_data))) {
    ESP_LOGE(TAG, "Failed to send engineering mode command");
    exit_config_mode_();
    return;
  }
  
  // 3. Wait for response with increased timeout
  std::vector<uint8_t> response;
  if (!read_response_(response, 2000)) {
    ESP_LOGE(TAG, "No response to engineering mode command");
    exit_config_mode_();
    return;
  }
  
  // 4. Validate the response
  char hex_buf[64] = {0};
  for (size_t i = 0; i < response.size() && i < 16; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGI(TAG, "Engineering mode response: %s", hex_buf);
  
  // Check if the response indicates success
  bool success = false;
  if (response.size() >= 2 && response[0] == 0x00 && response[1] == 0x00) {
    success = true;
    ESP_LOGI(TAG, "Engineering mode set successfully (standard ACK)");
  } else if (response.size() >= 3 && response[0] == 0x04 && response[2] == 0x12) {
    // Special case for engineering mode response as seen in documentation
    success = true;
    ESP_LOGI(TAG, "Engineering mode set successfully (device-specific response format)");
  }
  
  if (success) {
    // Important difference from previous implementation:
    // FIRST set the operating mode, so we correctly identify frames
    operating_mode_ = "Engineering";
    publish_operating_mode_();
    
    // Enable receiving engineering data
    engineering_data_enabled_ = true;
    
    // CRITICAL CORRECTION: From sniffed data, we see that we MUST exit config mode
    // for the device to start sending data frames!
    ESP_LOGI(TAG, "Exiting config mode to begin receiving engineering data frames");
    
    // Now exit config mode as seen in the official software's behavior
    exit_config_mode_();
    
    // Small delay
    delay(300);
    
    // 5. Clear any pending data again before receiving engineering frames
    flush();
    while (available()) {
      uint8_t c;
      read_byte(&c);
    }
    
    // List all configured energy gate sensors
    ESP_LOGI(TAG, "Configured energy gate sensors (%d):", energy_gate_sensors_.size());
    for (size_t i = 0; i < energy_gate_sensors_.size(); i++) {
      if (energy_gate_sensors_[i] != nullptr) {
        ESP_LOGI(TAG, "  Gate %d: sensor configured", i);
      }
    }
    
    ESP_LOGI(TAG, "Engineering mode activated - module should now send binary data frames");
  } else {
    ESP_LOGE(TAG, "Engineering mode command failed: Unexpected response");
    exit_config_mode_();
  }
}

// New method for directly setting normal mode without toggle logic
void HLKLD2402Component::set_normal_mode_direct() {
  // Check if already in normal mode to avoid unnecessary actions
  if (operating_mode_ == "Normal") {
    ESP_LOGI(TAG, "Already in normal mode. No action needed.");
    return;
  }
  
  // Call the existing normal mode setting function
  set_normal_mode();
}

void HLKLD2402Component::set_normal_mode() {
  ESP_LOGI(TAG, "Switching to normal mode...");
  
  // IMPORTANT: Disable engineering data processing flag when returning to normal mode
  engineering_data_enabled_ = false;
  
  // Enter config mode if not already in it
  if (!config_mode_ && !enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for normal mode");
    return;
  }
  
  // Set work mode
  if (set_work_mode_(MODE_NORMAL)) {
    ESP_LOGI(TAG, "Successfully switched to normal mode");
    
    // Always exit config mode when going to normal mode
    exit_config_mode_();
    
    // Clear any remaining data to ensure clean transition
    flush();
    while (available()) {
      uint8_t c;
      read_byte(&c);
    }
  } else {
    ESP_LOGE(TAG, "Failed to set normal mode");
    // Still try to exit config mode
    exit_config_mode_();
  }
}

void HLKLD2402Component::save_config() {
  ESP_LOGI(TAG, "Saving configuration...");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode");
    return;
  }
  
  if (save_configuration_()) {
    ESP_LOGI(TAG, "Configuration saved successfully");
  } else {
    ESP_LOGE(TAG, "Failed to save configuration");
  }
  
  exit_config_mode_();
}

bool HLKLD2402Component::save_configuration_() {
  ESP_LOGI(TAG, "Sending save configuration command...");
  
  // Clear any pending data first to ensure a clean state
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  // Send the save command with explicit flush
  if (!send_command_(CMD_SAVE_PARAMS)) {
    ESP_LOGE(TAG, "Failed to send save command");
    return false;
  }
  
  // IMPORTANT: Add a longer delay after sending the save command
  // The device needs more time to process flash operations
  delay(1000);  // Increase from 500ms to 1000ms
  
  std::vector<uint8_t> response;
  if (!read_response_(response, 3000)) {  // Increase timeout to 3 seconds
    ESP_LOGW(TAG, "No response to save command - this may indicate a firmware issue");
    
    // Retry with a direct send after a delay
    delay(500);
    if (!send_command_(CMD_SAVE_PARAMS)) {
      ESP_LOGE(TAG, "Failed to send retry save command");
      return false;
    }
    
    // Try reading the response again after a longer delay
    delay(1500);
    if (!read_response_(response, 3000)) {
      ESP_LOGE(TAG, "No response to retry save command");
      return false;
    }
  }
  
  // Log the complete response for debugging
  char hex_buf[128] = {0};
  for (size_t i = 0; i < response.size() && i < 32; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGI(TAG, "Save config response: %s", hex_buf);
  
  // Based on logs and protocol documentation, handle various response patterns:
  
  // Case 1: Standard ACK (00 00) as per documentation section 5.3
  if (response.size() >= 2 && response[0] == 0x00 && response[1] == 0x00) {
    ESP_LOGI(TAG, "Save configuration acknowledged with standard ACK");
    
    // Add a safety delay to ensure flash write completes
    delay(500);
    return true;
  }
  
  // Case 2: Actual device response format seen in logs
  // Format: [04 00][FD 01][00 00] - command echo pattern
  if (response.size() >= 6 &&
      response[0] == 0x04 && response[1] == 0x00 &&
      response[2] == 0xFD && 
      response[4] == 0x00 && response[5] == 0x00) {
    
    ESP_LOGI(TAG, "Save configuration acknowledged with device-specific format");
    
    // Add longer safety delay to ensure flash write completes
    delay(1000);
    return true;
  }
  
  // Case 3: Any other non-empty response (more permissive for future firmware updates)
  if (response.size() >= 2) {
    ESP_LOGW(TAG, "Received non-standard save response format but continuing");
    
    // Log detailed bytes for diagnostics
    ESP_LOGD(TAG, "Response details (first 6 bytes):");
    for (size_t i = 0; i < std::min(response.size(), size_t(6)); i++) {
      ESP_LOGD(TAG, "  Byte[%d] = 0x%02X", i, response[i]);
    }
    
    // Add a very conservative delay to ensure flash operations complete
    delay(1500);
    return true;
  }
  
  ESP_LOGW(TAG, "Unrecognized save configuration response format");
  return false;
}

// Update enable_auto_gain to use correct commands per documentation section 5.4
void HLKLD2402Component::enable_auto_gain() {
  ESP_LOGI(TAG, "Enabling auto gain...");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode");
    return;
  }
  
  if (!enable_auto_gain_()) {
    ESP_LOGE(TAG, "Failed to enable auto gain");
    exit_config_mode_();
    return;
  }
  
  // According to section 5.4, wait for completion command response (0xF0)
  ESP_LOGI(TAG, "Waiting for auto gain completion...");
  uint32_t start = millis();
  bool completion_received = false;
  
  // Use a 10 second timeout as auto gain may take time to complete
  while ((millis() - start) < 10000) {
    if (available() >= 12) { // Minimum expected frame size
      std::vector<uint8_t> response;
      if (read_response_(response)) {
        // Check for the completion notification frame (CMD_AUTO_GAIN_COMPLETE = 0xF0)
        if (response.size() >= 2 && response[0] == 0xF0 && response[1] == 0x00) {
          ESP_LOGI(TAG, "Auto gain adjustment completed");
          completion_received = true;
          break;
        }
      }
    }
    delay(100);
  }
  
  if (!completion_received) {
    ESP_LOGW(TAG, "Auto gain completion notification not received within timeout");
  }
  
  exit_config_mode_();
}

bool HLKLD2402Component::enable_auto_gain_() {
  // As per section 5.4, send the auto gain command
  if (!send_command_(CMD_AUTO_GAIN)) {
    ESP_LOGE(TAG, "Failed to send auto gain command");
    return false;
  }
  
  std::vector<uint8_t> response;
  if (!read_response_(response)) {
    ESP_LOGE(TAG, "No response to auto gain command");
    return false;
  }
  
  // According to the documentation, expect a standard ACK
  if (response.size() >= 2 && response[0] == 0x00 && response[1] == 0x00) {
    ESP_LOGI(TAG, "Auto gain command acknowledged");
    return true;
  }
  
  ESP_LOGE(TAG, "Invalid response to auto gain command");
  return false;
}

// Add serial number retrieval methods
void HLKLD2402Component::get_serial_number() {
  ESP_LOGI(TAG, "Getting serial number...");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode");
    return;
  }
  
  // Try HEX format first (newer firmware)
  if (!get_serial_number_hex_()) {
    // If that fails, try character format
    if (!get_serial_number_char_()) {
      ESP_LOGE(TAG, "Failed to get serial number");
    }
  }
  
  exit_config_mode_();
}

bool HLKLD2402Component::get_serial_number_hex_() {
  if (!send_command_(CMD_GET_SN_HEX)) {
    ESP_LOGE(TAG, "Failed to send hex SN command");
    return false;
  }
  
  std::vector<uint8_t> response;
  if (!read_response_(response)) {
    ESP_LOGE(TAG, "No response to hex SN command");
    return false;
  }
  
  // Per protocol section 5.2.4, response format:
  // 2 bytes ACK + 2 bytes length + N bytes SN
  if (response.size() >= 4 && response[0] == 0x00 && response[1] == 0x00) {
    uint16_t sn_length = response[2] | (response[3] << 8);
    
    if (response.size() >= 4 + sn_length) {
      // Format as hex string
      std::string sn;
      char temp[8];
      for (size_t i = 4; i < 4 + sn_length; i++) {
        sprintf(temp, "%02X", response[i]);
        sn += temp;
      }
      
      serial_number_ = sn;
      ESP_LOGI(TAG, "Serial number (hex): %s", sn.c_str());
      return true;
    }
  }
  
  return false;
}

bool HLKLD2402Component::get_serial_number_char_() {
  if (!send_command_(CMD_GET_SN_CHAR)) {
    ESP_LOGE(TAG, "Failed to send char SN command");
    return false;
  }
  
  std::vector<uint8_t> response;
  if (!read_response_(response)) {
    ESP_LOGE(TAG, "No response to char SN command");
    return false;
  }
  
  // Per protocol section 5.2.5, response format:
  // 2 bytes ACK + 2 bytes length + N bytes SN
  if (response.size() >= 4 && response[0] == 0x00 && response[1] == 0x00) {
    uint16_t sn_length = response[2] | (response[3] << 8);
    
    if (response.size() >= 4 + sn_length) {
      // Format as character string
      std::string sn;
      for (size_t i = 4; i < 4 + sn_length; i++) {
        sn += (char)response[i];
      }
      
      serial_number_ = sn;
      ESP_LOGI(TAG, "Serial number (char): %s", sn.c_str());
      return true;
    }
  }
  
  return false;
}

void HLKLD2402Component::check_power_interference() {
  ESP_LOGI(TAG, "Checking power interference status");
  
  // Clear any pending data first
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  // Flag to track if we entered config mode in this function
  bool entered_config_mode = false;
  
  if (!config_mode_) {
    if (!enter_config_mode_()) {
      ESP_LOGE(TAG, "Failed to enter config mode for power interference check");
      
      // Show ERROR state if the check fails
      if (this->power_interference_binary_sensor_ != nullptr) {
        this->power_interference_binary_sensor_->publish_state(true);  // Show interference/error
        ESP_LOGE(TAG, "Setting power interference to ON (ERROR) due to config mode failure");
      }
      return;
    }
    entered_config_mode = true;
  }
  
  // According to documentation, use GET_PARAMS command (0x0008) with parameter ID 0x0005
  ESP_LOGI(TAG, "Reading power interference parameter...");
  uint8_t param_data[2];
  param_data[0] = PARAM_POWER_INTERFERENCE & 0xFF;  // 0x05
  param_data[1] = (PARAM_POWER_INTERFERENCE >> 8) & 0xFF;  // 0x00  
  
  if (!send_command_(CMD_GET_PARAMS, param_data, sizeof(param_data))) {
    ESP_LOGE(TAG, "Failed to send power interference parameter query");
    
    if (this->power_interference_binary_sensor_ != nullptr) {
      this->power_interference_binary_sensor_->publish_state(true);  // Show interference/error
      ESP_LOGE(TAG, "Setting power interference to ON (ERROR) due to command failure");
    }
    
    // Exit config mode if we entered it
    if (entered_config_mode) {
      exit_config_mode_();
    }
    return;
  }
  
  // Wait for response
  delay(500);
  
  std::vector<uint8_t> response;
  if (!read_response_(response, 2000)) {
    ESP_LOGE(TAG, "No response to power interference parameter query");
    
    if (this->power_interference_binary_sensor_ != nullptr) {
      this->power_interference_binary_sensor_->publish_state(true);  // Show interference/error
      ESP_LOGE(TAG, "Setting power interference to ON (ERROR) due to timeout");
    }
    
    // Exit config mode if we entered it
    if (entered_config_mode) {
      exit_config_mode_();
    }
    return;
  }
  
  // Log the response for debugging
  char hex_buf[128] = {0};
  for (size_t i = 0; i < response.size() && i < 30; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGI(TAG, "Power interference response: %s", hex_buf);
  
  // According to documentation:
  // The response format is:
  // CMD (2 bytes) + ACK (2 bytes) + Parameter ID (2 bytes) + Parameter value (4 bytes)
  bool has_interference = false;
  
  // Based on the protocol documentation and our response:
  // 0: Not performed
  // 1: No interference
  // 2: Has interference
  if (response.size() >= 10) {
    // Parameter value is at offset 6-9, little endian
    uint32_t value = response[6] | (response[7] << 8) | (response[8] << 16) | (response[9] << 24);
    ESP_LOGI(TAG, "Power interference value: %u", value);
    
    if (value == 0) {
      ESP_LOGI(TAG, "Power interference check not performed");
      has_interference = false;
    } else if (value == 1) {
      ESP_LOGI(TAG, "No power interference detected");
      has_interference = false;
    } else if (value == 2) {
      ESP_LOGI(TAG, "Power interference detected");
      has_interference = true;
    } else {
      ESP_LOGW(TAG, "Unknown power interference value: %u", value);
      has_interference = (value != 1);  // Consider anything other than 1 as interference
    }
  } else {
    ESP_LOGW(TAG, "Invalid power interference parameter response format");
    has_interference = true;  // Assume interference on invalid response
  }
  
  // Update sensor state
  if (this->power_interference_binary_sensor_ != nullptr) {
    this->power_interference_binary_sensor_->publish_state(has_interference);
    ESP_LOGI(TAG, "Set power interference to %s based on parameter value", 
             has_interference ? "ON (interference detected)" : "OFF (no interference)");
  }
  
  // Exit config mode if we entered it in this function
  if (entered_config_mode) {
    exit_config_mode_();
  }
}

uint32_t HLKLD2402Component::db_to_threshold_(float db_value) {
  return static_cast<uint32_t>(pow(10, db_value / 10)); 
}

float HLKLD2402Component::threshold_to_db_(uint32_t threshold) {
  return 10 * log10(threshold);
}

void HLKLD2402Component::factory_reset() {
  ESP_LOGI(TAG, "Performing factory reset...");
  
  // Clear UART buffers before starting
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for factory reset");
    return;
  }
  
  // Add a delay after entering config mode
  delay(200);
  
  ESP_LOGI(TAG, "Resetting max distance to default (5m)");
  set_parameter_(PARAM_MAX_DISTANCE, 50);  // 5.0m = 50 (internal value is in decimeters)
  delay(200);  // Add delay between parameter setting
  
  ESP_LOGI(TAG, "Resetting target timeout to default (5s)");
  set_parameter_(PARAM_TIMEOUT, 5);
  delay(200);
  
  // Reset only trigger threshold for gate 0 as an example
  ESP_LOGI(TAG, "Resetting main threshold values");
  set_parameter_(PARAM_TRIGGER_THRESHOLD, 30);  // 30 = ~3.0 coefficient
  delay(200);  
  
  set_parameter_(PARAM_MICRO_THRESHOLD, 30);
  delay(200);  
  
  // Save configuration
  ESP_LOGI(TAG, "Saving factory reset configuration");
  if (send_command_(CMD_SAVE_PARAMS)) {
    std::vector<uint8_t> response;
    
    // Wait a bit longer for save operation
    delay(300);
    
    if (read_response_(response)) {
      // Log the response for debugging
      char hex_buf[64] = {0};
      for (size_t i = 0; i < response.size() && i < 16; i++) {
        sprintf(hex_buf + (i*3), "%02X ", response[i]);
      }
      ESP_LOGI(TAG, "Save config response: %s", hex_buf);
      ESP_LOGI(TAG, "Configuration saved successfully");
    } else {
      ESP_LOGW(TAG, "No response to save configuration command");
    }
  } else {
    ESP_LOGW(TAG, "Failed to send save command");
  }
  
  // Add a final delay before exiting config mode
  delay(500);
  
  // Use safer exit pattern
  ESP_LOGI(TAG, "Exiting config mode");
  send_command_(CMD_DISABLE_CONFIG);
  delay(200);
  config_mode_ = false;
  
  ESP_LOGI(TAG, "Factory reset completed");
}

// Make sure we have matching implementations for ALL protected methods
bool HLKLD2402Component::enter_config_mode_() {
  if (config_mode_)
    return true;
    
  ESP_LOGD(TAG, "Entering config mode...");
  
  // Clear any pending data first
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  // Try multiple times with delays
  for (int attempt = 0; attempt < 3; attempt++) {
    ESP_LOGI(TAG, "Config mode attempt %d", attempt + 1);
    
    // Send the command with no data
    if (!send_command_(CMD_ENABLE_CONFIG)) {
      ESP_LOGE(TAG, "Failed to send config mode command");
      delay(500);  // Wait before retrying
      continue;
    }
    
    // Delay slightly to ensure response has time to arrive
    delay(200);
    
    // Check for response with timeout
    uint32_t start = millis();
    while ((millis() - start) < 1000) {  // 1 second timeout
      if (available() >= 12) {  // Minimum expected response size with header/footer
        std::vector<uint8_t> response;
        if (read_response_(response)) {
          ESP_LOGI(TAG, "Received response to config mode command");
          
          // Dump the response bytes for debugging
          char hex_buf[128] = {0};
          for (size_t i = 0; i < response.size() && i < 20; i++) {
            sprintf(hex_buf + (i*3), "%02X ", response[i]);
          }
          ESP_LOGI(TAG, "Response: %s", hex_buf);
          
          // Looking at logs, the response is: "08 00 FF 01 00 00 02 00 20 00"
          // Format: Length (2) + Command ID (FF 01) + Status (00 00) + Protocol version (02 00) + Buffer size (20 00)
          if (response.size() >= 6 && 
              response[0] == 0xFF && response[1] == 0x01 && 
              response[2] == 0x00 && response[3] == 0x00) {
            config_mode_ = true;
            ESP_LOGI(TAG, "Successfully entered config mode");
            
            // Update operating mode
            operating_mode_ = "Config";
            publish_operating_mode_();
            return true;
          } else if (response.size() >= 6 && 
                    response[4] == 0x00 && response[5] == 0x00) {
            // Alternative format sometimes seen
            config_mode_ = true;
            ESP_LOGI(TAG, "Successfully entered config mode (alt format)");
            
            // Update operating mode
            operating_mode_ = "Config";
            publish_operating_mode_();
            return true;
          } else {
            ESP_LOGW(TAG, "Invalid config mode response format - expected status 00 00");
            
            // Trace each byte to help diagnose the issue
            ESP_LOGW(TAG, "Response details: %d bytes", response.size());
            for (size_t i = 0; i < response.size() && i < 10; i++) {
              ESP_LOGW(TAG, "  Byte[%d] = 0x%02X", i, response[i]);
            }
          }
        }
      }
      delay(50);  // Small delay between checks
    }
    
    ESP_LOGW(TAG, "No valid response to config mode command, retrying");
    delay(500);  // Wait before retrying
  }
  
  ESP_LOGE(TAG, "Failed to enter config mode after 3 attempts");
  return false;
}

bool HLKLD2402Component::exit_config_mode_() {
  if (!config_mode_)
    return true;
    
  ESP_LOGD(TAG, "Exiting config mode...");
  
  // Send exit command
  if (send_command_(CMD_DISABLE_CONFIG)) {
    // Brief wait for response, but don't wait too long
    delay(200);
    
    // Read any response but don't wait too long
    std::vector<uint8_t> response;
    bool got_response = read_response_(response, 300);
    if (got_response) {
      ESP_LOGI(TAG, "Got response to exit command");
    } else {
      // Don't treat this as an error - some firmware versions may not respond
      ESP_LOGI(TAG, "No response to exit command - this may be normal");
    }
  }
  
  // Always mark as exited regardless of response
  config_mode_ = false;
  ESP_LOGI(TAG, "Left config mode");
  
  // Update operating mode back to either Normal or Engineering
  if (operating_mode_ == "Config") {
    operating_mode_ = "Normal";  // Default to Normal when exiting config mode
    publish_operating_mode_();
  }
  
  // Clear any pending data to ensure clean state
  flush();
  while (available()) {
    uint8_t c;
    read_byte(&c);
  }
  
  return true;
}

bool HLKLD2402Component::set_parameter_(uint16_t param_id, uint32_t value) {
  ESP_LOGD(TAG, "Setting parameter 0x%04X to %u", param_id, value);
  
  uint8_t data[6];
  data[0] = param_id & 0xFF;
  data[1] = (param_id >> 8) & 0xFF;
  data[2] = value & 0xFF;
  data[3] = (value >> 8) & 0xFF;
  data[4] = (value >> 16) & 0xFF;
  data[5] = (value >> 24) & 0xFF;
  
  if (!send_command_(CMD_SET_PARAMS, data, sizeof(data))) {
    ESP_LOGE(TAG, "Failed to send set parameter command");
    return false;
  }
    
  // Add a small delay after sending command
  delay(100);
  
  std::vector<uint8_t> response;
  if (!read_response_(response)) {  // Use default timeout
    ESP_LOGE(TAG, "No response to set parameter command");
    return false;
  }
  
  // Log the response for debugging
  char hex_buf[64] = {0};
  for (size_t i = 0; i < response.size() && i < 16; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGD(TAG, "Set parameter response: %s", hex_buf);
  
  // Do basic error checking without being too strict on validation
  if (response.size() < 2) {
    ESP_LOGE(TAG, "Response too short");
    return false;
  }
  
  // Check for known error patterns
  bool has_error = false;
  if (response[0] == 0xFF && response[1] == 0xFF) {
    has_error = true;  // This typically indicates an error
  }
  
  if (has_error) {
    ESP_LOGE(TAG, "Parameter setting failed with error response");
    return false;
  }
  
  // For other responses, be permissive and assume success
  return true;
}

// Add these methods to configure thresholds for specific gates
bool HLKLD2402Component::set_motion_threshold(uint8_t gate, float db_value) {
  ESP_LOGI(TAG, "Setting motion threshold for gate %d to %.1f dB", gate, db_value);
  
  if (gate >= 16) {
    ESP_LOGE(TAG, "Invalid gate index %d (must be 0-15)", gate);
    return false;
  }
  
  // Clamp dB value to valid range (0-95 dB according to documentation)
  db_value = std::max(0.0f, std::min(95.0f, db_value));
  
  // Convert dB value to raw threshold
  uint32_t threshold = db_to_threshold_(db_value);
  
  // Gate-specific parameter ID: 0x0010 + gate number
  uint16_t param_id = PARAM_TRIGGER_THRESHOLD + gate;
  
  // Create a single parameter batch
  std::vector<std::pair<uint16_t, uint32_t>> params;
  params.push_back(std::make_pair(param_id, threshold));
  
  bool success = set_parameters_batch(params);
  if (success) {
    ESP_LOGI(TAG, "Successfully set motion threshold for gate %d to %.1f dB (raw: %u)",
             gate, db_value, threshold);
  } else {
    ESP_LOGE(TAG, "Failed to set motion threshold");
  }
  
  return success;
}

bool HLKLD2402Component::set_motion_thresholds(const std::map<uint8_t, float> &gate_thresholds) {
  if (gate_thresholds.empty()) {
    ESP_LOGW(TAG, "No motion thresholds to set");
    return false;
  }
  
  std::vector<std::pair<uint16_t, uint32_t>> params;
  
  for (const auto &entry : gate_thresholds) {
    uint8_t gate = entry.first;
    float db_value = entry.second;
    
    if (gate >= 16) {
      ESP_LOGW(TAG, "Skipping invalid gate index %d", gate);
      continue;
    }
    
    // Clamp dB value and convert to raw threshold
    db_value = std::max(0.0f, std::min(95.0f, db_value));
    uint32_t threshold = db_to_threshold_(db_value);
    
    // Gate-specific parameter ID
    uint16_t param_id = PARAM_TRIGGER_THRESHOLD + gate;
    params.push_back(std::make_pair(param_id, threshold));
    
    ESP_LOGI(TAG, "Adding motion threshold for gate %d: %.1f dB (raw: %u)", 
             gate, db_value, threshold);
  }
  
  return set_parameters_batch(params);
}

bool HLKLD2402Component::set_micromotion_thresholds(const std::map<uint8_t, float> &gate_thresholds) {
  if (gate_thresholds.empty()) {
    ESP_LOGW(TAG, "No micromotion thresholds to set");
    return false;
  }
  
  std::vector<std::pair<uint16_t, uint32_t>> params;
  
  for (const auto &entry : gate_thresholds) {
    uint8_t gate = entry.first;
    float db_value = entry.second;
    
    if (gate >= 16) {
      ESP_LOGW(TAG, "Skipping invalid gate index %d", gate);
      continue;
    }
    
    // Clamp dB value and convert to raw threshold
    db_value = std::max(0.0f, std::min(95.0f, db_value));
    uint32_t threshold = db_to_threshold_(db_value);
    
    // Gate-specific parameter ID
    uint16_t param_id = PARAM_MICRO_THRESHOLD + gate;
    params.push_back(std::make_pair(param_id, threshold));
    
    ESP_LOGI(TAG, "Adding micromotion threshold for gate %d: %.1f dB (raw: %u)", 
             gate, db_value, threshold);
  }
  
  return set_parameters_batch(params);
}

// Add new method for batch parameter setting
bool HLKLD2402Component::set_parameters_batch(const std::vector<std::pair<uint16_t, uint32_t>> &params) {
  ESP_LOGI(TAG, "Setting %d parameters in batch mode", params.size());
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for batch parameter setting");
    return false;
  }
  
  // Send all parameter commands in succession
  int success_count = 0;
  for (const auto &param : params) {
    uint16_t param_id = param.first;
    uint32_t value = param.second;
    
    // Prepare the command data
    uint8_t data[6];
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;
    data[2] = value & 0xFF;
    data[3] = (value >> 8) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    data[5] = (value >> 24) & 0xFF;
    
    ESP_LOGI(TAG, "Setting parameter 0x%04X to %u", param_id, value);
    
    // Send command with minimal delay between commands
    if (send_command_(CMD_SET_PARAMS, data, sizeof(data))) {
      success_count++;
      
      // Just a minimal delay between commands
      delay(50);
      
      // Clear any response data without waiting for it
      flush();
      while (available()) {
        uint8_t c;
        read_byte(&c);
      }
    } else {
      ESP_LOGW(TAG, "Failed to send parameter 0x%04X", param_id);
    }
  }
  
  // Save configuration after setting all parameters
  bool save_success = save_configuration_();
  if (save_success) {
    ESP_LOGI(TAG, "Successfully saved batch parameter configuration");
  } else {
    ESP_LOGW(TAG, "Failed to save batch parameter configuration");
  }
  
  // Exit config mode
  exit_config_mode_();
  
  // Return success if at least some parameters were set
  return (success_count > 0);
}

// Add a new method for batch parameter reading
bool HLKLD2402Component::get_parameters_batch_(const std::vector<uint16_t> &param_ids, std::vector<uint32_t> &values) {
  ESP_LOGI(TAG, "Reading %d parameters in batch mode", param_ids.size());
  
  // Prepare data: length of IDs array (2 bytes) followed by param IDs
  std::vector<uint8_t> data;
  uint16_t count = param_ids.size();
  data.push_back(count & 0xFF);
  data.push_back((count >> 8) & 0xFF);
  
  for (uint16_t id : param_ids) {
    data.push_back(id & 0xFF);
    data.push_back((id >> 8) & 0xFF);
  }
  
  if (!send_command_(CMD_GET_PARAMS, data.data(), data.size())) {
    ESP_LOGE(TAG, "Failed to send batch parameter query");
    return false;
  }
  
  // Wait for response
  delay(200);
  
  std::vector<uint8_t> response;
  if (!read_response_(response, 2000)) {
    ESP_LOGE(TAG, "No response to batch parameter query");
    return false;
  }
  
  // Log the response
  char hex_buf[128] = {0};
  for (size_t i = 0; i < response.size() && i < 30; i++) {
    sprintf(hex_buf + (i*3), "%02X ", response[i]);
  }
  ESP_LOGI(TAG, "Batch parameter response: %s", hex_buf);
  
  // Based on the serial capture, each 4-byte value is returned sequentially
  // The format should match what we saw in the capture
  if (response.size() >= param_ids.size() * 4) {
    values.clear();
    
    for (size_t i = 0; i < param_ids.size(); i++) {
      size_t offset = i * 4;
      uint32_t value = response[offset] | 
                      (response[offset+1] << 8) | 
                      (response[offset+2] << 16) | 
                      (response[offset+3] << 24);
      
      values.push_back(value);
      ESP_LOGI(TAG, "Parameter 0x%04X value: %u (0x%08X)", param_ids[i], value, value);
    }
    return true;
  }
  
  ESP_LOGE(TAG, "Invalid batch parameter response format or insufficient data");
  return false;
}

// Method to read all motion thresholds in one call
bool HLKLD2402Component::get_all_motion_thresholds() {
  ESP_LOGI(TAG, "Reading all motion thresholds");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for reading thresholds");
    return false;
  }
  
  // Prepare parameter IDs for motion threshold gates (0x0010 to 0x001F)
  std::vector<uint16_t> param_ids;
  for (uint16_t i = 0; i < 16; i++) {
    param_ids.push_back(PARAM_TRIGGER_THRESHOLD + i);
  }
  
  std::vector<uint32_t> values;
  bool success = get_parameters_batch_(param_ids, values);
  
  if (success) {
    ESP_LOGI(TAG, "Motion thresholds for all gates:");
    
    // Resize the cache vector if needed
    if (motion_threshold_values_.size() < values.size()) {
      motion_threshold_values_.resize(values.size(), 0);
    }
    
    // Process and publish each value
    for (size_t i = 0; i < values.size() && i < 16; i++) {
      float db_value = threshold_to_db_(values[i]);
      motion_threshold_values_[i] = db_value;
      
      ESP_LOGI(TAG, "  Gate %d: %u (%.1f dB)", i, values[i], db_value);
      
      // Publish to sensor if available
      if (i < motion_threshold_sensors_.size() && motion_threshold_sensors_[i] != nullptr) {
        motion_threshold_sensors_[i]->publish_state(db_value);
        ESP_LOGI(TAG, "Published motion threshold for gate %d: %.1f dB", i, db_value);
      }
    }
  }
  
  exit_config_mode_();
  return success;
}

// Method to read all micromotion thresholds in one call
bool HLKLD2402Component::get_all_micromotion_thresholds() {
  ESP_LOGI(TAG, "Reading all micromotion thresholds");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for reading thresholds");
    return false;
  }
  
  // Prepare parameter IDs for micromotion threshold gates (0x0030 to 0x003F)
  std::vector<uint16_t> param_ids;
  for (uint16_t i = 0; i < 16; i++) {
    param_ids.push_back(PARAM_MICRO_THRESHOLD + i);
  }
  
  std::vector<uint32_t> values;
  bool success = get_parameters_batch_(param_ids, values);
  
  if (success) {
    ESP_LOGI(TAG, "Micromotion thresholds for all gates:");
    
    // Resize the cache vector if needed
    if (micromotion_threshold_values_.size() < values.size()) {
      micromotion_threshold_values_.resize(values.size(), 0);
    }
    
    // Process and publish each value
    for (size_t i = 0; i < values.size() && i < 16; i++) {
      float db_value = threshold_to_db_(values[i]);
      micromotion_threshold_values_[i] = db_value;
      
      ESP_LOGI(TAG, "  Gate %d: %u (%.1f dB)", i, values[i], db_value);
      
      // Publish to sensor if available
      if (i < micromotion_threshold_sensors_.size() && micromotion_threshold_sensors_[i] != nullptr) {
        micromotion_threshold_sensors_[i]->publish_state(db_value);
        ESP_LOGI(TAG, "Published micromotion threshold for gate %d: %.1f dB", i, db_value);
      }
    }
  }
  
  exit_config_mode_();
  return success;
}

// Update calibration to match new command format and improve progress tracking
void HLKLD2402Component::calibrate() {
  calibrate_with_coefficients(3.0f, 3.0f, 3.0f);
}

// Update calibration to match new command format and improve progress tracking
bool HLKLD2402Component::calibrate_with_coefficients(float trigger_coeff, float hold_coeff, float micromotion_coeff) {
  ESP_LOGI(TAG, "Starting calibration with custom coefficients...");
  
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode");
    return false;
  }
  
  // Clamp coefficients to valid range (1.0 - 20.0)
  trigger_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, trigger_coeff));
  hold_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, hold_coeff));
  micromotion_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, micromotion_coeff));
  
  // According to section 5.2.9, each coefficient is multiplied by 10
  uint16_t trigger_value = static_cast<uint16_t>(trigger_coeff * 10.0f);
  uint16_t hold_value = static_cast<uint16_t>(hold_coeff * 10.0f);
  uint16_t micro_value = static_cast<uint16_t>(micromotion_coeff * 10.0f);
  
  // Prepare data according to the protocol: 3 coefficients, each 2 bytes
  uint8_t data[] = {
    static_cast<uint8_t>(trigger_value & 0xFF),
    static_cast<uint8_t>((trigger_value >> 8) & 0xFF),
    static_cast<uint8_t>(hold_value & 0xFF),
    static_cast<uint8_t>((hold_value >> 8) & 0xFF),
    static_cast<uint8_t>(micro_value & 0xFF),
    static_cast<uint8_t>((micro_value >> 8) & 0xFF)
  };
  
  ESP_LOGI(TAG, "Calibration coefficients - Trigger: %.1f, Hold: %.1f, Micro: %.1f", 
         trigger_coeff, hold_coeff, micromotion_coeff);
  
  if (send_command_(CMD_START_CALIBRATION, data, sizeof(data))) {
    ESP_LOGI(TAG, "Started calibration with custom coefficients");
    
    // Set calibration flags and initialize progress
    calibration_in_progress_ = true;
    calibration_progress_ = 0;
    last_calibration_check_ = millis() - 4000; // Check status almost immediately
    
    // Publish initial progress
    if (this->calibration_progress_sensor_ != nullptr) {
      this->calibration_progress_sensor_->publish_state(0);
    }
    return true;
  } else {
    ESP_LOGE(TAG, "Failed to start calibration");
    exit_config_mode_();
    return false;
  }
}

// ...existing code...

}  // namespace hlk_ld2402
}  // namespace esphome