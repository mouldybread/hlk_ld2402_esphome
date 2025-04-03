#include "hlk_ld2402.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2402 {

// Implementation for calibrate method
void HLKLD2402Component::calibrate() {
  ESP_LOGI(TAG, "Starting calibration with default coefficients...");
  // Use standard default values for calibration (trigger=3.0, hold=3.0, micromotion=3.0)
  calibrate_with_coefficients(3.0f, 3.0f, 3.0f);
}

// Implementation for get_all_motion_thresholds method
bool HLKLD2402Component::get_all_motion_thresholds() {
  ESP_LOGI(TAG, "Reading all motion thresholds");
  
  // Store the current operating mode to restore it later
  std::string previous_mode = operating_mode_;
  
  // Prepare parameter IDs to read
  std::vector<uint16_t> param_ids;
  for (uint16_t i = 0; i < 15; i++) {  // Read all 15 gates
    param_ids.push_back(PARAM_TRIGGER_THRESHOLD + i);
  }
  
  // Values will be stored here
  std::vector<uint32_t> values;
  
  // Enter config mode
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for reading thresholds");
    return false;
  }
  
  // Read all parameters in batch
  bool success = get_parameters_batch_(param_ids, values);
  
  if (success) {
    // Convert raw values to dB and store them
    motion_threshold_values_.clear();
    for (uint8_t i = 0; i < values.size(); i++) {
      float db_value = threshold_to_db_(values[i]);
      motion_threshold_values_.push_back(db_value);
      
      // Update the sensor if configured
      if (i < motion_threshold_sensors_.size() && motion_threshold_sensors_[i] != nullptr) {
        motion_threshold_sensors_[i]->publish_state(db_value);
        ESP_LOGI(TAG, "Published motion threshold for gate %d: %.1f dB", i, db_value);
      }
    }
  }
  
  // Exit config mode
  exit_config_mode_();
  
  return success;
}

// Implementation for get_all_static_thresholds method
bool HLKLD2402Component::get_all_static_thresholds() {
  ESP_LOGI(TAG, "Reading all static thresholds");
  
  // Store the current operating mode to restore it later
  std::string previous_mode = operating_mode_;
  
  // Prepare parameter IDs to read
  std::vector<uint16_t> param_ids;
  for (uint16_t i = 0; i < 15; i++) {  // Read all 15 gates
    param_ids.push_back(PARAM_STATIC_THRESHOLD + i);
  }
  
  // Values will be stored here
  std::vector<uint32_t> values;
  
  // Enter config mode
  if (!enter_config_mode_()) {
    ESP_LOGE(TAG, "Failed to enter config mode for reading thresholds");
    return false;
  }
  
  // Read all parameters in batch
  bool success = get_parameters_batch_(param_ids, values);
  
  if (success) {
    // Convert raw values to dB and store them
    static_threshold_values_.clear();
    for (uint8_t i = 0; i < values.size(); i++) {
      float db_value = threshold_to_db_(values[i]);
      static_threshold_values_.push_back(db_value);
      
      // Update the sensor if configured
      if (i < static_threshold_sensors_.size() && static_threshold_sensors_[i] != nullptr) {
        static_threshold_sensors_[i]->publish_state(db_value);
        ESP_LOGI(TAG, "Published static threshold for gate %d: %.1f dB", i, db_value);
      }
    }
  }
  
  // Exit config mode
  exit_config_mode_();
  
  return success;
}

// Implementation for helper method needed by the above functions
bool HLKLD2402Component::get_parameters_batch_(const std::vector<uint16_t> &param_ids, std::vector<uint32_t> &values) {
  ESP_LOGI(TAG, "Reading %d parameters in batch mode", param_ids.size());
  
  values.clear();
  
  // Check if we're in config mode, enter if not
  bool config_entered = false;
  if (!config_mode_) {
    if (!enter_config_mode_()) {
      ESP_LOGE(TAG, "Failed to enter config mode");
      return false;
    }
    config_entered = true;
  }
  
  bool success = true;
  
  // Read each parameter individually (we could optimize this later)
  for (const auto &param_id : param_ids) {
    uint32_t value = 0;
    if (get_parameter_(param_id, value)) {
      values.push_back(value);
      ESP_LOGI(TAG, "Successfully read parameter 0x%04X: %u (%.1f dB)", 
               param_id, value, threshold_to_db_(value));
    } else {
      // On failure, push a default value
      values.push_back(0);
      ESP_LOGW(TAG, "Failed to read parameter 0x%04X, using default (0)", param_id);
      success = false;
    }
  }
  
  // Exit config mode if we entered it
  if (config_entered) {
    exit_config_mode_();
  }
  
  return success;
}

}  // namespace hlk_ld2402
}  // namespace esphome
