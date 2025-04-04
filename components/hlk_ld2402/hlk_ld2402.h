#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

class HLKLD2402 : public esphome::PollingComponent, public esphome::uart::UARTDevice {
 public:
  HLKLD2402(esphome::uart::UARTComponent *parent) : 
    esphome::PollingComponent(1000), // 1 second update interval default
    esphome::uart::UARTDevice(parent) {}

  // Return the distance sensor instance
  esphome::sensor::Sensor *get_distance_sensor() { return &distance_sensor_; }

  // Setup and update methods
  void setup() override;
  void update() override;
  void loop() override;

  // Buffer handling
  void process_buffer_();
  void clear_buffer_();

  // Component variables
  static const uint8_t MAX_BUFFER_SIZE = 64;
  char buffer_[MAX_BUFFER_SIZE];
  size_t buffer_pos_{0};
  uint32_t last_read_time_{0};
  
 protected:
  esphome::sensor::Sensor distance_sensor_;
};
