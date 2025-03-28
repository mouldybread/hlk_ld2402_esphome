#pragma once

#include "esphome.h"

namespace esphome {
namespace hlk_ld2402 {

class HLKLD2402Component;  // Forward declaration

class HLKLD2402DistanceSensor : public sensor::Sensor, public Component {
 public:
  void set_parent(HLKLD2402Component *parent) { parent_ = parent; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
 protected:
  HLKLD2402Component *parent_;
};

class HLKLD2402BinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
  void set_parent(HLKLD2402Component *parent) { parent_ = parent; }
  void set_type(const std::string &type) { type_ = type; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
 protected:
  HLKLD2402Component *parent_;
  std::string type_;
};

class HLKLD2402Component : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_max_distance(float distance) { max_distance_ = distance; }
  void set_disappear_delay(uint16_t delay) { disappear_delay_ = delay; }

  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_presence_sensor(binary_sensor::BinarySensor *sensor) { presence_sensor_ = sensor; }
  void set_movement_sensor(binary_sensor::BinarySensor *sensor) { movement_sensor_ = sensor; }
  void set_micromovement_sensor(binary_sensor::BinarySensor *sensor) { micromovement_sensor_ = sensor; }

 protected:
  void process_data_(const std::vector<uint8_t> &data);
  void send_command_(const std::vector<uint8_t> &command);

  float max_distance_{8.5f};
  uint16_t disappear_delay_{30};

  sensor::Sensor *distance_sensor_{nullptr};
  binary_sensor::BinarySensor *presence_sensor_{nullptr};
  binary_sensor::BinarySensor *movement_sensor_{nullptr};
  binary_sensor::BinarySensor *micromovement_sensor_{nullptr};
  std::vector<uint8_t> buffer_;
};

}  // namespace hlk_ld2402
}  // namespace esphome