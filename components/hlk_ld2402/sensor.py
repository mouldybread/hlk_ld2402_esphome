import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    UNIT_PERCENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import HLKLD2402Component, CONF_HLK_LD2402_ID

CONF_THROTTLE = "throttle"
CONF_CALIBRATION_PROGRESS = "calibration_progress"
CONF_ENERGY_GATE = "energy_gate"  # Energy gate sensors
CONF_GATE_INDEX = "gate_index"     # Gate number (0-13)
CONF_MOTION_THRESHOLD = "motion_threshold"  # Motion threshold sensors
CONF_STATIC_THRESHOLD = "static_threshold"  # Static threshold sensors
# Removed micromotion backward compatibility

# Update schema to include threshold sensors
CONFIG_SCHEMA = sensor.sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_CALIBRATION_PROGRESS, default=False): cv.boolean,
    cv.Optional(CONF_ENERGY_GATE): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 14),  # Should be (0, 14) for 15 gates
    }),
    cv.Optional(CONF_MOTION_THRESHOLD): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
    # Only static option is now supported
    cv.Optional(CONF_STATIC_THRESHOLD): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await sensor.new_sensor(config)
    
    if CONF_ENERGY_GATE in config:
        gate_index = config[CONF_ENERGY_GATE][CONF_GATE_INDEX]
        cg.add(parent.set_energy_gate_sensor(gate_index, var))
    elif CONF_MOTION_THRESHOLD in config:
        gate_index = config[CONF_MOTION_THRESHOLD][CONF_GATE_INDEX]
        cg.add(parent.set_motion_threshold_sensor(gate_index, var))
    elif CONF_STATIC_THRESHOLD in config:
        gate_index = config[CONF_STATIC_THRESHOLD][CONF_GATE_INDEX]
        cg.add(parent.set_static_threshold_sensor(gate_index, var))
    elif config.get(CONF_CALIBRATION_PROGRESS):
        # This is a calibration progress sensor
        cg.add(parent.set_calibration_progress_sensor(var))
    else:
        # This is a regular distance sensor
        cg.add(parent.set_distance_sensor(var))
        if CONF_THROTTLE in config:
            cg.add(parent.set_distance_throttle(config[CONF_THROTTLE]))
