import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    CONF_ID,
)
from . import HLKLD2402Component, CONF_HLK_LD2402_ID, hlk_ld2402_ns

DEPENDENCIES = ["hlk_ld2402"]

# Create a dedicated sensor class for distance
HlkLd2402DistanceSensor = hlk_ld2402_ns.class_("HlkLd2402DistanceSensor", sensor.Sensor)

# Define a sensor platform in your component
CONF_DISTANCE = "distance"

# Register the component as a platform of sensor
CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_DISTANCE,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_CENTIMETER,
    ),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    
    if CONF_DISTANCE in config:
        var = cg.new_Pvariable(config[CONF_DISTANCE][CONF_ID])
        await sensor.register_sensor(var, config[CONF_DISTANCE])
        cg.add(parent.set_distance_sensor(var))