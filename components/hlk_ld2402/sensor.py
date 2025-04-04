import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    CONF_ID,
)
from . import HLKLD2402Component, CONF_HLK_LD2402_ID

DEPENDENCIES = ["hlk_ld2402"]

# Define a sensor platform in your component
CONF_DISTANCE = "distance"

# This is critical - we need to register the component as a platform of sensor
sensor.SENSOR_SCHEMA = sensor.sensor_schema()
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
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(parent.set_distance_sensor(sens))