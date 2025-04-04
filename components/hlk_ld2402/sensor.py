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

# Make this a proper platform
CONFIG_SCHEMA = sensor.sensor_schema(
    HlkLd2402DistanceSensor,
    device_class=DEVICE_CLASS_DISTANCE,
    state_class=STATE_CLASS_MEASUREMENT,
    unit_of_measurement=UNIT_CENTIMETER,
).extend({
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
})

async def to_code(config):
    var = await sensor.new_sensor(config)
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    cg.add(parent.set_distance_sensor(var))