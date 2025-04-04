import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_PRESENCE,
    CONF_ID,
)
from . import HLKLD2402Component, CONF_HLK_LD2402_ID, hlk_ld2402_ns

DEPENDENCIES = ["hlk_ld2402"]

# Create a dedicated binary sensor class for presence detection
HlkLd2402PresenceSensor = hlk_ld2402_ns.class_("HlkLd2402PresenceSensor", binary_sensor.BinarySensor)

# Make this a proper platform
CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    HlkLd2402PresenceSensor,
    device_class=DEVICE_CLASS_PRESENCE,
).extend({
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
})

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    cg.add(parent.set_presence_sensor(var))
