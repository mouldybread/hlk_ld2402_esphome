import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_DEVICE_CLASS,
    DEVICE_CLASS_PRESENCE,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_PROBLEM,
)
from . import CONF_HLK_LD2402_ID, HLKLD2402Component

DEPENDENCIES = ["hlk_ld2402"]

# Add missing configuration options
CONF_POWER_INTERFERENCE = "power_interference"
CONF_PRESENCE = "presence"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
    cv.GenerateID(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Optional(CONF_DEVICE_CLASS): cv.one_of(DEVICE_CLASS_PRESENCE, DEVICE_CLASS_MOTION, DEVICE_CLASS_PROBLEM),
    cv.Optional(CONF_POWER_INTERFERENCE): cv.boolean,
    cv.Optional(CONF_PRESENCE): cv.boolean,
})

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    
    if CONF_POWER_INTERFERENCE in config and config[CONF_POWER_INTERFERENCE]:
        cg.add(parent.set_power_interference_binary_sensor(var))
    elif CONF_PRESENCE in config and config[CONF_PRESENCE]:
        cg.add(parent.set_presence_binary_sensor(var))
    elif CONF_DEVICE_CLASS in config:
        if config[CONF_DEVICE_CLASS] == DEVICE_CLASS_PROBLEM:
            cg.add(parent.set_power_interference_binary_sensor(var))
        elif config[CONF_DEVICE_CLASS] == DEVICE_CLASS_PRESENCE:
            cg.add(parent.set_presence_binary_sensor(var))
        elif config[CONF_DEVICE_CLASS] == DEVICE_CLASS_MOTION:
            cg.add(parent.set_presence_binary_sensor(var))
    else:
        cg.add(parent.set_presence_binary_sensor(var))
