import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, CONF_HLK_LD2402_ID, CONF_TYPE

from .. import hlk_ld2402_ns, HLKLD2402Component

DEPENDENCIES = ["hlk_ld2402"]
AUTO_LOAD = ["hlk_ld2402"]

CONF_PRESENCE = "presence"
CONF_MOVEMENT = "movement"
CONF_MICROMOVEMENT = "micromovement"

HLKLD2402BinarySensor = hlk_ld2402_ns.class_(
    "HLKLD2402BinarySensor", binary_sensor.BinarySensor, cg.Component
)

def validate_binary_sensor(value):
    value = cv.Schema({
        cv.GenerateID(): cv.declare_id(HLKLD2402BinarySensor),
        cv.GenerateID(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
        cv.Required(CONF_TYPE): cv.one_of(
            CONF_PRESENCE, CONF_MOVEMENT, CONF_MICROMOVEMENT, lower=True
        ),
    })(value)
    
    if value[CONF_TYPE] == CONF_PRESENCE:
        value = binary_sensor.binary_sensor_schema(
            device_class="presence"
        )(value)
    return value

CONFIG_SCHEMA = validate_binary_sensor

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await binary_sensor.register_binary_sensor(var, config)
    
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(config[CONF_TYPE]))