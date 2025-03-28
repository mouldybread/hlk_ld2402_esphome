import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_DEVICE_CLASS,
    CONF_DISTANCE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_PRESENCE,
    DEVICE_CLASS_MOTION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_ACCURACY_DECIMALS,
    CONF_PLATFORM,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor"]

hlk_ld2402_ns = cg.esphome_ns.namespace("hlk_ld2402")
HLKLD2402Component = hlk_ld2402_ns.class_(
    "HLKLD2402Component", cg.Component, uart.UARTDevice
)

# Custom configs
CONF_PRESENCE = "presence"
CONF_MICROMOVEMENT = "micromovement"
CONF_MAX_DISTANCE = "max_distance"
CONF_TIMEOUT = "timeout"
CONF_UART_ID = "uart_id"
CONF_HLK_LD2402_ID = "hlk_ld2402_id"

# Main component schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2402Component),
    cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_MAX_DISTANCE, default=5.0): cv.float_range(min=0.7, max=10.0),
    cv.Optional(CONF_TIMEOUT, default=5): cv.int_range(min=0, max=65535),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_MAX_DISTANCE in config:
        cg.add(var.set_max_distance(config[CONF_MAX_DISTANCE]))
    if CONF_TIMEOUT in config:
        cg.add(var.set_timeout(config[CONF_TIMEOUT]))

# Define platform schemas without overriding global schemas
DISTANCE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CENTIMETER,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_DISTANCE,
    state_class=STATE_CLASS_MEASUREMENT,
).extend({
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
})

BINARY_SENSOR_SCHEMA = binary_sensor.binary_sensor_schema().extend({
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Required(CONF_DEVICE_CLASS): cv.one_of(DEVICE_CLASS_PRESENCE, DEVICE_CLASS_MOTION),
})

async def hlk_ld2402_sensor_to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await sensor.new_sensor(config)
    cg.add(parent.set_distance_sensor(var))

async def hlk_ld2402_binary_sensor_to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await binary_sensor.new_binary_sensor(config)
    if config[CONF_DEVICE_CLASS] == DEVICE_CLASS_PRESENCE:
        cg.add(parent.set_presence_binary_sensor(var))
    elif config[CONF_DEVICE_CLASS] == DEVICE_CLASS_MOTION:
        cg.add(parent.set_micromovement_binary_sensor(var))

# Register platforms properly - use function call style instead of decorators
sensor.register_sensor(
    "hlk_ld2402",
    DISTANCE_SENSOR_SCHEMA,
    hlk_ld2402_sensor_to_code,
)

binary_sensor.register_binary_sensor(
    "hlk_ld2402",
    BINARY_SENSOR_SCHEMA,
    hlk_ld2402_binary_sensor_to_code,
)