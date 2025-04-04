import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TIMEOUT, CONF_UPDATE_INTERVAL

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

hlk_ld2402_ns = cg.esphome_ns.namespace("hlk_ld2402")
HLKLD2402Component = hlk_ld2402_ns.class_("HLKLD2402Component", cg.Component, uart.UARTDevice)

# Component configuration keys
CONF_HLK_LD2402_ID = "hlk_ld2402_id"
CONF_MAX_DISTANCE = "max_distance"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2402Component),
    cv.Required(uart.CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_MAX_DISTANCE, default=10.0): cv.float_range(min=0.7, max=10.0),
    cv.Optional(CONF_TIMEOUT, default=5): cv.positive_int,
    cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Configure UART
    uart_component = await cg.get_variable(config[uart.CONF_UART_ID])
    cg.add(var.set_uart(uart_component))
    
    # Configure additional parameters
    cg.add(var.set_max_distance(config[CONF_MAX_DISTANCE]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    
    # Add update interval
    if CONF_UPDATE_INTERVAL in config:
        update_interval = config[CONF_UPDATE_INTERVAL]
        cg.add(var.set_update_interval(update_interval))