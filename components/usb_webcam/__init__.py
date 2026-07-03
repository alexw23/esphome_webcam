# SPDX-License-Identifier: MIT
# This file is derrived from esp32_camera component of ESPHome

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome import pins
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_FREQUENCY,
    CONF_ICON,
    CONF_ID,
    CONF_MODE,
    CONF_NAME,
    CONF_TRIGGER_ID,
)
from esphome.core import CORE, TimePeriod, ID
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome.components.esp32 import add_idf_component
from esphome.components import number, button, select
try:
  from esphome.cpp_helpers import setup_entity
except:
  from esphome.core.entity_helpers import setup_entity

DEPENDENCIES = ["esp32", "camera"]

AUTO_LOAD = ["camera", "psram", "number", "button", "select"]

usb_webcam_ns = cg.esphome_ns.namespace("usb_webcam")
USBWebCam = usb_webcam_ns.class_("USBWebCam", cg.PollingComponent, cg.EntityBase)
USBWebCamNumber = usb_webcam_ns.class_("USBWebCamNumber", number.Number)
USBWebCamButton = usb_webcam_ns.class_("USBWebCamButton", button.Button)
USBWebCamSelect = usb_webcam_ns.class_("USBWebCamSelect", select.Select)
USBWebCamStreamStartTrigger = usb_webcam_ns.class_(
    "USBWebCamStreamStartTrigger",
    automation.Trigger.template(),
)
USBWebCamStreamStopTrigger = usb_webcam_ns.class_(
    "USBWebCamStreamStopTrigger",
    automation.Trigger.template(),
)

CONF_IDLE_FRAMERATE = "idle_framerate"
CONF_DROP_FRAME_SIZE = "drop_frame_size"
CONF_FRAME_BUFFER_SIZE = "frame_buffer_size"

CONF_PROCESSING_UNIT_ID = "processing_unit_id"

# stream trigger
CONF_ON_STREAM_START = "on_stream_start"
CONF_ON_STREAM_STOP = "on_stream_stop"

CONFIG_SCHEMA = cv.ENTITY_BASE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(USBWebCam),
        cv.Optional(CONF_IDLE_FRAMERATE, default="0.1 fps"): cv.All(
            cv.framerate, cv.Range(min=0, max=1)
        ),
        cv.Optional(CONF_DROP_FRAME_SIZE, default="7000"): cv.All(
            cv.int_range(min=0, max=100000)
        ),
        cv.Optional(CONF_FRAME_BUFFER_SIZE, default="614400"): cv.All(
            cv.int_range(min=10240, max=2097152)
        ),
        cv.Optional(CONF_PROCESSING_UNIT_ID, default=2): cv.int_range(min=1, max=255),
        cv.Optional(CONF_ON_STREAM_START): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    USBWebCamStreamStartTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_STREAM_STOP): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    USBWebCamStreamStopTrigger
                ),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

def _final_validate(config):
    return

FINAL_VALIDATE_SCHEMA = _final_validate

async def to_code(config):
    cg.add_define("USE_CAMERA")
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)

    if config[CONF_IDLE_FRAMERATE] == 0:
        cg.add(var.set_idle_update_interval(0))
    else:
        cg.add(var.set_idle_update_interval(1000 / config[CONF_IDLE_FRAMERATE]))
    cg.add(var.set_drop_size(config[CONF_DROP_FRAME_SIZE]))
    cg.add(var.set_frame_buffer_size(config[CONF_FRAME_BUFFER_SIZE]))
    cg.add(var.set_processing_unit_id(config[CONF_PROCESSING_UNIT_ID]))

    parent_id = config[CONF_ID].id
    for ctrl_name, setter, selector_id in [
        ("brightness", "set_brightness_number", 0x02),
        ("contrast",   "set_contrast_number",   0x03),
        ("saturation", "set_saturation_number", 0x07),
        ("hue",        "set_hue_number",        0x06),
        ("sharpness",  "set_sharpness_number",  0x08),
    ]:
        num_id = ID(f"{parent_id}_{ctrl_name}", is_declaration=True, type=USBWebCamNumber)
        num_var = cg.new_Pvariable(num_id)
        cg.add(num_var.set_selector(selector_id))
        await number.register_number(
            num_var,
            {
                CONF_ID: num_id,
                CONF_NAME: ctrl_name.capitalize(),
                CONF_DISABLED_BY_DEFAULT: False,
                CONF_ICON: "",
                CONF_ENTITY_CATEGORY: "",
                CONF_MODE: number.NumberMode.NUMBER_MODE_AUTO,
            },
            min_value=-32768,
            max_value=32767,
            step=1,
        )
        cg.add(getattr(var, setter)(num_var))

    btn_id = ID(f"{parent_id}_stream_toggle", is_declaration=True, type=USBWebCamButton)
    btn_var = cg.new_Pvariable(btn_id)
    await button.register_button(btn_var, {
        CONF_ID: btn_id,
        CONF_NAME: "Stream Toggle",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: "mdi:camera",
        CONF_ENTITY_CATEGORY: "",
    })
    cg.add(var.set_stream_button(btn_var))

    sel_id = ID(f"{parent_id}_video_mode", is_declaration=True, type=USBWebCamSelect)
    sel_var = cg.new_Pvariable(sel_id)
    await select.register_select(sel_var, {
        CONF_ID: sel_id,
        CONF_NAME: "Video Mode",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: "mdi:video",
        CONF_ENTITY_CATEGORY: "",
    }, options=["(detecting...)"])
    cg.add(sel_var.set_parent(var))
    cg.add(var.set_mode_select(sel_var))

    cg.add_define("USE_USB_WEBCAM")

    # assert(CORE.is_esp_idf)
    add_idf_component(
            name="usb_host_uvc",
            repo="https://github.com/alexw23/esp-usb.git",
            path="host/class/uvc/usb_host_uvc"
    )

    for d, v in {
        #"CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT": True,
        "CONFIG_RTCIO_SUPPORT_RTC_GPIO_DESC": True,
        "CONFIG_USB_OTG_SUPPORTED": True,
        "CONFIG_SOC_USB_OTG_SUPPORTED": True,
        "CONFIG_SPIRAM_USE_MALLOC": True,
        "CONFIG_ESP_WIFI_IRAM_OPT": False,
        "CONFIG_ESP_WIFI_RX_IRAM_OPT": False,
    }.items():
        add_idf_sdkconfig_option(d, v)

    for conf in config.get(CONF_ON_STREAM_START, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_STREAM_STOP, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
