// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#ifdef USE_ESP32

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome::usb_webcam {

class USBWebCamNumber : public number::Number {
 public:
  void set_selector(uint8_t selector) { selector_ = selector; }

 protected:
  void control(float value) override;
  uint8_t selector_{0};
};

}  // namespace esphome::usb_webcam
#endif
