// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#ifdef USE_ESP32

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"

namespace esphome::usb_webcam {

class USBWebCamNumber : public number::Number {
 public:
  void set_selector(uint8_t selector) {
    selector_ = selector;
    // unique key per control selector
    pref_ = global_preferences->make_preference<float>(fnv1_hash("usb_webcam_ctrl") ^ selector);
  }

  ESPPreferenceObject pref_;

 protected:
  void control(float value) override;
  uint8_t selector_{0};
};

}  // namespace esphome::usb_webcam
#endif
