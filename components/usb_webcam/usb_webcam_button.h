// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#ifdef USE_ESP32

#include "esphome/components/button/button.h"

namespace esphome::usb_webcam {

class USBWebCamButton : public button::Button {
 protected:
  void press_action() override;
};

}  // namespace esphome::usb_webcam
#endif
