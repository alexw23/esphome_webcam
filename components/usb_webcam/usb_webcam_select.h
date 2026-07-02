// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#ifdef USE_ESP32

#include "esphome/components/select/select.h"
#include <vector>
#include <string>

namespace esphome::usb_webcam {

class USBWebCam;

struct VideoMode {
    uint16_t width;
    uint16_t height;
    uint16_t fps;
};

class USBWebCamSelect : public select::Select {
 public:
  void set_parent(USBWebCam *parent) { parent_ = parent; }
  std::vector<VideoMode> modes;
  std::vector<std::string> mode_labels;  // keeps label strings alive for traits c_str() pointers

 protected:
  void control(const std::string &value) override;
  USBWebCam *parent_{nullptr};
};

}  // namespace esphome::usb_webcam
#endif
