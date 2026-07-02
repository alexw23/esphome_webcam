// SPDX-License-Identifier: GPL-3.0-only
// This file is derrived from original esp32_camera component
// from ESPHome with modifications for usb_webcam component

#pragma once

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "usb_webcam_number.h"
#include "usb_webcam_button.h"
#include "usb_webcam_select.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>
#include <queue>


namespace esphome::usb_webcam {
using namespace esphome::camera;
/* ---------------- enum classes ---------------- */

typedef enum {
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_YUV422,    // 2BPP/YUV422
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
    PIXFORMAT_RGB888,    // 3BPP/RGB888
    PIXFORMAT_RAW,       // RAW
    PIXFORMAT_RGB444,    // 3BP2P/RGB444
    PIXFORMAT_RGB555,    // 3BP2P/RGB555
} pixformat_t;

/* ---------------- CameraImage class ---------------- */
typedef struct {
    uint8_t * buf;              // Pointer to the pixel data
    size_t len;                 // Length of the buffer in bytes
    size_t width;               // Width of the buffer in pixels
    size_t height;              // Height of the buffer in pixels
    pixformat_t format;         // Format of the pixel data
    struct timeval timestamp;   // Timestamp since boot of the first DMA buffer of the frame
} camera_fb_t;

// Pre-allocate space for 3 frames to allow for queuing
#define NUM_BUFFERS 3
static std::queue<uint8_t *> s_free_buffers;
static std::queue<camera_fb_t *> s_ready_buffers;
static SemaphoreHandle_t s_buffer_mutex = xSemaphoreCreateMutex();

class USBWebCam;

class USBWebCamImage : public camera::CameraImage {
 public:
  USBWebCamImage(camera_fb_t *buffer, uint8_t requester);
  ~USBWebCamImage();
  camera_fb_t *get_raw_buffer();
  uint8_t *get_data_buffer() override;
  size_t get_data_length() override;
  bool was_requested_by(camera::CameraRequester requester) const override;

 protected:
  camera_fb_t *buffer_;
  uint8_t requesters_;
};

struct CameraImageData {
  uint8_t *data;
  size_t length;
};

/* ---------------- CameraImageReader class ---------------- */
class USBWebCamImageReader : public camera::CameraImageReader {
 public:
  USBWebCamImageReader() {}
  ~USBWebCamImageReader() {}
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<USBWebCamImage> image_;
  size_t offset_{0};
};

/* ---------------- USBWebCam class ---------------- */
class USBWebCam : public camera::Camera {
 public:
  USBWebCam();

  int last_open_ret_ = -999;
  int last_start_ret_ = -999;
  int open_attempts_ = 0;
  bool stream_started_ = false;
  bool stream_opened_ = false;
  bool start_attempted_ = false;

  /* setters */
  void set_drop_size(uint32_t drop_size);
  void set_frame_buffer_size(uint32_t frame_buffer_size);
  void set_idle_update_interval(uint32_t idle_update_interval);
  /* -- camera controls */
  void set_processing_unit_id(uint8_t id) { processing_unit_id_ = id; }
  uint8_t get_processing_unit_id() const { return processing_unit_id_; }
  bool is_controls_probed() const { return controls_probed_; }
  void set_pu_controls_bitmap(uint32_t bm) { pu_controls_bitmap_ = bm; }
  void set_stream_button(USBWebCamButton *b) { stream_button_ = b; }
  void set_mode_select(USBWebCamSelect *s) { mode_select_ = s; }
  void change_video_mode(uint16_t width, uint16_t height, uint16_t fps);
  bool is_streaming() const { return stream_requesters_ != 0; }
  void set_brightness_number(USBWebCamNumber *n) { brightness_number_ = n; }
  void set_contrast_number(USBWebCamNumber *n)   { contrast_number_ = n; }
  void set_saturation_number(USBWebCamNumber *n) { saturation_number_ = n; }
  void set_hue_number(USBWebCamNumber *n)        { hue_number_ = n; }
  void set_sharpness_number(USBWebCamNumber *n)  { sharpness_number_ = n; }

  /* public API (derivated) */
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  /* public API (specific) */
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;
  void request_image(camera::CameraRequester requester) override;
  void update_camera_parameters();

  /// Add a listener to receive camera events
  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override;

  void add_stream_start_callback(std::function<void()> &&callback);
  void add_stream_stop_callback(std::function<void()> &&callback);

  ESPPreferenceObject mode_pref_;
  std::string pending_mode_label_;  // set by init task, published by loop()

 protected:
  /* internal methods */
  bool has_requested_image_() const;
  bool can_return_image_() const;

  static void camera_init_task(void *pv);

  /* attributes */
  uint32_t frame_buffer_size_{65536};
  volatile bool camera_init_done_{false};
  volatile bool controls_probed_{false};
  bool camera_ready_{false};
  /* -- framerates */
  uint32_t max_update_interval_{1000};
  uint32_t idle_update_interval_{15000};
  uint16_t current_width_{0};
  uint16_t current_height_{0};
  uint8_t processing_unit_id_{2};
  uint32_t pu_controls_bitmap_{0xFFFFFFFF};  // all supported until parsed
  USBWebCamButton *stream_button_{nullptr};
  USBWebCamSelect *mode_select_{nullptr};
  USBWebCamNumber *brightness_number_{nullptr};
  USBWebCamNumber *contrast_number_{nullptr};
  USBWebCamNumber *saturation_number_{nullptr};
  USBWebCamNumber *hue_number_{nullptr};
  USBWebCamNumber *sharpness_number_{nullptr};

  esp_err_t init_error_{ESP_OK};
  std::shared_ptr<USBWebCamImage> current_image_;
  uint8_t single_requesters_{0};
  uint8_t stream_requesters_{0};
  QueueHandle_t framebuffer_get_queue_;
  QueueHandle_t framebuffer_return_queue_;
  std::vector<camera::CameraListener *> listeners_;
  CallbackManager<void()> stream_start_callback_{};
  CallbackManager<void()> stream_stop_callback_{};

  uint64_t last_idle_request_{0};
  uint64_t last_update_{0};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern USBWebCam *global_usb_webcam;

class USBWebCamStreamStartTrigger : public Trigger<> {
 public:
  explicit USBWebCamStreamStartTrigger(USBWebCam *parent) {
    parent->add_stream_start_callback([this]() { this->trigger(); });
  }

 protected:
};
class USBWebCamStreamStopTrigger : public Trigger<> {
 public:
  explicit USBWebCamStreamStopTrigger(USBWebCam *parent) {
    parent->add_stream_stop_callback([this]() { this->trigger(); });
  }

 protected:
};

}  // namespace esphome::usb_webcam

#endif
