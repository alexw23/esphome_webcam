// SPDX-License-Identifier: GPL-3.0-only
// This file is derrived from esp32_camera component of ESPHome and from usb_camera_mic_spk example by Espressif
#ifdef USE_ESP32

#include "usb_webcam.h"
#include "esphome/components/camera/camera.h"
#include "usb/uvc_host.h"
#include "usb/usb_host.h"
#include "esp_timer.h"
#ifdef CONFIG_ESP32_S3_USB_OTG
#include "bsp/esp-bsp.h"
#endif

#include "esphome/core/log.h"
#include "driver/gpio.h"

#include <freertos/event_groups.h>
#include <freertos/task.h>


static const char *const TAG = "usb_webcam";

#define BIT0_FRAME_START     (0x01 << 0)
#define BIT1_NEW_FRAME_START (0x01 << 1)
#define BIT2_NEW_FRAME_END   (0x01 << 2)

namespace esphome::usb_webcam {

static uint32_t s_drop_frame_size = 0;
static camera_fb_t s_fb;
static uvc_host_stream_hdl_t stream_hdl = NULL;
static int s_frame_cb_count = 0;

void esp_camera_fb_return(camera_fb_t *fb)
{
    if (fb->buf != NULL) {
        heap_caps_free(fb->buf);
        fb->buf = NULL;
    }
}

static bool camera_frame_cb(const uvc_host_frame_t *frame, void *ptr)
{
    s_frame_cb_count++;
    ESP_LOGV(TAG, "FRAME_CB #%d format=%d len=%" PRIu32,
        s_frame_cb_count, frame->vs_format.format, frame->data_len);

    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) return true;
    if (frame->data_len < s_drop_frame_size) return true;

    // Drop incoming frame if previous one is still being consumed
    if (s_fb.buf != NULL) {
        return true;
    }

    // Try this temporarily to isolate if DMA is the cause
    uint8_t *copy = (uint8_t *)heap_caps_aligned_alloc(16, frame->data_len, MALLOC_CAP_SPIRAM);

    if (copy == NULL) {
        ESP_LOGW(TAG, "Frame copy alloc failed, dropping");
        return true;
    }
    memcpy(copy, frame->data, frame->data_len);

    s_fb.buf = copy;
    s_fb.len = frame->data_len;
    s_fb.width = frame->vs_format.h_res;
    s_fb.height = frame->vs_format.v_res;
    s_fb.format = PIXFORMAT_JPEG;

    return true;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    ESP_LOGV(TAG, "STREAM_EVENT type=%d", event->type);

    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB error");
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected");
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        stream_hdl = NULL;
        break;
    default:
        break;
    }
}

esp_err_t usb_host_drivers_install() {
#ifdef CONFIG_ESP32_S3_USB_OTG
  bsp_usb_mode_select_host();
  bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
#endif
  memset(&s_fb, 0, sizeof(camera_fb_t));

  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = 0,
  };
  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
      return err;
  }

  xTaskCreatePinnedToCore([](void *arg) {
      while (true) {
          uint32_t event_flags;
          usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
          if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
              usb_host_device_free_all();
          }
      }
  }, "usb_events", 4096, NULL, 4, NULL, 0);  // core 0

  const uvc_host_driver_config_t uvc_driver_config = {
      .driver_task_stack_size = 8 * 1024,
      .driver_task_priority = 4,
      .xCoreID = 0,
      .create_background_task = true,
  };
  err = uvc_host_install(&uvc_driver_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "uvc_host_install failed: %s", esp_err_to_name(err));
      return err;
  }
  return ESP_OK;
}

esp_err_t esp_camera_init(USBWebCamFrameSize fs, uint32_t fps, uint32_t frame_buffer_size) {

  uint16_t frame_width = 0;
  uint16_t frame_height = 0;
  switch (fs) {
    case USB_WEBCAM_SIZE_160X120:   frame_width = 160;  frame_height = 120; break;
    case USB_WEBCAM_SIZE_176X144:   frame_width = 176;  frame_height = 144; break;
    case USB_WEBCAM_SIZE_240X176:   frame_width = 240;  frame_height = 176; break;
    case USB_WEBCAM_SIZE_320X240:   frame_width = 320;  frame_height = 240; break;
    case USB_WEBCAM_SIZE_400X296:   frame_width = 400;  frame_height = 296; break;
    case USB_WEBCAM_SIZE_640X480:   frame_width = 640;  frame_height = 480; break;
    case USB_WEBCAM_SIZE_800X600:   frame_width = 800;  frame_height = 600; break;
    case USB_WEBCAM_SIZE_1024X768:  frame_width = 1024; frame_height = 768; break;
    case USB_WEBCAM_SIZE_1280X1024: frame_width = 1280; frame_height = 1024; break;
    case USB_WEBCAM_SIZE_1600X1200: frame_width = 1600; frame_height = 1200; break;
    case USB_WEBCAM_SIZE_1920X1080: frame_width = 1920; frame_height = 1080; break;
    case USB_WEBCAM_SIZE_720X1280:  frame_width = 720;  frame_height = 1280; break;
    case USB_WEBCAM_SIZE_864X1536:  frame_width = 864;  frame_height = 1536; break;
    case USB_WEBCAM_SIZE_2048X1536: frame_width = 2048; frame_height = 1536; break;
    case USB_WEBCAM_SIZE_2560X1440: frame_width = 2560; frame_height = 1440; break;
    case USB_WEBCAM_SIZE_2560X1600: frame_width = 2560; frame_height = 1600; break;
    case USB_WEBCAM_SIZE_1080X1920: frame_width = 1080; frame_height = 1920; break;
    case USB_WEBCAM_SIZE_2560X1920: frame_width = 2560; frame_height = 1920; break;
    default: return ESP_ERR_INVALID_ARG;
  }

  uvc_host_stream_config_t stream_config = {
      .event_cb = stream_callback,
      .frame_cb = camera_frame_cb,
      .user_ctx = NULL,
      .usb = {
          .dev_addr = 0,
          .vid = UVC_HOST_ANY_VID,
          .pid = UVC_HOST_ANY_PID,
          .uvc_stream_index = 0,
      },
      .vs_format = {
          .h_res = frame_width,
          .v_res = frame_height,
          .fps = 0, // let driver pick default fps from device descriptors
          .format = UVC_VS_FORMAT_MJPEG,
      },
      .advanced = {
          .number_of_frame_buffers = 2,
          .frame_size = 600000,
          .frame_heap_caps = MALLOC_CAP_SPIRAM,
          .number_of_urbs = 3,
          .urb_size = 3 * 1024,
          .user_frame_buffers = NULL,
      },
  };

  esp_err_t ret = ESP_ERR_NOT_FOUND;
  for (int attempt = 1; attempt <= 5 && ret != ESP_OK; attempt++) {
      global_usb_webcam->open_attempts_ = attempt;
      ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(10000), &stream_hdl);
      global_usb_webcam->last_open_ret_ = ret;
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uvc_host_stream_open attempt %d failed: %s", attempt, esp_err_to_name(ret));
          vTaskDelay(pdMS_TO_TICKS(2000));
      }
  }

  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "uvc_host_stream_open failed after 5 attempts: %s", esp_err_to_name(ret));
      return ret;
  }

  global_usb_webcam->stream_opened_ = true;
  ESP_LOGI(TAG, "Stream opened, deferring start to main loop");
  return ESP_OK;
}

/* ---------------- public API (derivated) ---------------- */
void USBWebCam::camera_init_task(void *pv) {
  USBWebCam *self = static_cast<USBWebCam *>(pv);

  // -- NEW: Sleep for 15 seconds so WiFi and Logger can connect --
  vTaskDelay(pdMS_TO_TICKS(10000));

  ESP_LOGI(TAG, "Starting USB stream open");
  self->init_error_ = esp_camera_init(self->frame_size, 1000 / self->max_update_interval_, self->frame_buffer_size_);
  self->camera_init_done_ = true;
  vTaskDelete(NULL);
}

void USBWebCam::setup() {
  global_usb_webcam = this;
  this->last_update_ = esp_timer_get_time();

  // Configure status LED
  gpio_reset_pin(GPIO_NUM_15);
  gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);

  // Enable verbose logging for UVC subsystem to diagnose enumeration issues
  esp_log_level_set("uvc", ESP_LOG_VERBOSE);
  esp_log_level_set("uvc-control", ESP_LOG_VERBOSE);
  esp_log_level_set("HCD DWC", ESP_LOG_VERBOSE);
  esp_log_level_set("USB HOST", ESP_LOG_VERBOSE);

  // Install USB host and UVC driver early (before WiFi claims interrupt slots)
  esp_err_t err = usb_host_drivers_install();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB driver install failed: %s", esp_err_to_name(err));
    this->init_error_ = err;
    this->camera_init_done_ = true;
    return;
  }

  // Defer slow stream open (device enumeration) to background task
  xTaskCreatePinnedToCore(USBWebCam::camera_init_task, "cam_init", 4096, this, 5, NULL, 0);
}

void USBWebCam::loop() {
  static uint32_t hb = 0;
  if (++hb % 100 == 0) {
    ESP_LOGV(TAG, "HEARTBEAT: init_done=%d ready=%d hdl=%p attempts=%d start_ret=%d s_frame_cb_count=%d (%s)",
    this->camera_init_done_, this->camera_ready_, stream_hdl, this->open_attempts_,
    this->last_start_ret_, s_frame_cb_count, esp_err_to_name(this->last_start_ret_));
  }
  static bool logged = false;
  if (this->camera_init_done_ && !logged) {
      logged = true;
      ESP_LOGI(TAG, "POST-BOOT: init_done=1 attempts=%d last_open_ret=%s stream_hdl=%p",
          this->open_attempts_, esp_err_to_name(this->last_open_ret_), stream_hdl);
  }
  if (!this->camera_init_done_) return;
  // Deferred stream start — runs once, after API is up, so errors are visible over WiFi
  if (this->stream_opened_ && !this->start_attempted_) {
    this->start_attempted_ = true;
    ESP_LOGI(TAG, "Attempting uvc_host_stream_start now...");
    esp_err_t ret = uvc_host_stream_start(stream_hdl);

    this->last_start_ret_ = ret;
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "stream_start FAILED: %d (%s)", ret, esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "stream_start OK — frames should flow now");

      ESP_LOGI(TAG, "Stream started — waiting for frames from callback");
      ESP_LOGI(TAG, "Checking compile-time log level...");
      // Force IDF to log everything including USB host internals
      esp_log_level_set("UVC", ESP_LOG_VERBOSE);
      esp_log_level_set("uvc", ESP_LOG_VERBOSE);  
      esp_log_level_set("USB_HOST", ESP_LOG_VERBOSE);
      esp_log_level_set("USBH", ESP_LOG_VERBOSE);
      esp_log_level_set("HCD", ESP_LOG_VERBOSE);
    }
  }
  if (!this->camera_ready_) {
    if (this->init_error_ != ESP_OK) {
      ESP_LOGE(TAG, "Setup Failed: %s", esp_err_to_name(this->init_error_));
      this->mark_failed();
    } else {
      ESP_LOGI(TAG, "Camera ready");
      this->camera_ready_ = true;
    }
  }
  // Always drain frames to prevent buffer starvation
  if (this->camera_ready_ && this->start_attempted_) {
    request_image(camera::IDLE);
  }
}

void USBWebCam::dump_config() {
  ESP_LOGCONFIG(TAG, "USB WebCam:");
  ESP_LOGCONFIG(TAG, "  Frame Size: %d", this->frame_size);
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %" PRIu32, this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  Max Update Interval: %" PRIu32, this->max_update_interval_);
  ESP_LOGCONFIG(TAG, "  Idle Update Interval: %" PRIu32, this->idle_update_interval_);
  ESP_LOGCONFIG(TAG, "  Stream Requesters: %d", this->stream_requesters_);
  if (this->init_error_ != ESP_OK) {
    ESP_LOGCONFIG(TAG, "  Init Error: %s", esp_err_to_name(this->init_error_));
  }
}

float USBWebCam::get_setup_priority() const { return setup_priority::BUS; }

/* ---------------- public API (specific) ---------------- */
void USBWebCam::start_stream(camera::CameraRequester requester) {
  uint8_t val = (1U << (uint32_t) requester);
  if (!this->stream_requesters_) {
    this->stream_start_callback_.call();
    this->stream_requesters_ |= val;
  }
  ESP_LOGD(TAG, "start_stream! %d", this->stream_requesters_);
}

void USBWebCam::stop_stream(camera::CameraRequester requester) {
  uint8_t old_mask = this->stream_requesters_;
  uint8_t val = (1U << (uint32_t) requester);
  this->stream_requesters_ &= ~val;
  if (old_mask && !this->stream_requesters_) {
    this->stream_stop_callback_.call();
    ESP_LOGD(TAG, "stop_stream! %d", this->stream_requesters_);
  }
}

camera_fb_t *esp_camera_fb_get()
{
    if (s_fb.buf == NULL) return nullptr;
    return &s_fb;
}

void USBWebCam::request_image(camera::CameraRequester requester) {
  if (!this->camera_ready_ || !this->start_attempted_ || stream_hdl == NULL) {
    return;
  }

  uint8_t val = (1U << (uint32_t) requester);
  uint32_t now = esp_timer_get_time();
  uint32_t mui = this->max_update_interval_ * 1000;

  if (this->stream_requesters_) {
    if ((now - this->last_update_) < mui) {
      if (this->current_image_)
          this->single_requesters_ |= val;
      return;
    }
  } else {
    mui = this->idle_update_interval_ * 1000;
    if ((now - this->last_update_) < mui) {
      if ((now - this->last_idle_request_) > mui || !this->current_image_) {
         this->last_idle_request_ = now;
         this->single_requesters_ |= val;
      }
      return;
    }
  }

  this->single_requesters_ |= val;

  if (stream_hdl == NULL) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    ESP_LOGV(TAG, "No frame ready yet");
    this->last_update_ = now;
    return;
  }

  ESP_LOGD(TAG, "fb %p, len %u, wh %ux%u", fb->buf, fb->len, fb->width, fb->height);

  std::shared_ptr<USBWebCamImage> image = std::make_shared<USBWebCamImage>(fb, this->single_requesters_);

  for (auto *listener : this->listeners_) {
    listener->on_camera_image(image);
  }

  this->current_image_ = image;
  this->single_requesters_ = 0;
  this->last_update_ = now;
}

void USBWebCam::update_camera_parameters() {
// TODO
}

void USBWebCam::add_stream_start_callback(std::function<void()> &&callback) {
  this->stream_start_callback_.add(std::move(callback));
}
void USBWebCam::add_stream_stop_callback(std::function<void()> &&callback) {
  this->stream_stop_callback_.add(std::move(callback));
}

camera::CameraImageReader *USBWebCam::create_image_reader() { return new USBWebCamImageReader(); }

/* ---------------- internal methods ---------------- */
bool USBWebCam::has_requested_image_() const { return this->single_requesters_ != 0; }

bool USBWebCam::can_return_image_() const { return this->current_image_.use_count() == 1; }


/* ---------------- setters ---------------- */

void USBWebCam::set_frame_size(USBWebCamFrameSize size) { this->frame_size = size; }
void USBWebCam::set_drop_size(uint32_t drop_size) { s_drop_frame_size = drop_size; }
void USBWebCam::set_frame_buffer_size(uint32_t frame_buffer_size) { this->frame_buffer_size_ = frame_buffer_size; }
void USBWebCam::set_max_update_interval(uint32_t max_update_interval) {
  this->max_update_interval_ = max_update_interval;
}
void USBWebCam::set_idle_update_interval(uint32_t idle_update_interval) {
  this->idle_update_interval_ = idle_update_interval;
}

USBWebCam::USBWebCam() {}


// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
USBWebCam *global_usb_webcam{nullptr};

/* ---------------- CameraImage class implementations ---------------- */
USBWebCamImage::USBWebCamImage(camera_fb_t *buffer, uint8_t requester) : camera::CameraImage(), buffer_(buffer), requesters_(requester) {}
camera_fb_t *USBWebCamImage::get_raw_buffer() { return this->buffer_; }
uint8_t *USBWebCamImage::get_data_buffer() { return this->buffer_->buf; }
size_t USBWebCamImage::get_data_length() { return this->buffer_->len; }
bool USBWebCamImage::was_requested_by(camera::CameraRequester requester) const {
  return (this->requesters_ & (1U << (uint32_t) requester)) != 0;
}


/* ---------------- CameraImageReader class implementations ---------------- */
void USBWebCamImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<USBWebCamImage>(image);
  this->offset_ = 0;
}
size_t USBWebCamImageReader::available() const {
  if (!this->image_) {
    return 0;
  }
  return this->image_->get_data_length() - this->offset_;
}
uint8_t *USBWebCamImageReader::peek_data_buffer() {
  if (!this->image_) {
    return nullptr;
  }
  return this->image_->get_data_buffer() + this->offset_;
}
void USBWebCamImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }
void USBWebCamImageReader::return_image() {
  if (!this->image_) {
    return;
  }
  if (this->image_.use_count() == 2) {
    esp_camera_fb_return(this->image_->get_raw_buffer());
  }
  this->image_.reset();
}

}  // namespace esphome::usb_webcam

#endif
