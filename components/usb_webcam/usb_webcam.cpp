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

#include <freertos/event_groups.h>
#include <freertos/task.h>


static const char *const TAG = "usb_webcam";

#define BIT0_FRAME_START     (0x01 << 0)
#define BIT1_NEW_FRAME_START (0x01 << 1)
#define BIT2_NEW_FRAME_END   (0x01 << 2)

namespace esphome::usb_webcam {

static EventGroupHandle_t s_evt_handle;
static uint32_t s_drop_frame_size = 0;
static camera_fb_t s_fb;
static uvc_host_stream_hdl_t stream_hdl = NULL;

camera_fb_t *esp_camera_fb_get()
{
    // Clear out any old flags before we wait
    xEventGroupClearBits(s_evt_handle, BIT1_NEW_FRAME_START | BIT2_NEW_FRAME_END);
    xEventGroupSetBits(s_evt_handle, BIT0_FRAME_START);
    xEventGroupWaitBits(s_evt_handle, BIT1_NEW_FRAME_START, true, true, portMAX_DELAY);
    return &s_fb;
}

void esp_camera_fb_return(camera_fb_t *fb)
{
    xEventGroupSetBits(s_evt_handle, BIT2_NEW_FRAME_END);
    return;
}

static bool camera_frame_cb(const uvc_host_frame_t *frame, void *ptr)
{
    if (!(xEventGroupGetBits(s_evt_handle) & BIT0_FRAME_START)) {
        return true;
    }

    // Once we accept the frame, we don't want to accept another until requested
    xEventGroupClearBits(s_evt_handle, BIT0_FRAME_START | BIT2_NEW_FRAME_END);
    ESP_LOGV(TAG, "uvc frame w = %d, h = %d, length = %u",
             frame->vs_format.h_res, frame->vs_format.v_res, frame->data_len);

    if(frame->data_len < s_drop_frame_size) {
      ESP_LOGV(TAG, "Dropping frame size %u < %u", frame->data_len, s_drop_frame_size);
      return true;
    }

    switch (frame->vs_format.format) {
    case UVC_VS_FORMAT_MJPEG:
        s_fb.buf = (uint8_t*)frame->data;
        s_fb.len = frame->data_len;
        s_fb.width = frame->vs_format.h_res;
        s_fb.height = frame->vs_format.v_res;
        s_fb.format = PIXFORMAT_JPEG;
        xEventGroupSetBits(s_evt_handle, BIT1_NEW_FRAME_START);
        ESP_LOGV(TAG, "send frame length %u", frame->data_len);
        xEventGroupWaitBits(s_evt_handle, BIT2_NEW_FRAME_END, true, true, portMAX_DELAY);
        ESP_LOGV(TAG, "send frame length %u done", frame->data_len);
        break;
    default:
        ESP_LOGW(TAG, "Format not supported");
        assert(0);
        break;
    }
    return true;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
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

esp_err_t esp_camera_init(USBWebCamFrameSize fs, uint32_t fps, uint32_t frame_buffer_size) {
#ifdef CONFIG_ESP32_S3_USB_OTG
  bsp_usb_mode_select_host();
  bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
#endif  
  memset(&s_fb, 0, sizeof(camera_fb_t));
  s_evt_handle = xEventGroupCreate();
  if (s_evt_handle == NULL) {
      ESP_LOGE(TAG, "Event group create failed");
      assert(0);
  }

  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LOWMED,
  };
  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
      return err;
  }
  
  const uvc_host_driver_config_t uvc_driver_config = {
      .driver_task_stack_size = 8 * 1024,
      .driver_task_priority = 6,
      .xCoreID = tskNO_AFFINITY,
      .create_background_task = true,
  };
  err = uvc_host_install(&uvc_driver_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "uvc_host_install failed: %s", esp_err_to_name(err));
      return err;
  }

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
          .vid = 0,
          .pid = 0,
          .uvc_stream_index = 0,
      },
      .vs_format = {
          .h_res = frame_width,
          .v_res = frame_height,
          .fps = (float)fps, // will be rounded to standard 5/10/15 depending on cam capabilities
          .format = UVC_VS_FORMAT_MJPEG,
      },
      .advanced = {
          .number_of_frame_buffers = 3,
          .frame_size = frame_buffer_size,
          .frame_heap_caps = MALLOC_CAP_SPIRAM,
          .number_of_urbs = 3,
          .urb_size = 4 * 1024,
          .user_frame_buffers = NULL,
      },
  };

  esp_err_t ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(5000), &stream_hdl);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "uvc_host_stream_open failed: %s", esp_err_to_name(ret));
      return ret;
  }

  ret = uvc_host_stream_start(stream_hdl);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "uvc_host_stream_start failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* ---------------- public API (derivated) ---------------- */
static void camera_init_task(void *pv) {
  USBWebCam *self = static_cast<USBWebCam *>(pv);
  self->init_error_ = esp_camera_init(self->frame_size, 1000 / self->max_update_interval_, self->frame_buffer_size_);
  self->camera_init_done_ = true;
  vTaskDelete(NULL);
}

void USBWebCam::setup() {
  global_usb_webcam = this;
  this->last_update_ = esp_timer_get_time();
  xTaskCreate(camera_init_task, "cam_init", 4096, this, 5, NULL);
}

void USBWebCam::loop() {
  if (!this->camera_init_done_) return;
  if (!this->camera_ready_) {
    if (this->init_error_ != ESP_OK) {
      ESP_LOGE(TAG, "Setup Failed: %s", esp_err_to_name(this->init_error_));
      this->mark_failed();
    } else {
      ESP_LOGI(TAG, "Camera ready");
      this->camera_ready_ = true;
    }
  }
  if (this->has_requested_image_() || this->stream_requesters_) {
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

float USBWebCam::get_setup_priority() const { return setup_priority::LATE; }

/* ---------------- public API (specific) ---------------- */
void USBWebCam::start_stream(camera::CameraRequester requester) {
  uint32_t val = (uint32_t) requester;
  if (!this->stream_requesters_)
    this->stream_start_callback_.call();
  this->stream_requesters_ |= val;
  // ESP_LOGD(TAG, "start_stream! %d", this->stream_requesters_);
}

void USBWebCam::stop_stream(camera::CameraRequester requester) {
  uint8_t old_mask = this->stream_requesters_;
  uint32_t val = (uint32_t) requester;
  this->stream_requesters_ &= ~val;
  if (old_mask && !this->stream_requesters_)
    this->stream_stop_callback_.call();
  // ESP_LOGD(TAG, "stop_stream! %d", this->stream_requesters_);
}

void USBWebCam::request_image(camera::CameraRequester requester) {
  uint32_t val = (uint32_t) requester;

  uint32_t now = esp_timer_get_time();
  uint32_t mui = this->max_update_interval_ * 1000;
  if (this->stream_requesters_) { // fast stream requested by web server
    if ((now - this->last_update_) < mui) {
      if (this->current_image_) // ensure quick start without black screen on web
          this->single_requesters_ |= val;
      return;
    }
  } else {
    mui = this->idle_update_interval_ * 1000;
    if ((now - this->last_update_) < mui) {
      if ((now - this->last_idle_request_) > mui || !this->current_image_) {
         this->last_idle_request_ = now;
         this->single_requesters_ |= val; // schedule request
      }
      return; // wait
    }
  }
  // take the request
  this->single_requesters_ |= val;

  esp_err_t err = ESP_OK;

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    ESP_LOGE(TAG, "Got nullptr returning from esp_camera_fb_get()");
    return;
  }
  // ESP_LOGI(TAG, "fb %p, len %u, wh %ux%u", fb->buf, fb->len, fb->width, fb->height);

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
  return (this->requesters_ & requester) != 0;
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
