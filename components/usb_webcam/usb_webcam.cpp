// SPDX-License-Identifier: GPL-3.0-only
// This file is derrived from esp32_camera component of ESPHome and from usb_camera_mic_spk example by Espressif
#ifdef USE_ESP32

#include "usb_webcam.h"
#include "usb_webcam_select.h"
#include "esphome/components/camera/camera.h"
#include "usb/uvc_host.h"
#include "usb/usb_host.h"
// forward-declare the private UVC control transfer API
extern "C" esp_err_t uvc_host_usb_ctrl(uvc_host_stream_hdl_t stream_hdl, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, uint8_t *data);
#include "esp_timer.h"
#ifdef CONFIG_ESP32_S3_USB_OTG
#include "bsp/esp-bsp.h"
#endif

#include "esphome/core/log.h"
#include "driver/gpio.h"

#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <algorithm>


static const char *const TAG = "usb_webcam";

#define BIT0_FRAME_START     (0x01 << 0)
#define BIT1_NEW_FRAME_START (0x01 << 1)
#define BIT2_NEW_FRAME_END   (0x01 << 2)

namespace esphome::usb_webcam {

static uint32_t fetch_pu_bitmap(uint8_t unit_id);
static std::vector<VideoMode> parse_uvc_formats();

static uint32_t s_drop_frame_size = 0;
static camera_fb_t s_fb;
static uvc_host_stream_hdl_t stream_hdl = NULL;
static int s_frame_cb_count = 0;

void esp_camera_fb_return(camera_fb_t *fb)
{
    if (fb && fb->buf != NULL) {
        xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
        s_free_buffers.push(fb->buf);
        xSemaphoreGive(s_buffer_mutex);
        fb->buf = NULL;
    }
    delete fb;
}

static bool camera_frame_cb(const uvc_host_frame_t *frame, void *ptr)
{
    s_frame_cb_count++;

    ESP_LOGV(TAG, "frame_cb: frame=%p, data=%p, len=%zu, format=%d, h_res=%d, v_res=%d, fps=%.2f",
             frame, frame->data, frame->data_len, frame->vs_format.format,
             frame->vs_format.h_res, frame->vs_format.v_res, frame->vs_format.fps);

    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) return true;

    if (xSemaphoreTake(s_buffer_mutex, 0) == pdTRUE) {

        // Only process if we have an empty slot available
        if (!s_free_buffers.empty()) {
            uint8_t *buf = s_free_buffers.front();
            s_free_buffers.pop();
            memcpy(buf, frame->data, frame->data_len);

            camera_fb_t *fb = new camera_fb_t();
            fb->buf = buf;
            fb->len = frame->data_len;
            fb->width = frame->vs_format.h_res;
            fb->height = frame->vs_format.v_res;
            fb->format = PIXFORMAT_JPEG;

            s_ready_buffers.push(fb);
        }

        xSemaphoreGive(s_buffer_mutex);
    } else {
        ESP_LOGV(TAG, "Frame dropped: Mutex busy");
    }
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
  }, "usb_events", 4096, NULL, 15, NULL, tskNO_AFFINITY);

  const uvc_host_driver_config_t uvc_driver_config = {
      .driver_task_stack_size = 8 * 1024,
      .driver_task_priority = 3,
      .xCoreID = tskNO_AFFINITY,
      .create_background_task = true,
  };
  err = uvc_host_install(&uvc_driver_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "uvc_host_install failed: %s", esp_err_to_name(err));
      return err;
  }
  return ESP_OK;
}

static esp_err_t open_stream(uint16_t frame_width, uint16_t frame_height, uint16_t fps, uint32_t frame_buffer_size) {
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
          .fps = (float)fps,
          .format = UVC_VS_FORMAT_MJPEG,
      },
      .advanced = {
          .number_of_frame_buffers = 3,
          .frame_size = 614400,
          .frame_heap_caps = MALLOC_CAP_SPIRAM,
          .number_of_urbs = 4,
          .urb_size = 10 * 1024,
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
  return ESP_OK;
}

/* ---------------- public API (derivated) ---------------- */
void USBWebCam::camera_init_task(void *pv) {
  USBWebCam *self = static_cast<USBWebCam *>(pv);

  vTaskDelay(pdMS_TO_TICKS(10000));

  // Wait for USB device to enumerate (up to 15s)
  ESP_LOGI(TAG, "Waiting for USB device to enumerate...");
  uint8_t addr_list[8];
  int num_dev = 0;
  for (int i = 0; i < 150 && num_dev == 0; i++) {
      usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_dev);
      if (num_dev == 0) vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (num_dev == 0) {
      ESP_LOGE(TAG, "No USB device found after 15s");
      self->init_error_ = ESP_ERR_NOT_FOUND;
      self->camera_init_done_ = true;
      vTaskDelete(NULL);
      return;
  }
  ESP_LOGI(TAG, "USB device found, parsing video modes...");

  // Parse available MJPEG modes from descriptor
  auto modes = parse_uvc_formats();
  if (modes.empty()) {
      ESP_LOGE(TAG, "No MJPEG video modes found in descriptor");
      self->init_error_ = ESP_ERR_NOT_FOUND;
      self->camera_init_done_ = true;
      vTaskDelete(NULL);
      return;
  }

  // Sort ascending by pixel count, then fps
  std::sort(modes.begin(), modes.end(), [](const VideoMode &a, const VideoMode &b) {
      uint32_t pa = (uint32_t)a.width * a.height;
      uint32_t pb = (uint32_t)b.width * b.height;
      return pa != pb ? pa < pb : a.fps < b.fps;
  });

  // Populate select options
  if (self->mode_select_) {
      self->mode_select_->modes = modes;
      self->mode_select_->mode_labels.clear();
      for (auto &m : modes) {
          char buf[32];
          snprintf(buf, sizeof(buf), "%dx%d @ %dfps", m.width, m.height, m.fps);
          self->mode_select_->mode_labels.push_back(buf);
      }
      FixedVector<const char *> opts;
      for (auto &lbl : self->mode_select_->mode_labels)
          opts.push_back(lbl.c_str());
      self->mode_select_->traits.set_options(opts);
  }

  // Pick mode: NVS preference or lowest res (modes[0])
  VideoMode chosen = modes[0];
  {
      VideoMode saved{};
      if (self->mode_pref_.load(&saved) && saved.width != 0) {
          for (auto &m : modes) {
              if (m.width == saved.width && m.height == saved.height && m.fps == saved.fps) {
                  chosen = m;
                  ESP_LOGI(TAG, "Restored saved mode %dx%d @ %dfps", chosen.width, chosen.height, chosen.fps);
                  break;
              }
          }
      }
  }

  ESP_LOGI(TAG, "Opening stream at %dx%d @ %dfps", chosen.width, chosen.height, chosen.fps);
  self->init_error_ = open_stream(chosen.width, chosen.height, chosen.fps, self->frame_buffer_size_);

  if (self->init_error_ == ESP_OK) {
      self->current_width_  = chosen.width;
      self->current_height_ = chosen.height;
      self->max_update_interval_ = 1000 / chosen.fps;
      self->set_pu_controls_bitmap(fetch_pu_bitmap(self->get_processing_unit_id()));

      if (self->mode_select_) {
          char buf[32];
          snprintf(buf, sizeof(buf), "%dx%d @ %dfps", chosen.width, chosen.height, chosen.fps);
          self->mode_select_->publish_state(buf);
      }
  }

  self->camera_init_done_ = true;
  vTaskDelete(NULL);
}

void USBWebCam::setup() {
  global_usb_webcam = this;
  this->last_update_ = esp_timer_get_time();

  mode_pref_ = global_preferences->make_preference<VideoMode>(fnv1_hash("usb_webcam_mode"));

  // Configure status LED
  gpio_reset_pin(GPIO_NUM_15);
  gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);

  esp_log_level_set("uvc", ESP_LOG_VERBOSE);
  esp_log_level_set("uvc-control", ESP_LOG_VERBOSE);
  esp_log_level_set("HCD DWC", ESP_LOG_VERBOSE);
  esp_log_level_set("USB HOST", ESP_LOG_VERBOSE);

  esp_err_t err = usb_host_drivers_install();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB driver install failed: %s", esp_err_to_name(err));
    this->init_error_ = err;
    this->camera_init_done_ = true;
    return;
  }

  xTaskCreatePinnedToCore(USBWebCam::camera_init_task, "cam_init", 4096, this, 5, NULL, 0);

  for (int i = 0; i < NUM_BUFFERS; i++) {
    uint8_t *buf = (uint8_t *)heap_caps_aligned_alloc(16, frame_buffer_size_, MALLOC_CAP_SPIRAM);
    if (buf) {
      s_free_buffers.push(buf);
    }
  }

  s_buffer_mutex = xSemaphoreCreateMutex();
}

void USBWebCam::loop() {
  static uint32_t hb = 0;
  if (++hb % 100 == 0) {
    ESP_LOGD(TAG, "HEARTBEAT: init_done=%d ready=%d hdl=%p attempts=%d start_ret=%d s_frame_cb_count=%d (%s)",
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
  // Deferred stream start — runs once after init, so errors are visible over WiFi
  if (this->stream_opened_ && !this->start_attempted_) {
    this->start_attempted_ = true;
    ESP_LOGI(TAG, "Attempting uvc_host_stream_start now...");
    esp_err_t ret = uvc_host_stream_start(stream_hdl);

    this->last_start_ret_ = ret;
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "stream_start FAILED: %d (%s)", ret, esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "Stream started — waiting for frames from callback");
      xTaskCreate([](void *arg) {
        static_cast<USBWebCam *>(arg)->update_camera_parameters();
        vTaskDelete(NULL);
      }, "cam_ctrl", 4096, this, 3, NULL);
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
  ESP_LOGCONFIG(TAG, "  Current Mode: %dx%d", this->current_width_, this->current_height_);
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
    ESP_LOGD(TAG, "start_stream! %d", this->stream_requesters_);
  }

  this->stream_requesters_ |= val;
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

  camera_fb_t *fb = nullptr;

  xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
  if (!s_ready_buffers.empty()) {
    fb = s_ready_buffers.front();
    s_ready_buffers.pop();
  }
  xSemaphoreGive(s_buffer_mutex);

  if (fb == nullptr) {
    ESP_LOGV(TAG, "No frame ready yet");
    return;
  }

  std::shared_ptr<USBWebCamImage> image = std::make_shared<USBWebCamImage>(fb, this->single_requesters_);

  for (auto *listener : this->listeners_) {
    listener->on_camera_image(image);
  }

  this->current_image_ = image;
  this->single_requesters_ = 0;
  this->last_update_ = now;
}

void USBWebCam::change_video_mode(uint16_t width, uint16_t height, uint16_t fps) {
    struct Args { USBWebCam *self; uint16_t width, height, fps; };
    auto *args = new Args{this, width, height, fps};
    xTaskCreate([](void *arg) {
        auto *a = static_cast<Args *>(arg);
        ESP_LOGI(TAG, "Changing mode to %dx%d @ %dfps", a->width, a->height, a->fps);

        if (stream_hdl) {
            uvc_host_stream_stop(stream_hdl);
            uvc_host_stream_close(stream_hdl);
            stream_hdl = NULL;
        }

        esp_err_t ret = open_stream(a->width, a->height, a->fps, a->self->frame_buffer_size_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reopen stream: %s", esp_err_to_name(ret));
        } else {
            a->self->current_width_  = a->width;
            a->self->current_height_ = a->height;
            a->self->max_update_interval_ = 1000 / a->fps;

            VideoMode vm{a->width, a->height, a->fps};
            a->self->mode_pref_.save(&vm);

            ret = uvc_host_stream_start(stream_hdl);
            if (ret == ESP_OK) {
                a->self->update_camera_parameters();
            } else {
                ESP_LOGE(TAG, "stream_start failed after mode change: %s", esp_err_to_name(ret));
            }
        }
        delete a;
        vTaskDelete(NULL);
    }, "cam_mode", 4096, args, 3, NULL);
}

// UVC 1.1 Table A-14: selector → bmControls bit position
static int pu_selector_to_bit(uint8_t selector) {
    static const int8_t map[] = {
        -1,  // 0x00
         8,  // 0x01 Backlight Compensation
         0,  // 0x02 Brightness
         1,  // 0x03 Contrast
         9,  // 0x04 Gain
        10,  // 0x05 Power Line Frequency
         2,  // 0x06 Hue
         3,  // 0x07 Saturation
         4,  // 0x08 Sharpness
         5,  // 0x09 Gamma
         6,  // 0x0A White Balance Temperature
         7,  // 0x0B White Balance Component
    };
    if (selector >= sizeof(map)) return -1;
    return map[selector];
}

static bool pu_supported(uint32_t bitmap, uint8_t selector) {
    int bit = pu_selector_to_bit(selector);
    return (bit >= 0) && ((bitmap >> bit) & 1);
}

// Walk the USB config descriptor to find the Processing Unit with the given bUnitID
// and return its bmControls packed into a uint32_t.
static uint32_t fetch_pu_bitmap(uint8_t unit_id) {
    usb_host_client_handle_t temp_client;
    const usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 1,
        .async = {
            .client_event_callback = [](const usb_host_client_event_msg_t *, void *) {},
            .callback_arg = nullptr,
        },
    };
    if (usb_host_client_register(&cfg, &temp_client) != ESP_OK) {
        ESP_LOGW(TAG, "Could not register temp USB client for descriptor parse");
        return 0xFFFFFFFF;
    }

    uint8_t addr_list[8];
    int num_dev = 0;
    uint32_t bitmap = 0xFFFFFFFF;

    usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_dev);
    for (int i = 0; i < num_dev && bitmap == 0xFFFFFFFF; i++) {
        usb_device_handle_t dev;
        if (usb_host_device_open(temp_client, addr_list[i], &dev) != ESP_OK) continue;

        const usb_config_desc_t *cfg_desc;
        if (usb_host_get_active_config_descriptor(dev, &cfg_desc) == ESP_OK) {
            const uint8_t *p   = (const uint8_t *)cfg_desc;
            const uint8_t *end = p + cfg_desc->wTotalLength;
            while (p + 2 <= end) {
                uint8_t len  = p[0];
                uint8_t type = p[1];
                if (len < 2 || p + len > end) break;
                // CS_INTERFACE = 0x24, PU subtype = 0x05
                if (type == 0x24 && len >= 9 && p[2] == 0x05 && p[3] == unit_id) {
                    uint8_t ctrl_size = p[7];
                    uint32_t bm = 0;
                    for (uint8_t b = 0; b < ctrl_size && b < 4; b++)
                        bm |= (uint32_t)p[8 + b] << (b * 8);
                    bitmap = bm;
                    ESP_LOGI(TAG, "PU unit=%d bmControls=0x%08" PRIx32, unit_id, bm);
                    break;
                }
                p += len;
            }
        }
        usb_host_device_close(temp_client, dev);
    }

    usb_host_client_deregister(temp_client);
    return bitmap;
}

// Walk the USB config descriptor collecting all VS_FRAME_MJPEG entries (subtype 0x07).
// Returns a list of {width, height, fps} for every advertised MJPEG mode.
static std::vector<VideoMode> parse_uvc_formats() {
    std::vector<VideoMode> modes;

    usb_host_client_handle_t temp_client;
    const usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 1,
        .async = {
            .client_event_callback = [](const usb_host_client_event_msg_t *, void *) {},
            .callback_arg = nullptr,
        },
    };
    if (usb_host_client_register(&cfg, &temp_client) != ESP_OK) {
        ESP_LOGW(TAG, "Could not register temp USB client for format parse");
        return modes;
    }

    uint8_t addr_list[8];
    int num_dev = 0;
    usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_dev);

    ESP_LOGI(TAG, "parse_uvc_formats: scanning %d device(s)", num_dev);
    for (int i = 0; i < num_dev; i++) {
        usb_device_handle_t dev;
        if (usb_host_device_open(temp_client, addr_list[i], &dev) != ESP_OK) {
            ESP_LOGW(TAG, "parse_uvc_formats: could not open device %d", i);
            continue;
        }

        const usb_config_desc_t *cfg_desc;
        if (usb_host_get_active_config_descriptor(dev, &cfg_desc) != ESP_OK) {
            ESP_LOGW(TAG, "parse_uvc_formats: could not get config descriptor for device %d", i);
            usb_host_device_close(temp_client, dev);
            continue;
        }

        ESP_LOGI(TAG, "parse_uvc_formats: descriptor total length=%d", cfg_desc->wTotalLength);
        const uint8_t *p   = (const uint8_t *)cfg_desc;
        const uint8_t *end = p + cfg_desc->wTotalLength;
        int cs_iface_count = 0;

        while (p + 2 <= end) {
            uint8_t len  = p[0];
            uint8_t type = p[1];
            if (len < 2 || p + len > end) break;

            // CS_INTERFACE = 0x24, VS_FRAME_MJPEG subtype = 0x07
            if (type == 0x24) {
                cs_iface_count++;
                uint8_t subtype = p[2];
                if (subtype == 0x07) {
                    // VS_FRAME_MJPEG — fixed header is 26 bytes minimum
                    if (len < 27) {
                        ESP_LOGW(TAG, "VS_FRAME_MJPEG descriptor too short: len=%d", len);
                        p += len;
                        continue;
                    }
                    uint16_t w = (uint16_t)(p[5] | (p[6] << 8));
                    uint16_t h = (uint16_t)(p[7] | (p[8] << 8));
                    uint8_t  interval_type = p[25];
                    ESP_LOGI(TAG, "VS_FRAME_MJPEG: %dx%d interval_type=%d len=%d", w, h, interval_type, len);

                    if (interval_type == 0) {
                        // Continuous: [26..29]=min, [30..33]=max (100ns units)
                        if (len >= 38) {
                            uint32_t t_min = p[26] | ((uint32_t)p[27] << 8) | ((uint32_t)p[28] << 16) | ((uint32_t)p[29] << 24);
                            uint32_t t_max = p[30] | ((uint32_t)p[31] << 8) | ((uint32_t)p[32] << 16) | ((uint32_t)p[33] << 24);
                            ESP_LOGI(TAG, "  continuous: t_min=%" PRIu32 " t_max=%" PRIu32, t_min, t_max);
                            static const uint16_t common_fps[] = {5, 10, 15, 20, 25, 30, 60};
                            for (uint16_t fps : common_fps) {
                                uint32_t t = 10000000u / fps;
                                if (t >= t_min && t <= t_max) {
                                    modes.push_back({w, h, fps});
                                    ESP_LOGI(TAG, "  -> %dx%d @ %dfps", w, h, fps);
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "  continuous descriptor too short for interval fields: len=%d", len);
                        }
                    } else {
                        // Discrete: interval_type intervals of 4 bytes each starting at p[26]
                        for (uint8_t n = 0; n < interval_type; n++) {
                            uint8_t off = 26 + n * 4;
                            if (off + 4 > len) break;
                            uint32_t t = p[off] | ((uint32_t)p[off+1] << 8) | ((uint32_t)p[off+2] << 16) | ((uint32_t)p[off+3] << 24);
                            if (t == 0) continue;
                            uint16_t fps = (uint16_t)((10000000u + t / 2) / t);
                            if (fps > 0) {
                                modes.push_back({w, h, fps});
                                ESP_LOGI(TAG, "  -> %dx%d @ %dfps (interval=%" PRIu32 ")", w, h, fps, t);
                            }
                        }
                    }
                }
            }
            p += len;
        }
        ESP_LOGI(TAG, "parse_uvc_formats: saw %d CS_INTERFACE descriptors", cs_iface_count);
        usb_host_device_close(temp_client, dev);
    }

    usb_host_client_deregister(temp_client);
    ESP_LOGI(TAG, "parse_uvc_formats: found %d MJPEG modes total", (int)modes.size());
    return modes;
}

static esp_err_t pu_req(uvc_host_stream_hdl_t hdl, uint8_t unit_id, uint8_t selector, uint8_t bRequest, int16_t *out) {
    uint8_t data[2] = {0};
    uint8_t bmRequestType = (bRequest == 0x01) ? 0x21 : 0xA1;
    esp_err_t err = uvc_host_usb_ctrl(hdl, bmRequestType, bRequest, (uint16_t)(selector << 8), (uint16_t)(unit_id << 8), 2, data);
    if (err == ESP_OK && out) *out = (int16_t)(data[0] | (data[1] << 8));
    return err;
}

static esp_err_t pu_set(uvc_host_stream_hdl_t hdl, uint8_t unit_id, uint8_t selector, int16_t value) {
    uint8_t data[2] = {(uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    return uvc_host_usb_ctrl(hdl, 0x21, 0x01, (uint16_t)(selector << 8), (uint16_t)(unit_id << 8), 2, data);
}

void USBWebCam::update_camera_parameters() {
    if (stream_hdl == NULL) return;

    struct { uint8_t selector; const char *name; USBWebCamNumber *number; } controls[] = {
        {0x02, "brightness", brightness_number_},
        {0x03, "contrast",   contrast_number_},
        {0x07, "saturation", saturation_number_},
        {0x06, "hue",        hue_number_},
        {0x08, "sharpness",  sharpness_number_},
    };

    global_usb_webcam->controls_probed_ = false;
    for (auto &ctrl : controls) {
        if (!ctrl.number) continue;
        if (!pu_supported(pu_controls_bitmap_, ctrl.selector)) {
            ESP_LOGD(TAG, "Control '%s' not in PU bmControls, skipping", ctrl.name);
            ctrl.number->publish_state(NAN);
            continue;
        }
        int16_t current, min_val, max_val;
        if (pu_req(stream_hdl, processing_unit_id_, ctrl.selector, 0x81, &current) != ESP_OK) {
            ESP_LOGW(TAG, "Control '%s' GET_CUR failed unexpectedly", ctrl.name);
            ctrl.number->publish_state(NAN);
            continue;
        }
        bool has_range = (pu_req(stream_hdl, processing_unit_id_, ctrl.selector, 0x82, &min_val) == ESP_OK &&
                          pu_req(stream_hdl, processing_unit_id_, ctrl.selector, 0x83, &max_val) == ESP_OK);
        if (has_range) {
            ESP_LOGI(TAG, "Control '%s': current=%d, min=%d, max=%d", ctrl.name, current, min_val, max_val);
            ctrl.number->traits.set_min_value(min_val);
            ctrl.number->traits.set_max_value(max_val);
        } else {
            ESP_LOGI(TAG, "Control '%s': current=%d (range unavailable)", ctrl.name, current);
        }
        float restored;
        if (ctrl.number->pref_.load(&restored) && restored >= min_val && restored <= max_val) {
            ESP_LOGI(TAG, "Restoring '%s' = %.0f", ctrl.name, restored);
            pu_set(stream_hdl, processing_unit_id_, ctrl.selector, (int16_t)restored);
            ctrl.number->publish_state(restored);
        } else {
            ctrl.number->publish_state(current);
        }
    }
    global_usb_webcam->controls_probed_ = true;
}

void USBWebCamNumber::control(float value) {
    if (stream_hdl == NULL || global_usb_webcam == nullptr) return;
    if (!global_usb_webcam->is_controls_probed()) {
        ESP_LOGW(TAG, "Controls not yet probed, ignoring");
        return;
    }
    int16_t min_v = (int16_t)traits.get_min_value();
    int16_t max_v = (int16_t)traits.get_max_value();
    if (value < min_v || value > max_v) {
        ESP_LOGE(TAG, "Value %.0f out of range [%d, %d]", value, min_v, max_v);
        return;
    }
    esp_err_t err = pu_set(stream_hdl, global_usb_webcam->get_processing_unit_id(), selector_, (int16_t)value);
    if (err == ESP_OK) {
        publish_state(value);
        pref_.save(&value);
        ESP_LOGI(TAG, "Control 0x%02x = %d", selector_, (int)value);
    } else {
        ESP_LOGW(TAG, "Failed to set control 0x%02x: %s", selector_, esp_err_to_name(err));
    }
}

void USBWebCamSelect::control(const std::string &value) {
    for (auto &m : modes) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%dx%d @ %dfps", m.width, m.height, m.fps);
        if (value == buf) {
            publish_state(value);
            if (parent_) parent_->change_video_mode(m.width, m.height, m.fps);
            return;
        }
    }
    ESP_LOGW(TAG, "Unknown video mode selected: %s", value.c_str());
}

void USBWebCamButton::press_action() {
    if (stream_hdl == NULL) return;
    xTaskCreate([](void *) {
        if (global_usb_webcam->is_streaming()) {
            ESP_LOGI(TAG, "Button: stopping stream");
            uvc_host_stream_stop(stream_hdl);
        } else {
            ESP_LOGI(TAG, "Button: starting stream");
            uvc_host_stream_start(stream_hdl);
        }
        vTaskDelete(NULL);
    }, "cam_btn", 4096, nullptr, 3, NULL);
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

void USBWebCam::set_drop_size(uint32_t drop_size) { s_drop_frame_size = drop_size; }
void USBWebCam::set_frame_buffer_size(uint32_t frame_buffer_size) { this->frame_buffer_size_ = frame_buffer_size; }
void USBWebCam::set_idle_update_interval(uint32_t idle_update_interval) {
  this->idle_update_interval_ = idle_update_interval;
}

USBWebCam::USBWebCam() {}


// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
USBWebCam *global_usb_webcam{nullptr};

/* ---------------- CameraImage class implementations ---------------- */
USBWebCamImage::USBWebCamImage(camera_fb_t *buffer, uint8_t requester) : camera::CameraImage(), buffer_(buffer), requesters_(requester) {}
USBWebCamImage::~USBWebCamImage() {
    if (buffer_) {
        esp_camera_fb_return(buffer_);
        buffer_ = nullptr;
    }
}
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
    if (!this->image_) return;
    this->image_.reset();
}

}  // namespace esphome::usb_webcam

#endif
