#pragma once
// Minimal host substitute for the pinned ESP-IDF APIs used by SeaTalkBus.
// These tests exercise driver decisions, not electrical timing or IDF internals.
#include <cstddef>
#include <cstdint>
#define IRAM_ATTR
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_FAIL=-1;
using gpio_num_t=int;
enum gpio_mode_t { GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2,
                   GPIO_MODE_INPUT_OUTPUT=3, GPIO_MODE_INPUT_OUTPUT_OD=7 };
struct gpio_config_t { uint64_t pin_bit_mask=0; gpio_mode_t mode=GPIO_MODE_INPUT; };
constexpr int INPUT=1, CHANGE=2;
constexpr int SIG_GPIO_OUT_IDX=256, RMT_SIG_OUT0_IDX=87;
int gpio_get_level(gpio_num_t);
esp_err_t gpio_config(const gpio_config_t *);
esp_err_t gpio_set_level(gpio_num_t,int);
esp_err_t gpio_set_direction(gpio_num_t,gpio_mode_t);
void esp_rom_gpio_connect_out_signal(uint32_t,uint32_t,bool,bool);
void pinMode(int,int);
void attachInterruptArg(int,void (*)(void *),void *,int);
uint32_t micros();
uint32_t esp_random();
int64_t esp_timer_get_time();
using esp_timer_handle_t=void *;
struct esp_timer_create_args_t { void (*callback)(void *)=nullptr; void *arg=nullptr; const char *name=nullptr; };
esp_err_t esp_timer_create(const esp_timer_create_args_t *,esp_timer_handle_t *);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t,uint64_t);
esp_err_t esp_timer_stop(esp_timer_handle_t);
using RingbufHandle_t=void *;
void *xRingbufferReceive(RingbufHandle_t,size_t *,uint32_t);
void vRingbufferReturnItem(RingbufHandle_t,void *);
enum rmt_channel_t { RMT_CHANNEL_0=0, RMT_CHANNEL_4=4 };
enum rmt_mode_t { RMT_MODE_RX, RMT_MODE_TX };
enum rmt_idle_level_t { RMT_IDLE_LEVEL_LOW, RMT_IDLE_LEVEL_HIGH };
struct rmt_item32_t { uint32_t duration0:15; uint32_t level0:1; uint32_t duration1:15; uint32_t level1:1; };
struct rmt_config_t {
  gpio_num_t gpio_num; rmt_channel_t channel; rmt_mode_t rmt_mode;
  int clk_div=0, mem_block_num=0;
  struct { bool filter_en=false; int filter_ticks_thresh=0, idle_threshold=0; } rx_config;
  struct { bool idle_output_en=false; rmt_idle_level_t idle_level=RMT_IDLE_LEVEL_LOW; } tx_config;
};
#define RMT_DEFAULT_CONFIG_RX(pin,channel) (rmt_config_t{pin,channel,RMT_MODE_RX,0,0,{}, {}})
#define RMT_DEFAULT_CONFIG_TX(pin,channel) (rmt_config_t{pin,channel,RMT_MODE_TX,0,0,{}, {}})
esp_err_t rmt_config(const rmt_config_t *);
esp_err_t rmt_driver_install(rmt_channel_t,size_t,int);
esp_err_t rmt_get_ringbuf_handle(rmt_channel_t,RingbufHandle_t *);
esp_err_t rmt_rx_start(rmt_channel_t,bool);
esp_err_t rmt_tx_stop(rmt_channel_t);
esp_err_t rmt_wait_tx_done(rmt_channel_t,uint32_t);
esp_err_t rmt_write_items(rmt_channel_t,const rmt_item32_t *,int,bool);
