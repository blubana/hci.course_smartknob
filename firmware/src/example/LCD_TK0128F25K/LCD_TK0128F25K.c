/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "sdkconfig.h"

#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_compiler.h"

#define TK0128F25K_CMD_RAMCTRL               0xb0
#define TK0128F25K_DATA_LITTLE_ENDIAN_BIT   (1 << 3)

static const char *TAG = "lcd_panel.tk0128f25k";

static esp_err_t panel_tk0128f25k_del(esp_lcd_panel_t *panel);
static esp_err_t panel_tk0128f25k_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_tk0128f25k_init(esp_lcd_panel_t *panel);
static esp_err_t panel_tk0128f25k_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                          const void *color_data);
static esp_err_t panel_tk0128f25k_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_tk0128f25k_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_tk0128f25k_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_tk0128f25k_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_tk0128f25k_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_tk0128f25k_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;    // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val;    // save current value of LCD_CMD_COLMOD register
    uint8_t ramctl_val_1;
    uint8_t ramctl_val_2;
} tk0128f25k_panel_t;

esp_err_t
esp_lcd_new_panel_tk0128f25k(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                         esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    tk0128f25k_panel_t *tk0128f25k = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    // leak detection of tk0128f25k because saving tk0128f25k->base address
    ESP_COMPILER_DIAGNOSTIC_PUSH_IGNORE("-Wanalyzer-malloc-leak")
    tk0128f25k = calloc(1, sizeof(tk0128f25k_panel_t));
    ESP_GOTO_ON_FALSE(tk0128f25k, ESP_ERR_NO_MEM, err, TAG, "no mem for tk0128f25k panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        tk0128f25k->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        tk0128f25k->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported RGB element order");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        tk0128f25k->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        tk0128f25k->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    tk0128f25k->ramctl_val_1 = 0x00;
    tk0128f25k->ramctl_val_2 = 0xf0;    // Use big endian by default
    if ((panel_dev_config->data_endian) == LCD_RGB_DATA_ENDIAN_LITTLE) {
        // Use little endian
        tk0128f25k->ramctl_val_2 |= TK0128F25K_DATA_LITTLE_ENDIAN_BIT;
    }

    tk0128f25k->io = io;
    tk0128f25k->fb_bits_per_pixel = fb_bits_per_pixel;
    tk0128f25k->reset_gpio_num = panel_dev_config->reset_gpio_num;
    tk0128f25k->reset_level = panel_dev_config->flags.reset_active_high;
    tk0128f25k->base.del = panel_tk0128f25k_del;
    tk0128f25k->base.reset = panel_tk0128f25k_reset;
    tk0128f25k->base.init = panel_tk0128f25k_init;
    tk0128f25k->base.draw_bitmap = panel_tk0128f25k_draw_bitmap;
    tk0128f25k->base.invert_color = panel_tk0128f25k_invert_color;
    tk0128f25k->base.set_gap = panel_tk0128f25k_set_gap;
    tk0128f25k->base.mirror = panel_tk0128f25k_mirror;
    tk0128f25k->base.swap_xy = panel_tk0128f25k_swap_xy;
    tk0128f25k->base.disp_on_off = panel_tk0128f25k_disp_on_off;
    tk0128f25k->base.disp_sleep = panel_tk0128f25k_sleep;
    *ret_panel = &(tk0128f25k->base);
    ESP_LOGD(TAG, "new tk0128f25k panel @%p", tk0128f25k);

    return ESP_OK;

err:
    if (tk0128f25k) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(tk0128f25k);
    }
    return ret;
    ESP_COMPILER_DIAGNOSTIC_POP("-Wanalyzer-malloc-leak")
}

static esp_err_t panel_tk0128f25k_del(esp_lcd_panel_t *panel)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);

    if (tk0128f25k->reset_gpio_num >= 0) {
        gpio_reset_pin(tk0128f25k->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del tk0128f25k panel @%p", tk0128f25k);
    free(tk0128f25k);
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_reset(esp_lcd_panel_t *panel)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;

    // perform hardware reset
    if (tk0128f25k->reset_gpio_num >= 0) {
        gpio_set_level(tk0128f25k->reset_gpio_num, tk0128f25k->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(tk0128f25k->reset_gpio_num, !tk0128f25k->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "io tx param failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_init(esp_lcd_panel_t *panel)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first

    // 初始化命令序列
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xFE, NULL, 0), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xEF, NULL, 0), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xEB, (uint8_t[]){0x14}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x84, (uint8_t[]){0x60}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x85, (uint8_t[]){0xFF}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x86, (uint8_t[]){0xFF}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x87, (uint8_t[]){0xFF}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8E, (uint8_t[]){0xFF}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8F, (uint8_t[]){0xFF}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x88, (uint8_t[]){0x0A}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x89, (uint8_t[]){0x23}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8A, (uint8_t[]){0x00}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8B, (uint8_t[]){0x80}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8C, (uint8_t[]){0x01}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x8D, (uint8_t[]){0x03}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xB6, (uint8_t[]){0x00, 0x00}, 2), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x36, (uint8_t[]){0x40}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x3A, (uint8_t[]){0x05}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x90, (uint8_t[]){0x08, 0x08, 0x08, 0x08}, 4), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xBA, (uint8_t[]){0x0A}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xBD, (uint8_t[]){0x06}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xBC, (uint8_t[]){0x00}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xFF, (uint8_t[]){0x60, 0x01, 0x04}, 3), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xC3, (uint8_t[]){0x18}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xC4, (uint8_t[]){0x18}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xC9, (uint8_t[]){0x3F}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xBE, (uint8_t[]){0x11}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xE1, (uint8_t[]){0x10, 0x0E}, 2), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xDF, (uint8_t[]){0x21, 0x0C, 0x02}, 3), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xF0, (uint8_t[]){0x4C, 0x10, 0x09, 0x09, 0x86, 0x32}, 6), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xF1, (uint8_t[]){0x48, 0x75, 0x95, 0x2E, 0x34, 0x8F}, 6), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xF2, (uint8_t[]){0x4C, 0x10, 0x09, 0x09, 0x86, 0x32}, 6), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xF3, (uint8_t[]){0x48, 0x75, 0x95, 0x2E, 0x34, 0x8F}, 6), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xED, (uint8_t[]){0x1B, 0x0B}, 2), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xAE, (uint8_t[]){0x77}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xCD, (uint8_t[]){0x63}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x70, (uint8_t[]){0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03}, 9), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xE8, (uint8_t[]){0x34}, 1), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x62, (uint8_t[]){0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}, 12), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x63, (uint8_t[]){0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}, 12), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x64, (uint8_t[]){0x3B, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x0A}, 7), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x66, (uint8_t[]){0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x67, (uint8_t[]){0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x74, (uint8_t[]){0x10, 0x69, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x98, (uint8_t[]){0x3E, 0x07}, 2), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x99, (uint8_t[]){0x3E, 0x07}, 2), TAG, "io tx param failed");


    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x36, (uint8_t[]){0x48}, 1), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x35, NULL, 0), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x21, NULL, 0), TAG, "io tx param failed");

    // 延时
    vTaskDelay(pdMS_TO_TICKS(1));

    // Sleep Out命令
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0), TAG, "io tx param failed");

    // 延时
    vTaskDelay(pdMS_TO_TICKS(1));

    // Display On命令
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0), TAG, "io tx param failed");

    // 延时
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                          const void *color_data)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;

    x_start += tk0128f25k->x_gap;
    x_end += tk0128f25k->x_gap;
    y_start += tk0128f25k->y_gap;
    y_end += tk0128f25k->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * tk0128f25k->fb_bits_per_pixel / 8;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "io tx color failed");

    return ESP_OK;
}

esp_err_t lcd_draw_image_rgb565(esp_lcd_panel_handle_t panel, uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height, const uint16_t *image_data)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    
    // 计算结束坐标
    uint16_t x_end = x_start + width - 1;
    uint16_t y_end = y_start + height - 1;
    
    // 应用偏移量
    x_start += tk0128f25k->x_gap;
    x_end += tk0128f25k->x_gap;
    y_start += tk0128f25k->y_gap;
    y_end += tk0128f25k->y_gap;
    
    // 设置显示窗口
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){
        (x_start >> 8) & 0xFF, x_start & 0xFF,
        (x_end >> 8) & 0xFF, x_end & 0xFF
    }, 4), TAG, "Failed to set column address");
    
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){
        (y_start >> 8) & 0xFF, y_start & 0xFF,
        (y_end >> 8) & 0xFF, y_end & 0xFF
    }, 4), TAG, "Failed to set row address");
    
    // 传输图像数据
    size_t data_len = width * height * 2;
    return esp_lcd_panel_io_tx_color(io, 0x2C, image_data, data_len);
}

static esp_err_t panel_tk0128f25k_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    if (mirror_x) {
        tk0128f25k->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        tk0128f25k->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        tk0128f25k->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        tk0128f25k->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        tk0128f25k->madctl_val
    }, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    if (swap_axes) {
        tk0128f25k->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        tk0128f25k->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        tk0128f25k->madctl_val
    }, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    tk0128f25k->x_gap = x_gap;
    tk0128f25k->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    int command = 0;
    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_tk0128f25k_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    tk0128f25k_panel_t *tk0128f25k = __containerof(panel, tk0128f25k_panel_t, base);
    esp_lcd_panel_io_handle_t io = tk0128f25k->io;
    int command = 0;
    if (sleep) {
        command = LCD_CMD_SLPIN;
    } else {
        command = LCD_CMD_SLPOUT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}
