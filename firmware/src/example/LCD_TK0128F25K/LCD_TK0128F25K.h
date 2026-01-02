/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_panel_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define WHITE						0xFFFF
// #define BLACK						0x0000	  
// #define BLUE						0x001F  
// #define BRED						0XF81F
// #define GRED						0XFFE0
// #define GBLUE						0X07FF
// #define RED							0xF800
// #define MAGENTA					    0xF81F
// #define GREEN						0x07E0
// #define CYAN						0x7FFF
// #define YELLOW					    0xFFE0
// #define BROWN						0XBC40 //棕色
// #define BRRED						0XFC07 //棕红色
// #define GRAY						0X8430 //灰色

#define COLOR_RED       0x00F8    // 原来的0xF800交换字节
#define COLOR_GREEN     0xE007    // 原来的0x07E0交换字节  
#define COLOR_BLUE      0x1F00    // 原来的0x001F交换字节
#define COLOR_YELLOW    0xE0FF    // 原来的0xFFE0交换字节
#define COLOR_CYAN      0xFF07    // 原来的0x07FF交换字节
#define COLOR_MAGENTA   0x1FF8    // 原来的0xF81F交换字节
#define COLOR_WHITE     0xFFFF    // 白色不变
#define COLOR_BLACK     0x0000    // 黑色不变

/**
 * @brief Create LCD panel for model ST7789
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_tk0128f25k(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief 显示RGB565图像
 * @param panel LCD面板句柄
 * @param x_start 起始X坐标
 * @param y_start 起始Y坐标
 * @param width 图像宽度
 * @param height 图像高度
 * @param image_data RGB565图像数据
 */
esp_err_t lcd_draw_image_rgb565(esp_lcd_panel_handle_t panel, uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height, const uint16_t *image_data);


#ifdef __cplusplus
}
#endif
