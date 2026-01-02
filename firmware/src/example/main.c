/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "touch_CTP/touch_CTP.h"
#include "LCD_TK0128F25K/LCD_TK0128F25K.h"
#include "icon.h"

/* LCD size */
#define EXAMPLE_LCD_H_RES   (240)
#define EXAMPLE_LCD_V_RES   (240)

#define EXAMPLE_I2C_NUM                 1          // I2C number
#define EXAMPLE_TOUCH_I2C_CLK_HZ        (400000)
#define EXAMPLE_I2C_SCL                 (GPIO_NUM_40)
#define EXAMPLE_I2C_SDA                 (GPIO_NUM_39)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM          (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ     (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS         (8)
#define EXAMPLE_LCD_PARAM_BITS       (8)
#define EXAMPLE_LCD_COLOR_SPACE      (LCD_RGB_ELEMENT_ORDER_BGR)
#define EXAMPLE_LCD_BITS_PER_PIXEL   (16)
#define EXAMPLE_LCD_BL_ON_LEVEL      (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK       (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_MOSI       (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_RST        (GPIO_NUM_NC)
#define EXAMPLE_LCD_GPIO_DC         (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_CS         (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_BL         (GPIO_NUM_NC)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static char *TAG = "example";

static esp_lcd_panel_io_handle_t tp_io_handle = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

static void lcd_touc_init(void) 
{
    i2c_master_bus_handle_t codec_i2c_bus;

    // Initialize I2C peripheral
    i2c_master_bus_config_t i2c_bus_cfg1 = {
        .i2c_port = EXAMPLE_I2C_NUM,
        .sda_io_num = EXAMPLE_I2C_SDA,
        .scl_io_num = EXAMPLE_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg1, &codec_i2c_bus));

    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_TOUCH_ADDRESS, 
        .control_phase_bytes = 1,           
        .dc_bit_offset = 0,                 
        .lcd_cmd_bits = 8,                  
        .flags =                            
        {                                   
            .disable_control_phase = 1,     
        },
        .scl_speed_hz = 400000,                              
    };

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");

    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(codec_i2c_bus, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_touch(tp_io_handle, &tp_cfg, &touch_handle));
}

/* 初始化SPI */
static esp_err_t spi_init(void) 
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = EXAMPLE_LCD_GPIO_MOSI;
    buscfg.miso_io_num = GPIO_NUM_NC;
    buscfg.sclk_io_num = EXAMPLE_LCD_GPIO_SCLK;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    return ESP_OK;
}

/**
 * @brief 在指定矩形区域填充颜色
 * @param panel LCD面板句柄
 * @param x_start 区域起始X坐标
 * @param y_start 区域起始Y坐标
 * @param x_end 区域结束X坐标（不包含）
 * @param y_end 区域结束Y坐标（不包含）
 * @param color RGB565颜色值
 * @return esp_err_t 执行结果
 */
esp_err_t fill_rect(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, uint16_t color)
{
    // 参数检查
    if (x_start < 0 || y_start < 0 || x_end > 240 || y_end > 240 || 
        x_start >= x_end || y_start >= y_end) {
        ESP_LOGE(TAG, "Invalid rectangle parameters: (%d,%d)-(%d,%d)", 
                 x_start, y_start, x_end, y_end);
        return ESP_ERR_INVALID_ARG;
    }

    int width = x_end - x_start;
    int height = y_end - y_start;

    // 创建行缓冲区
    uint16_t *line_buffer = malloc(width * sizeof(uint16_t));
    if (!line_buffer) {
        ESP_LOGE(TAG, "Failed to allocate line buffer");
        return ESP_ERR_NO_MEM;
    }

    // 填充行缓冲区
    for (int i = 0; i < width; i++) {
        line_buffer[i] = color;
    }

    // 逐行绘制
    for (int y = y_start; y < y_end; y++) {
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel, x_start, y, x_end, y + 1, line_buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw line %d: %s", y, esp_err_to_name(ret));
            free(line_buffer);
            return ret;
        }
    }

    free(line_buffer);
    ESP_LOGD(TAG, "Filled rect (%d,%d)-(%d,%d) with color 0x%04X", 
             x_start, y_start, x_end, y_end, color);
    return ESP_OK;
}

/**
 * @brief 填充整个屏幕
 */
esp_err_t fill_screen(esp_lcd_panel_handle_t panel, uint16_t color)
{
    return fill_rect(panel, 0, 0, 240, 240, color);
}

/**
 * @brief 最近邻插值放大
 * @param src_width 原图宽度
 * @param src_height 原图高度
 * @param src_data 原图数据(RGB565)
 * @param dest_width 目标宽度
 * @param dest_height 目标高度
 * @return 放大后的图像数据
 */
uint16_t *image_scale_nearest(uint16_t src_width, uint16_t src_height, const uint16_t *src_data,
                             uint16_t dest_width, uint16_t dest_height)
{
    uint16_t *dest_data = heap_caps_malloc(dest_width * dest_height * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!dest_data) return NULL;
    
    for (uint16_t y = 0; y < dest_height; y++) {
        for (uint16_t x = 0; x < dest_width; x++) {
            // 计算原图坐标
            uint16_t src_x = (x * src_width) / dest_width;
            uint16_t src_y = (y * src_height) / dest_height;
            
            // 获取最近像素
            uint32_t src_index = src_y * src_width + src_x;
            uint32_t dest_index = y * dest_width + x;
            dest_data[dest_index] = src_data[src_index];
        }
    }
    
    return dest_data;
}

/**
 * @brief 显示放大后的图片
 * @param scale_method 缩放方法：0-最近邻, 1-双线性
 */
esp_err_t lcd_draw_image_scaled(esp_lcd_panel_handle_t panel, uint16_t x, uint16_t y,
                               uint16_t src_width, uint16_t src_height, const uint16_t *src_data,
                               uint16_t dest_width, uint16_t dest_height)
{
    uint16_t *scaled_data = NULL;
    
    scaled_data = image_scale_nearest(src_width, src_height, src_data, dest_width, dest_height);

    
    if (!scaled_data) {
        ESP_LOGE(TAG, "Failed to scale image");
        return ESP_ERR_NO_MEM;
    }
    
    // 显示缩放后的图像
    esp_err_t ret = lcd_draw_image_rgb565(panel, x, y, dest_width, dest_height, scaled_data);
    
    // 释放内存
    heap_caps_free(scaled_data);
    
    return ret;
}


/**
 * @brief 绘制水平线
 */
esp_err_t draw_hline(esp_lcd_panel_handle_t panel, int x_start, int x_end, int y, uint16_t color)
{
    return fill_rect(panel, x_start, y, x_end, y + 1, color);
}

/**
 * @brief 绘制垂直线
 */
esp_err_t draw_vline(esp_lcd_panel_handle_t panel, int x, int y_start, int y_end, uint16_t color)
{
    return fill_rect(panel, x, y_start, x + 1, y_end, color);
}

/**
 * @brief 绘制矩形边框
 */
esp_err_t draw_rect(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, uint16_t color)
{
    esp_err_t ret = ESP_OK;
    
    // 上边
    ret = draw_hline(panel, x_start, x_end, y_start, color);
    if (ret != ESP_OK) return ret;
    
    // 下边
    ret = draw_hline(panel, x_start, x_end, y_end - 1, color);
    if (ret != ESP_OK) return ret;
    
    // 左边
    ret = draw_vline(panel, x_start, y_start, y_end, color);
    if (ret != ESP_OK) return ret;
    
    // 右边
    ret = draw_vline(panel, x_end - 1, y_start, y_end, color);
    
    return ret;
}

/**
 * @brief 绘制圆角矩形（填充）
 */
esp_err_t fill_round_rect(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, int radius, uint16_t color)
{
    // 先填充中间矩形区域
    ESP_ERROR_CHECK(fill_rect(panel, x_start + radius, y_start, x_end - radius, y_end, color));
    ESP_ERROR_CHECK(fill_rect(panel, x_start, y_start + radius, x_end, y_end - radius, color));
    
    // 填充四个角的矩形区域
    ESP_ERROR_CHECK(fill_rect(panel, x_start, y_start, x_start + radius, y_start + radius, color));
    ESP_ERROR_CHECK(fill_rect(panel, x_end - radius, y_start, x_end, y_start + radius, color));
    ESP_ERROR_CHECK(fill_rect(panel, x_start, y_end - radius, x_start + radius, y_end, color));
    ESP_ERROR_CHECK(fill_rect(panel, x_end - radius, y_end - radius, x_end, y_end, color));
    
    return ESP_OK;
}

static esp_err_t lcd_init(void)
{
    /* LCD backlight */
    if (EXAMPLE_LCD_GPIO_BL >= 0) {
        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    }

    // 液晶屏控制IO初始化
    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io));

    // 初始化液晶屏驱动芯片
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .rgb_ele_order = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };

    ESP_LOGI(TAG, "Install TK0134 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_tk0128f25k(lcd_io, &panel_config, &lcd_panel));
    
    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_disp_on_off(lcd_panel, true);
    esp_lcd_panel_invert_color(lcd_panel, true);

    return ESP_OK;
}

void touch_task(void *pvParameters)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)pvParameters;
    
    init_touch_ui_state(96, 96);
    
    while (1) {
        if (process_touch_and_gestures(tp)) {
            ESP_LOGD(TAG, "Touch/gesture processed");
        }
        
        vTaskDelay(pdMS_TO_TICKS(30)); 
    }
    
    vTaskDelete(NULL);
}

// 在显示任务中
void display_task(void *pvParameters)
{
    while (1) {
        touch_ui_state_t *state = get_touch_ui_state();
        
        // if (state->needs_redraw) {
            if (state->double_state && !state->slide_state) {
                for (int i = 0; i <= 4; i++) {
                    // 计算当前尺寸
                    uint16_t current_size = (12 * i) + 48;
                    
                    // 计算居中偏移量
                    uint16_t x_offset = state->x_offset - ((current_size - 48) / 2);
                    uint16_t y_offset = state->y_offset - ((current_size - 48) / 2);
                    
                    // 边界检查
                    x_offset = MIN(MAX(x_offset, 0), 240 - current_size);
                    y_offset = MIN(MAX(y_offset, 0), 240 - current_size);
                    
                    lcd_draw_image_scaled(lcd_panel, x_offset, y_offset, 48, 48, (const uint16_t *)gImage_shiba_inu_icon, current_size, current_size);
                    vTaskDelay(pdMS_TO_TICKS(60));
                }

                for (int i = 3; i >= 0; i--) {
                    // 计算当前尺寸
                    uint16_t current_size = (12 * i) + 48;
                    
                    // 计算偏移量
                    uint16_t x_offset = state->x_offset - ((current_size - 48) / 2);
                    uint16_t y_offset = state->y_offset - ((current_size - 48) / 2);
                    
                    // 边界检查
                    x_offset = MIN(MAX(x_offset, 0), 240 - current_size);
                    y_offset = MIN(MAX(y_offset, 0), 240 - current_size);
                    
                    // 清除
                    fill_rect(lcd_panel, x_offset - 5, y_offset - 5, current_size + x_offset + 10, current_size + y_offset + 10, COLOR_WHITE);
                    vTaskDelay(pdMS_TO_TICKS(5)); // 短暂延迟确保清屏完成
                    // 绘制当前帧
                    lcd_draw_image_scaled(lcd_panel, x_offset, y_offset, 48, 48, (const uint16_t *)gImage_shiba_inu_icon, current_size, current_size);
                    vTaskDelay(pdMS_TO_TICKS(55));
                }

                state->double_state = false;
            }
            
            if (state->slide_state && !state->double_state) {
                lcd_draw_image_rgb565(lcd_panel, state->x_offset, state->y_offset, 48, 48, (const uint16_t *)gImage_shiba_inu_icon);
                fill_rect(lcd_panel, state->c_x_offset, state->c_y_offset, state->c_x_offset + state->width, state->c_y_offset + state->height, COLOR_WHITE);

                state->slide_state = false;
            }
            
        //     reset_touch_redraw_flag();
        // }
        
        vTaskDelay(pdMS_TO_TICKS(18)); 
    }
    
    vTaskDelete(NULL);
}

void app_main(void)
{
    lcd_touc_init();
    spi_init();
    lcd_init();
    
    // 清屏
    fill_screen(lcd_panel, COLOR_WHITE);

    lcd_draw_image_rgb565(lcd_panel, 96, 96, 48, 48, (const uint16_t *)gImage_shiba_inu_icon);

    xTaskCreate(touch_task, "touch_task", 4096, (void *)touch_handle, 5, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
}
