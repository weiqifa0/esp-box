/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bsp/esp-bsp.h"
#include "display.h"
#include "es7210.h"
//#include "es8311.h"
#include "fft_convert.h"
#include "usb_headset.h"
#include "page_fft.h"

#define ES7210_ADDR       0x40
#define ES7210_ADC_VOLUME 0

i2s_chan_handle_t i2s_tx_chan;
i2s_chan_handle_t i2s_rx_chan;
//static es8311_handle_t es8311_dev = NULL;
static es7210_dev_handle_t es7210_handle = NULL;

//static void _es8311_init()
//{
//    /* Create and configure ES8311 I2C driver */
//    es8311_dev = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
//    const es8311_clock_config_t clk_cfg = BSP_ES8311_SCLK_CONFIG(SAMPLE_RATE);
//    es8311_init(es8311_dev, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
//    es8311_voice_volume_set(es8311_dev, DEFAULT_VOLUME, NULL);
//}

static void _es7210_init()
{
    const es7210_i2c_config_t es7210_cfg = {
        .i2c_addr = ES7210_ADDR,
        .i2c_port = BSP_I2C_NUM,
    };

    es7210_new_codec(&es7210_cfg, &es7210_handle);
    es7210_codec_config_t codec_conf = {
        .i2s_format = ES7210_I2S_FMT_I2S,
        .mclk_ratio = 256,
        .sample_rate_hz = SAMPLE_RATE,
        .bit_width = WIDTH,
        .mic_bias = ES7210_MIC_BIAS_2V87,
        .mic_gain = ES7210_MIC_GAIN_30DB,
        .flags.tdm_enable = false,
    };

    es7210_config_codec(es7210_handle, &codec_conf);
    es7210_config_volume(es7210_handle, ES7210_ADC_VOLUME);
}

#define TAG "ESP32S3"
/*用定时器给LVGL提供时钟*/
static void lv_tick_task(void *arg)
{
	(void)arg;
	lv_tick_inc(10);
}

SemaphoreHandle_t xGuiSemaphore;

static void gui_task(void *arg)
{
	xGuiSemaphore = xSemaphoreCreateMutex();
	lv_init(); // lvgl内核初始化

	lvgl_driver_init(); // lvgl显示接口初始化
	/*外部PSRAM方式*/
	// lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	// lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

	/*内部DMA方式*/
	lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
	lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

	// static lv_color_t buf1[DISP_BUF_SIZE];
	// static lv_color_t buf2[DISP_BUF_SIZE];
	static lv_disp_buf_t disp_buf;
	uint32_t size_in_px = DISP_BUF_SIZE;
	lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = disp_driver_flush;
	disp_drv.buffer = &disp_buf;
	lv_disp_drv_register(&disp_drv);

	// lv_indev_drv_t indev_drv;
	// lv_indev_drv_init(&indev_drv);
	// indev_drv.read_cb = touch_driver_read;
	// indev_drv.type = LV_INDEV_TYPE_POINTER;
	// lv_indev_drv_register(&indev_drv);

	// esp_register_freertos_tick_hook(lv_tick_task);
	/* Create and start a periodic timer interrupt to call lv_tick_inc */
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &lv_tick_task,
		.name = "periodic_gui"};
	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));

	FFT_Setup();
	while (1)
	{
		/* Delay 1 tick (assumes FreeRTOS tick is 10ms */
		vTaskDelay(pdMS_TO_TICKS(10));
		/* Try to take the semaphore, call lvgl related function on success */
		if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
		{
		 	//  page.Running();
			lv_task_handler();
			xSemaphoreGive(xGuiSemaphore);
		}
	}
}

void app_main(void)
{
    /* Initialize I2C (for touch and audio) */
    bsp_i2c_init();

    /* Initialize display */
    display_lcd_init();

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    /* Init fft */
    fft_convert_init();

    /* Init es8311 */
    //_es8311_init();

    /* Init es7210 */
    //_es7210_init();

    /* Configure I2S peripheral and Power Amplifier */
    // i2s_std_config_t i2s_config = {
    //     .clk_cfg = {
    //         .sample_rate_hz = SAMPLE_RATE,
    //         .clk_src = I2S_CLK_SRC_DEFAULT,
    //         .mclk_multiple = I2S_MCLK_MULTIPLE_384,
    //     },
    //     .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)WIDTH, (i2s_slot_mode_t)CHANNEL),
    //     .gpio_cfg = BSP_I2S_GPIO_CFG,
    // };
		xTaskCreatePinnedToCore(&gui_task, "gui_task", 1024 * 8, NULL, 6, NULL, 0);
    bsp_audio_init(&i2s_config, &i2s_tx_chan, &i2s_rx_chan);
    bsp_audio_poweramp_enable(true);
    usb_headset_init();
}
