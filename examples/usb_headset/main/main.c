/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bsp/esp-bsp.h"
#include "display.h"
#include "fft_convert.h"
#include "usb_headset.h"

i2s_chan_handle_t i2s_tx_chan;
i2s_chan_handle_t i2s_rx_chan;

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

    /* Configure I2S peripheral and Power Amplifier */
    i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)WIDTH, (i2s_slot_mode_t)CHANNEL),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    bsp_audio_init(&i2s_config, &i2s_tx_chan, &i2s_rx_chan);
    bsp_audio_poweramp_enable(true);
    usb_headset_init();
}