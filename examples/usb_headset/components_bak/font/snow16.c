#include "lvgl/lvgl.h"

/*******************************************************************************
 * Size: 16 px
 * Bpp: 4
 * Opts: 
 ******************************************************************************/

#ifndef SNOW16
#define SNOW16 1
#endif

#if SNOW16

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t gylph_bitmap[] = {
    /* U+E612 "" */
    0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x9, 0x0, 0x0, 0x0, 0x0, 0x0, 0xaa, 0x83,
    0x0, 0x0, 0x10, 0x70, 0x1b, 0x40, 0x34, 0x10,
    0x59, 0x70, 0x9, 0x0, 0x89, 0x80, 0x3a, 0xc8,
    0x9, 0x5, 0x96, 0x60, 0x10, 0x2, 0x9f, 0xa4,
    0x0, 0x10, 0x0, 0x1, 0x9f, 0xa3, 0x0, 0x0,
    0x39, 0x39, 0x19, 0x6, 0xab, 0x60, 0x4a, 0xc0,
    0x9, 0x0, 0x67, 0x70, 0x20, 0x80, 0x6, 0x30,
    0x34, 0x20, 0x0, 0x0, 0xa7, 0x92, 0x0, 0x0,
    0x0, 0x0, 0x9, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x1, 0x0, 0x0, 0x0,

    /* U+E69C "" */
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x55, 0x54, 0x40, 0x0, 0x0, 0x79, 0xd2, 0x58,
    0xb7, 0x0, 0x3, 0xaf, 0xe5, 0xa8, 0xfb, 0x40,
    0x2, 0x77, 0xcd, 0xee, 0x87, 0x30, 0x8, 0x1b,
    0xcf, 0xfe, 0xb5, 0x80, 0x2, 0x4, 0x9f, 0xfd,
    0x41, 0x20, 0x5, 0xee, 0xf9, 0xbc, 0xee, 0x70,
    0x0, 0x5f, 0xd5, 0xa8, 0xfa, 0x0, 0x0, 0x35,
    0xd0, 0x18, 0x86, 0x0, 0x0, 0x0, 0x56, 0x51,
    0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

    /* U+E69D "" */
    0x0, 0x0, 0x15, 0x0, 0x0, 0x0, 0x2, 0x6b,
    0x40, 0x0, 0x0, 0x74, 0xff, 0x85, 0x21, 0x5e,
    0xe4, 0x6c, 0xd, 0xcc, 0xc, 0xfc, 0x7b, 0x8f,
    0xf4, 0x5, 0x26, 0xff, 0xa2, 0x42, 0xc, 0x9c,
    0xee, 0xe9, 0xb5, 0x1c, 0xf8, 0x4b, 0x1f, 0xf5,
    0x47, 0xd2, 0x9e, 0x3b, 0x88, 0x0, 0x14, 0xee,
    0xc1, 0x0, 0x0, 0x0, 0x3a, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,

    /* U+E7CE "" */
    0x0, 0x0, 0x1, 0x40, 0x0, 0x0, 0x0, 0x0,
    0x5, 0x29, 0x31, 0x0, 0x0, 0x0, 0x30, 0x7c,
    0xcc, 0x11, 0x10, 0x3, 0xc, 0x0, 0x8d, 0x10,
    0x93, 0x40, 0x5c, 0xc4, 0x2, 0x90, 0xd, 0xba,
    0x3, 0x9f, 0xc3, 0x29, 0x8, 0xed, 0x60, 0x44,
    0x2, 0xab, 0xdc, 0x50, 0x26, 0x0, 0x0, 0x3,
    0xef, 0x80, 0x0, 0x0, 0x6c, 0x9a, 0xa4, 0x96,
    0xc7, 0xbc, 0x0, 0x8f, 0x70, 0x29, 0x1, 0xec,
    0x30, 0x86, 0xb1, 0x3, 0xa0, 0xa, 0x4b, 0x10,
    0x8, 0x2, 0xdf, 0x70, 0x53, 0x0, 0x0, 0x0,
    0xa5, 0x9a, 0x30, 0x0, 0x0, 0x0, 0x0, 0x29,
    0x0, 0x0, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 256, .box_w = 12, .box_h = 14, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 84, .adv_w = 256, .box_w = 12, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 156, .adv_w = 256, .box_w = 10, .box_h = 12, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 216, .adv_w = 256, .box_w = 13, .box_h = 14, .ofs_x = 2, .ofs_y = -1}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x8a, 0x8b, 0x1bc
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 58898, .range_length = 445, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 4, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

/*Store all the custom data of the font*/
static lv_font_fmt_txt_dsc_t font_dsc = {
    .glyph_bitmap = gylph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
lv_font_t snow16 = {
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 14,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0)
    .underline_position = 0,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if SNOW16*/

