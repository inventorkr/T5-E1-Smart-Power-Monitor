#include "tuya_cloud_types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tal_api.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_i2c.h"
#include "tkl_pinmux.h"

#include "lvgl.h"
#include "lv_vendor.h"
#include "board_com_api.h"

/* -------------------- font fallback -------------------- */
#if LV_FONT_MONTSERRAT_32
    #define FONT_UNIT_BIG   (&lv_font_montserrat_32)
#elif LV_FONT_MONTSERRAT_24
    #define FONT_UNIT_BIG   (&lv_font_montserrat_24)
#else
    #define FONT_UNIT_BIG   (&lv_font_montserrat_14)
#endif

#if LV_FONT_MONTSERRAT_24
    #define FONT_UNIT_SMALL (&lv_font_montserrat_24)
#else
    #define FONT_UNIT_SMALL (&lv_font_montserrat_14)
#endif

#if LV_FONT_MONTSERRAT_24
    #define FONT_TITLE      (&lv_font_montserrat_24)
#else
    #define FONT_TITLE      (&lv_font_montserrat_14)
#endif

/* -------------------- colors -------------------- */
#define COL_BG         0x000000
#define COL_RING       0xD9FDFF
#define COL_RED        0xFF4A4A
#define COL_YELLOW     0xE8F33A
#define COL_CYAN       0xA8F6FF
#define COL_POWER      0xD7FF2F
#define COL_CURR       0xDCE7FF
#define COL_TITLE      0xF7A73C
#define COL_PANEL      0x4C535A
#define COL_BAT        0xE8E8E8
#define COL_PLUS       0xFF5252
#define COL_SEG_OFF    0x223238
#define COL_OK         0x7CFF7C
#define COL_ERR        0xFF8080
#define COL_WARN       0xFFD36B

/* -------------------- INA226 config -------------------- */
#define INA226_ADDR              0x40
#define INA226_SCL_PIN           16
#define INA226_SDA_PIN           17

#define INA226_REG_CONFIG        0x00
#define INA226_REG_SHUNT_VOLT    0x01
#define INA226_REG_BUS_VOLT      0x02
#define INA226_REG_POWER         0x03
#define INA226_REG_CURRENT       0x04
#define INA226_REG_CALIB         0x05
#define INA226_REG_MFG_ID        0xFE
#define INA226_REG_DIE_ID        0xFF

/*
 * AVG = 16
 * VBUSCT = 1.1ms
 * VSHCT  = 1.1ms
 * MODE   = Shunt and Bus, Continuous
 */
#define INA226_CONFIG_VALUE      0x4527

/*
 * R100 shunt on INA226 module:
 * Rshunt = 0.100 ohm
 * Safe measurable display range = 0A to about 0.8A
 * At 0.8A: Vshunt = 0.8 * 0.1 = 0.08V = 80mV
 */
#define INA226_SHUNT_OHMS        0.100f
#define INA226_CURRENT_LSB       0.0001f
#define INA226_CAL_VALUE         0x0200

/*
 * Calibration for the module shown in the photo:
 * Shunt marked R100 = 0.1 ohm.
 * Your reference test:
 * - real supply around 12.00V
 * - display before correction around 13.26V
 * - 5W load was displayed around 50W with the wrong 0.002 ohm setting
 *
 * BUS factor: 12.00 / 13.26 = 0.90498
 * SHUNT factor is left at 1.0 first for honest raw shunt measurement.
 * If your 5W load still reads low, use 1.33 instead of 1.0.
 */
#define SHUNT_MV_CAL_FACTOR      1.0f
#define BUS_VOLT_CAL_FACTOR      0.90498f

/* INA226 shunt measurement range */
#define INA226_MAX_SHUNT_MV      81.92f
#define INA226_OVRANGE_WARN_MV   80.0f

/* -------------------- UI scale -------------------- */
#define MAX_CURRENT_A            0.8f
#define MAX_POWER_W              15.0f

/* -------------------- behavior -------------------- */
#define CURRENT_DEADBAND_A       0.02f
#define POWER_DEADBAND_W         0.20f
#define SHUNT_ZERO_WINDOW_MV     0.03f
#define FILTER_ALPHA             0.20f

/* -------------------- globals -------------------- */
static lv_obj_t *g_top_slot[4];
static lv_obj_t *g_curr_slot[4];
static lv_obj_t *g_top_unit = NULL;
static lv_obj_t *g_curr_unit = NULL;

static lv_obj_t *g_seg_bg[6];
static lv_obj_t *g_seg_fill[6];

static lv_obj_t *g_status_label = NULL;
static lv_obj_t *g_voltage_label = NULL;
static lv_obj_t *g_debug_label = NULL;

static TUYA_I2C_NUM_E g_i2c_port = TUYA_I2C_NUM_0;
static uint8_t g_ina226_addr = INA226_ADDR;
static BOOL_T g_ina226_ready = FALSE;

static float g_power_w = 0.0f;
static float g_current = 0.0f;
static float g_voltage = 0.0f;

static int16_t g_last_raw_shunt = 0;
static uint16_t g_last_raw_bus = 0;
static int16_t g_last_raw_current = 0;
static uint16_t g_last_raw_power = 0;
static BOOL_T g_overrange = FALSE;

/* -------------------- gauge segments -------------------- */
typedef struct {
    int start;
    int end;
    uint32_t color;
    int width;
} gauge_seg_t;

static const gauge_seg_t g_gauge_segs[6] = {
    {180, 214, COL_RED,    16},
    {216, 252, COL_YELLOW, 16},
    {254, 288, 0xC7FBFF,   16},
    {300, 332, COL_CYAN,   12},
    {336, 366, COL_CYAN,   12},
    {370, 400, COL_CYAN,   12},
};

/* -------------------- helpers -------------------- */
static float absf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float lowpass(float prev, float now, float alpha)
{
    return prev + alpha * (now - prev);
}

/* -------------------- UI helpers -------------------- */
static lv_obj_t *make_arc(lv_obj_t *parent, int size, int start, int end, uint32_t color, int width)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_remove_style_all(arc);
    lv_obj_set_size(arc, size, size);
    lv_obj_center(arc);

    lv_arc_set_rotation(arc, 0);
    lv_arc_set_bg_angles(arc, start, end);
    lv_arc_set_value(arc, 0);

    lv_obj_set_style_arc_width(arc, width, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_KNOB);

    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return arc;
}

static lv_obj_t *make_progress_seg(lv_obj_t *parent, int size, int start, int end,
                                   uint32_t color, int width, bool background)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_remove_style_all(arc);
    lv_obj_set_size(arc, size, size);
    lv_obj_center(arc);

    lv_arc_set_rotation(arc, 0);
    lv_arc_set_bg_angles(arc, start, end);
    lv_arc_set_range(arc, 0, 1000);
    lv_arc_set_value(arc, background ? 1000 : 0);

    if (background) {
        lv_obj_set_style_arc_width(arc, width, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, lv_color_hex(COL_SEG_OFF), LV_PART_MAIN);
        lv_obj_set_style_arc_opa(arc, 120, LV_PART_MAIN);

        lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(arc, 0, LV_PART_KNOB);
    } else {
        lv_obj_set_style_arc_width(arc, 0, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);

        lv_obj_set_style_arc_width(arc, width, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
        lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);

        lv_obj_set_style_arc_width(arc, 0, LV_PART_KNOB);
    }

    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return arc;
}

static lv_obj_t *make_seg(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_color_t color)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_radius(obj, h / 2, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    return obj;
}

static void seg_map(char c, bool seg[7])
{
    memset(seg, 0, sizeof(bool) * 7);

    switch (c) {
        case '0': seg[0]=seg[1]=seg[2]=seg[3]=seg[4]=seg[5]=true; break;
        case '1': seg[1]=seg[2]=true; break;
        case '2': seg[0]=seg[1]=seg[6]=seg[4]=seg[3]=true; break;
        case '3': seg[0]=seg[1]=seg[6]=seg[2]=seg[3]=true; break;
        case '4': seg[5]=seg[6]=seg[1]=seg[2]=true; break;
        case '5': seg[0]=seg[5]=seg[6]=seg[2]=seg[3]=true; break;
        case '6': seg[0]=seg[5]=seg[6]=seg[4]=seg[2]=seg[3]=true; break;
        case '7': seg[0]=seg[1]=seg[2]=true; break;
        case '8': seg[0]=seg[1]=seg[2]=seg[3]=seg[4]=seg[5]=seg[6]=true; break;
        case '9': seg[0]=seg[1]=seg[2]=seg[3]=seg[5]=seg[6]=true; break;
        case '-': seg[6]=true; break;
        default: break;
    }
}

static lv_obj_t *create_digit_custom(lv_obj_t *parent, char c, bool with_dot,
                                     lv_color_t color, int t, int w, int h)
{
    bool seg[7];
    seg_map(c, seg);

    lv_coord_t total_w = w + 2 * t + (with_dot ? (t + 3) : 0);
    lv_coord_t total_h = 2 * h + 3 * t;

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, total_w, total_h);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    if(seg[0]) make_seg(cont, t, 0, w, t, color);
    if(seg[1]) make_seg(cont, w + t, t, t, h, color);
    if(seg[2]) make_seg(cont, w + t, h + 2 * t, t, h, color);
    if(seg[3]) make_seg(cont, t, 2 * h + 2 * t, w, t, color);
    if(seg[4]) make_seg(cont, 0, h + 2 * t, t, h, color);
    if(seg[5]) make_seg(cont, 0, t, t, h, color);
    if(seg[6]) make_seg(cont, t, h + t, w, t, color);

    if(with_dot) {
        make_seg(cont, w + 2 * t + 3, total_h - t, t, t, color);
    }

    return cont;
}

static lv_obj_t *make_slot(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *slot = lv_obj_create(parent);
    lv_obj_remove_style_all(slot);
    lv_obj_set_pos(slot, x, y);
    lv_obj_set_size(slot, w, h);
    lv_obj_clear_flag(slot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return slot;
}

static void set_slot_digit(lv_obj_t *slot, char c, bool with_dot, lv_color_t color, int t, int w, int h)
{
    lv_obj_clean(slot);
    if(c == ' ') return;
    lv_obj_t *d = create_digit_custom(slot, c, with_dot, color, t, w, h);
    lv_obj_center(d);
}

static void ui_set_progress_from_current(float current_a)
{
    float abs_a = current_a;
    if (abs_a < 0.0f) abs_a = -abs_a;

    float pct = abs_a / MAX_CURRENT_A;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    if (pct <= 0.0001f) {
        lv_arc_set_value(g_seg_fill[0], 90);
        lv_arc_set_value(g_seg_fill[1], 0);
        lv_arc_set_value(g_seg_fill[2], 0);
        lv_arc_set_value(g_seg_fill[3], 0);
        lv_arc_set_value(g_seg_fill[4], 0);
        lv_arc_set_value(g_seg_fill[5], 0);
        return;
    }

    float scaled = pct * 6.0f;

    for (int i = 0; i < 6; i++) {
        float local = scaled - (float)i;
        if (local <= 0.0f) {
            lv_arc_set_value(g_seg_fill[i], 0);
        } else if (local >= 1.0f) {
            lv_arc_set_value(g_seg_fill[i], 1000);
        } else {
            lv_arc_set_value(g_seg_fill[i], (int)(local * 1000.0f));
        }
    }
}

static void ui_set_status_text(const char *txt, lv_color_t color)
{
    lv_vendor_disp_lock();
    lv_label_set_text(g_status_label, txt);
    lv_obj_set_style_text_color(g_status_label, color, 0);
    lv_vendor_disp_unlock();
}

static void ui_set_debug_text(int16_t raw_shunt, uint16_t raw_bus)
{
    char dbg[64];
    snprintf(dbg, sizeof(dbg), "RS=%d RB=%u", raw_shunt, raw_bus);

    lv_vendor_disp_lock();
    lv_label_set_text(g_debug_label, dbg);
    lv_vendor_disp_unlock();
}

static void ui_set_values(float power_w, float current_a, float voltage_v)
{
    char pbuf[8];
    char abuf[8];
    char vbuf[32];

    if (power_w < 0.0f) power_w = 0.0f;
    if (power_w > MAX_POWER_W) power_w = MAX_POWER_W;

    if (current_a < -MAX_CURRENT_A) current_a = -MAX_CURRENT_A;
    if (current_a >  MAX_CURRENT_A) current_a =  MAX_CURRENT_A;

    snprintf(pbuf, sizeof(pbuf), "%4.0f", power_w);
    snprintf(abuf, sizeof(abuf), "%4.1f", current_a);
    snprintf(vbuf, sizeof(vbuf), "V=%.2fV", voltage_v);

    lv_vendor_disp_lock();

    set_slot_digit(g_top_slot[0], pbuf[0], false, lv_color_hex(COL_POWER), 4, 18, 24);
    set_slot_digit(g_top_slot[1], pbuf[1], false, lv_color_hex(COL_POWER), 4, 18, 24);
    set_slot_digit(g_top_slot[2], pbuf[2], false, lv_color_hex(COL_POWER), 4, 18, 24);
    set_slot_digit(g_top_slot[3], pbuf[3], false, lv_color_hex(COL_POWER), 4, 18, 24);

    set_slot_digit(g_curr_slot[0], abuf[0], false, lv_color_hex(COL_CURR), 4, 16, 20);
    set_slot_digit(g_curr_slot[1], abuf[1], true,  lv_color_hex(COL_CURR), 4, 16, 20);
    set_slot_digit(g_curr_slot[2], abuf[3], false, lv_color_hex(COL_CURR), 4, 16, 20);
    set_slot_digit(g_curr_slot[3], ' ', false, lv_color_hex(COL_CURR), 4, 16, 20);

    lv_label_set_text(g_voltage_label, vbuf);
    ui_set_progress_from_current(current_a);

    lv_vendor_disp_unlock();
}

/* -------------------- INA226 low-level -------------------- */
static OPERATE_RET ina226_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)(value & 0xFF);

    return tkl_i2c_master_send(g_i2c_port, g_ina226_addr, buf, 3, TRUE);
}

static OPERATE_RET ina226_read_reg(uint8_t reg, uint16_t *value)
{
    OPERATE_RET rt;
    uint8_t tx = reg;
    uint8_t rx[2] = {0};

    rt = tkl_i2c_master_send(g_i2c_port, g_ina226_addr, &tx, 1, FALSE);
    if (rt != OPRT_OK) return rt;

    rt = tkl_i2c_master_receive(g_i2c_port, g_ina226_addr, rx, 2, TRUE);
    if (rt != OPRT_OK) return rt;

    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return OPRT_OK;
}

static OPERATE_RET ina226_configure(void)
{
    OPERATE_RET rt;

    rt = ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_VALUE);
    if (rt != OPRT_OK) return rt;

    rt = ina226_write_reg(INA226_REG_CALIB, INA226_CAL_VALUE);
    if (rt != OPRT_OK) return rt;

    return OPRT_OK;
}

static OPERATE_RET ina226_read_measurements(float *current_a, float *voltage_v, float *power_w, float *shunt_mv)
{
    OPERATE_RET rt;
    uint16_t raw_shunt_u = 0;
    uint16_t raw_bus_u = 0;
    uint16_t raw_current_u = 0;
    uint16_t raw_power_u = 0;

    float raw_shunt_mv = 0.0f;
    float raw_bus_v = 0.0f;
    float corrected_shunt_mv = 0.0f;
    float corrected_bus_v = 0.0f;

    rt = ina226_write_reg(INA226_REG_CALIB, INA226_CAL_VALUE);
    if (rt != OPRT_OK) return rt;

    rt = ina226_read_reg(INA226_REG_SHUNT_VOLT, &raw_shunt_u);
    if (rt != OPRT_OK) return rt;

    rt = ina226_read_reg(INA226_REG_BUS_VOLT, &raw_bus_u);
    if (rt != OPRT_OK) return rt;

    (void)ina226_read_reg(INA226_REG_CURRENT, &raw_current_u);
    (void)ina226_read_reg(INA226_REG_POWER, &raw_power_u);

    g_last_raw_shunt = (int16_t)raw_shunt_u;
    g_last_raw_bus = raw_bus_u;
    g_last_raw_current = (int16_t)raw_current_u;
    g_last_raw_power = raw_power_u;

    raw_shunt_mv = ((float)g_last_raw_shunt) * 0.0025f;
    raw_bus_v    = ((float)g_last_raw_bus) * 0.00125f;

    corrected_shunt_mv = raw_shunt_mv * SHUNT_MV_CAL_FACTOR;
    corrected_bus_v    = raw_bus_v * BUS_VOLT_CAL_FACTOR;

    *shunt_mv  = corrected_shunt_mv;
    *voltage_v = corrected_bus_v;

    *current_a = (corrected_shunt_mv / 1000.0f) / INA226_SHUNT_OHMS;
    *power_w   = corrected_bus_v * (*current_a);

    if (absf_local(corrected_shunt_mv) >= INA226_OVRANGE_WARN_MV) {
        g_overrange = TRUE;
    } else {
        g_overrange = FALSE;
    }

    PR_NOTICE("INA226_R100_0P8A: raw_shunt=%d raw_bus=%u raw_current=%d raw_power=%u raw_shunt_mv=%.5f corr_shunt_mv=%.5f raw_bus_v=%.4f corr_bus_v=%.4f current=%.5fA power=%.5fW limit=%d",
              g_last_raw_shunt,
              g_last_raw_bus,
              g_last_raw_current,
              g_last_raw_power,
              raw_shunt_mv,
              corrected_shunt_mv,
              raw_bus_v,
              corrected_bus_v,
              *current_a,
              *power_w,
              g_overrange);

    return OPRT_OK;
}

static void i2c_pinmux_for_port(TUYA_I2C_NUM_E port)
{
    if (port == TUYA_I2C_NUM_0) {
        tkl_io_pinmux_config(INA226_SCL_PIN, TUYA_IIC0_SCL);
        tkl_io_pinmux_config(INA226_SDA_PIN, TUYA_IIC0_SDA);
    }
}

static OPERATE_RET i2c_try_init_port(TUYA_I2C_NUM_E port)
{
    TUYA_IIC_BASE_CFG_T cfg;

    i2c_pinmux_for_port(port);

    cfg.role = TUYA_IIC_MODE_MASTER;
    cfg.speed = TUYA_IIC_BUS_SPEED_100K;
    cfg.addr_width = TUYA_IIC_ADDRESS_7BIT;

    return tkl_i2c_init(port, &cfg);
}

static BOOL_T ina226_detect_and_init(void)
{
    uint16_t cfg_reg = 0;
    uint16_t mfg_id = 0;
    uint16_t die_id = 0;

    g_i2c_port = TUYA_I2C_NUM_0;
    g_ina226_addr = INA226_ADDR;

    if (i2c_try_init_port(g_i2c_port) != OPRT_OK) {
        PR_ERR("I2C init failed");
        return FALSE;
    }

    /* First try the original address from the old code: 0x40 */
    if (ina226_read_reg(INA226_REG_CONFIG, &cfg_reg) != OPRT_OK) {
        PR_ERR("INA226 not found at 0x%02X, scanning 0x40..0x4F", INA226_ADDR);

        BOOL_T found = FALSE;
        for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
            g_ina226_addr = addr;
            if (ina226_read_reg(INA226_REG_CONFIG, &cfg_reg) == OPRT_OK) {
                PR_NOTICE("I2C DEVICE FOUND AT 0x%02X cfg=0x%04X", addr, cfg_reg);
                found = TRUE;
                break;
            }
        }

        if (found == FALSE) {
            g_ina226_addr = INA226_ADDR;
            PR_ERR("INA226 scan failed. Check SDA=17, SCL=16, VCC, GND, and pull-up resistors.");
            return FALSE;
        }
    }

    if (ina226_configure() != OPRT_OK) {
        PR_ERR("INA226 configure failed at addr=0x%02X", g_ina226_addr);
        return FALSE;
    }

    (void)ina226_read_reg(INA226_REG_MFG_ID, &mfg_id);
    (void)ina226_read_reg(INA226_REG_DIE_ID, &die_id);

    PR_NOTICE("INA226_R100_0P8A OK: addr=0x%02X cfg=0x%04X mfg=0x%04X die=0x%04X",
              g_ina226_addr, cfg_reg, mfg_id, die_id);

    return TRUE;
}

/* -------------------- timer -------------------- */
static void sensor_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    float current_a = 0.0f;
    float voltage_v = 0.0f;
    float power_w = 0.0f;
    float shunt_mv = 0.0f;

    if (g_ina226_ready == TRUE) {
        if (ina226_read_measurements(&current_a, &voltage_v, &power_w, &shunt_mv) == OPRT_OK) {

            if (absf_local(shunt_mv) < SHUNT_ZERO_WINDOW_MV) {
                current_a = 0.0f;
            }

            if (absf_local(current_a) < CURRENT_DEADBAND_A) {
                current_a = 0.0f;
            }

            if (power_w < 0.0f) {
                power_w = -power_w;
            }

            if (power_w < POWER_DEADBAND_W) {
                power_w = 0.0f;
            }

            g_current = lowpass(g_current, current_a, FILTER_ALPHA);
            g_voltage = lowpass(g_voltage, voltage_v, FILTER_ALPHA);
            g_power_w = lowpass(g_power_w, power_w, FILTER_ALPHA);

            if (absf_local(g_current) < CURRENT_DEADBAND_A) g_current = 0.0f;
            if (g_power_w < POWER_DEADBAND_W) g_power_w = 0.0f;
            if (g_voltage < 0.02f) g_voltage = 0.0f;

            if (g_overrange == TRUE) {
                ui_set_status_text("0.8A LIMIT REACHED", lv_color_hex(COL_WARN));
            } else {
                ui_set_status_text("R100 SHUNT 0.8A MODE", lv_color_hex(COL_OK));
            }

            ui_set_debug_text(g_last_raw_shunt, g_last_raw_bus);
            ui_set_values(g_power_w, g_current, g_voltage);
            return;
        }

        ui_set_status_text("INA226 READ FAIL", lv_color_hex(COL_ERR));
        ui_set_debug_text(g_last_raw_shunt, g_last_raw_bus);
    } else {
        ui_set_status_text("INA226 NOT FOUND", lv_color_hex(COL_ERR));
        ui_set_debug_text(0, 0);
    }

    ui_set_values(0.0f, 0.0f, 0.0f);
}

/* -------------------- UI build -------------------- */
static void create_gauge_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    make_arc(scr, 355, 135, 405, COL_RING, 8);

    for (int i = 0; i < 6; i++) {
        g_seg_bg[i] = make_progress_seg(scr, 315,
                                        g_gauge_segs[i].start,
                                        g_gauge_segs[i].end,
                                        g_gauge_segs[i].color,
                                        g_gauge_segs[i].width,
                                        true);
    }

    for (int i = 0; i < 6; i++) {
        g_seg_fill[i] = make_progress_seg(scr, 315,
                                          g_gauge_segs[i].start,
                                          g_gauge_segs[i].end,
                                          g_gauge_segs[i].color,
                                          g_gauge_segs[i].width,
                                          false);
    }

    lv_obj_t *inner = lv_obj_create(scr);
    lv_obj_remove_style_all(inner);
    lv_obj_set_size(inner, 270, 270);
    lv_obj_center(inner);
    lv_obj_set_style_radius(inner, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(inner, lv_color_hex(0x060606), 0);
    lv_obj_set_style_bg_opa(inner, LV_OPA_COVER, 0);
    lv_obj_clear_flag(inner, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *gloss = lv_obj_create(inner);
    lv_obj_remove_style_all(gloss);
    lv_obj_set_size(gloss, 250, 72);
    lv_obj_align(gloss, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_radius(gloss, 36, 0);
    lv_obj_set_style_bg_color(gloss, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_bg_opa(gloss, 90, 0);

    lv_obj_t *bat = lv_obj_create(inner);
    lv_obj_remove_style_all(bat);
    lv_obj_set_size(bat, 66, 24);
    lv_obj_align(bat, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_border_width(bat, 2, 0);
    lv_obj_set_style_border_color(bat, lv_color_hex(COL_BAT), 0);
    lv_obj_set_style_bg_opa(bat, LV_OPA_TRANSP, 0);

    lv_obj_t *bat_tip = lv_obj_create(inner);
    lv_obj_remove_style_all(bat_tip);
    lv_obj_set_size(bat_tip, 8, 10);
    lv_obj_align_to(bat_tip, bat, LV_ALIGN_OUT_RIGHT_MID, 3, 0);
    lv_obj_set_style_bg_color(bat_tip, lv_color_hex(COL_BAT), 0);
    lv_obj_set_style_bg_opa(bat_tip, LV_OPA_COVER, 0);

    lv_obj_t *bat_plus = lv_label_create(inner);
    lv_obj_set_style_text_font(bat_plus, FONT_TITLE, 0);
    lv_obj_set_style_text_color(bat_plus, lv_color_hex(COL_PLUS), 0);
    lv_label_set_text(bat_plus, "+");
    lv_obj_align_to(bat_plus, bat, LV_ALIGN_CENTER, 0, -1);

    /* top = watt */
    lv_obj_t *top_group = lv_obj_create(inner);
    lv_obj_remove_style_all(top_group);
    lv_obj_set_size(top_group, 250, 78);
    lv_obj_align(top_group, LV_ALIGN_CENTER, 0, -12);

    g_top_slot[0] = make_slot(top_group,   0, 0, 28, 64);
    g_top_slot[1] = make_slot(top_group,  32, 0, 28, 64);
    g_top_slot[2] = make_slot(top_group,  64, 0, 28, 64);
    g_top_slot[3] = make_slot(top_group,  96, 0, 28, 64);

    g_top_unit = lv_label_create(top_group);
    lv_obj_set_style_text_font(g_top_unit, FONT_UNIT_BIG, 0);
    lv_obj_set_style_text_color(g_top_unit, lv_color_hex(COL_POWER), 0);
    lv_label_set_text(g_top_unit, "W");
    lv_obj_set_pos(g_top_unit, 138, 14);

    /* bottom = current */
    lv_obj_t *curr_group = lv_obj_create(inner);
    lv_obj_remove_style_all(curr_group);
    lv_obj_set_size(curr_group, 220, 54);
    lv_obj_align(curr_group, LV_ALIGN_CENTER, 0, 62);

    g_curr_slot[0] = make_slot(curr_group,   0, 0, 28, 56);
    g_curr_slot[1] = make_slot(curr_group,  32, 0, 36, 56);
    g_curr_slot[2] = make_slot(curr_group,  72, 0, 28, 56);
    g_curr_slot[3] = make_slot(curr_group, 104, 0, 28, 56);

    g_curr_unit = lv_label_create(curr_group);
    lv_obj_set_style_text_font(g_curr_unit, FONT_UNIT_SMALL, 0);
    lv_obj_set_style_text_color(g_curr_unit, lv_color_hex(COL_CURR), 0);
    lv_label_set_text(g_curr_unit, "A");
    lv_obj_set_pos(g_curr_unit, 140, 12);

    /* status bar */
    lv_obj_t *status_bar = lv_obj_create(scr);
    lv_obj_remove_style_all(status_bar);
    lv_obj_set_size(status_bar, 300, 26);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 14);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x20252A), 0);
    lv_obj_set_style_bg_opa(status_bar, 220, 0);
    lv_obj_set_style_radius(status_bar, 6, 0);

    g_status_label = lv_label_create(status_bar);
    lv_obj_set_style_text_font(g_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(COL_OK), 0);
    lv_label_set_text(g_status_label, "BOOT");
    lv_obj_center(g_status_label);

    /* voltage bar */
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, 282, 40);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -42);
    lv_obj_set_style_bg_color(bar, lv_color_hex(COL_PANEL), 0);
    lv_obj_set_style_bg_opa(bar, 220, 0);
    lv_obj_set_style_radius(bar, 8, 0);

    g_voltage_label = lv_label_create(bar);
    lv_obj_set_style_text_font(g_voltage_label, FONT_TITLE, 0);
    lv_obj_set_style_text_color(g_voltage_label, lv_color_hex(COL_TITLE), 0);
    lv_label_set_text(g_voltage_label, "V=0.00V");
    lv_obj_center(g_voltage_label);

    /* debug bar */
    lv_obj_t *dbg_bar = lv_obj_create(scr);
    lv_obj_remove_style_all(dbg_bar);
    lv_obj_set_size(dbg_bar, 220, 24);
    lv_obj_align(dbg_bar, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(dbg_bar, lv_color_hex(0x1E1E1E), 0);
    lv_obj_set_style_bg_opa(dbg_bar, 200, 0);
    lv_obj_set_style_radius(dbg_bar, 6, 0);

    g_debug_label = lv_label_create(dbg_bar);
    lv_obj_set_style_text_font(g_debug_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_debug_label, lv_color_hex(0xBFCAD4), 0);
    lv_label_set_text(g_debug_label, "RS=0 RB=0");
    lv_obj_center(g_debug_label);
}

void user_main(void)
{
    tal_log_init(TAL_LOG_LEVEL_DEBUG, 4096, (TAL_LOG_OUTPUT_CB)tkl_log_output);

    PR_NOTICE("=== INA226 R100 SHUNT 0.8A START ===");
    PR_NOTICE("Project name:        %s", PROJECT_NAME);
    PR_NOTICE("App version:         %s", PROJECT_VERSION);
    PR_NOTICE("Compile time:        %s", __DATE__);
    PR_NOTICE("TuyaOpen version:    %s", OPEN_VERSION);
    PR_NOTICE("TuyaOpen commit-id:  %s", OPEN_COMMIT);
    PR_NOTICE("Platform chip:       %s", PLATFORM_CHIP);
    PR_NOTICE("Platform board:      %s", PLATFORM_BOARD);
    PR_NOTICE("Platform commit-id:  %s", PLATFORM_COMMIT);

    board_register_hardware();
    lv_vendor_init(DISPLAY_NAME);

    lv_vendor_disp_lock();
    create_gauge_ui();
    lv_vendor_disp_unlock();

    g_ina226_ready = ina226_detect_and_init();

    if (g_ina226_ready == TRUE) {
        ui_set_status_text("R100 SHUNT 0.8A MODE", lv_color_hex(COL_OK));
    } else {
        ui_set_status_text("INA226 NOT FOUND", lv_color_hex(COL_ERR));
    }

    ui_set_values(0.0f, 0.0f, 0.0f);
    lv_timer_create(sensor_timer_cb, 250, NULL);

    lv_vendor_start(5, 1024 * 8);
}

#if OPERATING_SYSTEM == SYSTEM_LINUX

void main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    user_main();
    while (1) {
        tal_system_sleep(500);
    }
}

#else

static THREAD_HANDLE ty_app_thread = NULL;

static void tuya_app_thread(void *arg)
{
    (void)arg;
    user_main();
    tal_thread_delete(ty_app_thread);
    ty_app_thread = NULL;
}

void tuya_app_main(void)
{
    THREAD_CFG_T thrd_param;

    memset(&thrd_param, 0, sizeof(THREAD_CFG_T));
    thrd_param.stackDepth = 1024 * 4;
    thrd_param.priority = THREAD_PRIO_1;
    thrd_param.thrdname = "tuya_app_main";

    tal_thread_create_and_start(&ty_app_thread, NULL, NULL, tuya_app_thread, NULL, &thrd_param);
}

#endif
