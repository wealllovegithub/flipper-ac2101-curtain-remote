/**
 * @file ac2101_curtain_remote.c
 * @brief 40 kBaud 2-FSK RF capture & remote app for AC2101-01A / Allesin motorized blinds.
 *
 * Flipper's stock Sub-GHz Read RAW pipeline drops pulses <= 50 us (SubGhzWorker's 30 us
 * glitch filter + raw decoder's te_short = 50 us) and uses a 16 us TIM2 input filter.
 * This app configures the CC1101 with the FSK_40K_19K 40 kBaud / +/-19 kHz preset, tunes
 * TIM2's input filter down to 8 us, captures directly via furi_hal_subghz async RX with a
 * pre-trigger ring buffer + RSSI gate, and replays lossless bursts directly from RAM.
 */

#include <assets_icons.h>
#include <dolphin/dolphin.h>
#include <flipper_format/flipper_format.h>
#include <flipper_format/flipper_format_i.h>
#include <furi.h>
#include <furi_hal_subghz.h>
#include <gui/elements.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <lib/drivers/cc1101_regs.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>
#include <lib/subghz/devices/devices.h>
#include <lib/toolbox/stream/stream.h>
#include <lib/toolbox/strint.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32wbxx_ll_tim.h>
#include <storage/storage.h>
#include <string.h>

#define AC2101_FOLDER            EXT_PATH("subghz/AC2101")
#define AC2101_FREQ_PRIMARY      433914000U
#define AC2101_FREQ_ALT          433920000U
#define AC2101_RING_LEN          2048U
#define AC2101_CAP_MAX_PULSES    22000U
#define AC2101_CAP_FALLBACK      16384U
#define AC2101_PRETRIG_US        4000U
#define AC2101_SPLIT_CHUNK_US    30000U
#define AC2101_RAW_LINE_ITEMS    512U
#define AC2101_MAX_BURST_MS      1200U
#define AC2101_TRAIL_SILENCE_MS  160U
#define AC2101_MIN_AUTO_BURST_MS 220U
#define AC2101_DEFAULT_THRESH    (-70)
#define AC2101_MIN_THRESH        (-85)
#define AC2101_MAX_THRESH        (-50)

typedef enum {
    Ac2101CmdOpen = 0,
    Ac2101CmdStop,
    Ac2101CmdClose,
    Ac2101CmdCount,
} Ac2101CmdId;

typedef enum {
    Ac2101ViewRemote = 0,
    Ac2101ViewCapture,
} Ac2101ViewId;

typedef enum {
    Ac2101EventCapDone = 1,
} Ac2101CustomEvent;

typedef enum {
    CapPhaseArmed = 0,
    CapPhaseRecording,
    CapPhaseSaving,
} CapPhase;

typedef struct {
    const char* name;
    const char* file_name;
    const char* short_sym;
    CanvasDirection tri_dir;
    bool has_triangle;
} Ac2101SlotDef;

typedef struct {
    bool exists;
    uint32_t pulses;
    uint32_t duration_ms;
    uint16_t min_pulse_us;
} Ac2101SlotMeta;

typedef struct {
    uint8_t selected;
    uint32_t freq_hz;
    bool tx_active;
    Ac2101SlotMeta slots[Ac2101CmdCount];
    char status[28];
} RemoteViewModel;

typedef struct {
    Ac2101CmdId cmd;
    uint32_t freq_hz;
    CapPhase phase;
    int32_t cur_rssi_dbm;
    int32_t thresh_dbm;
    uint32_t pulses;
    uint32_t ms;
    uint16_t min_us;
} CaptureViewModel;

typedef struct {
    Gui* gui;
    NotificationApp* notifications;
    ViewDispatcher* view_dispatcher;
    View* remote_view;
    View* capture_view;
    const SubGhzDevice* radio_device;

    uint32_t freq_hz;
    int32_t thresh_dbm;
    Ac2101CmdId active_cap_cmd;
    Ac2101SlotMeta slots[Ac2101CmdCount];

    /* TX state */
    FuriTimer* tx_poll_timer;
    int16_t* tx_buf;
    volatile uint32_t tx_len;
    volatile uint32_t tx_idx;
    volatile bool tx_active;

    /* RX capture state */
    FuriThread* cap_thread;
    int16_t* ring_buf;
    int16_t* cap_buf;
    uint32_t cap_max_pulses;
    volatile uint32_t ring_head;
    volatile uint32_t ring_count;
    volatile uint32_t cap_count;
    volatile uint32_t cap_total_us;
    volatile uint16_t cap_min_us;
    volatile CapPhase cap_phase;
    volatile bool cap_abort;
    volatile bool cap_manual_trig;
    volatile bool cap_manual_stop;
} Ac2101App;

static const Ac2101SlotDef AC2101_SLOT_DEFS[Ac2101CmdCount] = {
    [Ac2101CmdOpen] =
        {.name = "Open",
         .file_name = "Open.sub",
         .short_sym = "OPEN",
         .tri_dir = CanvasDirectionBottomToTop,
         .has_triangle = true},
    [Ac2101CmdStop] =
        {.name = "Stop",
         .file_name = "Stop.sub",
         .short_sym = "STOP",
         .tri_dir = CanvasDirectionLeftToRight,
         .has_triangle = false},
    [Ac2101CmdClose] =
        {.name = "Close",
         .file_name = "Close.sub",
         .short_sym = "CLOSE",
         .tri_dir = CanvasDirectionTopToBottom,
         .has_triangle = true},
};

/* Exact FSK_40K_19K CC1101 preset: 39.97 kBaud (25 us), +/-19.04 kHz dev, 203.12 kHz RX BW */
static const uint8_t AC2101_FSK_40K_PRESET[] = {
    CC1101_IOCFG0,   0x0D, /* GDO0: async serial data output / TX input */
    CC1101_FIFOTHR,  0x07,
    CC1101_PKTCTRL0, 0x32, /* Async serial continuous mode */
    CC1101_FSCTRL1,  0x06, /* IF = 152.34 kHz */
    CC1101_DEVIATN,  0x34, /* +/-19.043 kHz deviation */
    CC1101_MDMCFG0,  0x00,
    CC1101_MDMCFG1,  0x00,
    CC1101_MDMCFG2,  0x00, /* 2-FSK, no sync gate */
    CC1101_MDMCFG3,  0x93, /* DRATE_M = 147 */
    CC1101_MDMCFG4,  0x8A, /* CHANBW = 203.12 kHz, DRATE = 39.970 kBaud */
    CC1101_MCSM0,    0x18, /* Autocalibrate on IDLE -> RX/TX */
    CC1101_FOCCFG,   0x16, /* Frequency offset compensation */
    CC1101_AGCCTRL0, 0x91,
    CC1101_AGCCTRL1, 0x00,
    CC1101_AGCCTRL2, 0x07,
    CC1101_WORCTRL,  0xFB,
    CC1101_FREND0,   0x10,
    CC1101_FREND1,   0x56,
    0x00,            0x00, /* End register pairs */
    0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* +10 dBm at 433 MHz */
};

static const NotificationSequence sequence_set_cyan_255 = {
    &message_red_0,
    &message_green_255,
    &message_blue_255,
    &message_do_not_reset,
    NULL,
};

static void ac2101_slot_path(Ac2101CmdId cmd, FuriString* out_path) {
    furi_string_printf(out_path, "%s/%s", AC2101_FOLDER, AC2101_SLOT_DEFS[cmd].file_name);
}

static uint32_t ac2101_count_sub_chunks(const int16_t* raw, uint32_t raw_len, uint16_t* out_min_us) {
    /* Consecutive same-sign int16 entries represent one continuous >30ms pulse split to fit int16. */
    uint32_t chunks = 0;
    uint16_t min_us = 0xFFFF;
    uint32_t acc = 0;
    int8_t acc_sign = 0;

    for(uint32_t i = 0; i < raw_len; i++) {
        int32_t v = raw[i];
        if(v == 0) continue;
        int8_t s = (v > 0) ? 1 : -1;
        uint32_t mag = (uint32_t)(v > 0 ? v : -v);
        if(s == acc_sign) {
            acc += mag;
        } else {
            if(acc_sign != 0) {
                chunks++;
                if(acc < min_us) min_us = (uint16_t)acc;
            }
            acc_sign = s;
            acc = mag;
        }
    }
    if(acc_sign != 0) {
        chunks++;
        if(acc < min_us) min_us = (uint16_t)acc;
    }
    if(out_min_us) *out_min_us = (chunks > 0) ? min_us : 0;
    return chunks;
}

static bool ac2101_save_slot_sub(
    Ac2101CmdId cmd,
    const int16_t* raw,
    uint32_t raw_len,
    uint32_t freq_hz) {
    if(!raw || raw_len == 0) return false;

    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_simply_mkdir(storage, EXT_PATH("subghz"));
    storage_simply_mkdir(storage, AC2101_FOLDER);

    FlipperFormat* ff = flipper_format_buffered_file_alloc(storage);
    FuriString* path = furi_string_alloc();
    ac2101_slot_path(cmd, path);

    bool ok = false;
    do {
        if(!flipper_format_buffered_file_open_always(ff, furi_string_get_cstr(path))) break;
        if(!flipper_format_write_header_cstr(ff, "Flipper SubGhz RAW File", 1)) break;
        if(!flipper_format_write_uint32(ff, "Frequency", &freq_hz, 1)) break;
        if(!flipper_format_write_string_cstr(ff, "Preset", "FuriHalSubGhzPresetCustom")) break;
        if(!flipper_format_write_string_cstr(ff, "Custom_preset_module", "CC1101")) break;
        if(!flipper_format_write_hex(
               ff, "Custom_preset_data", AC2101_FSK_40K_PRESET, sizeof(AC2101_FSK_40K_PRESET)))
            break;
        if(!flipper_format_write_string_cstr(ff, "Protocol", "RAW")) break;

        int32_t* line_buf = malloc(AC2101_RAW_LINE_ITEMS * sizeof(int32_t));
        if(!line_buf) break;

        uint32_t line_idx = 0;
        int32_t acc = 0;
        int8_t acc_sign = 0;
        bool write_ok = true;

        for(uint32_t i = 0; i < raw_len && write_ok; i++) {
            int32_t v = raw[i];
            if(v == 0) continue;
            int8_t s = (v > 0) ? 1 : -1;
            if(s == acc_sign) {
                acc += v;
            } else {
                if(acc_sign != 0) {
                    line_buf[line_idx++] = acc;
                    if(line_idx == AC2101_RAW_LINE_ITEMS) {
                        write_ok = flipper_format_write_int32(ff, "RAW_Data", line_buf, line_idx);
                        line_idx = 0;
                    }
                }
                acc_sign = s;
                acc = v;
            }
        }
        if(write_ok && acc_sign != 0) {
            line_buf[line_idx++] = acc;
        }
        if(write_ok && line_idx > 0) {
            write_ok = flipper_format_write_int32(ff, "RAW_Data", line_buf, line_idx);
        }

        free(line_buf);
        ok = write_ok;
    } while(false);

    flipper_format_buffered_file_close(ff);
    flipper_format_free(ff);
    furi_string_free(path);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

static bool ac2101_parse_sub_stats_and_load(
    Ac2101CmdId cmd,
    Ac2101SlotMeta* out_meta,
    int16_t** out_buf,
    uint32_t* out_len) {
    memset(out_meta, 0, sizeof(Ac2101SlotMeta));
    if(out_buf) *out_buf = NULL;
    if(out_len) *out_len = 0;

    Storage* storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* ff = flipper_format_buffered_file_alloc(storage);
    FuriString* path = furi_string_alloc();
    FuriString* line = furi_string_alloc();
    ac2101_slot_path(cmd, path);

    bool loaded = false;
    uint32_t chunks_needed = 0;
    uint32_t logical_pulses = 0;
    uint64_t total_us = 0;
    uint16_t min_us = 0xFFFF;

    if(flipper_format_buffered_file_open_existing(ff, furi_string_get_cstr(path))) {
        Stream* stream = flipper_format_get_raw_stream(ff);
        /* Pass 1: compute total stats and exact split int16 array capacity */
        while(stream_read_line(stream, line)) {
            const char* str = furi_string_get_cstr(line);
            if(strncmp(str, "RAW_Data:", 9) != 0) continue;
            char* p = (char*)(str + 9);
            int32_t val = 0;
            while(strint_to_int32(p, &p, &val, 10) == StrintParseNoError) {
                if(val != 0) {
                    uint32_t mag = (uint32_t)(val > 0 ? val : -val);
                    if(mag > 1000000U) mag = 1000000U;
                    logical_pulses++;
                    total_us += mag;
                    if(mag < min_us) min_us = (uint16_t)mag;
                    chunks_needed += (mag + AC2101_SPLIT_CHUNK_US - 1U) / AC2101_SPLIT_CHUNK_US;
                }
                if(*p == ',') p++;
            }
        }

        if(logical_pulses > 0) {
            out_meta->exists = true;
            out_meta->pulses = logical_pulses;
            out_meta->duration_ms = (uint32_t)(total_us / 1000U);
            out_meta->min_pulse_us = (min_us == 0xFFFF) ? 0 : min_us;
            loaded = true;

            if(out_buf && out_len && chunks_needed > 0) {
                int16_t* buf = malloc(chunks_needed * sizeof(int16_t));
                if(buf) {
                    stream_rewind(stream);
                    uint32_t idx = 0;
                    while(stream_read_line(stream, line) && idx < chunks_needed) {
                        const char* str = furi_string_get_cstr(line);
                        if(strncmp(str, "RAW_Data:", 9) != 0) continue;
                        char* p = (char*)(str + 9);
                        int32_t val = 0;
                        while(strint_to_int32(p, &p, &val, 10) == StrintParseNoError) {
                            if(val != 0) {
                                int32_t sign = (val > 0) ? 1 : -1;
                                uint32_t rem = (uint32_t)(val > 0 ? val : -val);
                                if(rem > 1000000U) rem = 1000000U;
                                while(rem > AC2101_SPLIT_CHUNK_US && idx < chunks_needed) {
                                    buf[idx++] = (int16_t)(sign * (int32_t)AC2101_SPLIT_CHUNK_US);
                                    rem -= AC2101_SPLIT_CHUNK_US;
                                }
                                if(rem > 0 && idx < chunks_needed) {
                                    buf[idx++] = (int16_t)(sign * (int32_t)rem);
                                }
                            }
                            if(*p == ',') p++;
                        }
                    }
                    *out_buf = buf;
                    *out_len = idx;
                } else {
                    loaded = false;
                }
            }
        }
        flipper_format_buffered_file_close(ff);
    }

    flipper_format_free(ff);
    furi_string_free(line);
    furi_string_free(path);
    furi_record_close(RECORD_STORAGE);
    return loaded;
}

static void ac2101_refresh_all_slots(Ac2101App* app) {
    for(uint8_t i = 0; i < Ac2101CmdCount; i++) {
        ac2101_parse_sub_stats_and_load((Ac2101CmdId)i, &app->slots[i], NULL, NULL);
    }
    with_view_model(
        app->remote_view,
        RemoteViewModel * m,
        {
            memcpy(m->slots, app->slots, sizeof(app->slots));
            m->freq_hz = app->freq_hz;
        },
        true);
}

/* ==================== RX Capture Engine ==================== */

static inline void ac2101_push_cap_sample(Ac2101App* app, bool level, uint32_t duration) {
    if(duration > 1000000U) duration = 1000000U;
    if(duration < 0xFFFFU && (uint16_t)duration < app->cap_min_us) {
        app->cap_min_us = (uint16_t)duration;
    }
    app->cap_total_us += duration;

    int32_t sign = level ? 1 : -1;
    uint32_t rem = duration;
    uint32_t max_p = app->cap_max_pulses;
    while(rem > AC2101_SPLIT_CHUNK_US && app->cap_count < max_p) {
        app->cap_buf[app->cap_count++] = (int16_t)(sign * (int32_t)AC2101_SPLIT_CHUNK_US);
        rem -= AC2101_SPLIT_CHUNK_US;
    }
    if(rem > 0 && app->cap_count < max_p) {
        app->cap_buf[app->cap_count++] = (int16_t)(sign * (int32_t)rem);
    }
}

static inline void ac2101_push_ring_sample(Ac2101App* app, bool level, uint32_t duration) {
    if(duration > AC2101_SPLIT_CHUNK_US) duration = AC2101_SPLIT_CHUNK_US;
    int16_t sample = (int16_t)(level ? (int32_t)duration : -(int32_t)duration);
    app->ring_buf[app->ring_head] = sample;
    app->ring_head = (app->ring_head + 1U) % AC2101_RING_LEN;
    if(app->ring_count < AC2101_RING_LEN) app->ring_count++;
}

static void ac2101_rx_isr(bool level, uint32_t duration, void* context) {
    Ac2101App* app = context;
    if(duration == 0) return;

    CapPhase phase = app->cap_phase;
    if(phase == CapPhaseArmed) {
        ac2101_push_ring_sample(app, level, duration);
    } else if(phase == CapPhaseRecording) {
        ac2101_push_cap_sample(app, level, duration);
    }
}

static void ac2101_rewind_pretrigger_locked(Ac2101App* app) {
    uint32_t count = app->ring_count;
    if(count == 0) return;

    uint32_t pre_us = 0;
    uint32_t take = 0;
    uint32_t idx = app->ring_head;

    while(take < count && pre_us < AC2101_PRETRIG_US) {
        idx = (idx == 0) ? (AC2101_RING_LEN - 1U) : (idx - 1U);
        int32_t v = app->ring_buf[idx];
        pre_us += (uint32_t)(v > 0 ? v : -v);
        take++;
    }

    uint32_t cur = idx;
    for(uint32_t i = 0; i < take && app->cap_count < app->cap_max_pulses; i++) {
        int32_t v = app->ring_buf[cur];
        bool lvl = (v > 0);
        uint32_t dur = (uint32_t)(v > 0 ? v : -v);
        ac2101_push_cap_sample(app, lvl, dur);
        cur = (cur + 1U) % AC2101_RING_LEN;
    }
}

static int32_t ac2101_capture_worker(void* context) {
    Ac2101App* app = context;
    Ac2101CmdId cmd = app->active_cap_cmd;
    uint32_t freq_hz = app->freq_hz;

    app->ring_buf = malloc(AC2101_RING_LEN * sizeof(int16_t));
    app->cap_max_pulses = AC2101_CAP_MAX_PULSES;
    app->cap_buf = malloc(app->cap_max_pulses * sizeof(int16_t));
    if(!app->cap_buf) {
        app->cap_max_pulses = AC2101_CAP_FALLBACK;
        app->cap_buf = malloc(app->cap_max_pulses * sizeof(int16_t));
    }
    if(!app->ring_buf || !app->cap_buf) {
        free(app->ring_buf);
        free(app->cap_buf);
        app->ring_buf = NULL;
        app->cap_buf = NULL;
        with_view_model(
            app->remote_view,
            RemoteViewModel * m,
            { snprintf(m->status, sizeof(m->status), "Low RAM Err"); },
            true);
        view_dispatcher_send_custom_event(app->view_dispatcher, Ac2101EventCapDone);
        return 0;
    }

    app->ring_head = 0;
    app->ring_count = 0;
    app->cap_count = 0;
    app->cap_total_us = 0;
    app->cap_min_us = 0xFFFF;
    app->cap_phase = CapPhaseArmed;
    app->cap_abort = false;
    app->cap_manual_trig = false;
    app->cap_manual_stop = false;

    subghz_devices_reset(app->radio_device);
    subghz_devices_idle(app->radio_device);
    subghz_devices_load_preset(
        app->radio_device, FuriHalSubGhzPresetCustom, (uint8_t*)AC2101_FSK_40K_PRESET);
    subghz_devices_set_frequency(app->radio_device, freq_hz);
    subghz_devices_flush_rx(app->radio_device);

    subghz_devices_start_async_rx(app->radio_device, ac2101_rx_isr, app);

    /*
     * Reduce TIM2 hardware input capture filter from Flipper's 16 us (FDIV32_N8)
     * down to 12 us (FDIV32_N6 @ 16 MHz f_DTS = 500 kHz * 6 = 12 us) so 25 us symbols
     * pass with >2x timing margin while sub-12 us noise glitches are rejected in HW.
     */
    FURI_CRITICAL_ENTER();
    LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N6);
    LL_TIM_ClearFlag_CC1(TIM2);
    LL_TIM_ClearFlag_CC2(TIM2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    FURI_CRITICAL_EXIT();

    uint32_t settle_until = furi_get_tick() + 45U;
    uint32_t last_ui_tick = 0;
    uint32_t trig_tick = 0;
    uint32_t last_high_tick = 0;
    uint32_t last_high_idx = 0;
    uint8_t rssi_streak = 0;
    bool triggered_manually = false;

    while(!app->cap_abort) {
        uint32_t now = furi_get_tick();
        float rssi_f = subghz_devices_get_rssi(app->radio_device);
        int32_t rssi_dbm = (int32_t)rssi_f;
        int32_t thresh = app->thresh_dbm;

        if(app->cap_phase == CapPhaseArmed) {
            bool rssi_hit = (now >= settle_until) && (rssi_dbm >= thresh);
            rssi_streak = rssi_hit ? (uint8_t)(rssi_streak + 1U) : 0U;

            if(app->cap_manual_trig || rssi_streak >= 2U) {
                triggered_manually = app->cap_manual_trig;
                app->cap_manual_trig = false;
                FURI_CRITICAL_ENTER();
                if(!triggered_manually) {
                    ac2101_rewind_pretrigger_locked(app);
                }
                app->cap_phase = CapPhaseRecording;
                last_high_idx = app->cap_count;
                FURI_CRITICAL_EXIT();

                trig_tick = now;
                last_high_tick = now;
                notification_message(app->notifications, &sequence_set_cyan_255);
            }
        } else if(app->cap_phase == CapPhaseRecording) {
            uint32_t cur_count = app->cap_count;
            if(rssi_dbm >= thresh) {
                last_high_tick = now;
                last_high_idx = cur_count;
            }

            bool silence_done = (!triggered_manually) &&
                                ((now - last_high_tick) >= AC2101_TRAIL_SILENCE_MS) &&
                                ((now - trig_tick) >= AC2101_MIN_AUTO_BURST_MS) &&
                                (cur_count > 16U);
            bool max_time = (now - trig_tick) >= AC2101_MAX_BURST_MS;
            bool buf_full = (cur_count >= app->cap_max_pulses - 4U);

            if(app->cap_manual_stop || silence_done || max_time || buf_full) {
                FURI_CRITICAL_ENTER();
                app->cap_phase = CapPhaseSaving;
                if(silence_done && last_high_idx > 16U) {
                    uint32_t keep = last_high_idx + 20U;
                    if(keep < app->cap_count) app->cap_count = keep;
                }
                FURI_CRITICAL_EXIT();
                break;
            }
        }

        if((now - last_ui_tick) >= 45U) {
            last_ui_tick = now;
            CapPhase p = app->cap_phase;
            uint32_t cnt = app->cap_count;
            uint32_t ms = app->cap_total_us / 1000U;
            uint16_t min_u = (app->cap_min_us == 0xFFFF) ? 0 : app->cap_min_us;
            with_view_model(
                app->capture_view,
                CaptureViewModel * cm,
                {
                    cm->cmd = cmd;
                    cm->freq_hz = freq_hz;
                    cm->phase = p;
                    cm->cur_rssi_dbm = rssi_dbm;
                    cm->thresh_dbm = thresh;
                    cm->pulses = cnt;
                    cm->ms = ms;
                    cm->min_us = min_u;
                },
                true);
        }

        furi_delay_ms(1);
    }

    subghz_devices_stop_async_rx(app->radio_device);
    subghz_devices_idle(app->radio_device);
    subghz_devices_sleep(app->radio_device);
    notification_message(app->notifications, &sequence_reset_blue);

    bool saved = false;
    uint32_t final_len = app->cap_count;
    if(!app->cap_abort && final_len > 8U) {
        with_view_model(
            app->capture_view,
            CaptureViewModel * cm,
            { cm->phase = CapPhaseSaving; },
            true);

        saved = ac2101_save_slot_sub(cmd, app->cap_buf, final_len, freq_hz);
        if(saved) {
            uint16_t min_u = 0;
            uint32_t merged_cnt = ac2101_count_sub_chunks(app->cap_buf, final_len, &min_u);
            uint32_t sum_us = 0;
            for(uint32_t i = 0; i < final_len; i++) {
                int32_t v = app->cap_buf[i];
                sum_us += (uint32_t)(v > 0 ? v : -v);
            }
            app->slots[cmd].exists = true;
            app->slots[cmd].pulses = merged_cnt;
            app->slots[cmd].duration_ms = sum_us / 1000U;
            app->slots[cmd].min_pulse_us = min_u;

            with_view_model(
                app->remote_view,
                RemoteViewModel * rm,
                {
                    memcpy(rm->slots, app->slots, sizeof(app->slots));
                    snprintf(
                        rm->status,
                        sizeof(rm->status),
                        "Saved %s",
                        AC2101_SLOT_DEFS[cmd].name);
                },
                true);
            dolphin_deed(DolphinDeedSubGhzRawRec);
        }
    }

    if(!app->cap_abort && !saved) {
        with_view_model(
            app->remote_view,
            RemoteViewModel * rm,
            { snprintf(rm->status, sizeof(rm->status), "Cap empty"); },
            true);
    }

    free(app->ring_buf);
    free(app->cap_buf);
    app->ring_buf = NULL;
    app->cap_buf = NULL;

    if(!app->cap_abort) {
        view_dispatcher_send_custom_event(app->view_dispatcher, Ac2101EventCapDone);
    }
    return 0;
}

/* ==================== TX Replay Engine ==================== */

static LevelDuration ac2101_tx_yield_cb(void* context) {
    Ac2101App* app = context;
    if(!app->tx_active || !app->tx_buf || app->tx_idx >= app->tx_len) {
        return level_duration_reset();
    }
    int32_t v = app->tx_buf[app->tx_idx++];
    if(v > 0) {
        return level_duration_make(true, (uint32_t)v);
    } else if(v < 0) {
        return level_duration_make(false, (uint32_t)(-v));
    }
    return level_duration_reset();
}

static void ac2101_stop_tx(Ac2101App* app, const char* status_msg) {
    if(app->tx_poll_timer) {
        furi_timer_stop(app->tx_poll_timer);
    }
    if(app->tx_active) {
        app->tx_active = false;
        subghz_devices_stop_async_tx(app->radio_device);
        subghz_devices_idle(app->radio_device);
        subghz_devices_sleep(app->radio_device);
        notification_message(app->notifications, &sequence_reset_blue);
    }
    if(app->tx_buf) {
        free(app->tx_buf);
        app->tx_buf = NULL;
    }
    app->tx_len = 0;
    app->tx_idx = 0;

    with_view_model(
        app->remote_view,
        RemoteViewModel * m,
        {
            m->tx_active = false;
            if(status_msg) snprintf(m->status, sizeof(m->status), "%s", status_msg);
        },
        true);
}

static void ac2101_tx_poll_timer_cb(void* context) {
    Ac2101App* app = context;
    if(!app->tx_active) return;

    if(subghz_devices_is_async_complete_tx(app->radio_device)) {
        ac2101_stop_tx(app, "Sent OK");
    }
}

static bool ac2101_start_tx(Ac2101App* app, Ac2101CmdId cmd) {
    if(cmd >= Ac2101CmdCount || app->tx_active) return false;

    Ac2101SlotMeta meta;
    int16_t* buf = NULL;
    uint32_t len = 0;
    if(!ac2101_parse_sub_stats_and_load(cmd, &meta, &buf, &len) || !buf || len == 0) {
        free(buf);
        with_view_model(
            app->remote_view,
            RemoteViewModel * m,
            { snprintf(m->status, sizeof(m->status), "Load Err"); },
            true);
        return false;
    }

    subghz_devices_reset(app->radio_device);
    subghz_devices_idle(app->radio_device);
    subghz_devices_load_preset(
        app->radio_device, FuriHalSubGhzPresetCustom, (uint8_t*)AC2101_FSK_40K_PRESET);
    subghz_devices_set_frequency(app->radio_device, app->freq_hz);

    if(!subghz_devices_set_tx(app->radio_device)) {
        subghz_devices_idle(app->radio_device);
        subghz_devices_sleep(app->radio_device);
        free(buf);
        with_view_model(
            app->remote_view,
            RemoteViewModel * m,
            { snprintf(m->status, sizeof(m->status), "TX Blocked"); },
            true);
        return false;
    }

    app->tx_buf = buf;
    app->tx_len = len;
    app->tx_idx = 0;
    app->tx_active = true;

    if(!subghz_devices_start_async_tx(app->radio_device, ac2101_tx_yield_cb, app)) {
        app->tx_active = false;
        subghz_devices_idle(app->radio_device);
        subghz_devices_sleep(app->radio_device);
        free(app->tx_buf);
        app->tx_buf = NULL;
        app->tx_len = 0;
        with_view_model(
            app->remote_view,
            RemoteViewModel * m,
            { snprintf(m->status, sizeof(m->status), "TX Failed"); },
            true);
        return false;
    }

    notification_message(app->notifications, &sequence_set_only_blue_255);
    dolphin_deed(DolphinDeedSubGhzSend);
    furi_timer_start(app->tx_poll_timer, 20);

    with_view_model(
        app->remote_view,
        RemoteViewModel * m,
        {
            m->tx_active = true;
            snprintf(m->status, sizeof(m->status), "TX %s...", AC2101_SLOT_DEFS[cmd].name);
        },
        true);
    return true;
}

/* ==================== UI Drawing & Input ==================== */

static void ac2101_draw_slot_card(
    Canvas* canvas,
    const Ac2101SlotDef* def,
    const Ac2101SlotMeta* meta,
    bool selected,
    bool sending,
    uint8_t y) {
    const uint8_t x = 1;
    const uint8_t w = 62;
    const uint8_t h = 24;

    if(sending) {
        elements_slightly_rounded_box(canvas, x, y, w, h);
        canvas_set_color(canvas, ColorWhite);
    } else {
        elements_slightly_rounded_frame(canvas, x, y, w, h);
        if(selected) {
            canvas_draw_rframe(canvas, x + 1, y + 1, w - 2, h - 2, 2);
        }
    }

    /* Glyph */
    if(def->has_triangle) {
        canvas_draw_triangle(canvas, x + 8, y + 8, 7, 4, def->tri_dir);
    } else {
        canvas_draw_box(canvas, x + 5, y + 5, 6, 6);
    }

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, x + 16, y + 12, def->name);

    char sub_line[24];
    if(sending) {
        snprintf(sub_line, sizeof(sub_line), "SENDING...");
    } else if(meta->exists) {
        snprintf(
            sub_line,
            sizeof(sub_line),
            "%lums %uu",
            (unsigned long)meta->duration_ms,
            (unsigned)meta->min_pulse_us);
    } else {
        snprintf(sub_line, sizeof(sub_line), "-- empty --");
    }
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, x + 5, y + 21, sub_line);

    canvas_set_color(canvas, ColorBlack);
}

static void ac2101_remote_draw_cb(Canvas* canvas, void* context) {
    RemoteViewModel* m = context;
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    /* Two-line header (fits 64px cleanly without overlap) */
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 32, 1, AlignCenter, AlignTop, "AC2101-01A");
    canvas_set_font(canvas, FontSecondary);
    const char* freq_lbl =
        (m->freq_hz == AC2101_FREQ_PRIMARY) ? "433.914 MHz" : "433.920 MHz";
    canvas_draw_str_aligned(canvas, 32, 11, AlignCenter, AlignTop, freq_lbl);
    canvas_draw_line(canvas, 0, 20, 63, 20);

    /* Three command cards */
    for(uint8_t i = 0; i < Ac2101CmdCount; i++) {
        ac2101_draw_slot_card(
            canvas,
            &AC2101_SLOT_DEFS[i],
            &m->slots[i],
            m->selected == i,
            m->tx_active && (m->selected == i),
            (uint8_t)(22 + i * 26));
    }

    /* Footer */
    canvas_draw_line(canvas, 0, 100, 63, 100);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 32, 110, AlignCenter, AlignBottom, m->status);
    canvas_draw_str(canvas, 1, 119, "OK:Tx  R:Rec");
    canvas_draw_str(canvas, 1, 127, "L:Freq");
    canvas_draw_icon(canvas, 39, 120, &I_Pin_back_arrow_10x8);
    canvas_draw_str(canvas, 50, 127, "Ex");
}

static bool ac2101_remote_input_cb(InputEvent* event, void* context) {
    Ac2101App* app = context;
    if(event->key == InputKeyBack) {
        return false;
    }
    if(event->type != InputTypeShort && event->type != InputTypePress) {
        return true;
    }

    if(event->key == InputKeyUp && event->type == InputTypePress) {
        if(!app->tx_active) {
            with_view_model(
                app->remote_view,
                RemoteViewModel * m,
                { m->selected = (m->selected == 0) ? (Ac2101CmdCount - 1U) : (m->selected - 1U); },
                true);
        }
    } else if(event->key == InputKeyDown && event->type == InputTypePress) {
        if(!app->tx_active) {
            with_view_model(
                app->remote_view,
                RemoteViewModel * m,
                { m->selected = (m->selected + 1U) % Ac2101CmdCount; },
                true);
        }
    } else if(event->key == InputKeyLeft && event->type == InputTypeShort) {
        if(!app->tx_active) {
            app->freq_hz = (app->freq_hz == AC2101_FREQ_PRIMARY) ? AC2101_FREQ_ALT :
                                                                   AC2101_FREQ_PRIMARY;
            with_view_model(
                app->remote_view,
                RemoteViewModel * m,
                {
                    m->freq_hz = app->freq_hz;
                    snprintf(
                        m->status,
                        sizeof(m->status),
                        "%s MHz",
                        (app->freq_hz == AC2101_FREQ_PRIMARY) ? "433.914" : "433.920");
                },
                true);
        }
    } else if(event->key == InputKeyRight && event->type == InputTypeShort) {
        if(!app->tx_active) {
            uint8_t sel = 0;
            with_view_model(
                app->remote_view,
                RemoteViewModel * m,
                { sel = m->selected; },
                false);
            app->active_cap_cmd = (Ac2101CmdId)sel;
            view_dispatcher_switch_to_view(app->view_dispatcher, Ac2101ViewCapture);
        }
    } else if(event->key == InputKeyOk && event->type == InputTypeShort) {
        if(app->tx_active) return true;
        uint8_t sel = 0;
        bool has_sig = false;
        with_view_model(
            app->remote_view,
            RemoteViewModel * m,
            {
                sel = m->selected;
                has_sig = m->slots[sel].exists;
            },
            false);
        if(has_sig) {
            ac2101_start_tx(app, (Ac2101CmdId)sel);
        } else {
            app->active_cap_cmd = (Ac2101CmdId)sel;
            view_dispatcher_switch_to_view(app->view_dispatcher, Ac2101ViewCapture);
        }
    }
    return true;
}

static void ac2101_capture_draw_cb(Canvas* canvas, void* context) {
    CaptureViewModel* m = context;
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    char buf[32];
    canvas_set_font(canvas, FontPrimary);
    snprintf(buf, sizeof(buf), "REC: %s", AC2101_SLOT_DEFS[m->cmd].name);
    canvas_draw_str(canvas, 1, 10, buf);

    canvas_set_font(canvas, FontSecondary);
    snprintf(
        buf,
        sizeof(buf),
        "%s  40k",
        (m->freq_hz == AC2101_FREQ_PRIMARY) ? "433.914" : "433.920");
    canvas_draw_str(canvas, 1, 20, buf);
    canvas_draw_line(canvas, 0, 22, 63, 22);

    /* RSSI & threshold bar (-110 .. -30 dBm mapped to 0..60 px) */
    snprintf(buf, sizeof(buf), "RSSI:%lddBm", (long)m->cur_rssi_dbm);
    canvas_draw_str(canvas, 1, 33, buf);

    int32_t clamped = m->cur_rssi_dbm;
    if(clamped < -110) clamped = -110;
    if(clamped > -30) clamped = -30;
    uint8_t bar_w = (uint8_t)(((clamped + 110) * 60) / 80);
    canvas_draw_frame(canvas, 1, 36, 62, 8);
    if(bar_w > 0) canvas_draw_box(canvas, 2, 37, bar_w, 6);

    int32_t t_clamp = m->thresh_dbm;
    if(t_clamp < -110) t_clamp = -110;
    if(t_clamp > -30) t_clamp = -30;
    uint8_t trig_x = (uint8_t)(2 + (((t_clamp + 110) * 60) / 80));
    canvas_draw_line(canvas, trig_x, 34, trig_x, 45);

    snprintf(buf, sizeof(buf), "Trig:%lddBm", (long)m->thresh_dbm);
    canvas_draw_str(canvas, 1, 55, buf);
    canvas_draw_line(canvas, 0, 58, 63, 58);

    if(m->phase == CapPhaseArmed) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 32, 70, AlignCenter, AlignBottom, "ARMED");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 32, 82, AlignCenter, AlignBottom, "Hold remote");
        canvas_draw_str_aligned(canvas, 32, 91, AlignCenter, AlignBottom, "6-12in + hold");
        canvas_draw_str_aligned(canvas, 32, 100, AlignCenter, AlignBottom, "button 1 sec");
        canvas_draw_str(canvas, 1, 117, "Up/Dn:Trig");
        canvas_draw_str(canvas, 1, 126, "OK:Man B:Esc");
    } else if(m->phase == CapPhaseRecording) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 32, 71, AlignCenter, AlignBottom, "CAPTURING");
        canvas_set_font(canvas, FontSecondary);
        snprintf(buf, sizeof(buf), "Edges:%lu", (unsigned long)m->pulses);
        canvas_draw_str(canvas, 1, 84, buf);
        snprintf(
            buf, sizeof(buf), "Time:%lums", (unsigned long)m->ms);
        canvas_draw_str(canvas, 1, 94, buf);
        snprintf(buf, sizeof(buf), "Min:%uus", (unsigned)m->min_us);
        canvas_draw_str(canvas, 1, 104, buf);
        canvas_draw_str(canvas, 1, 125, "OK:Stop B:Esc");
    } else {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 32, 82, AlignCenter, AlignBottom, "SAVING...");
    }
}

static bool ac2101_capture_input_cb(InputEvent* event, void* context) {
    Ac2101App* app = context;
    if(event->key == InputKeyBack) {
        return false;
    }
    if(event->type != InputTypeShort && event->type != InputTypePress) {
        return true;
    }

    if(app->cap_phase == CapPhaseArmed) {
        if(event->key == InputKeyUp && event->type == InputTypePress) {
            if(app->thresh_dbm < AC2101_MAX_THRESH) app->thresh_dbm += 5;
            with_view_model(
                app->capture_view,
                CaptureViewModel * cm,
                { cm->thresh_dbm = app->thresh_dbm; },
                true);
        } else if(event->key == InputKeyDown && event->type == InputTypePress) {
            if(app->thresh_dbm > AC2101_MIN_THRESH) app->thresh_dbm -= 5;
            with_view_model(
                app->capture_view,
                CaptureViewModel * cm,
                { cm->thresh_dbm = app->thresh_dbm; },
                true);
        } else if(event->key == InputKeyOk && event->type == InputTypeShort) {
            app->cap_manual_trig = true;
        }
    } else if(app->cap_phase == CapPhaseRecording) {
        if(event->key == InputKeyOk && event->type == InputTypeShort) {
            app->cap_manual_stop = true;
        }
    }
    return true;
}

static void ac2101_capture_enter_cb(void* context) {
    Ac2101App* app = context;
    if(app->cap_thread) return;

    with_view_model(
        app->capture_view,
        CaptureViewModel * cm,
        {
            cm->cmd = app->active_cap_cmd;
            cm->freq_hz = app->freq_hz;
            cm->phase = CapPhaseArmed;
            cm->cur_rssi_dbm = -100;
            cm->thresh_dbm = app->thresh_dbm;
            cm->pulses = 0;
            cm->ms = 0;
            cm->min_us = 0;
        },
        true);

    app->cap_thread = furi_thread_alloc_ex("Ac2101Cap", 4096, ac2101_capture_worker, app);
    furi_thread_start(app->cap_thread);
}

static void ac2101_capture_exit_cb(void* context) {
    Ac2101App* app = context;
    if(app->cap_thread) {
        app->cap_abort = true;
        furi_thread_join(app->cap_thread);
        furi_thread_free(app->cap_thread);
        app->cap_thread = NULL;
    }
}

static uint32_t ac2101_remote_prev_cb(void* context) {
    UNUSED(context);
    return VIEW_NONE;
}

static uint32_t ac2101_capture_prev_cb(void* context) {
    Ac2101App* app = context;
    app->cap_abort = true;
    return Ac2101ViewRemote;
}

static bool ac2101_custom_event_cb(void* context, uint32_t event) {
    Ac2101App* app = context;
    if(event == Ac2101EventCapDone) {
        view_dispatcher_switch_to_view(app->view_dispatcher, Ac2101ViewRemote);
        return true;
    }
    return false;
}

static Ac2101App* ac2101_app_alloc(void) {
    Ac2101App* app = malloc(sizeof(Ac2101App));
    memset(app, 0, sizeof(Ac2101App));
    app->freq_hz = AC2101_FREQ_PRIMARY;
    app->thresh_dbm = AC2101_DEFAULT_THRESH;

    app->gui = furi_record_open(RECORD_GUI);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    subghz_devices_init();
    app->radio_device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
    if(app->radio_device) {
        subghz_devices_begin(app->radio_device);
        subghz_devices_sleep(app->radio_device);
    }

    app->tx_poll_timer = furi_timer_alloc(ac2101_tx_poll_timer_cb, FuriTimerTypePeriodic, app);

    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_custom_event_callback(app->view_dispatcher, ac2101_custom_event_cb);
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);

    app->remote_view = view_alloc();
    view_set_context(app->remote_view, app);
    view_set_orientation(app->remote_view, ViewOrientationVertical);
    view_allocate_model(app->remote_view, ViewModelTypeLocking, sizeof(RemoteViewModel));
    with_view_model(
        app->remote_view,
        RemoteViewModel * m,
        {
            memset(m, 0, sizeof(RemoteViewModel));
            m->freq_hz = app->freq_hz;
            snprintf(m->status, sizeof(m->status), "Ready");
        },
        true);
    view_set_draw_callback(app->remote_view, ac2101_remote_draw_cb);
    view_set_input_callback(app->remote_view, ac2101_remote_input_cb);
    view_set_previous_callback(app->remote_view, ac2101_remote_prev_cb);
    view_dispatcher_add_view(app->view_dispatcher, Ac2101ViewRemote, app->remote_view);

    app->capture_view = view_alloc();
    view_set_context(app->capture_view, app);
    view_set_orientation(app->capture_view, ViewOrientationVertical);
    view_allocate_model(app->capture_view, ViewModelTypeLocking, sizeof(CaptureViewModel));
    view_set_draw_callback(app->capture_view, ac2101_capture_draw_cb);
    view_set_input_callback(app->capture_view, ac2101_capture_input_cb);
    view_set_enter_callback(app->capture_view, ac2101_capture_enter_cb);
    view_set_exit_callback(app->capture_view, ac2101_capture_exit_cb);
    view_set_previous_callback(app->capture_view, ac2101_capture_prev_cb);
    view_dispatcher_add_view(app->view_dispatcher, Ac2101ViewCapture, app->capture_view);

    ac2101_refresh_all_slots(app);
    view_dispatcher_switch_to_view(app->view_dispatcher, Ac2101ViewRemote);
    return app;
}

static void ac2101_app_free(Ac2101App* app) {
    ac2101_capture_exit_cb(app);
    ac2101_stop_tx(app, NULL);
    furi_timer_free(app->tx_poll_timer);

    view_dispatcher_remove_view(app->view_dispatcher, Ac2101ViewCapture);
    view_dispatcher_remove_view(app->view_dispatcher, Ac2101ViewRemote);
    view_free(app->capture_view);
    view_free(app->remote_view);
    view_dispatcher_free(app->view_dispatcher);

    if(app->radio_device) {
        subghz_devices_end(app->radio_device);
    }
    subghz_devices_deinit();

    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    free(app);
}

int32_t ac2101_curtain_remote_app(void* p) {
    UNUSED(p);
    Ac2101App* app = ac2101_app_alloc();
    view_dispatcher_run(app->view_dispatcher);
    ac2101_app_free(app);
    return 0;
}
