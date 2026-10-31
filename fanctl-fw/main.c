/*
 * fanctl firmware v2.2 - XIAO RP2040, two blowers, USB CDC control
 *
 * Driver stage (pick at build time, cmake -DFANCTL_DRIVER=...):
 *   DRV8871  (default) One DRV8871 per blower. IN1 <- EN (GP6, shared),
 *            IN2 <- PWM inverted: forward / brake (slow decay). OUT2 stays
 *            low, so the blower's ground stays at ground.
 *   LOWSIDE  MOSFET trigger module. Active-high PWM. Needs a flyback
 *            Schottky across each blower.
 *
 * Pins:
 *   GP0 (D6)  PWM1        slice 0 A
 *   GP1 (D7)  PWM2        slice 0 B
 *   GP6 (D4)  EN          DRV8871 IN1, both channels (unused for LOWSIDE)
 *   GP7 (D5)  TACH1       slice 3 B, falling-edge counter
 *   GP3 (D10) TACH2       slice 1 B, falling-edge counter
 *
 * USB CDC protocol, one command per line, case-insensitive:
 *   SET <d1> [<d2>]    duty % per blower (one value = both)       -> STAT
 *   GET                                                           -> STAT
 *   MIN <m1> [<m2>]    minimum running duty %, saved to flash     -> CFG
 *   TACH <t1> [<t2>]   1 = blower has tach (enables stall
 *                      recovery), 0 = none; saved to flash        -> CFG
 *   CFG                                                           -> CFG
 *   PROBE <ms> [<f>]   debug: blower f (default 1) at 100 % for ms,
 *                      tach falling-edge times in ms              -> PROBE
 *   ID                                                            -> ID FANCTL 2.2 <driver>
 *
 *   STAT mode=HOST d1=70 rpm1=2040 st1=OK d2=30 rpm2=1280 st2=OK
 *   CFG min1=30 min2=30 tach1=1 tach2=1
 *   PROBE fan=1 ms=50 n=3 t=2.62,26.46,50.01
 *
 *   mode: HOST | LOST (no SET yet, or none for HOST_TIMEOUT_MS; both blowers 100 %)
 *   d:    output level actually applied (see speed levels), not the request
 *   st:   OFF   duty 0
 *         OK    running
 *         KICK  start-up or recovery kick at 100 %
 *         STALL no tach after STALL_RETRIES recovery kicks; held at
 *               100 %, retried every STALL_RETRY_MS, clears when tach returns
 *
 * Speed levels: with its supply chopped, the BFB1012HH only runs at a few
 * distinct speeds (~1280 rpm anywhere from 20-60 %, ~2000-2450 at 70-90 %,
 * ~4200 at 100 %), and at 80-90 % its speed depends on whether it came from
 * above or below. Requests are therefore mapped onto three levels, with
 * hysteresis so a request near a threshold doesn't flap:
 *   LOW  30 %   request 1-49   (back down from MID below 45)
 *   MID  70 %   request 50-89  (back down from HIGH below 85)
 *   HIGH 100 %  request 90-100
 *
 * Tach: the blower's tach output is only valid while its supply is steady.
 * At 0 % and 100 % the falling-edge counter is used continuously. At LOW and
 * MID the counter overreads, so the blower is briefly bumped to 100 % (at
 * most PROBE_MAX_MS, ending after two tach edges) PROBE_SETTLE_MS after the
 * level is reached, again PROBE_FOLLOWUP_MS later (slowing down to a lower
 * level takes several seconds), and then every PROBE_INTERVAL_MS; rpm is the
 * time between the two edges, and STAT reports the last such sample.
 *
 * Stall recovery (only for channels with tach enabled): a running blower
 * that shows 0 rpm for STALL_DETECT_MS on the counter, or fails
 * PROBE_FAILS samples in a row (retried every PROBE_RETRY_MS), gets
 * RECOVER_KICK_MS at 100 % and then returns to its setting. The level is
 * honoured in the reported duty throughout; st= shows what the output is
 * actually doing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define DRV_LOWSIDE 1
#define DRV_DRV8871 2
#ifndef FANCTL_DRIVER
#define FANCTL_DRIVER DRV_DRV8871
#endif

/* ---------------- configuration ---------------- */
#define PIN_PWM1           0
#define PIN_PWM2           1
#define PIN_EN             6
#define PIN_TACH1          7
#define PIN_TACH2          3

#if FANCTL_DRIVER == DRV_DRV8871
#define PWM_FREQ_HZ        25000
#define DRIVER_NAME        "DRV8871"
#else
#define PWM_FREQ_HZ        20000   /* MOSFET modules are rated to 20 kHz */
#define DRIVER_NAME        "LOWSIDE"
#endif

#define TACH_PULSES_PER_REV 2      /* BFB1012HH: 4-pole, 2 pulses/rev */
#define TACH_WINDOW_MS     500     /* rpm resolution: 60 rpm */

#define HOST_TIMEOUT_MS    10000
#define LOST_DUTY          100     /* duty with no host */

#define LEVEL_LOW          30
#define LEVEL_MID          70
#define LEVEL_HIGH         100
#define MID_UP             50      /* request >= this: LOW -> MID */
#define MID_DOWN           45      /* request <  this: MID -> LOW */
#define HIGH_UP            90      /* request >= this: -> HIGH */
#define HIGH_DOWN          85      /* request <  this: HIGH -> MID */

#define DEFAULT_MIN_DUTY   30      /* floor under LOW; MIN raises it */
#define START_KICK_MS      1000    /* 100 % when starting from stop */
#define RECOVER_KICK_MS    1500    /* 100 % after a detected stall */
#define STALL_DETECT_MS    2000    /* 0 rpm this long while running = stall */
#define STALL_RETRIES      3       /* recovery kicks before latching STALL */
#define STALL_RETRY_MS     30000   /* latched: try normal duty again */

#define PROBE_SETTLE_MS    3000    /* first sample after reaching a level */
#define PROBE_FOLLOWUP_MS  7000    /* second sample, once the speed has settled */
#define PROBE_STAGGER_MS   500     /* fan 2 samples this much after fan 1 */
#define PROBE_INTERVAL_MS  30000
#define PROBE_MAX_MS       70      /* two edges at 1000 rpm need <= 60 ms */
#define PROBE_RETRY_MS     2000    /* after a failed sample */
#define PROBE_FAILS        3       /* failed samples in a row = stall */
#define PROBE_MIN_RPM      1000    /* slower than this counts as failed */
#define PROBE_BLANK_US     300     /* ignore tach edges closer than this */

/* ---------------- persistent settings (last flash sector) ---------------- */
#define CFG_MAGIC   0x46414E31u               /* "FAN1" */
#define CFG_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint8_t  min_duty[2];
    uint8_t  tach[2];
    uint32_t check;
} cfg_t;

static cfg_t cfg;

static uint32_t cfg_check(const cfg_t *c) {
    return c->magic ^ ((uint32_t)c->min_duty[0] << 0) ^ ((uint32_t)c->min_duty[1] << 8) ^
           ((uint32_t)c->tach[0] << 16) ^ ((uint32_t)c->tach[1] << 24) ^ 0xA5A55A5Au;
}

static void cfg_load(void) {
    const cfg_t *f = (const cfg_t *)(XIP_BASE + CFG_OFFSET);
    if (f->magic == CFG_MAGIC && f->check == cfg_check(f) &&
        f->min_duty[0] <= 100 && f->min_duty[1] <= 100 &&
        f->tach[0] <= 1 && f->tach[1] <= 1) {
        cfg = *f;
        return;
    }
    cfg.magic = CFG_MAGIC;
    cfg.min_duty[0] = cfg.min_duty[1] = DEFAULT_MIN_DUTY;
    cfg.tach[0] = cfg.tach[1] = 1;
    cfg.check = cfg_check(&cfg);
}

static void cfg_save(void) {
    const cfg_t *f = (const cfg_t *)(XIP_BASE + CFG_OFFSET);
    cfg.check = cfg_check(&cfg);
    if (memcmp(f, &cfg, sizeof cfg) == 0) return;      /* unchanged: no wear */

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, &cfg, sizeof cfg);
    watchdog_update();
    uint32_t irq = save_and_disable_interrupts();      /* no XIP during erase */
    flash_range_erase(CFG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CFG_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
}

/* ---------------- state ---------------- */
typedef enum { MODE_HOST, MODE_LOST } run_mode_t;
typedef enum { ST_OFF, ST_OK, ST_KICK, ST_STALL } fan_state_t;

typedef struct {
    uint        pin_pwm, pin_tach, tach_slice;
    int         duty_set;        /* host request, % */
    int         level;           /* LEVEL_* picked from the request, 0 = off */
    int         duty_out;        /* reported duty (level after MIN), % */
    fan_state_t st;
    bool        chopped;         /* output between 0 and 100 %: counter invalid */
    uint32_t    kick_until;      /* end of current kick */
    uint32_t    watch_from;      /* ignore the tach counter until this time */
    uint32_t    zero_since;      /* first 0-rpm reading while running, 0 = none */
    uint32_t    stall_retry_at;  /* latched STALL: next normal-duty attempt */
    uint32_t    probe_at;        /* next tach sample while chopped */
    int         probe_fails;
    bool        probe_settled;   /* follow-up sample done since the last change */
    int         retries;
    uint32_t    rpm;
} fan_t;

static fan_t fans[2] = {
    { .pin_pwm = PIN_PWM1, .pin_tach = PIN_TACH1 },
    { .pin_pwm = PIN_PWM2, .pin_tach = PIN_TACH2 },
};
static uint       pwm_slice;
static uint32_t   pwm_top;
static run_mode_t mode = MODE_LOST;
static uint32_t   last_host_ms;

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }
static inline bool after(uint32_t t, uint32_t ref) { return (int32_t)(t - ref) >= 0; }

/* ---------------- hardware ---------------- */
static void set_level(fan_t *f, int pct) {
    /* DRV8871 build inverts the output in the PWM block, so "pct" is always
     * the fraction of time the blower is driven. */
    pwm_set_gpio_level(f->pin_pwm, (uint16_t)((pwm_top + 1) * (uint32_t)pct / 100));
}

static void pwm_out_init(void) {
    pwm_slice = pwm_gpio_to_slice_num(PIN_PWM1);       /* GP0/GP1 share slice 0 */
    pwm_top = clock_get_hz(clk_sys) / PWM_FREQ_HZ - 1;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_wrap(&c, pwm_top);
#if FANCTL_DRIVER == DRV_DRV8871
    pwm_config_set_output_polarity(&c, true, true);    /* IN2 low = drive */
#endif
    pwm_init(pwm_slice, &c, false);
    for (int i = 0; i < 2; i++) {
        set_level(&fans[i], 0);
        gpio_set_function(fans[i].pin_pwm, GPIO_FUNC_PWM);
    }
    pwm_set_enabled(pwm_slice, true);
#if FANCTL_DRIVER == DRV_DRV8871
    sleep_ms(1);
    gpio_put(PIN_EN, 1);    /* IN1 high: IN2 now selects drive / brake */
#endif
}

static void tach_init(fan_t *f) {
    gpio_set_function(f->pin_tach, GPIO_FUNC_PWM);
    gpio_pull_up(f->pin_tach);
    f->tach_slice = pwm_gpio_to_slice_num(f->pin_tach);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&c, PWM_DIV_B_FALLING);
    pwm_config_set_clkdiv(&c, 1.0f);
    pwm_init(f->tach_slice, &c, true);
}

/* Drive the blower at 100 % for up to ms, timestamping tach falling edges
 * (us from the start), stopping after max_edges. Blocks; leaves the output
 * at 100 % - the caller restores it. */
static int capture_edges(fan_t *f, int ms, uint32_t *edge_us, int max_edges) {
    int n = 0;
    watchdog_update();
    set_level(f, 100);
    uint32_t t0 = time_us_32(), last_fall = t0 - 1000000;
    bool prev = gpio_get(f->pin_tach);
    while (n < max_edges && time_us_32() - t0 < (uint32_t)ms * 1000) {
        bool lvl = gpio_get(f->pin_tach);
        uint32_t t = time_us_32();
        if (prev && !lvl && t - last_fall >= PROBE_BLANK_US) {
            edge_us[n++] = t - t0;
            last_fall = t;
        }
        prev = lvl;
    }
    return n;
}

/* one tach sample while chopped: rpm from two edges, 0 if too slow / none */
static uint32_t sample_rpm(fan_t *f) {
    uint32_t e[2];
    if (capture_edges(f, PROBE_MAX_MS, e, 2) < 2) return 0;
    uint32_t rpm = 60000000u / ((e[1] - e[0]) * TACH_PULSES_PER_REV);
    return rpm < PROBE_MIN_RPM ? 0 : rpm;
}

/* ---------------- control ---------------- */
static int pick_level(int level, int req) {
    if (req <= 0) return 0;
    if (req >= HIGH_UP) return LEVEL_HIGH;
    switch (level) {
    case LEVEL_HIGH: return req < MID_DOWN ? LEVEL_LOW : req < HIGH_DOWN ? LEVEL_MID : LEVEL_HIGH;
    case LEVEL_MID:  return req < MID_DOWN ? LEVEL_LOW : LEVEL_MID;
    default:         return req >= MID_UP ? LEVEL_MID : LEVEL_LOW;
    }
}

static int target_duty(int idx) {
    fan_t *f = &fans[idx];
    f->level = pick_level(f->level, mode == MODE_HOST ? f->duty_set : LOST_DUTY);
    int d = f->level;
    if (d > 0 && d < cfg.min_duty[idx]) d = cfg.min_duty[idx];
    return d > 100 ? 100 : d;
}

static void kick(fan_t *f, uint32_t t, uint32_t ms) {
    f->st = ST_KICK;
    f->chopped = false;
    f->kick_until = t + ms;
    f->zero_since = 0;
    set_level(f, 100);
}

/* back to normal running: give the tach time before trusting it again */
static void resume(int idx, uint32_t t) {
    fan_t *f = &fans[idx];
    f->st = ST_OK;
    f->watch_from = t + TACH_WINDOW_MS + 100;
    f->probe_at = t + PROBE_SETTLE_MS + idx * PROBE_STAGGER_MS;
    f->probe_fails = 0;
    f->probe_settled = false;
    f->zero_since = 0;
}

static void stalled(fan_t *f, uint32_t t) {
    if (f->retries < STALL_RETRIES) {
        f->retries++;
        kick(f, t, RECOVER_KICK_MS);
    } else {
        f->st = ST_STALL;                          /* give up: hold 100 % */
        f->chopped = false;
        f->watch_from = t + TACH_WINDOW_MS + 100;  /* skip a window with chopped junk */
        f->stall_retry_at = t + STALL_RETRY_MS;
        set_level(f, 100);
    }
}

static void update_fan(int idx) {
    fan_t *f = &fans[idx];
    int target = target_duty(idx);
    uint32_t t = now_ms();
    bool tach = cfg.tach[idx];

    if (target == 0) {
        f->st = ST_OFF;
        f->chopped = false;
        f->duty_out = 0;
        f->retries = 0;
        f->zero_since = 0;
        set_level(f, 0);
        return;
    }
    if (f->st == ST_OK && target != f->duty_out) resume(idx, t);   /* new level */
    f->duty_out = target;

    switch (f->st) {
    case ST_OFF:                                   /* starting from stop */
        f->retries = 0;
        kick(f, t, START_KICK_MS);
        return;

    case ST_KICK:
        if (!after(t, f->kick_until)) return;      /* still kicking */
        resume(idx, t);
        break;

    case ST_STALL:
        set_level(f, 100);
        if (after(t, f->watch_from) && f->rpm > 0) {   /* recovered */
            f->retries = 0;
            resume(idx, t);
        } else if (after(t, f->stall_retry_at)) {
            f->retries = 0;                        /* fresh round of kicks */
            kick(f, t, RECOVER_KICK_MS);
        }
        return;

    case ST_OK:
        break;
    }

    /* ST_OK: run at target, watch for a stall */
    set_level(f, target);
    f->chopped = target < 100;
    if (!tach) { f->zero_since = 0; return; }

    if (f->chopped) {                              /* sampled tach */
        if (!after(t, f->probe_at)) return;
        f->rpm = sample_rpm(f);
        set_level(f, target);
        t = now_ms();
        if (f->rpm > 0) {
            f->probe_fails = 0;
            f->retries = 0;
            f->probe_at = t + (f->probe_settled ? PROBE_INTERVAL_MS : PROBE_FOLLOWUP_MS);
            f->probe_settled = true;
        } else if (++f->probe_fails < PROBE_FAILS) {
            f->probe_at = t + PROBE_RETRY_MS;
        } else {
            stalled(f, t);
        }
        return;
    }

    /* 100 %: continuous counter */
    if (!after(t, f->watch_from)) { f->zero_since = 0; return; }
    if (f->rpm > 0) {
        f->zero_since = 0;
        f->retries = 0;
        return;
    }
    if (f->zero_since == 0) { f->zero_since = t | 1; return; }
    if (!after(t, f->zero_since + STALL_DETECT_MS)) return;
    stalled(f, t);
}

static void update_all(void) { update_fan(0); update_fan(1); }

/* ---------------- protocol ---------------- */
static const char *mode_name(void) {
    return mode == MODE_HOST ? "HOST" : "LOST";
}

static const char *st_name(fan_state_t s) {
    static const char *n[] = { "OFF", "OK", "KICK", "STALL" };
    return n[s];
}

static void print_stat(void) {
    printf("STAT mode=%s d1=%d rpm1=%lu st1=%s d2=%d rpm2=%lu st2=%s\n",
           mode_name(),
           fans[0].duty_out, (unsigned long)fans[0].rpm, st_name(fans[0].st),
           fans[1].duty_out, (unsigned long)fans[1].rpm, st_name(fans[1].st));
}

static void print_cfg(void) {
    printf("CFG min1=%u min2=%u tach1=%u tach2=%u\n",
           cfg.min_duty[0], cfg.min_duty[1], cfg.tach[0], cfg.tach[1]);
}

static void print_probe(int idx, int ms) {
    uint32_t e[40];
    int n = capture_edges(&fans[idx], ms, e, 40);
    update_fan(idx);                               /* back to normal output */
    printf("PROBE fan=%d ms=%d n=%d t=", idx + 1, ms, n);
    for (int i = 0; i < n; i++)
        printf("%s%lu.%02lu", i ? "," : "", (unsigned long)(e[i] / 1000),
               (unsigned long)(e[i] % 1000 / 10));
    printf("\n");
}

/* parse "<a> [<b>]" with each value in [lo, hi]; one value applies to both */
static bool parse_pair(char *p, int lo, int hi, int out[2]) {
    char *end;
    for (int i = 0; i < 2; i++) {
        while (*p == ' ') p++;
        if (*p == 0) {
            if (i == 0) return false;
            out[1] = out[0];
            return true;
        }
        long v = strtol(p, &end, 10);
        if (end == p || v < lo || v > hi) return false;
        out[i] = (int)v;
        p = end;
    }
    while (*p == ' ') p++;
    return *p == 0;
}

static void handle_line(char *s) {
    for (char *p = s; *p; p++) *p = (char)toupper((unsigned char)*p);
    while (*s == ' ') s++;
    int v[2];

    if (strncmp(s, "SET", 3) == 0) {
        if (!parse_pair(s + 3, 0, 100, v)) { printf("ERR SET 0-100 [0-100]\n"); return; }
        fans[0].duty_set = v[0];
        fans[1].duty_set = v[1];
        mode = MODE_HOST;
        last_host_ms = now_ms();
        update_all();
        print_stat();
    } else if (strcmp(s, "GET") == 0) {
        print_stat();
    } else if (strncmp(s, "MIN", 3) == 0) {
        if (!parse_pair(s + 3, 0, 100, v)) { printf("ERR MIN 0-100 [0-100]\n"); return; }
        cfg.min_duty[0] = (uint8_t)v[0];
        cfg.min_duty[1] = (uint8_t)v[1];
        cfg_save();
        update_all();
        print_cfg();
    } else if (strncmp(s, "TACH", 4) == 0) {
        if (!parse_pair(s + 4, 0, 1, v)) { printf("ERR TACH 0|1 [0|1]\n"); return; }
        cfg.tach[0] = (uint8_t)v[0];
        cfg.tach[1] = (uint8_t)v[1];
        cfg_save();
        print_cfg();
    } else if (strcmp(s, "CFG") == 0) {
        print_cfg();
    } else if (strncmp(s, "PROBE", 5) == 0) {
        char *end;
        long ms = strtol(s + 5, &end, 10);
        long fan = *end ? strtol(end, &end, 10) : 1;
        if (ms < 1 || ms > 500 || fan < 1 || fan > 2) { printf("ERR PROBE 1-500 [1|2]\n"); return; }
        print_probe((int)fan - 1, (int)ms);
    } else if (strcmp(s, "ID") == 0) {
        printf("ID FANCTL 2.2 %s\n", DRIVER_NAME);
    } else if (*s) {
        printf("ERR unknown\n");
    }
}

static void poll_usb(void) {
    static char buf[48];
    static size_t len = 0;
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (len) { buf[len] = 0; handle_line(buf); len = 0; }
        } else if (len < sizeof buf - 1) {
            buf[len++] = (char)c;
        } else {
            len = 0;
        }
    }
}

/* ---------------- main ---------------- */
int main(void) {
    const uint off_pins[] = { PIN_PWM1, PIN_PWM2, PIN_EN };
    for (size_t i = 0; i < 3; i++) {
        gpio_init(off_pins[i]);
        gpio_set_dir(off_pins[i], GPIO_OUT);
        gpio_put(off_pins[i], 0);
    }

    cfg_load();
    stdio_init_all();
    tach_init(&fans[0]);
    tach_init(&fans[1]);
    pwm_out_init();
    watchdog_enable(2000, true);

    uint32_t t_tach = now_ms(), t_ctrl = 0;

    while (true) {
        watchdog_update();
        poll_usb();
        uint32_t t = now_ms();

        if (mode == MODE_HOST && t - last_host_ms > HOST_TIMEOUT_MS)
            mode = MODE_LOST;

        if (t - t_tach >= TACH_WINDOW_MS) {
            uint32_t dt = t - t_tach;
            for (int i = 0; i < 2; i++) {
                uint16_t n = pwm_get_counter(fans[i].tach_slice);
                pwm_set_counter(fans[i].tach_slice, 0);
                if (!fans[i].chopped)              /* chopped: last sample stands */
                    fans[i].rpm = (uint32_t)n * 60000u / (dt * TACH_PULSES_PER_REV);
            }
            t_tach = t;
        }

        if (t - t_ctrl >= 100) {
            t_ctrl = t;
            update_all();
        }
        sleep_ms(1);
    }
}
