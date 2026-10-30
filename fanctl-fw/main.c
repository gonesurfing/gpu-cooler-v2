/*
 * fanctl firmware v2 - XIAO RP2040, two blowers, USB CDC control
 *
 * Driver stage (pick one at build time, cmake -DFANCTL_DRIVER=...):
 *   LOWSIDE            MOSFET trigger module, low-side switch.
 *            Active-high PWM on the trigger input. Needs a flyback
 *            Schottky across each blower.
 *   DRV8871  (default) One DRV8871 per blower. IN1 <- EN (GP6, shared), IN2 <- PWM
 *            inverted, giving forward/brake slow decay. No flyback diode.
 *
 * Pins:
 *   GP0 (D6)  PWM1        slice 0 A
 *   GP1 (D7)  PWM2        slice 0 B
 *   GP6 (D4)  EN          DRV8871 IN1 for both channels (unused for LOWSIDE)
 *   GP3 (D10) TACH1       slice 1 B, falling-edge counter
 *   GP7 (D5)  TACH2       slice 3 B, falling-edge counter
 *
 * USB CDC protocol, one command per line, case-insensitive:
 *   SET <d1> [<d2>]   duty % per blower (one value = both)  -> STAT
 *   GET                                                     -> STAT
 *   ID                                                      -> ID FANCTL 2.0 <driver>
 *
 *   STAT mode=HOST d1=45 rpm1=1820 d2=60 rpm2=1950
 *   mode: HOST | LOST (no SET yet, or none for HOST_TIMEOUT_MS; both blowers 100 %)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"

#define DRV_LOWSIDE 1
#define DRV_DRV8871 2
#ifndef FANCTL_DRIVER
#define FANCTL_DRIVER DRV_DRV8871
#endif

/* ---------------- configuration ---------------- */
#define PIN_PWM1           0
#define PIN_PWM2           1
#define PIN_EN             6
#define PIN_TACH1          3
#define PIN_TACH2          7

#if FANCTL_DRIVER == DRV_DRV8871
#define PWM_FREQ_HZ        25000
#define DRIVER_NAME        "DRV8871"
#else
#define PWM_FREQ_HZ        20000   /* MOSFET modules are rated to 20 kHz */
#define DRIVER_NAME        "LOWSIDE"
#endif

#define TACH_PULSES_PER_REV 2
#define TACH_WINDOW_MS     1000

#define HOST_TIMEOUT_MS    10000
#define LOST_DUTY          100     /* duty with no host */
#define MIN_RUN_DUTY       20      /* nonzero duty is raised to this */
#define KICK_MS            500     /* full power when starting from stop */

/* ---------------- state ---------------- */
typedef enum { MODE_HOST, MODE_LOST } run_mode_t;

typedef struct {
    uint     pin_pwm, pin_tach, tach_slice;
    int      duty_set;       /* host request, % */
    int      duty_out;       /* reported duty, % */
    bool     kicking;
    uint32_t kick_until;
    uint32_t rpm;
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

/* ---------------- hardware ---------------- */
static void set_level(fan_t *f, int pct) {
    /* With DRV8871 the output is inverted in hardware (pwm polarity), so the
     * same level means "drive fraction" in both builds. */
    pwm_set_gpio_level(f->pin_pwm, (uint16_t)((pwm_top + 1) * (uint32_t)pct / 100));
}

static void pwm_out_init(void) {
    pwm_slice = pwm_gpio_to_slice_num(PIN_PWM1);      /* GP0/GP1 share slice 0 */
    pwm_top = clock_get_hz(clk_sys) / PWM_FREQ_HZ - 1;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_wrap(&c, pwm_top);
#if FANCTL_DRIVER == DRV_DRV8871
    pwm_config_set_output_polarity(&c, true, true);   /* IN2 low = drive */
#endif
    pwm_init(pwm_slice, &c, false);
    for (int i = 0; i < 2; i++) {
        set_level(&fans[i], 0);
        gpio_set_function(fans[i].pin_pwm, GPIO_FUNC_PWM);
    }
    pwm_set_enabled(pwm_slice, true);

#if FANCTL_DRIVER == DRV_DRV8871
    sleep_ms(1);
    gpio_put(PIN_EN, 1);   /* IN1 high: from here IN2 selects drive/brake */
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

/* ---------------- control ---------------- */
static int target_duty(const fan_t *f) {
    int d = mode == MODE_HOST ? f->duty_set : LOST_DUTY;
    if (d > 0 && d < MIN_RUN_DUTY) d = MIN_RUN_DUTY;
    return d > 100 ? 100 : d;
}

static void update_fan(fan_t *f) {
    int target = target_duty(f);
    uint32_t t = now_ms();

    if (target == 0) { f->kicking = false; f->duty_out = 0; set_level(f, 0); return; }
    if (f->duty_out == 0 && !f->kicking) { f->kicking = true; f->kick_until = t + KICK_MS; }
    f->duty_out = target;
    if (f->kicking) {
        if ((int32_t)(f->kick_until - t) > 0) { set_level(f, 100); return; }
        f->kicking = false;
    }
    set_level(f, target);
}

static void update_all(void) { update_fan(&fans[0]); update_fan(&fans[1]); }

/* ---------------- protocol ---------------- */
static const char *mode_name(void) {
    return mode == MODE_HOST ? "HOST" : "LOST";
}

static void print_stat(void) {
    printf("STAT mode=%s d1=%d rpm1=%lu d2=%d rpm2=%lu\n", mode_name(),
           fans[0].duty_out, (unsigned long)fans[0].rpm,
           fans[1].duty_out, (unsigned long)fans[1].rpm);
}

static bool parse_duty(char **p, int *out) {
    char *end;
    long v = strtol(*p, &end, 10);
    if (end == *p || v < 0 || v > 100) return false;
    *out = (int)v;
    *p = end;
    return true;
}

static void handle_line(char *s) {
    for (char *p = s; *p; p++) *p = (char)toupper((unsigned char)*p);
    while (*s == ' ') s++;

    if (strncmp(s, "SET", 3) == 0) {
        char *p = s + 3;
        int d1, d2;
        if (!parse_duty(&p, &d1)) { printf("ERR range 0-100\n"); return; }
        while (*p == ' ') p++;
        if (*p == 0) d2 = d1;
        else if (!parse_duty(&p, &d2)) { printf("ERR range 0-100\n"); return; }
        fans[0].duty_set = d1;
        fans[1].duty_set = d2;
        mode = MODE_HOST;
        last_host_ms = now_ms();
        update_all();
        print_stat();
    } else if (strcmp(s, "GET") == 0) {
        print_stat();
    } else if (strcmp(s, "ID") == 0) {
        printf("ID FANCTL 2.0 %s\n", DRIVER_NAME);
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
    /* hold everything off until configured */
    const uint off_pins[] = { PIN_PWM1, PIN_PWM2, PIN_EN };
    for (size_t i = 0; i < 3; i++) {
        gpio_init(off_pins[i]);
        gpio_set_dir(off_pins[i], GPIO_OUT);
        gpio_put(off_pins[i], 0);
    }

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

        if (t - t_ctrl >= 100) {
            t_ctrl = t;
            update_all();
        }

        if (t - t_tach >= TACH_WINDOW_MS) {
            uint32_t dt = t - t_tach;
            for (int i = 0; i < 2; i++) {
                uint16_t n = pwm_get_counter(fans[i].tach_slice);
                pwm_set_counter(fans[i].tach_slice, 0);
                fans[i].rpm = (uint32_t)n * 60000u / (dt * TACH_PULSES_PER_REV);
            }
            t_tach = t;
        }
        sleep_ms(1);
    }
}
