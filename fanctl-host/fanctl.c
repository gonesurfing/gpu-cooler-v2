/*
 * fanctl - GPU blower controller host daemon (C, no dependencies beyond libc)
 *
 * Reads GPU temperatures through NVML (libnvidia-ml.so.1, loaded at runtime
 * with dlopen, so no CUDA headers are needed to build), maps each blower's
 * GPUs through its own fan curve, and sends "SET d1 d2" to the XIAO RP2040
 * controller over USB CDC. Reads back STAT (tach RPM).
 *
 * Build:   gcc -O2 -Wall -o fanctl fanctl.c -ldl
 * Run:     fanctl [-c /etc/fanctl.conf] [-p /dev/fanctl] [-v] [-1]
 *          -1  print one status line from the controller and exit
 *
 * Failure handling:
 *   - A blower whose GPU can't be read (NVML error, GPU gone) gets fail_duty.
 *   - Serial errors: close, retry every 5 s.
 *   - If this program stops, the controller notices after 10 s and runs
 *     both blowers at 100 % (mode=LOST).
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define MAX_PTS  8
#define MAX_GPUS 8

/* ------------------------------------------------------------ logging */
static int verbose, use_prefix;

static void logmsg(int prio, const char *fmt, ...) {
    /* journald understands "<N>" priority prefixes on stderr */
    if (use_prefix) fprintf(stderr, "<%d>", prio);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}
#define LOG_ERR  3
#define LOG_WARN 4
#define LOG_INFO 6

/* ------------------------------------------------------------ config */
typedef struct {
    int    ngpu;                 /* 0 = fan not used (duty 0) */
    int    gpu[MAX_GPUS];
    int    npts;
    double t[MAX_PTS], d[MAX_PTS];
    int    tach;                 /* 1 = blower has a tach wire */
} fan_cfg_t;

typedef struct {
    char      port[256];
    double    interval;          /* s, must be < 10 (controller timeout) */
    double    ramp_down;         /* max % decrease per update */
    int       fail_duty;
    int       stall_min_duty;    /* warn if duty >= this and rpm == 0 */
    fan_cfg_t fan[2];
} cfg_t;

static int parse_curve(const char *s, fan_cfg_t *f) {
    f->npts = 0;
    while (*s) {
        double t, d;
        int used;
        if (f->npts >= MAX_PTS || sscanf(s, " %lf : %lf%n", &t, &d, &used) != 2) return -1;
        if (d < 0 || d > 100 || (f->npts && t <= f->t[f->npts - 1])) return -1;
        f->t[f->npts] = t;
        f->d[f->npts] = d;
        f->npts++;
        s += used;
        while (*s == ' ' || *s == ',') s++;
    }
    return f->npts >= 2 ? 0 : -1;
}

static int parse_gpus(const char *s, fan_cfg_t *f) {
    f->ngpu = 0;
    if (strcmp(s, "none") == 0) return 0;
    while (*s) {
        char *end;
        long v = strtol(s, &end, 10);
        if (end == s || v < 0 || f->ngpu >= MAX_GPUS) return -1;
        f->gpu[f->ngpu++] = (int)v;
        s = end;
        while (*s == ' ' || *s == ',') s++;
    }
    return f->ngpu ? 0 : -1;
}

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

static int load_config(const char *path, cfg_t *c) {
    memset(c, 0, sizeof *c);
    strcpy(c->port, "/dev/fanctl");
    c->interval = 2.0;
    c->ramp_down = 3.0;
    c->fail_duty = 100;
    c->stall_min_duty = 30;
    parse_gpus("0,1", &c->fan[0]);
    parse_gpus("2,3", &c->fan[1]);
    for (int i = 0; i < 2; i++) {
        parse_curve("40:0,50:30,70:70,82:100", &c->fan[i]);
        c->fan[i].tach = 1;
    }

    FILE *fp = fopen(path, "r");
    if (!fp) {
        if (errno == ENOENT) { logmsg(LOG_WARN, "no %s, using defaults", path); return 0; }
        logmsg(LOG_ERR, "%s: %s", path, strerror(errno));
        return -1;
    }
    char line[512];
    int ln = 0, bad = 0;
    while (fgets(line, sizeof line, fp)) {
        ln++;
        char *h = strchr(line, '#');
        if (h) *h = 0;
        char *s = trim(line);
        if (!*s) continue;
        char *eq = strchr(s, '=');
        if (!eq) { logmsg(LOG_ERR, "%s:%d: expected key=value", path, ln); bad = 1; continue; }
        *eq = 0;
        char *k = trim(s), *v = trim(eq + 1);
        int ok = 1;
        if      (!strcmp(k, "port"))           ok = snprintf(c->port, sizeof c->port, "%s", v) < (int)sizeof c->port;
        else if (!strcmp(k, "interval"))       c->interval = atof(v);
        else if (!strcmp(k, "ramp_down"))      c->ramp_down = atof(v);
        else if (!strcmp(k, "fail_duty"))      c->fail_duty = atoi(v);
        else if (!strcmp(k, "stall_min_duty")) c->stall_min_duty = atoi(v);
        else if (!strncmp(k, "fan", 3) && (k[3] == '1' || k[3] == '2') && k[4] == '_') {
            fan_cfg_t *f = &c->fan[k[3] - '1'];
            const char *sub = k + 5;
            if      (!strcmp(sub, "gpus"))  ok = parse_gpus(v, f) == 0;
            else if (!strcmp(sub, "curve")) ok = parse_curve(v, f) == 0;
            else if (!strcmp(sub, "tach"))  f->tach = atoi(v);
            else ok = 0;
        } else ok = 0;
        if (!ok) { logmsg(LOG_ERR, "%s:%d: bad setting '%s'", path, ln, k); bad = 1; }
    }
    fclose(fp);
    if (c->interval <= 0 || c->interval >= 10) {
        logmsg(LOG_ERR, "interval must be between 0 and 10 s");
        bad = 1;
    }
    if (c->fail_duty < 0 || c->fail_duty > 100) { logmsg(LOG_ERR, "fail_duty 0-100"); bad = 1; }
    return bad ? -1 : 0;
}

static double curve_duty(const fan_cfg_t *f, double t) {
    if (t <= f->t[0]) return f->d[0];
    for (int i = 1; i < f->npts; i++)
        if (t <= f->t[i])
            return f->d[i - 1] + (t - f->t[i - 1]) / (f->t[i] - f->t[i - 1]) * (f->d[i] - f->d[i - 1]);
    return f->d[f->npts - 1];
}

/* ------------------------------------------------------------ NVML */
typedef int nvmlReturn_t;               /* NVML_SUCCESS == 0 */
typedef struct nvmlDevice_st *nvmlDevice_t;
#define NVML_TEMPERATURE_GPU 0

static struct {
    void *lib;
    int   up;
    nvmlReturn_t (*init)(void);
    nvmlReturn_t (*shutdown)(void);
    nvmlReturn_t (*count)(unsigned int *);
    nvmlReturn_t (*handle)(unsigned int, nvmlDevice_t *);
    nvmlReturn_t (*temp)(nvmlDevice_t, int, unsigned int *);
    const char  *(*errstr)(nvmlReturn_t);
} nv;

static const char *nv_err(nvmlReturn_t r) {
    static char buf[32];
    if (nv.errstr) return nv.errstr(r);
    snprintf(buf, sizeof buf, "NVML error %d", r);
    return buf;
}

static int nvml_up(void) {
    if (nv.up) return 0;
    if (!nv.lib) {
        nv.lib = dlopen("libnvidia-ml.so.1", RTLD_NOW);
        if (!nv.lib) { logmsg(LOG_ERR, "dlopen libnvidia-ml.so.1: %s", dlerror()); return -1; }
        *(void **)&nv.init     = dlsym(nv.lib, "nvmlInit_v2");
        *(void **)&nv.shutdown = dlsym(nv.lib, "nvmlShutdown");
        *(void **)&nv.count    = dlsym(nv.lib, "nvmlDeviceGetCount_v2");
        *(void **)&nv.handle   = dlsym(nv.lib, "nvmlDeviceGetHandleByIndex_v2");
        *(void **)&nv.temp     = dlsym(nv.lib, "nvmlDeviceGetTemperature");
        *(void **)&nv.errstr   = dlsym(nv.lib, "nvmlErrorString");
        if (!nv.init || !nv.shutdown || !nv.count || !nv.handle || !nv.temp) {
            logmsg(LOG_ERR, "libnvidia-ml.so.1 is missing required symbols");
            dlclose(nv.lib);
            memset(&nv, 0, sizeof nv);
            return -1;
        }
    }
    nvmlReturn_t r = nv.init();
    if (r) { logmsg(LOG_ERR, "nvmlInit: %s", nv_err(r)); return -1; }
    unsigned int n = 0;
    nv.count(&n);
    logmsg(LOG_INFO, "NVML up, %u GPU(s)", n);
    nv.up = 1;
    return 0;
}

static void nvml_down(void) {
    if (nv.up) nv.shutdown();
    nv.up = 0;
}

/* hottest temperature among a fan's GPUs; -1 on any failure */
static double fan_temp(const fan_cfg_t *f, char *desc, size_t dlen) {
    double hot = -273;
    size_t off = 0;
    desc[0] = 0;
    for (int i = 0; i < f->ngpu; i++) {
        nvmlDevice_t dev;
        unsigned int t;
        nvmlReturn_t r = nv.handle((unsigned)f->gpu[i], &dev);
        if (!r) r = nv.temp(dev, NVML_TEMPERATURE_GPU, &t);
        static int gpu_bad[64];                         /* log on state change only */
        int g = f->gpu[i] & 63;
        if (r) {
            if (!gpu_bad[g]) logmsg(LOG_WARN, "GPU %d: %s - blower to fail_duty", f->gpu[i], nv_err(r));
            gpu_bad[g] = 1;
            return -1;
        }
        if (gpu_bad[g]) logmsg(LOG_INFO, "GPU %d: readable again", f->gpu[i]);
        gpu_bad[g] = 0;
        if (t > hot) hot = t;
        if (off < dlen)
            off += (size_t)snprintf(desc + off, dlen - off, "%s%u", i ? "," : "", t);
    }
    return hot;
}

/* ------------------------------------------------------------ serial */
static int ser_fd = -1;
static char rxbuf[256];
static size_t rxlen;

static void ser_close(void) {
    if (ser_fd >= 0) close(ser_fd);
    ser_fd = -1;
    rxlen = 0;
}

static int ser_readline(char *out, size_t olen, int timeout_ms) {
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (;;) {
        char *nl = memchr(rxbuf, '\n', rxlen);
        if (nl) {
            size_t n = (size_t)(nl - rxbuf);
            size_t c = n < olen - 1 ? n : olen - 1;
            memcpy(out, rxbuf, c);
            out[c] = 0;
            if (c && out[c - 1] == '\r') out[c - 1] = 0;
            memmove(rxbuf, nl + 1, rxlen - n - 1);
            rxlen -= n + 1;
            if (out[0]) return 0;
            continue;                                   /* skip blank lines */
        }
        if (rxlen == sizeof rxbuf) rxlen = 0;           /* garbage, drop */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int left = timeout_ms - (int)((now.tv_sec - t0.tv_sec) * 1000 +
                                      (now.tv_nsec - t0.tv_nsec) / 1000000);
        if (left <= 0) return -1;
        struct pollfd p = { .fd = ser_fd, .events = POLLIN };
        int pr = poll(&p, 1, left);
        if (pr < 0 && errno == EINTR) continue;
        if (pr <= 0) return -1;
        if (p.revents & (POLLERR | POLLHUP)) return -1;
        ssize_t r = read(ser_fd, rxbuf + rxlen, sizeof rxbuf - rxlen);
        if (r <= 0) return -1;
        rxlen += (size_t)r;
    }
}

/* send one command, return the reply line */
static int ser_cmd(const char *cmd, char *reply, size_t rlen) {
    char out[64];
    int n = snprintf(out, sizeof out, "%s\n", cmd);
    if (write(ser_fd, out, (size_t)n) != n) return -1;
    return ser_readline(reply, rlen, 1000);
}

static int ser_open(const char *port) {
    ser_fd = open(port, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (ser_fd < 0) { logmsg(LOG_ERR, "open %s: %s", port, strerror(errno)); return -1; }
    struct termios tio;
    if (tcgetattr(ser_fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, B115200);                      /* ignored by CDC */
        tio.c_cflag |= CLOCAL | CREAD;
        tcsetattr(ser_fd, TCSANOW, &tio);
    }
    tcflush(ser_fd, TCIOFLUSH);
    char id[128];
    if (ser_cmd("ID", id, sizeof id) < 0 || strncmp(id, "ID FANCTL", 9) != 0) {
        logmsg(LOG_ERR, "%s: no FANCTL controller (reply: '%s')", port, id);
        ser_close();
        return -1;
    }
    logmsg(LOG_INFO, "connected to %s: %s", port, id);
    return 0;
}

/* pull "key=value" out of a STAT line */
static int stat_get(const char *line, const char *key, char *val, size_t vlen) {
    size_t kl = strlen(key);
    for (const char *p = line; (p = strstr(p, key)); p += kl) {
        if ((p == line || p[-1] == ' ') && p[kl] == '=') {
            p += kl + 1;
            size_t n = strcspn(p, " ");
            if (n >= vlen) n = vlen - 1;
            memcpy(val, p, n);
            val[n] = 0;
            return 0;
        }
    }
    strcpy(val, "?");
    return -1;
}

/* ------------------------------------------------------------ main */
static volatile sig_atomic_t running = 1;
static void on_signal(int s) { (void)s; running = 0; }

static double mono(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    const char *cfg_path = "/etc/fanctl.conf", *port_override = NULL;
    int once = 0, opt;
    while ((opt = getopt(argc, argv, "c:p:v1h")) != -1) {
        switch (opt) {
        case 'c': cfg_path = optarg; break;
        case 'p': port_override = optarg; break;
        case 'v': verbose = 1; break;
        case '1': once = 1; break;
        default:
            fprintf(stderr, "usage: %s [-c config] [-p port] [-v] [-1]\n", argv[0]);
            return 2;
        }
    }
    use_prefix = !isatty(STDERR_FILENO);

    cfg_t cfg;
    if (load_config(cfg_path, &cfg) < 0) return 1;
    if (port_override) snprintf(cfg.port, sizeof cfg.port, "%s", port_override);

    if (once) {
        char reply[256];
        if (ser_open(cfg.port) < 0 || ser_cmd("GET", reply, sizeof reply) < 0) return 1;
        printf("%s\n", reply);
        return 0;
    }

    struct sigaction sa = { .sa_handler = on_signal };
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    double duty[2] = { -1, -1 };     /* -1 = not yet set */
    int    last_logged[2] = { -1, -1 }, stall_n[2] = { 0, 0 };
    double t_log = 0, t_nvml_retry = 0, t_nvml_reinit = 0;

    while (running) {
        double t_start = mono();

        if (ser_fd < 0 && ser_open(cfg.port) < 0) {
            for (int i = 0; i < 50 && running; i++) usleep(100000);
            continue;
        }

        /* --- temperatures -> duty --- */
        int nv_ok = 1;
        if (!nv.up && t_start >= t_nvml_retry) {
            if (nvml_up() < 0) t_nvml_retry = t_start + 10;
        }
        char temps[2][64];
        for (int i = 0; i < 2; i++) {
            fan_cfg_t *f = &cfg.fan[i];
            double target;
            strcpy(temps[i], "-");
            if (f->ngpu == 0) {
                target = 0;
            } else {
                double t = nv.up ? fan_temp(f, temps[i], sizeof temps[i]) : -1;
                if (t < 0) {
                    target = cfg.fail_duty;
                    nv_ok = 0;
                    strcpy(temps[i], "FAIL");
                } else {
                    target = curve_duty(f, t);
                }
            }
            /* spin up immediately, spin down gradually */
            if (duty[i] < 0 || target >= duty[i]) duty[i] = target;
            else duty[i] = fmax(target, duty[i] - cfg.ramp_down);
        }
        /* a GPU that keeps failing may need NVML re-initialised (driver reset,
         * card back on the bus) - try that at most every 30 s */
        if (!nv_ok && nv.up && t_start >= t_nvml_reinit) {
            nvml_down();
            t_nvml_retry = 0;
            t_nvml_reinit = t_start + 30;
        }

        /* --- send to controller --- */
        char cmd[32], reply[256];
        snprintf(cmd, sizeof cmd, "SET %d %d", (int)lround(duty[0]), (int)lround(duty[1]));
        if (ser_cmd(cmd, reply, sizeof reply) < 0 || strncmp(reply, "STAT", 4) != 0) {
            logmsg(LOG_ERR, "controller did not answer '%s' - reconnecting", cmd);
            ser_close();
            continue;
        }

        /* --- report --- */
        char rpm[2][16];
        stat_get(reply, "rpm1", rpm[0], sizeof rpm[0]);
        stat_get(reply, "rpm2", rpm[1], sizeof rpm[1]);

        int changed = 0;
        for (int i = 0; i < 2; i++) {
            int d = (int)lround(duty[i]);
            if (abs(d - last_logged[i]) >= 5) changed = 1;
            fan_cfg_t *f = &cfg.fan[i];
            if (f->tach && cfg.stall_min_duty > 0 && d >= cfg.stall_min_duty && !strcmp(rpm[i], "0")) {
                if (++stall_n[i] == 3 || stall_n[i] % 30 == 0)
                    logmsg(LOG_WARN, "fan%d: no tach at %d%% duty - stalled or unplugged?", i + 1, d);
            } else {
                stall_n[i] = 0;
            }
        }
        if (verbose || changed || t_start - t_log >= 60) {
            logmsg(LOG_INFO, "fan1 gpu[%s] %d%% %s rpm | fan2 gpu[%s] %d%% %s rpm",
                   temps[0], (int)lround(duty[0]), rpm[0],
                   temps[1], (int)lround(duty[1]), rpm[1]);
            last_logged[0] = (int)lround(duty[0]);
            last_logged[1] = (int)lround(duty[1]);
            t_log = t_start;
        }

        while (running && mono() - t_start < cfg.interval) usleep(100000);
    }

    ser_close();
    nvml_down();
    logmsg(LOG_INFO, "exiting");
    return 0;
}
