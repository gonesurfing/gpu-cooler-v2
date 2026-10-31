/*
 * fake libnvidia-ml.so.1 - lets fanctl run on a machine without NVIDIA GPUs
 *
 * GPU temperatures come from a text file (FAKE_NVML_TEMPS, default
 * ./fake-temps), re-read on every query so edits take effect on the next
 * update. One whitespace-separated value per GPU, in index order:
 *
 *     45 47 60 62        GPUs 0-3
 *     45 fail 60 62      GPU 1 unreadable (fanctl -> fail_duty)
 *
 * The GPU count comes from the file at init; a missing file makes init fail
 * like an unloaded driver.
 *
 * Build:   make fake-nvml
 * Run:     DYLD_LIBRARY_PATH=test ./fanctl ...    (macOS)
 *          LD_LIBRARY_PATH=test ./fanctl ...      (Linux)
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef int nvmlReturn_t;
typedef struct nvmlDevice_st *nvmlDevice_t;

#define NVML_SUCCESS                0
#define NVML_ERROR_INVALID_ARGUMENT 2
#define NVML_ERROR_DRIVER_NOT_LOADED 9
#define NVML_ERROR_GPU_IS_LOST      15
#define MAX_GPUS                    16

static unsigned ngpu;

/* temps[i] < 0 = unreadable; returns the number of entries, -1 if no file */
static int read_temps(int temps[MAX_GPUS]) {
    const char *path = getenv("FAKE_NVML_TEMPS");
    FILE *fp = fopen(path ? path : "fake-temps", "r");
    if (!fp) return -1;
    int n = 0;
    char tok[32];
    while (n < MAX_GPUS && fscanf(fp, "%31s", tok) == 1) {
        char *end;
        long v = strtol(tok, &end, 10);
        temps[n++] = (*end || v < 0) ? -1 : (int)v;
    }
    fclose(fp);
    return n;
}

nvmlReturn_t nvmlInit_v2(void) {
    int t[MAX_GPUS], n = read_temps(t);
    if (n < 0) return NVML_ERROR_DRIVER_NOT_LOADED;
    ngpu = (unsigned)n;
    return NVML_SUCCESS;
}

nvmlReturn_t nvmlShutdown(void) { return NVML_SUCCESS; }

nvmlReturn_t nvmlDeviceGetCount_v2(unsigned int *count) {
    *count = ngpu;
    return NVML_SUCCESS;
}

nvmlReturn_t nvmlDeviceGetHandleByIndex_v2(unsigned int index, nvmlDevice_t *dev) {
    if (index >= ngpu) return NVML_ERROR_INVALID_ARGUMENT;
    *dev = (nvmlDevice_t)(uintptr_t)(index + 1);        /* never NULL */
    return NVML_SUCCESS;
}

nvmlReturn_t nvmlDeviceGetTemperature(nvmlDevice_t dev, int sensor, unsigned int *temp) {
    (void)sensor;
    unsigned idx = (unsigned)(uintptr_t)dev - 1;
    int t[MAX_GPUS], n = read_temps(t);
    if (n < 0 || idx >= (unsigned)n || t[idx] < 0) return NVML_ERROR_GPU_IS_LOST;
    *temp = (unsigned)t[idx];
    return NVML_SUCCESS;
}

const char *nvmlErrorString(nvmlReturn_t r) {
    switch (r) {
    case NVML_SUCCESS:                 return "Success";
    case NVML_ERROR_INVALID_ARGUMENT:  return "Invalid Argument";
    case NVML_ERROR_DRIVER_NOT_LOADED: return "Driver Not Loaded (fake: no temps file)";
    case NVML_ERROR_GPU_IS_LOST:       return "GPU is lost (fake: 'fail' in temps file)";
    default:                           return "Unknown Error";
    }
}
