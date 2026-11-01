# gpu-cooler-v2

Temperature-controlled blower cooling for passively cooled datacenter GPUs
(built for a pair of Tesla K80s).

A Seeed XIAO RP2040 drives two 12 V blowers through DRV8871 motor drivers and
reads their tach signals. A small C daemon on the Linux host reads GPU
temperatures through NVML, maps them through a per-blower fan curve, and sends
duty settings to the controller over USB.

    GPU temps (NVML) -> fanctl daemon -> USB CDC -> XIAO RP2040 -> DRV8871 -> blowers
                                      <- STAT (duty, RPM) <-

## Layout

| Path | What it is |
|---|---|
| [`fanctl-fw/`](fanctl-fw) | RP2040 firmware (Pico SDK, CMake) |
| [`fanctl-host/`](fanctl-host) | Host daemon, config, udev rule, systemd unit |

Wiring, build and install steps, and failure behaviour are documented in
[`fanctl-host/README.md`](fanctl-host/README.md).

## Requirements

- **Firmware:** Pico SDK 2.3.1 and its toolchain, installed by the Raspberry
  Pi Pico VS Code extension (or set `PICO_SDK_PATH` yourself)
- **Host:** Linux with the NVIDIA driver (`libnvidia-ml.so.1`), systemd, a C
  compiler. No CUDA toolkit needed; NVML is loaded at runtime.

## Quick start

    # firmware
    cd fanctl-fw && mkdir build && cd build
    cmake .. && make
    # hold BOOT, plug in the XIAO, copy fanctl.uf2 to the RPI-RP2 drive

    # host
    cd fanctl-host
    make && sudo make install
    journalctl -u fanctl -f

Then edit `/etc/fanctl.conf` to map each blower to its GPUs (check the order
with `nvidia-smi -L`) and tune the fan curves.

## Safety

The controller fails toward cooling: at power-up until the first command
arrives, and whenever the daemon stops talking to it for 10 s, both blowers
run at 100 %. A blower whose GPU can't be read also goes to full speed. If a
blower stalls, the firmware kicks it at full power to restart it and, if that
fails, holds it at 100 % and reports the stall to the daemon's log.

## License

MIT, see [LICENSE](LICENSE). Provided as is, with no warranty: if your GPUs
cook, that is on you.
