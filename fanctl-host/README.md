# fanctl - GPU blower controller

XIAO RP2040 + two DRV8871 boards drive two 12 V blowers; a small C daemon on
the host reads GPU temps through NVML and sets each blower's duty over USB.

## Wiring (DRV8871 boards)

| XIAO pin | Goes to |
|---|---|
| D6 / GP0 | Driver 1 **IN2** (PWM, inverted in firmware) |
| D7 / GP1 | Driver 2 **IN2** |
| D4 / GP6 | **IN1** on both drivers (enable) |
| D10 / GP3 | Blower 1 tach (via BAT54 + 10k pull-up to 3V3 + 1k series) |
| D5 / GP7 | Blower 2 tach (same) |
| GND | Driver GND (logic) |

Each driver: VM/GND to 12 V. Blower + to OUT1, blower - to OUT2.
No flyback diode needed. At boot IN1 = IN2 = low, so the driver is asleep
and the blower is off until firmware is running.

Check the ILIM resistor on your boards: I_limit = 64 / R_ILIM(kOhm).
30k gives about 2.1 A, a good limit for 1.5 A blowers.

USB: XIAO USB-C to the motherboard's internal 9-pin USB 2.0 header.

## Firmware

    cd fanctl-fw
    mkdir build && cd build
    cmake .. -DFANCTL_DRIVER=DRV8871      # or LOWSIDE for a MOSFET module
    make
    # hold BOOT, plug in, copy fanctl.uf2 to the RPI-RP2 drive

Test by hand: `screen /dev/ttyACM0` then type `ID`, `GET`, `SET 40 60`.

## Host

    cd fanctl-host
    make
    sudo make install       # binary, udev rule, service; starts fanctl
    journalctl -u fanctl -f

`make install` copies `fanctl.conf` to `/etc/` only if it isn't there yet,
so re-running it after an update keeps your edits. `sudo make uninstall`
removes everything except `/etc/fanctl.conf`.

`fanctl -1` prints one status line. `fanctl -v` logs every update.

Confirm GPU order with `nvidia-smi -L` and set `fan1_gpus` / `fan2_gpus`
so each blower follows its own K80.

## Failure behaviour

| Event | Result |
|---|---|
| GPU unreadable (NVML error, card lost) | that blower to `fail_duty` (100 %) |
| fanctl stops / host hangs | controller: both blowers 100 % after 10 s |
| Controller powers up / resets | both blowers 100 % until the first `SET` |
| Firmware hangs | watchdog reset; drivers asleep during reset, then 100 % |
| Tach reads 0 at >= 30 % duty | warning in the journal |
