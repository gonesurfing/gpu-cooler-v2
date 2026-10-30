# fanctl - dual GPU blower controller

XIAO RP2040 + two DRV8871 modules drive two Delta BFB1012HH blowers (12 V,
1.25 A, 2 pulses/rev tach). A small C daemon on the host reads GPU temps
through NVML and sets each blower's duty over USB.

Schematic: `fan_controller.kicad_sch` (rev C, carrier board for the modules).

## Wiring

| From | To |
|---|---|
| PSU 4-pin peripheral (Molex) | J1: 1 = +12V, 2/3 = GND, 4 (+5V) unused |
| +12V | F1 / F2 (PTC 2 A hold, >= 16 V) -> each module's VM |
| XIAO D4 / GP6 (EN) | IN1 on both modules |
| XIAO D6 / GP0 (PWM1) | module 1 IN2 |
| XIAO D7 / GP1 (PWM2) | module 2 IN2 |
| Module OUT1 / OUT2 | blower red (+) / black (-) |
| Blower blue (FG) | 10k pull-up to 3V3, then 1k to GPIO, 47 nF GPIO-to-GND |
| XIAO D10 / GP3, D5 / GP7 | TACH1, TACH2 |
| XIAO USB-C | motherboard internal 9-pin USB 2.0 header |

- R1-R3 (100k) hold IN1/IN2 low during boot, so the DRV8871s sleep and the
  blowers stay off until the firmware is running.
- Tach pull-up goes to 3V3, never 12 V. No blocking diode is needed with the
  DRV8871: OUT2 is held low, so the blower's ground stays at ground.
- Module current limit: I = 64 / R_ILIM(kOhm). 30k gives ~2.1 A.
- Optional filtered DC (if the blower misbehaves on chopped 12 V): fit
  L (220 uH, Isat >= 2.5 A) + C (100 uF) and remove the 0R link, then raise
  MIN to ~60 % (the blower needs >= 7 V).

## Firmware

    cd fanctl-fw
    mkdir build && cd build
    cmake ..            # -DFANCTL_DRIVER=LOWSIDE for a MOSFET module instead
    make
    # first time: hold B, plug in, copy fanctl.uf2 to RPI-RP2
    # after that the Pico VS Code extension / picotool can reboot it over USB

### Commands (USB serial, any terminal)

| Command | Reply |
|---|---|
| `SET 40 60` | duty % per blower (`SET 40` = both) -> `STAT` |
| `GET` | `STAT mode=HOST d1=40 rpm1=1600 st1=OK d2=60 rpm2=2400 st2=OK` |
| `MIN 35 35` | minimum running duty, saved to flash -> `CFG` |
| `TACH 1 1` | 1 = tach wired (stall recovery on), saved -> `CFG` |
| `CFG` | `CFG min1=35 min2=35 tach1=1 tach2=1` |
| `ID` | `ID FANCTL 2.1 DRV8871` |

`st`: `OFF`, `OK`, `KICK` (start-up or recovery kick at 100 %), `STALL`
(no tach after 3 recovery kicks: held at 100 %, retried every 30 s, clears
by itself when the tach comes back).

`mode`: `HOST` or `LOST` (no SET yet since power-up, or none for 10 s: both
blowers at 100 %).

## Calibrating the minimum duty (do this once per blower)

With the fanctl service stopped (`sudo systemctl stop fanctl`):

1. `fanctl -s "SET 100"`, then step down 5 % at a time
   (`fanctl -s "SET 70"`, `fanctl -s "SET 65"`, ...) and check `fanctl -1`
   after ~5 s each step. Note the duty where a blower's rpm drops to 0, or
   where it falls back to `KICK` repeatedly.
2. `fanctl -s "MIN <stall+10> <stall+10>"`, e.g. `MIN 35 40`.
3. `sudo systemctl start fanctl`.

Starting from stop is handled by the 1 s kick, so MIN only has to keep a
blower running, not start it.

## Host daemon

    make
    sudo make install       # binary, udev rule, service; starts fanctl
    journalctl -u fanctl -f

`make install` copies `fanctl.conf` to `/etc/` only if it isn't there yet,
so re-running it after an update keeps your edits. `sudo make uninstall`
removes everything except `/etc/fanctl.conf`.

`fanctl -1` prints one status line, `fanctl -s "CMD"` sends one command,
`fanctl -v` logs every update. Set `fan1_gpus` / `fan2_gpus` in
`/etc/fanctl.conf` so each blower follows its own K80 (`nvidia-smi -L`).

## Failure behaviour

| Event | Result |
|---|---|
| GPU unreadable (NVML error, card lost) | that blower to `fail_duty` (100 %) |
| Blower stops (tach 0 while running) | firmware recovery kicks; after 3, held at 100 % and logged as STALLED |
| fanctl stops / host hangs | after 10 s: both blowers 100 % |
| Controller powers up / resets | both blowers 100 % until the first `SET` |
| Firmware hangs | watchdog reset; drivers asleep during reset, then 100 % |
| Idle / cold | never off: host curve floor 35 % |
