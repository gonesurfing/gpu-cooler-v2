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
| XIAO D5 / GP7, D10 / GP3 | TACH1, TACH2 |
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
| `GET` | `STAT mode=HOST d1=30 rpm1=1280 st1=OK d2=70 rpm2=2040 st2=OK` |
| `MIN 35 35` | floor under the LOW speed, saved to flash -> `CFG` |
| `TACH 1 1` | 1 = tach wired (stall recovery on), saved -> `CFG` |
| `CFG` | `CFG min1=35 min2=35 tach1=1 tach2=1` |
| `PROBE 100` | debug: blower 1 at 100 % for 100 ms, tach edge times -> `PROBE fan=1 ms=100 n=5 t=5.70,29.44,...` |
| `ID` | `ID FANCTL 2.2 DRV8871` |

`st`: `OFF`, `OK`, `KICK` (start-up or recovery kick at 100 %), `STALL`
(no tach after 3 recovery kicks: held at 100 %, retried every 30 s, clears
by itself when the tach comes back).

`mode`: `HOST` or `LOST` (no SET yet since power-up, or none for 10 s: both
blowers at 100 %).

`d` is the level actually applied, not the request.

### Three speeds

With its 12 V chopped by PWM, the BFB1012HH only runs at a few distinct
speeds, measured on the bench:

| Duty | Speed |
|---|---|
| 20-60 % | ~1280 rpm (all the same) |
| 70 % | ~2000 rpm |
| 80-90 % | ~2250-2450 rpm coming from below, ~4200 coming from 100 % |
| 100 % | ~4200 rpm |

So the firmware maps each request onto three levels and never parks in the
80-90 % band:

| Request | Level |
|---|---|
| 1-49 % | LOW 30 % (~1280 rpm) |
| 50-89 % | MID 70 % (~2000 rpm) |
| 90-100 % | HIGH 100 % (~4200 rpm) |

On the way down it switches at 45 % (MID -> LOW) and 85 % (HIGH -> MID), so
a request hovering at a threshold doesn't flap. `MIN` raises the LOW level
if a blower needs more than 30 % to keep running; 20-60 % all run the same
speed, so the default 30 % costs nothing.

### Tach while chopped

The blower's tach output comes from its own driver chip, which runs off the
chopped supply, so it is only trustworthy at 0 % and 100 %; at LOW and MID
the falling-edge counter overreads. At those levels the firmware samples
instead: 3 s and 10 s after reaching a level (slowing down takes several
seconds), then every 30 s, it runs the blower
at 100 % for at most 70 ms (usually 20-50 ms), times two tach edges, and
reports that rpm. The tach is valid within a millisecond of steady power and
the impeller barely speeds up in that time, so the sample matches the real
speed. A sample with no edges (or under 1000 rpm) is retried after 2 s;
three misses in a row start the stall recovery below.

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

## Testing without GPUs (e.g. on a Mac)

`make fake-nvml` builds `test/libnvidia-ml.so.1`, a stand-in NVML that reads
GPU temperatures from a text file, one value per GPU (`fail` = unreadable).
The file is re-read every update, so edit it while fanctl runs:

    make && make fake-nvml
    echo "45 47 60 62" > fake-temps
    DYLD_LIBRARY_PATH=test ./fanctl -c fanctl.conf -p /dev/cu.usbmodem* -v
    # Linux: LD_LIBRARY_PATH=test ./fanctl -c fanctl.conf -p /dev/ttyACM0 -v

On macOS use the `/dev/cu.*` port, not `/dev/tty.*`.

## Failure behaviour

| Event | Result |
|---|---|
| GPU unreadable (NVML error, card lost) | that blower to `fail_duty` (100 %) |
| Blower stops (tach 0 at 100 %, or 3 failed samples at LOW / MID) | firmware recovery kicks; after 3, held at 100 % and logged as STALLED |
| fanctl stops / host hangs | after 10 s: both blowers 100 % |
| Controller powers up / resets | both blowers 100 % until the first `SET` |
| Firmware hangs | watchdog reset; drivers asleep during reset, then 100 % |
| Idle / cold | never off: host curve floor 35 % |
