# ublox-gnss-config

Headless configurator for **u-blox GNSS receivers** (M8 / M9 / M10) — sets the update rate,
constellations, NMEA output and navigation profile in one command, and **saves it to the
receiver's flash** so it survives a power cycle.

One Python file, no GUI, works on **Windows, Linux and macOS**.

*[Русская версия](README.ru.md)*

```
$ ublox-setup --rate 10 --yes

Scanning ports: /dev/ttyUSB0
  [/dev/ttyUSB0] u-blox gen 10  hw=000A0000 fw=SPG 5.10 protver=34.10 (link baud 9600)

Profile: rate=10Hz  talker=GP  dynmodel=automotive  min-elev=10
         systems=GPS,GLO,GAL,BDS,QZSS,SBAS  save=Flash+BBR+RAM

=== /dev/ttyUSB0 (gen 10) ===
  Link type           : uart1  ->  configuring: uart1  (link baud will change)
  Estimated NMEA load : ~4928 B/s of 11520 B/s at 115200 baud (43%)
  ! M10 cannot run GLONASS+BeiDou together -> BeiDou disabled, GLONASS kept
  systems applied     : GPS,GLO,GAL,QZSS,SBAS
  uart1               : OK
  gnss                : OK
  nav                 : OK
  rate                : OK
  nmea                : OK
  sbas                : OK
  itfm                : OK
  assistnow           : OK
  msgout-uart1        : OK
  readback:
    CFG_RATE_MEAS                      = 100
    CFG_NAVSPG_DYNMODEL                = 4
    CFG_NMEA_MAINTALKERID              = 1
    CFG_UART1_BAUDRATE                 = 115200
    CFG_MSGOUT_NMEA_ID_GGA_UART1       = 1
  link baud now       : 115200
  RESULT: OK

Receiver: u-blox M10 (SPG 5.10, protver 34.10)  /dev/ttyUSB0 @ 115200, 10 Hz
Fix:  3D / DGPS(SBAS)   sats used 14   PDOP 1.6  HDOP 0.9  VDOP 1.3
Pos:  55.123456 N  82.987654 E   alt 142.3 m   speed 0.4 km/h   course 55 deg
UTC:  2026-07-24 12:00:00
Est. accuracy: 1.2 m horiz

Constellation   in view   used   C/N0 avg / best
GPS                  11      8   39 / 47 dBHz
GLONASS               8      5   36 / 44 dBHz
Galileo               5      3   35 / 40 dBHz
QZSS                  1      0   31 / 31 dBHz
SBAS                  2      0   34 / 35 dBHz
total                27     16

Link:  2.0 kB/s = 17% of 115200, 0 bad checksums, 0 truncated
Rates: GGA 10.0  RMC 10.0  VTG 10.0  GST 10.0  GSA 6.0  GSV 2.0 Hz
```

## Why

u-blox receivers ship at **1 Hz**, with a conservative navigation profile and a pile of NMEA
sentences you probably don't want. The official way to change that is **u-center** — a Windows
GUI. That is fine once; it is painful when you have a fleet of devices, a headless Linux box, a
Raspberry Pi, or a Mac.

The old scripts you find online drive the **legacy `CFG-*` messages**, which **no longer exist**
on M9 and M10. This tool speaks the modern **`CFG-VALSET`** key/value interface, so the same
command works on gen 8 (FW 3.01+), gen 9 and gen 10 — and it verifies what it wrote.

## What it does

- **Detects** every connected receiver (scans serial ports, probes 115200 / 38400 / 9600,
  reads `UBX-MON-VER`) and identifies the generation from `hwVersion`.
- **Applies a profile**: update rate, GNSS constellations, NMEA version and sentence set,
  dynamic model, elevation mask, SBAS corrections, interference monitor, AssistNow.
- **Writes to RAM + BBR + Flash**, so the configuration survives power-off.
- **Reads back** key settings from the receiver to confirm they took effect.
- **Reports what the receiver is actually receiving**: fix type, satellites in view and used
  per constellation, C/N0, DOP values, position, accuracy estimate — plus link statistics
  (bytes/s, % of the serial line, checksum errors, per-sentence Hz).
- Handles per-generation quirks automatically (see [Constellation limits](#constellation-limits)).

## Supported hardware

| Generation | `hwVersion` | Typical modules | Status |
|---|---|---|---|
| **M10** | `000A0000` | MAX-M10S, MIA-M10, SAM-M10Q, NEO-M10 | fully supported |
| **M9** | `00190000` | ZED-F9P/F9R (as a plain receiver), NEO-M9N | fully supported |
| **M8** | `00080000` | NEO-M8N/M8U, CAM-M8 | supported when protocol version >= 27 (FW 3.01+) |
| M6 / M7 / M8 with old FW | — | NEO-6M, NEO-7M | **not supported** — no `CFG-VALSET`; use u-center |

Connection: the receiver's **native USB** (CDC-ACM) or its **UART** through any USB-serial
bridge (PL2303, CP210x, CH340/CH341, FTDI).

## Install

You need **Python 3.10+** and two packages: [`pyserial`](https://pypi.org/project/pyserial/)
and [`pyubx2`](https://pypi.org/project/pyubx2/). The 3.10 floor comes from `pyubx2`/`pynmeagps`, not from this tool.

### Windows

1. Install Python from [python.org](https://www.python.org/downloads/) — tick
   **"Add python.exe to PATH"** in the installer.
2. Open **PowerShell** or **cmd** and run:

   ```powershell
   pip install pyserial pyubx2
   curl -L -o ublox_setup.py https://raw.githubusercontent.com/odmin4eg/ublox-gnss-config/main/ublox_setup.py
   python ublox_setup.py --list
   ```

3. **Drivers.** Windows 10/11 recognises u-blox native USB and CP210x/FTDI out of the box.
   For a cheap **PL2303** or **CH340** adapter you may need the vendor driver
   ([CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html),
   [PL2303](https://www.prolific.com.tw/US/ShowProduct.aspx?p_id=225&pcid=41)).
   Check **Device Manager → Ports (COM & LPT)** — the port must appear without a yellow "!".
4. **Close u-center before running this tool** — Windows gives exclusive access to a COM port,
   and u-center holds it.

### Linux

```bash
# Debian / Ubuntu / Raspberry Pi OS
sudo apt install python3-pip
pip3 install --user pyserial pyubx2        # or: sudo apt install python3-serial && pip3 install --user pyubx2

wget https://raw.githubusercontent.com/odmin4eg/ublox-gnss-config/main/ublox_setup.py
python3 ublox_setup.py --list
```

**Serial port permissions.** Without this you get `Permission denied: '/dev/ttyUSB0'`:

```bash
sudo usermod -aG dialout $USER     # Debian/Ubuntu; on Arch/Fedora the group is 'uucp'
# log out and back in (or: newgrp dialout) for the group to take effect
```

**If `gpsd` is running**, it grabs the port and will fight you for it — and its u-blox driver
may renegotiate the rate in RAM. Stop it first:

```bash
sudo systemctl stop gpsd gpsd.socket
```

Note: on many distributions the packaged `pyubx2` is old or missing — prefer `pip`. If your
system is "externally managed" (PEP 668, e.g. Ubuntu 24.04) use `pipx` or a venv:

```bash
python3 -m venv ~/.venvs/ublox && ~/.venvs/ublox/bin/pip install pyserial pyubx2
~/.venvs/ublox/bin/python ublox_setup.py --list
```

### macOS

```bash
brew install python                        # or use the python.org installer
pip3 install pyserial pyubx2

curl -L -O https://raw.githubusercontent.com/odmin4eg/ublox-gnss-config/main/ublox_setup.py
python3 ublox_setup.py --list
```

macOS needs **no driver** for u-blox native USB, CP210x or FTDI (built in since Big Sur).
CH340 and old PL2303 clones do need a vendor kext/driver.

Always use the **`/dev/cu.*`** device, never `/dev/tty.*` — `tty.*` blocks waiting for a
carrier signal. The tool already prefers `cu.*` when scanning.

### As a package (any OS)

```bash
pipx install git+https://github.com/odmin4eg/ublox-gnss-config
# or: pip install git+https://github.com/odmin4eg/ublox-gnss-config
ublox-setup --list
```

## Quick start

```bash
# what is plugged in?
ublox-setup --list

# interactive: asks for rate and constellations, then configures everything it finds
ublox-setup

# the common case: 10 Hz, saved to flash, no questions
ublox-setup --rate 10 --yes

# one specific receiver, 5 Hz, pedestrian profile
ublox-setup --port /dev/ttyUSB0 --rate 5 --dynmodel pedestrian --yes

# see what would be sent, touch nothing
ublox-setup --dry-run --rate 10

# try it without committing to flash (reverts on power cycle)
ublox-setup --no-save --yes

# just look at the receiver: live fix / satellites / signal strength
ublox-setup --monitor --port COM3

# just check the link quality of an already-configured receiver
ublox-setup --check 10 --port /dev/ttyUSB0
```

## Command line

| Flag | Default | Meaning |
|---|---|---|
| `--list` | — | list serial ports (device, VID:PID, serial number, guessed type) and exit |
| `--port PORT` | all found | configure one specific port instead of scanning |
| `--rate HZ` | `10` | navigation/output rate, 1–25 Hz |
| `--systems LIST` | all | comma list of `GPS,GLO,GAL,BDS,QZSS,SBAS` |
| `--talker gp\|gn` | `gp` | NMEA main talker ID — `$GPxxx` or standard multi-GNSS `$GNxxx` |
| `--dynmodel NAME` | `automotive` | `portable`, `stationary`, `pedestrian`, `automotive`, `sea`, `airborne1g/2g/4g` |
| `--min-elev DEG` | `10` | elevation mask — ignore satellites below this angle |
| `--iface auto\|uart1\|usb\|both` | `auto` | which receiver interface gets the NMEA output configured |
| `--baud N` | `115200` | target UART1 baud rate |
| `--no-save` | off | apply to RAM only (test; reverts on power cycle) |
| `--dry-run` | off | print every key/value that would be sent, change nothing |
| `--monitor [SEC]` | — | live fix / satellite / signal view, Ctrl-C to exit |
| `--check [SEC]` | `5` | one-shot report on an already-configured receiver, configures nothing |
| `--no-check` | off | skip the report after configuring |
| `--raw` | off | add a few raw NMEA sample lines to the report |
| `--yes` | off | no prompts, no confirmation (for scripts and CI) |

Exit codes: **0** success · **1** nothing found or usage error · **2** a receiver failed to
apply the configuration cleanly.

## Why the "gp" talker default

With multiple constellations enabled, u-blox emits `$GNGGA` / `$GNRMC` (GN = mixed GNSS).
A surprising number of consumers — Android head units, older loggers, hand-rolled parsers,
some navigation apps — filter on `GPGGA` / `GPRMC` and silently show **no position** when
they see `GN`. Forcing the main talker to `GP` makes those work.

If your software is standards-correct and you want proper multi-GNSS talkers, use
`--talker gn`. Either way per-constellation `GSV` sentences keep their own talker
(`$GPGSV`, `$GLGSV`, `$GAGSV`, …), so the satellite report below stays meaningful.

## What it sets, and why

| Setting | Value | Why |
|---|---|---|
| UART1 baud | 115200 | headroom for 10 Hz multi-GNSS NMEA |
| Protocols | in: UBX + NMEA, out: NMEA + UBX | NMEA for your consumer; UBX kept so the receiver can still be polled and reconfigured |
| Rate | 10 Hz | smooth track, precise time-tagging of events |
| NMEA version | 4.1 | modern multi-GNSS sentences and talker IDs |
| Main talker ID | `GP` | compatibility (see above) |
| GSV talker ID | GNSS-specific | keeps the per-constellation satellite breakdown |
| High precision | on | more decimals in latitude/longitude |
| Sentences on | GGA, RMC, VTG, GST at full rate; GSA, GSV decimated to ~2 Hz | GGA = position, RMC = speed/time are needed every cycle; GSA/GSV are bulky and change slowly |
| Sentences off | GLL, GNS, ZDA, GBS, DTM, GRS, VLW | redundant — pure bandwidth |
| Dynamic model | automotive | filter tuned for a vehicle: smoother track, fewer outliers at speed |
| Elevation mask | 10° | cuts multipath from buildings |
| SBAS | ranging + differential corrections + integrity | WAAS / EGNOS / SDCM / MSAS accuracy improvement |
| Interference monitor | on | detects and mitigates CW/broadband jamming (cheap chargers, jammers) |
| AssistNow Autonomous | on | fast re-fix after tunnels, bridges, parking garages |
| Saved to | RAM + BBR + Flash | survives a full power-off |

Deliberately **left at factory defaults**: `PDOP`/`TDOP` output filters and the `MINCNO`
signal-strength mask. Tightening them looks good on paper but costs you the fix in tunnels,
urban canyons and under trees. The 10° elevation mask already removes most of the junk.

See [README.ru.md](README.ru.md) for a key-by-key reference of every `CFG-VALSET` value the
tool writes.

## Constellation limits

- **M9** runs all six systems concurrently: GPS + GLONASS + Galileo + BeiDou + QZSS + SBAS.
- **M10** (single band) **cannot run GLONASS and BeiDou at the same time** — a hardware
  limitation; the receiver simply NAKs the request. The tool detects the generation and
  **drops BeiDou, keeps GLONASS**, printing a note. Use `--systems GPS,BDS,GAL,QZSS,SBAS`
  if you would rather keep BeiDou.

More constellations means more visible satellites, better geometry (lower DOP) and a faster,
more stable fix — which is exactly what you want in a city or under trees.

## Bandwidth

10 Hz multi-GNSS NMEA is not free. The tool estimates the load before writing and warns you
if it exceeds ~80% of the serial line, and the report after configuring shows what it really
came out as. Measured on an M10 at 10 Hz over a 115200 UART with five constellations:
**~17% channel utilisation, 0 bad checksums, 0 truncated lines**.

If you push the rate up or the baud down and the estimate goes over 100%, sentences *will*
get truncated. Fix it by raising `--baud`, lowering `--rate`, or dropping constellations.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `--list` shows no ports | cable is charge-only; driver missing (Device Manager on Windows); on Linux check `dmesg \| tail` after plugging in |
| `Permission denied: '/dev/ttyUSB0'` | add yourself to `dialout` (or `uucp`) and re-login |
| `Could not open port ... Access is denied` (Windows) | u-center or another program is holding the COM port — close it |
| `no u-blox response` on a port that exists | it is a different device; or the receiver is at an unusual baud (the tool probes 115200/38400/9600); or you are on `/dev/tty.*` instead of `/dev/cu.*` on macOS |
| Configuration applies, but the rate drops back to 1 Hz | something else is reconfiguring the receiver at runtime — `gpsd`'s u-blox driver does this in RAM. Our settings are in flash, so they return on the next power cycle; stop `gpsd` or re-run this tool afterwards |
| Everything is `OK` but there is no fix | you are indoors. A cold start needs ~30 s of open sky. `--monitor` shows satellites in view even without a fix — if C/N0 values appear, the antenna is fine |
| `gen 8 ... needs the legacy CFG-* interface` | pre-3.01 firmware; use u-center, or update the firmware |
| Rate changes but sentences look garbled | the link is over its bandwidth budget — see [Bandwidth](#bandwidth) |

## Safety notes

- Everything is written to **flash** by default. Use `--no-save` to test in RAM first — a
  power cycle then restores the previous configuration.
- This tool does not touch the firmware and cannot brick a receiver. Worst case you write a
  configuration you don't like; re-run it with different flags, or restore factory defaults
  from u-center.
- When the link is the very UART being reconfigured, the baud change is applied **first** and
  the port is reopened at the new speed. If the tool is interrupted at exactly that moment,
  the receiver may be left at the new baud while you expect the old one — just re-run,
  detection probes several baud rates.

## Credits

The setting profile started from the well-known u-center recipe for Android head units
discussed on the 4PDA forum; this tool reimplements it over the modern `CFG-VALSET`
interface so it can run headless on any OS and on M9/M10, where the legacy `CFG-*` messages
no longer exist. Protocol work is done by [pyubx2](https://github.com/semuconsulting/pyubx2).

## License

MIT — see [LICENSE](LICENSE).
