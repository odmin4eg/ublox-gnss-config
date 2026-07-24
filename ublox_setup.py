#!/usr/bin/env python3
"""
ublox_setup - one-shot configurator for u-blox M8 / M9 / M10 GNSS receivers.

Detects connected u-blox receivers, identifies the generation via UBX-MON-VER,
and applies a sane "moving vehicle" profile through the modern CFG-VALSET
configuration interface (generation 9 and 10; generation 8 from firmware 3.01,
protocol version 27, onward).

Profile applied (all of it adjustable from the command line):
  - Update rate 1..25 Hz (default 10); GSA/GSV decimated to ~2 Hz
  - NMEA 4.1, high-precision coordinates, main talker ID GP or GN
  - GGA/RMC/VTG/GST at full rate; GLL/GNS/ZDA/GBS/DTM/GRS/VLW off
  - Dynamic model (default automotive), minimum SV elevation (default 10 deg)
  - SBAS ranging + differential corrections + integrity on
  - Interference/jamming monitor on, AssistNow Autonomous on
  - UART1 at the chosen baud (default 115200), UBX+NMEA both directions
  - Saved to RAM + BBR + Flash so it survives a power cycle (--no-save = RAM)

Per-generation limits are handled automatically: a single-band M10 cannot run
GLONASS and BeiDou concurrently, so BeiDou is dropped (GLONASS kept).

After configuring, the tool captures the live stream for a few seconds and
prints a receiver report: fix type, satellites used/in view per constellation,
DOPs, position, estimated accuracy, and link statistics -- so you can see not
just that settings were written but that the receiver is actually receiving.

Setup:
    pip install pyserial pyubx2

Usage:
    python3 ublox_setup.py --list                 # show serial ports and exit
    python3 ublox_setup.py                        # interactive, all receivers
    python3 ublox_setup.py --rate 10 --yes        # non-interactive
    python3 ublox_setup.py --port COM5 --rate 5 --talker gn --yes
    python3 ublox_setup.py --iface both --dynmodel sea --min-elev 5 --yes
    python3 ublox_setup.py --dry-run --rate 10 --iface both --yes
    python3 ublox_setup.py --check 10 --port /dev/ttyUSB0   # one-shot report
    python3 ublox_setup.py --monitor --port COM3  # live view, Ctrl-C to stop
    python3 ublox_setup.py --monitor 30           # live view for 30 seconds
    python3 ublox_setup.py --no-save --yes        # test in RAM only

Exit codes: 0 = success, 1 = nothing found / usage error,
            2 = one or more receivers failed to apply cleanly.
--monitor always exits 0 unless the port cannot be opened.
"""

import argparse
import math
import os
import platform
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

# --- dependencies (fail with a friendly message, not a traceback) ------------

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write(
        "Missing dependency: pyserial\n"
        "Install with:  pip install pyserial pyubx2\n")
    sys.exit(1)

try:
    from pyubx2 import (
        UBXMessage, UBXReader, POLL,
        SET_LAYER_RAM, SET_LAYER_BBR, SET_LAYER_FLASH,
        POLL_LAYER_RAM, TXN_NONE,
    )
except ImportError:
    sys.stderr.write(
        "Missing dependency: pyubx2\n"
        "Install with:  pip install pyserial pyubx2\n")
    sys.exit(1)

ALL_LAYERS = SET_LAYER_RAM | SET_LAYER_BBR | SET_LAYER_FLASH  # bitmask 7

# --- constants ---------------------------------------------------------------

# USB vendor IDs used to rank / classify serial ports.
UBLOX_VID = 0x1546                    # u-blox native USB (CDC-ACM)
BRIDGE_VIDS = {                       # common USB-UART bridge chips
    0x067B: "PL2303",
    0x10C4: "CP210x",
    0x1A86: "CH34x",
    0x0403: "FTDI",
}

# GNSS system -> CFG-SIGNAL enable keys (system enable + L1-band signal enable)
SYSTEM_KEYS = {
    "GPS":  ["CFG_SIGNAL_GPS_ENA", "CFG_SIGNAL_GPS_L1CA_ENA"],
    "GLO":  ["CFG_SIGNAL_GLO_ENA", "CFG_SIGNAL_GLO_L1_ENA"],
    "GAL":  ["CFG_SIGNAL_GAL_ENA", "CFG_SIGNAL_GAL_E1_ENA"],
    "BDS":  ["CFG_SIGNAL_BDS_ENA", "CFG_SIGNAL_BDS_B1_ENA"],
    "QZSS": ["CFG_SIGNAL_QZSS_ENA", "CFG_SIGNAL_QZSS_L1CA_ENA"],
    "SBAS": ["CFG_SIGNAL_SBAS_ENA", "CFG_SIGNAL_SBAS_L1CA_ENA"],
}
ALL_SYSTEMS = list(SYSTEM_KEYS.keys())

# CFG-NAVSPG-DYNMODEL values (u-blox interface description)
DYNMODELS = {
    "portable": 0, "stationary": 2, "pedestrian": 3, "automotive": 4,
    "sea": 5, "airborne1g": 6, "airborne2g": 7, "airborne4g": 8,
}

# CFG-NMEA-MAINTALKERID: 0 = auto (GN for multi-GNSS), 1 = force GP.
# GP matters for legacy parsers / head units that filter on GPGGA/GPRMC.
TALKERS = {"gp": 1, "gn": 0}

# hwVersion string (from MON-VER) -> receiver generation
HW_GEN = {"00080000": 8, "00190000": 9, "000A0000": 10}

# Rough NMEA sentence sizes in bytes (incl. $, checksum, CR LF) used by the
# bandwidth estimator. GSA is emitted once per constellation (NMEA 4.1),
# GSV typically ~3 sentences of ~70 bytes per constellation.
FULLRATE_BYTES = 82 + 80 + 50 + 60        # GGA + RMC + VTG + GST per epoch
PER_TALKER_BYTES = 66 + 3 * 70            # GSA + GSV per constellation

MON_VER_BAUDS = (115200, 38400, 9600)     # link baud probe order


@dataclass
class Profile:
    """Everything the user chose; consumed by build_config()."""
    rate: int = 10
    systems: List[str] = field(default_factory=lambda: list(ALL_SYSTEMS))
    ifaces: List[str] = field(default_factory=lambda: ["uart1"])
    talker: str = "gp"
    dynmodel: str = "automotive"
    min_elev: int = 10
    baud: int = 115200                    # target UART1 baud
    save: bool = True                     # write RAM+BBR+Flash vs RAM only


# =============================================================================
# Port discovery / classification
# =============================================================================

def classify_port(p) -> str:
    """Categorize a pyserial ListPortInfo: ublox / bridge / usb / system."""
    if p.vid == UBLOX_VID:
        return "u-blox USB"
    if p.vid in BRIDGE_VIDS:
        return "USB-UART bridge (%s)" % BRIDGE_VIDS[p.vid]
    if p.vid is not None:
        return "USB serial"
    return "system port"


def _macos_fix(device: str) -> str:
    """On macOS prefer the callout device (/dev/cu.*) over /dev/tty.*."""
    if platform.system() == "Darwin" and device.startswith("/dev/tty."):
        return "/dev/cu." + device[len("/dev/tty."):]
    return device


def _macos_skip(device: str) -> bool:
    """Skip macOS Bluetooth / debug-console pseudo ports."""
    lower = device.lower()
    return any(s in lower for s in ("bluetooth", "debug-console", "wlan-debug"))


def enumerate_ports() -> List[dict]:
    """All serial ports, u-blox first, then bridges, then the rest."""
    entries = []
    for p in list_ports.comports():
        device = _macos_fix(p.device)
        if platform.system() == "Darwin" and _macos_skip(device):
            continue
        cat = classify_port(p)
        rank = {"u-blox USB": 0, "USB-UART bridge": 1}.get(cat.split(" (")[0], 2)
        if cat == "system port":
            rank = 3
        entries.append({
            "device": device,
            "description": p.description or "",
            "vid": p.vid, "pid": p.pid,
            "serial": p.serial_number or "",
            "category": cat,
            "rank": rank,
        })
    entries.sort(key=lambda e: (e["rank"], e["device"]))
    return entries


def candidate_ports() -> List[dict]:
    """Ports worth probing automatically.

    Legacy motherboard UARTs (no USB VID, e.g. Linux /dev/ttyS*) are excluded
    from the automatic scan -- probing dozens of dead ports takes minutes.
    They can still be targeted explicitly with --port.
    """
    return [e for e in enumerate_ports() if e["vid"] is not None]


def link_kind(port: str, entries: Optional[List[dict]] = None) -> str:
    """How are we attached to the receiver? 'usb', 'uart1' or 'unknown'.

    'usb'   = the receiver's own USB interface (CDC-ACM); baud is meaningless
              on this link and must never be "changed" under ourselves.
    'uart1' = a USB-UART bridge (or a real UART) wired to the receiver's
              UART1 -- reconfiguring UART1 baud drops this very link.
    """
    if entries is None:
        entries = enumerate_ports()
    for e in entries:
        if e["device"] == port:
            if e["vid"] == UBLOX_VID:
                return "usb"
            if e["vid"] in BRIDGE_VIDS:
                return "uart1"
            break
    # Fall back to name heuristics for ports pyserial did not classify.
    lower = port.lower()
    if "ttyacm" in lower or "usbmodem" in lower:
        return "usb"
    if "ttyusb" in lower or "usbserial" in lower or "uart" in lower:
        return "uart1"
    return "unknown"


def print_port_list() -> int:
    entries = enumerate_ports()
    if not entries:
        # Listing is informational: "nothing plugged in" is an answer, not an
        # error, so it must not fail scripts or CI.
        print("No serial ports found.")
        return 0
    # Motherboard UARTs (/dev/ttyS*, no USB VID) are almost never the receiver
    # and a PC can expose dozens of them -- summarize instead of listing.
    usb = [e for e in entries if e["vid"] is not None]
    legacy = [e for e in entries if e["vid"] is None]
    if usb:
        print("%-18s %-9s %-14s %-26s %s" %
              ("DEVICE", "VID:PID", "SERIAL", "CATEGORY", "DESCRIPTION"))
        for e in usb:
            print("%-18s %-9s %-14s %-26s %s" %
                  (e["device"], "%04X:%04X" % (e["vid"], e["pid"]),
                   e["serial"] or "-", e["category"], e["description"]))
    else:
        print("No USB serial ports found -- is the receiver plugged in?")
    if legacy:
        names = ", ".join(e["device"] for e in legacy[:6])
        more = " ..." if len(legacy) > 6 else ""
        print("\n%d built-in/system port(s) not scanned automatically: %s%s"
              % (len(legacy), names, more))
        print("Target one explicitly with --port if the receiver is on a real UART.")
    return 0


def open_port(port: str, baud: int, timeout: float = 0.3) -> serial.Serial:
    """Open a serial port, or raise SystemExit with a platform-specific hint."""
    try:
        return serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException as exc:
        print("ERROR: cannot open %s: %s" % (port, exc))
        system = platform.system()
        if system == "Linux":
            print("  Hint: your user may lack permission on the port.")
            print("        sudo usermod -aG dialout $USER   (or 'uucp' on Arch)")
            print("        then log out and back in.")
        elif system == "Windows":
            print("  Hint: the port may be held open by another program")
            print("        (u-center?), or the driver is not installed.")
        elif system == "Darwin":
            print("  Hint: use the /dev/cu.* device, not /dev/tty.*.")
        raise SystemExit(1)


# =============================================================================
# Low-level UBX I/O
# =============================================================================

def _parse_ubx(buf: bytes) -> list:
    """Extract complete UBX frames from a byte buffer by hand (ignores NMEA).

    Scanning for the 0xB5 0x62 sync directly avoids feeding NMEA to UBXReader
    (which spews 'stream terminated' warnings on truncated lines) and makes
    ACK detection reliable amid a heavy NMEA stream.
    """
    out = []
    i, n = 0, len(buf)
    while i < n - 7:
        if buf[i] == 0xB5 and buf[i + 1] == 0x62:
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 6 + length + 2
            if end <= n:
                try:
                    out.append(UBXReader.parse(buf[i:end]))
                except Exception:
                    pass
                i = end
                continue
            break
        i += 1
    return out


def read_until(stream, want, timeout: float = 3.0):
    """Read raw bytes (bounded by serial timeout) into a buffer, then parse.

    Never feeds the live stream to UBXReader directly, so a baud-mismatch
    garbage stream cannot cause an unbounded blocking read.
    """
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        chunk = stream.read(256)
        if chunk:
            buf += chunk
            for m in _parse_ubx(buf):
                if m.identity in want:
                    return m
            if len(buf) > 16384:
                buf = buf[-4096:]
        elif not buf:
            continue
    return None


def raw_has_signal(stream, dur: float = 1.5) -> bool:
    """True if the stream carries valid NMEA or a UBX sync within dur."""
    end = time.time() + dur
    buf = b""
    while time.time() < end:
        buf += stream.read(256)
        if b"\xb5\x62" in buf or b"$G" in buf:
            return True
        if len(buf) > 8192:
            break
    return False


def detect(port: str, bauds=MON_VER_BAUDS):
    """Poll MON-VER at several bauds. Returns (info_dict, baud) or (None, None)."""
    for baud in bauds:
        try:
            s = serial.Serial(port, baud, timeout=0.3)
        except Exception:
            return None, None
        time.sleep(0.15)
        s.reset_input_buffer()
        s.write(UBXMessage("MON", "MON-VER", POLL).serialize())
        ver = read_until(s, {"MON-VER"}, timeout=2.0)
        s.close()
        if ver:
            sw = bytes(ver.swVersion).split(b"\x00")[0].decode(errors="replace")
            hw = bytes(ver.hwVersion).split(b"\x00")[0].decode(errors="replace")
            protver = None
            i = 1
            # firmware details ride in the MON-VER extension strings
            while hasattr(ver, "extension_%02d" % i):
                ext = bytes(getattr(ver, "extension_%02d" % i)) \
                    .split(b"\x00")[0].decode(errors="replace")
                if ext.startswith("PROTVER="):
                    protver = ext.split("=", 1)[1]
                i += 1
            gen = HW_GEN.get(hw.upper())
            return {"sw": sw, "hw": hw, "protver": protver, "gen": gen}, baud
    return None, None


def valset_supported(info: dict) -> Tuple[bool, List[str]]:
    """Can this receiver take CFG-VALSET? Returns (proceed, notes).

    Generation 9/10 always support it. Generation 8 gained it in firmware
    3.01 (protocol version 27); older gen-8 firmware and receivers with an
    unrecognized hwVersion need the legacy CFG-* interface (u-center).
    """
    gen = info.get("gen")
    if gen in (9, 10):
        return True, []
    protver = None
    try:
        protver = float((info.get("protver") or "").split()[0])
    except (ValueError, IndexError):
        pass
    label = ("generation 8" if gen == 8
             else "unknown generation (hwVersion %s)" % info.get("hw"))
    if protver is not None and protver >= 27:
        return True, ["%s with PROTVER %s: CFG-VALSET should work, "
                      "but this path is less tested" % (label, protver)]
    return False, [
        "%s (PROTVER %s) does not support CFG-VALSET." % (label, protver),
        "It needs the legacy CFG-* interface -- configure it with u-center.",
    ]


# =============================================================================
# Configuration profile
# =============================================================================

def adjust_systems(gen, systems: List[str]) -> Tuple[List[str], List[str]]:
    """Apply per-generation GNSS capability limits. Returns (systems, notes)."""
    sysset = list(systems)
    notes = []
    # Single-band M10 cannot run GLONASS and BeiDou concurrently (VALSET
    # would NAK) -- prefer GLONASS.
    if gen == 10 and "GLO" in sysset and "BDS" in sysset:
        sysset.remove("BDS")
        notes.append("M10 cannot run GLONASS+BeiDou together -> "
                     "BeiDou disabled, GLONASS kept")
    return sysset, notes


def gsx_decim(rate: int) -> int:
    """GSA/GSV output divider: decimate to ~2 Hz so they don't hog the UART."""
    return max(1, round(rate / 2))


def build_config(profile: Profile) -> List[Tuple[str, List[Tuple[str, int]]]]:
    """Return list of (group_name, [(key, value), ...]) applied in order.

    Each group is sent as one CFG-VALSET. UART1 link settings go LAST because
    the baud change may drop our own connection when we are talking to the
    receiver over that very UART (the caller reorders in that case).
    """
    meas = int(round(1000 / profile.rate))    # CFG-RATE-MEAS period, ms
    decim = gsx_decim(profile.rate)

    gnss = []
    for sysname, keys in SYSTEM_KEYS.items():
        on = 1 if sysname in profile.systems else 0
        for k in keys:
            gnss.append((k, on))

    groups = [
        ("gnss", gnss),
        ("nav", [
            ("CFG_NAVSPG_DYNMODEL", DYNMODELS[profile.dynmodel]),
            ("CFG_NAVSPG_INFIL_MINELEV", profile.min_elev),  # degrees
        ]),
        ("rate", [
            ("CFG_RATE_MEAS", meas),
            ("CFG_RATE_NAV", 1),              # one nav solution per measurement
        ]),
        ("nmea", [
            ("CFG_NMEA_PROTVER", 41),         # NMEA 4.1
            ("CFG_NMEA_HIGHPREC", 1),         # extra coordinate decimals
            ("CFG_NMEA_MAINTALKERID", TALKERS[profile.talker]),
            # GSV keeps its GNSS-specific talker (GPGSV/GLGSV/...) even when
            # the main talker is forced to GP -- without this the satellite
            # report could not tell the constellations apart.
            ("CFG_NMEA_GSVTALKERID", 0),
        ]),
        ("sbas", [
            ("CFG_SBAS_USE_RANGING", 1),
            ("CFG_SBAS_USE_DIFFCORR", 1),
            ("CFG_SBAS_USE_INTEGRITY", 1),
        ]),
        ("itfm", [
            # jamming/interference monitor (thresholds left at defaults)
            ("CFG_ITFM_ENABLE", 1),
        ]),
        ("assistnow", [
            ("CFG_ANA_USE_ANA", 1),           # AssistNow Autonomous: faster refix
        ]),
    ]

    # Message rates per output interface. Key suffix _UART1 / _USB selects
    # the physical port each rate applies to.
    for iface in profile.ifaces:
        sfx = {"uart1": "UART1", "usb": "USB"}[iface]
        groups.append(("msgout-%s" % iface, [
            ("CFG_MSGOUT_NMEA_ID_GGA_%s" % sfx, 1),      # position
            ("CFG_MSGOUT_NMEA_ID_RMC_%s" % sfx, 1),      # speed/course/date
            ("CFG_MSGOUT_NMEA_ID_VTG_%s" % sfx, 1),      # ground speed
            ("CFG_MSGOUT_NMEA_ID_GST_%s" % sfx, 1),      # error statistics
            ("CFG_MSGOUT_NMEA_ID_GSA_%s" % sfx, decim),  # DOP, slow-changing
            ("CFG_MSGOUT_NMEA_ID_GSV_%s" % sfx, decim),  # sats in view, bulky
            ("CFG_MSGOUT_NMEA_ID_GLL_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_GNS_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_ZDA_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_GBS_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_DTM_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_GRS_%s" % sfx, 0),
            ("CFG_MSGOUT_NMEA_ID_VLW_%s" % sfx, 0),
        ]))

    if "usb" in profile.ifaces:
        groups.append(("usb-prot", [
            ("CFG_USBINPROT_UBX", 1),
            ("CFG_USBINPROT_NMEA", 1),
            ("CFG_USBOUTPROT_UBX", 1),
            ("CFG_USBOUTPROT_NMEA", 1),
        ]))

    if "uart1" in profile.ifaces:
        groups.append(("uart1", [
            ("CFG_UART1INPROT_UBX", 1),       # keep UBX in: re-config over wire
            ("CFG_UART1INPROT_NMEA", 1),
            ("CFG_UART1OUTPROT_UBX", 1),      # keep UBX out: ACK/poll/verify
            ("CFG_UART1OUTPROT_NMEA", 1),
            ("CFG_UART1_BAUDRATE", profile.baud),
        ]))

    return groups


# =============================================================================
# Bandwidth estimate
# =============================================================================

def estimate_load(profile: Profile) -> Tuple[int, int, float]:
    """Estimated NMEA (bytes/sec, capacity bytes/sec, utilization 0..1).

    Accounts for constellation count: GSA is one sentence per constellation
    in NMEA 4.1, GSV is several. 1 byte on a UART = 10 bits (start+8+stop).
    """
    talkers = len([s for s in profile.systems
                   if s in ("GPS", "GLO", "GAL", "BDS")]) or 1
    gsx_hz = profile.rate / gsx_decim(profile.rate)
    bps = int(profile.rate * FULLRATE_BYTES
              + gsx_hz * talkers * PER_TALKER_BYTES)
    capacity = profile.baud // 10
    return bps, capacity, bps / capacity


def print_load(profile: Profile) -> None:
    bps, capacity, util = estimate_load(profile)
    print("  Estimated NMEA load : ~%d B/s of %d B/s at %d baud (%.0f%%)"
          % (bps, capacity, profile.baud, util * 100))
    if "uart1" in profile.ifaces and util > 0.8:
        print("  WARNING: estimated load exceeds 80% of UART capacity --")
        print("           expect truncated sentences. Raise --baud or lower --rate.")


# =============================================================================
# Applying the configuration
# =============================================================================

def apply_group(stream, cfg, layers=ALL_LAYERS, retries: int = 1):
    """Send one CFG-VALSET, wait for ACK. Under a heavy 10 Hz NMEA stream the
    ACK can be missed, so retry once. Returns True/False/None (no response --
    the command may still have applied; see readback)."""
    for _ in range(retries + 1):
        msg = UBXMessage.config_set(layers, TXN_NONE, cfg)
        stream.reset_input_buffer()
        stream.write(msg.serialize())
        ack = read_until(stream, {"ACK-ACK", "ACK-NAK"}, timeout=2.5)
        if ack is not None:
            return ack.identity == "ACK-ACK"
        time.sleep(0.1)
    return None


def configure(port: str, baud: int, gen, profile: Profile, uart_is_link: bool):
    """Apply the profile to one receiver. Returns a result dict."""
    layers = ALL_LAYERS if profile.save else SET_LAYER_RAM
    systems, notes = adjust_systems(gen, profile.systems)
    prof = Profile(**{**profile.__dict__, "systems": systems})
    groups = build_config(prof)

    # When we are connected over the very UART we reconfigure, raise the baud
    # FIRST and reconnect, then send everything else at the new speed (avoids
    # flooding the old slow link once the rate/messages are enabled).
    if uart_is_link:
        groups = ([g for g in groups if g[0] == "uart1"]
                  + [g for g in groups if g[0] != "uart1"])

    s = open_port(port, baud)
    time.sleep(0.2)
    results = {}
    for name, cfg in groups:
        if name == "uart1" and uart_is_link:
            new_baud = dict(cfg)["CFG_UART1_BAUDRATE"]
            s.reset_input_buffer()
            s.write(UBXMessage.config_set(layers, TXN_NONE, cfg).serialize())
            time.sleep(0.6)               # let the frame flush at the old baud
            s.close()
            time.sleep(0.4)
            s = open_port(port, new_baud)
            time.sleep(0.2)
            results[name] = raw_has_signal(s, dur=2.0)
            baud = new_baud
        else:
            results[name] = apply_group(s, cfg, layers)

    # Verify a few key values back from the RAM layer.
    verify = {
        "CFG_RATE_MEAS": int(round(1000 / prof.rate)),
        "CFG_NAVSPG_DYNMODEL": DYNMODELS[prof.dynmodel],
        "CFG_NMEA_MAINTALKERID": TALKERS[prof.talker],
    }
    if "uart1" in prof.ifaces:
        verify["CFG_UART1_BAUDRATE"] = prof.baud
        verify["CFG_MSGOUT_NMEA_ID_GGA_UART1"] = 1
    if "usb" in prof.ifaces:
        verify["CFG_MSGOUT_NMEA_ID_GGA_USB"] = 1
    s.reset_input_buffer()
    s.write(UBXMessage.config_poll(POLL_LAYER_RAM, 0,
                                   list(verify.keys())).serialize())
    vget = read_until(s, {"CFG-VALGET"}, timeout=2.0)
    readback, mismatches = {}, []
    if vget:
        for k, expect in verify.items():
            if hasattr(vget, k):
                got = getattr(vget, k)
                readback[k] = got
                if got != expect:
                    mismatches.append("%s: expected %s, got %s"
                                      % (k, expect, got))

    s.close()

    clean = (all(v is not False for v in results.values())
             and not mismatches and bool(readback))
    return {
        "results": results, "readback": readback, "mismatches": mismatches,
        "baud": baud, "notes": notes, "systems": systems, "clean": clean,
    }


# =============================================================================
# NMEA state parser (hand-rolled -- no third-party NMEA library)
# =============================================================================

# GSV talker prefix -> constellation. GN carries no constellation info.
GSV_TALKER = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
              "GB": "BeiDou", "BD": "BeiDou",  # BD = legacy BeiDou talker
              "GQ": "QZSS", "GI": "NavIC"}

# NMEA 4.1 GSA systemId (last field) -> constellation.
GSA_SYSID = {1: "GPS", 2: "GLONASS", 3: "Galileo",
             4: "BeiDou", 5: "QZSS", 6: "NavIC"}

FIX_QUALITY = {0: "NONE", 1: "GPS", 2: "DGPS(SBAS)", 4: "RTK fix",
               5: "RTK float", 6: "dead reckoning"}
FIX_MODE = {1: "no fix", 2: "2D", 3: "3D"}

CONST_ORDER = ["GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "NavIC", "SBAS"]
CONST_LETTER = {"GPS": "G", "GLONASS": "R", "Galileo": "E", "BeiDou": "B",
                "QZSS": "Q", "NavIC": "I", "SBAS": "S"}


def _sbas_prn(prn: int) -> bool:
    """SBAS satellites ride in the GP talker with these PRN ranges."""
    return 33 <= prn <= 64 or 120 <= prn <= 158


def _f(s: str) -> Optional[float]:
    try:
        return float(s)
    except (ValueError, TypeError):
        return None


def _i(s: str) -> Optional[int]:
    try:
        return int(s)
    except (ValueError, TypeError):
        return None


def _dm_to_deg(v: str, hemi: str) -> Optional[float]:
    """NMEA ddmm.mmmm / dddmm.mmmm + hemisphere -> signed decimal degrees."""
    x = _f(v)
    if x is None:
        return None
    deg = int(x / 100)
    val = deg + (x - deg * 100) / 60.0
    return -val if hemi in ("S", "W") else val


@dataclass
class GnssState:
    """Everything we learned about the receiver from the NMEA stream."""
    fix_quality: Optional[int] = None     # GGA: 0/1/2/4/5/6
    sats_used: Optional[int] = None       # GGA: satellites in solution
    hdop: Optional[float] = None          # GGA
    alt: Optional[float] = None           # GGA: meters above MSL
    lat: Optional[float] = None
    lon: Optional[float] = None
    speed_kmh: Optional[float] = None     # RMC (knots * 1.852)
    course: Optional[float] = None        # RMC
    utc_time: Optional[str] = None        # RMC hhmmss.ss
    utc_date: Optional[str] = None        # RMC ddmmyy
    rmc_valid: Optional[bool] = None      # RMC status A/V
    fix_mode: int = 1                     # GSA: 1 none / 2 2D / 3 3D (max)
    pdop: Optional[float] = None          # GSA
    gsa_hdop: Optional[float] = None
    vdop: Optional[float] = None
    used: Dict[str, Set[int]] = field(default_factory=dict)  # per systemId
    used_any: Set[int] = field(default_factory=set)          # no systemId
    # sats: constellation -> {prn: {"el","az","cno","ts"}}
    sats: Dict[str, Dict[int, dict]] = field(default_factory=dict)
    horiz_acc: Optional[float] = None     # GST: sqrt(lat_sig^2 + lon_sig^2)

    def used_all(self) -> Set[int]:
        out = set(self.used_any)
        for s in self.used.values():
            out |= s
        return out

    def used_for(self, const: str) -> Set[int]:
        """Used-PRN set for one constellation. With NMEA 4.1 systemId-tagged
        GSA this is exact; legacy untagged GSA falls back to the global set
        (PRN collisions across constellations are then possible)."""
        return self.used.get(const, self.used_any)

    def has_fix(self) -> bool:
        return (self.fix_quality or 0) > 0 or self.fix_mode >= 2

    def in_view(self, max_age: Optional[float] = None,
                now: Optional[float] = None):
        """{constellation: {prn: satdict}}, optionally dropping stale sats."""
        if max_age is None:
            return self.sats
        now = now if now is not None else time.time()
        out = {}
        for const, sats in self.sats.items():
            keep = {p: d for p, d in sats.items()
                    if now - d.get("ts", now) <= max_age}
            if keep:
                out[const] = keep
        return out


def _parse_gga(f: List[str], st: GnssState) -> None:
    if len(f) < 10:
        return
    q = _i(f[6])
    if q is not None:
        st.fix_quality = q
    n = _i(f[7])
    if n is not None:
        st.sats_used = n
    st.hdop = _f(f[8]) or st.hdop
    st.alt = _f(f[9]) if _f(f[9]) is not None else st.alt
    lat = _dm_to_deg(f[2], f[3])            # f[1] is UTC time
    lon = _dm_to_deg(f[4], f[5])
    if lat is not None and lon is not None:
        st.lat, st.lon = lat, lon


def _parse_rmc(f: List[str], st: GnssState) -> None:
    if len(f) < 10:
        return
    st.utc_time = f[1] or st.utc_time
    if f[2] in ("A", "V"):
        st.rmc_valid = (f[2] == "A")
    kn = _f(f[7])
    if kn is not None:
        st.speed_kmh = kn * 1.852
    c = _f(f[8])
    if c is not None:
        st.course = c
    st.utc_date = f[9] or st.utc_date


def _parse_gsa(f: List[str], st: GnssState) -> None:
    if len(f) < 18:
        return
    mode = _i(f[2])
    if mode in (1, 2, 3):
        st.fix_mode = max(st.fix_mode, mode)
    prns = set()
    for x in f[3:15]:
        p = _i(x)
        if p is not None:
            prns.add(p)
    st.pdop = _f(f[15]) if _f(f[15]) is not None else st.pdop
    st.gsa_hdop = _f(f[16]) if _f(f[16]) is not None else st.gsa_hdop
    st.vdop = _f(f[17]) if _f(f[17]) is not None else st.vdop
    const = GSA_SYSID.get(_i(f[18])) if len(f) > 18 else None
    if const:
        st.used.setdefault(const, set()).update(prns)
    else:
        st.used_any.update(prns)


def _parse_gsv(f: List[str], st: GnssState, now: float) -> None:
    const = GSV_TALKER.get(f[0][:2])
    if const is None or len(f) < 4:
        return  # GN talker (or unknown) carries no constellation info
    sat_fields = f[4:]
    if len(sat_fields) % 4 == 1:
        sat_fields = sat_fields[:-1]  # NMEA 4.1 trailing signalId
    for i in range(0, len(sat_fields) - 3, 4):
        prn = _i(sat_fields[i])
        if prn is None:
            continue
        bucket = ("SBAS" if const == "GPS" and _sbas_prn(prn) else const)
        st.sats.setdefault(bucket, {})[prn] = {
            "el": _i(sat_fields[i + 1]),
            "az": _i(sat_fields[i + 2]),
            "cno": _i(sat_fields[i + 3]),
            "ts": now,
        }


def _parse_gst(f: List[str], st: GnssState) -> None:
    if len(f) < 8:
        return
    lat_sig, lon_sig = _f(f[6]), _f(f[7])
    if lat_sig is not None and lon_sig is not None:
        st.horiz_acc = math.sqrt(lat_sig ** 2 + lon_sig ** 2)


# =============================================================================
# Stream capture / link statistics (shared by --check, --monitor, post-config)
# =============================================================================

def new_counters() -> dict:
    return {"bytes": 0, "seconds": 0.0, "bad": 0, "trunc": 0,
            "types": {}, "raw_samples": []}


def process_line(line: str, counters: dict, st: GnssState,
                 now: Optional[float] = None) -> None:
    """Verify one NMEA line's checksum, count it, feed it to the parser.

    Tolerant by design: anything malformed bumps a counter and is skipped;
    nothing here may raise on garbage input.
    """
    if not line.startswith("$"):
        return                              # UBX or other binary in between
    now = now if now is not None else time.time()
    body, sep, ck = line[1:].partition("*")
    if not sep or len(ck) < 2:
        counters["trunc"] += 1
        return
    x = 0
    for ch in body:
        x ^= ord(ch)
    try:
        ok = (x == int(ck[:2], 16))
    except ValueError:
        ok = False
    if not ok:
        counters["bad"] += 1
        return
    f = body.split(",")
    ident = f[0]                            # e.g. GPGGA, GLGSV, PUBX
    if len(ident) < 5 or ident.startswith("P"):
        return                              # proprietary or too short
    stype = ident[2:5]                      # sentence type, talker-agnostic
    counters["types"][stype] = counters["types"].get(stype, 0) + 1
    if len(counters["raw_samples"]) < 4:
        counters["raw_samples"].append(line)
    try:
        if stype == "GGA":
            _parse_gga(f, st)
        elif stype == "RMC":
            _parse_rmc(f, st)
        elif stype == "GSA":
            _parse_gsa(f, st)
        elif stype == "GSV":
            _parse_gsv(f, st, now)
        elif stype == "GST":
            _parse_gst(f, st)
    except Exception:
        pass                                # never let a weird field crash us


def analyze_capture(raw: bytes, seconds: float,
                    st: Optional[GnssState] = None) -> Tuple[dict, GnssState]:
    """One-shot analysis of a captured byte blob."""
    st = st or GnssState()
    counters = new_counters()
    counters["bytes"] = len(raw)
    counters["seconds"] = seconds
    parts = raw.decode("ascii", "replace").split("\n")
    for l in parts[1:-1]:                   # drop partial edge lines
        process_line(l.rstrip("\r"), counters, st)
    return counters, st


def capture(port: str, baud: int, seconds: float) -> bytes:
    s = open_port(port, baud, timeout=0.3)
    time.sleep(0.2)
    s.reset_input_buffer()
    end = time.time() + seconds
    raw = b""
    while time.time() < end:
        raw += s.read(1024)
    s.close()
    return raw


# =============================================================================
# Receiver report rendering
# =============================================================================

RATE_ORDER = ["GGA", "RMC", "VTG", "GST", "GSA", "GSV"]


def _fmt_link(counters: dict, baud: int, capacity_known: bool) -> List[str]:
    secs = counters["seconds"] or 1.0
    bps = counters["bytes"] / secs
    if capacity_known and baud:
        cap = "%.1f kB/s = %.0f%% of %d" % (bps / 1000, bps / (baud / 10) * 100,
                                            baud)
    else:
        cap = "%.1f kB/s (native USB, capacity n/a)" % (bps / 1000)
    lines = ["Link:  %s, %d bad checksums, %d truncated"
             % (cap, counters["bad"], counters["trunc"])]
    if counters["types"]:
        order = ([t for t in RATE_ORDER if t in counters["types"]]
                 + sorted(t for t in counters["types"] if t not in RATE_ORDER))
        lines.append("Rates: " + "  ".join(
            "%s %.1f" % (t, counters["types"][t] / secs) for t in order)
            + " Hz")
    else:
        lines.append("Rates: no valid NMEA seen (wrong baud? output off?)")
    return lines


def render_report(st: GnssState, counters: dict, baud: int,
                  capacity_known: bool, header: Optional[str] = None,
                  max_age: Optional[float] = None,
                  raw_samples: bool = False) -> List[str]:
    """Build the satellite/fix report as a list of ASCII lines."""
    out = []
    if header:
        out.append(header)

    sats = st.in_view(max_age)
    total_view = sum(len(v) for v in sats.values())

    # --- fix line ------------------------------------------------------------
    if st.has_fix():
        # 2D/3D comes from GSA, the quality flag from GGA; show what we have.
        parts = []
        if st.fix_mode >= 2:
            parts.append(FIX_MODE[st.fix_mode])
        if st.fix_quality:
            parts.append(FIX_QUALITY.get(st.fix_quality, "?"))
        bits = ["Fix:  " + " / ".join(parts)]
        if st.sats_used is not None:
            bits.append("sats used %d" % st.sats_used)
        dops = []
        if st.pdop is not None:
            dops.append("PDOP %.1f" % st.pdop)
        if st.gsa_hdop is not None or st.hdop is not None:
            dops.append("HDOP %.1f" % (st.gsa_hdop if st.gsa_hdop is not None
                                       else st.hdop))
        if st.vdop is not None:
            dops.append("VDOP %.1f" % st.vdop)
        out.append("   ".join(bits) + ("   " + "  ".join(dops) if dops else ""))
        # --- position (only with a fix) --------------------------------------
        if st.lat is not None and st.lon is not None:
            pos = "Pos:  %.6f %s  %.6f %s" % (
                abs(st.lat), "N" if st.lat >= 0 else "S",
                abs(st.lon), "E" if st.lon >= 0 else "W")
            if st.alt is not None:
                pos += "   alt %.1f m" % st.alt
            if st.speed_kmh is not None:
                pos += "   speed %.1f km/h" % st.speed_kmh
            if st.course is not None:
                pos += "   course %.0f deg" % st.course
            out.append(pos)
        if st.utc_date and st.utc_time:
            d, t = st.utc_date, st.utc_time
            out.append("UTC:  20%s-%s-%s %s:%s:%s" %
                       (d[4:6], d[2:4], d[0:2], t[0:2], t[2:4], t[4:6]))
        if st.horiz_acc is not None:
            out.append("Est. accuracy: %.1f m horiz" % st.horiz_acc)
    else:
        out.append("Fix:  NONE - %d sats used, %d in view "
                   "(indoors? antenna not connected? "
                   "cold start needs ~30 s of open sky)"
                   % (st.sats_used or 0, total_view))

    # --- per-constellation table --------------------------------------------
    if sats:
        out.append("")
        out.append("%-15s %7s %6s   %s"
                   % ("Constellation", "in view", "used", "C/N0 avg / best"))
        tot_used = 0
        for const in CONST_ORDER:
            if const not in sats:
                continue
            prns = sats[const]
            used_set = st.used_for(const)
            n_used = len(set(prns) & used_set)
            tot_used += n_used
            cnos = [d["cno"] for d in prns.values() if d.get("cno") is not None]
            cno = ("%2d / %2d dBHz" % (sum(cnos) / len(cnos), max(cnos))
                   if cnos else "-- / -- dBHz")
            out.append("%-15s %7d %6d   %s" % (const, len(prns), n_used, cno))
        out.append("%-15s %7d %6d" % ("total", total_view, tot_used))

    # --- link statistics -----------------------------------------------------
    out.append("")
    out.extend(_fmt_link(counters, baud, capacity_known))
    if raw_samples and counters["raw_samples"]:
        out.append("Raw sample:")
        for l in counters["raw_samples"]:
            out.append("  " + l)
    return out


def render_cno_bars(st: GnssState, max_sats: int = 12,
                    max_age: Optional[float] = None) -> List[str]:
    """ASCII C/N0 bar chart of the strongest satellites."""
    rows = []
    for const, prns in st.in_view(max_age).items():
        used_set = st.used_for(const)
        for prn, d in prns.items():
            if d.get("cno") is not None:
                rows.append((d["cno"], CONST_LETTER.get(const, "?"), prn,
                             prn in used_set))
    rows.sort(reverse=True)
    out = []
    for cno, letter, prn, used in rows[:max_sats]:
        bar = "#" * max(0, min(20, int(round(cno / 55.0 * 20))))
        out.append("  %s%02d  %2d dBHz [%-20s]%s"
                   % (letter, prn, cno, bar, " used" if used else ""))
    return out


def make_header(port: str, baud: int, info: Optional[dict] = None,
                rate: Optional[int] = None) -> str:
    where = "%s @ %d" % (port, baud)
    if rate:
        where += ", %d Hz" % rate
    if info:
        gen = {8: "M8", 9: "M9", 10: "M10"}.get(info.get("gen"), "gen ?")
        return "Receiver: u-blox %s (%s, protver %s)  %s" % (
            gen, info.get("sw"), info.get("protver"), where)
    return "Port: %s" % where


def capture_and_report(port: str, baud: int, seconds: float,
                       capacity_known: bool, header: Optional[str] = None,
                       raw_samples: bool = False) -> GnssState:
    """One capture powering both link stats and the satellite report."""
    raw = capture(port, baud, seconds)
    counters, st = analyze_capture(raw, seconds)
    for line in render_report(st, counters, baud, capacity_known,
                              header=header, raw_samples=raw_samples):
        print(line)
    return st


# =============================================================================
# --check and --monitor modes
# =============================================================================

def _resolve_check_port(args) -> Optional[Tuple[str, int, bool, bool]]:
    """Pick (port, baud, capacity_known, signal_seen) for check/monitor."""
    if args.port:
        port = args.port
    else:
        cands = candidate_ports()
        if not cands:
            print("No serial ports found. Use --port.")
            return None
        port = cands[0]["device"]
        print("No --port given, using first candidate: %s" % port)
    kind = link_kind(port)
    if kind == "usb":
        return port, args.baud, False, True
    for b in dict.fromkeys(list(MON_VER_BAUDS) + [args.baud]):
        s = open_port(port, b)
        time.sleep(0.15)
        s.reset_input_buffer()
        got = raw_has_signal(s, dur=1.2)
        s.close()
        if got:
            return port, b, True, True
    return port, args.baud, True, False


def run_check_mode(args) -> int:
    """Standalone --check: one capture, one report."""
    resolved = _resolve_check_port(args)
    if resolved is None:
        return 1
    port, baud, capacity_known, signal = resolved
    if not signal:
        print("No data seen on %s at any common baud." % port)
        return 1
    seconds = args.check
    print("Checking %s at %d baud for %.1f s..." % (port, baud, seconds))
    print("")
    capture_and_report(port, baud, seconds, capacity_known,
                       header=make_header(port, baud),
                       raw_samples=args.raw)
    return 0


def _enable_vt() -> bool:
    """Enable ANSI escape handling. POSIX terminals: already on. Windows:
    try SetConsoleMode(ENABLE_VIRTUAL_TERMINAL_PROCESSING); an old console
    that refuses gets the plain-reprint fallback instead of escape soup."""
    if os.name != "nt":
        return True
    try:
        import ctypes
        k = ctypes.windll.kernel32
        h = k.GetStdHandle(-11)             # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if not k.GetConsoleMode(h, ctypes.byref(mode)):
            return False
        return bool(k.SetConsoleMode(h, mode.value | 0x0004))
    except Exception:
        return False


def run_monitor_mode(args) -> int:
    """Live satellite/fix view, ~1 frame per second, Ctrl-C to stop."""
    resolved = _resolve_check_port(args)
    if resolved is None:
        return 1
    port, baud, capacity_known, _signal = resolved
    duration = args.monitor          # 0 = until Ctrl-C
    use_ansi = sys.stdout.isatty() and _enable_vt()

    st = GnssState()
    frames = deque(maxlen=5)         # rolling window for link/rate stats
    header = make_header(port, baud)
    s = open_port(port, baud, timeout=0.2)
    tail = b""
    start = time.time()
    last_counters = new_counters()

    def window_counters() -> dict:
        agg = new_counters()
        for c in frames:
            agg["bytes"] += c["bytes"]
            agg["seconds"] += c["seconds"]
            agg["bad"] += c["bad"]
            agg["trunc"] += c["trunc"]
            for t, n in c["types"].items():
                agg["types"][t] = agg["types"].get(t, 0) + n
            agg["raw_samples"] = c["raw_samples"] or agg["raw_samples"]
        return agg

    def frame_lines(final: bool = False) -> List[str]:
        lines = render_report(st, last_counters, baud, capacity_known,
                              header=header, max_age=10.0,
                              raw_samples=args.raw and final)
        bars = render_cno_bars(st, max_age=10.0)
        if bars:
            lines += ["", "Strongest satellites:"] + bars
        if not final:
            lines += ["", "(Ctrl-C to stop)"]
        return lines

    try:
        while duration <= 0 or time.time() - start < duration:
            f_start = time.time()
            counters = new_counters()
            while time.time() - f_start < 1.0:
                chunk = s.read(512)
                if not chunk:
                    continue
                counters["bytes"] += len(chunk)
                tail += chunk
                lines = tail.split(b"\n")
                tail = lines.pop()
                for l in lines:
                    process_line(l.decode("ascii", "replace").rstrip("\r"),
                                 counters, st)
                if len(tail) > 4096:
                    tail = tail[-1024:]
            counters["seconds"] = time.time() - f_start
            frames.append(counters)
            last_counters = window_counters()
            body = "\n".join(frame_lines())
            if use_ansi:
                # home + print + clear rest of screen (less flicker than 2J)
                sys.stdout.write("\x1b[H" + body + "\x1b[0J\n")
            else:
                sys.stdout.write("\n---- %s ----\n%s\n"
                                 % (time.strftime("%H:%M:%S"), body))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    print("\n==== final summary ====")
    for line in frame_lines(final=True):
        print(line)
    return 0


# =============================================================================
# CLI
# =============================================================================

def print_plan(profile: Profile, gen=None) -> None:
    """Human-readable dump of every key/value group that would be sent."""
    systems, notes = adjust_systems(gen, profile.systems)
    prof = Profile(**{**profile.__dict__, "systems": systems})
    for note in notes:
        print("  ! %s" % note)
    if gen is None and "GLO" in systems and "BDS" in systems:
        print("  (note: on an M10, BeiDou would be dropped in favor of GLONASS)")
    print("  Save layers         : %s"
          % ("RAM + BBR + Flash" if prof.save else "RAM only"))
    print_load(prof)
    for name, cfg in build_config(prof):
        print("  [%s]" % name)
        for k, v in cfg:
            print("    %-34s = %s" % (k, v))


def ask_rate(default: int = 10) -> int:
    ans = input("\nUpdate rate in Hz, 1..25 [%d]: " % default).strip()
    if not ans:
        return default
    try:
        rate = int(ans)
        if 1 <= rate <= 25:
            return rate
    except ValueError:
        pass
    print("Invalid rate, using %d Hz." % default)
    return default


def ask_systems() -> List[str]:
    print("\nGNSS systems (M10 drops BeiDou automatically when GLONASS is on):")
    print("  [1] All: GPS+GLONASS+Galileo+BeiDou+QZSS+SBAS (default)")
    print("  [2] GPS+GLONASS+Galileo+QZSS+SBAS (no BeiDou)")
    print("  [3] Custom (comma list of: " + ",".join(ALL_SYSTEMS) + ")")
    ans = input("Choose 1/2/3 [1]: ").strip()
    if ans == "2":
        return ["GPS", "GLO", "GAL", "QZSS", "SBAS"]
    if ans == "3":
        raw = input("Systems: ").strip()
        return [x.strip().upper() for x in raw.split(",") if x.strip()]
    return list(ALL_SYSTEMS)


def resolve_ifaces(choice: str, kind: str) -> Tuple[List[str], bool]:
    """Map --iface + link kind to (ifaces_to_configure, uart_is_our_link).

    auto: native USB link -> configure BOTH usb and uart1 (uart1 settings are
    harmless over USB and useful for the other cable), no baud dance on our
    link; USB-UART bridge (or unknown) -> uart1 only, and it IS our link, so
    the baud-change-then-reconnect dance applies.
    """
    if choice == "auto":
        ifaces = ["uart1", "usb"] if kind == "usb" else ["uart1"]
    elif choice == "both":
        ifaces = ["uart1", "usb"]
    else:
        ifaces = [choice]
    uart_is_link = ("uart1" in ifaces) and kind != "usb"
    return ifaces, uart_is_link


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="ublox_setup",
        description="Configure u-blox GNSS receivers (M8/M9/M10) via CFG-VALSET.",
        epilog="After configuring, a 5 s capture prints a satellite/fix report "
               "(fix type, sats used/in view per constellation, DOPs, position, "
               "link stats). Standalone: --check for a one-shot report, "
               "--monitor for a live view refreshed every second "
               "(Ctrl-C to stop). Examples: "
               "'ublox_setup.py --rate 10 --yes' | "
               "'ublox_setup.py --check --port COM5' | "
               "'ublox_setup.py --monitor --port /dev/ttyUSB0'")
    ap.add_argument("--port", help="serial port (default: scan all USB ports)")
    ap.add_argument("--list", action="store_true",
                    help="list serial ports with classification and exit")
    ap.add_argument("--iface", choices=["auto", "uart1", "usb", "both"],
                    default="auto",
                    help="which receiver interface(s) to configure (default auto)")
    ap.add_argument("--rate", type=int, metavar="HZ",
                    help="update rate 1..25 Hz (default 10)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="target UART1 baud (default 115200)")
    ap.add_argument("--talker", choices=["gp", "gn"], default="gp",
                    help="NMEA main talker ID: gp = force $GPxxx for legacy "
                         "parsers; gn = receiver default (default gp)")
    ap.add_argument("--dynmodel", choices=sorted(DYNMODELS), default="automotive",
                    help="dynamic platform model (default automotive)")
    ap.add_argument("--min-elev", type=int, default=10, metavar="DEG",
                    help="minimum satellite elevation in degrees (default 10)")
    ap.add_argument("--systems", metavar="LIST",
                    help="comma list of: " + ",".join(ALL_SYSTEMS))
    ap.add_argument("--no-save", action="store_true",
                    help="apply to RAM only (reverts on power cycle)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the key/value groups that would be sent, "
                         "then exit without touching any port")
    ap.add_argument("--check", type=float, nargs="?", const=5.0, metavar="SEC",
                    help="capture the stream for SEC seconds (default 5), "
                         "print the satellite/fix report and exit; "
                         "no configuration is applied")
    ap.add_argument("--monitor", type=float, nargs="?", const=0.0,
                    metavar="SEC",
                    help="live satellite/fix view refreshed ~1x/sec until "
                         "Ctrl-C (or for SEC seconds); no configuration "
                         "is applied")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the automatic satellite/fix report after "
                         "configuring")
    ap.add_argument("--raw", action="store_true",
                    help="include a few raw NMEA sample lines in reports")
    ap.add_argument("--yes", action="store_true",
                    help="no confirmation / no interactive prompts")
    return ap


def _run(args) -> int:
    if args.list:
        return print_port_list()
    if args.monitor is not None:
        return run_monitor_mode(args)
    if args.check is not None:
        return run_check_mode(args)

    if args.rate is not None and not 1 <= args.rate <= 25:
        print("ERROR: --rate must be between 1 and 25 Hz.")
        return 1
    if args.min_elev < 0 or args.min_elev > 90:
        print("ERROR: --min-elev must be between 0 and 90 degrees.")
        return 1

    interactive = sys.stdin.isatty() and not args.yes and not args.dry_run
    rate = args.rate if args.rate is not None else \
        (ask_rate() if interactive else 10)
    if args.systems:
        systems = [x.strip().upper() for x in args.systems.split(",") if x.strip()]
    else:
        systems = ask_systems() if interactive else list(ALL_SYSTEMS)
    bad = [x for x in systems if x not in SYSTEM_KEYS]
    if bad:
        print("Unknown systems: %s (valid: %s)" % (bad, ",".join(ALL_SYSTEMS)))
        return 1

    base = Profile(rate=rate, systems=systems, talker=args.talker,
                   dynmodel=args.dynmodel, min_elev=args.min_elev,
                   baud=args.baud, save=not args.no_save)

    # --- dry run: show the plan, touch nothing --------------------------------
    if args.dry_run:
        kind = link_kind(args.port) if args.port else "unknown"
        if args.iface == "auto" and kind == "unknown":
            ifaces = ["uart1", "usb"]
            print("Dry run (no port opened). --iface auto without a known port:"
                  " showing BOTH uart1 and usb groups.")
        else:
            ifaces, _ = resolve_ifaces(args.iface, kind)
            print("Dry run (no port opened).")
        base.ifaces = ifaces
        print("Profile: rate=%dHz  iface=%s  talker=%s  dynmodel=%s  "
              "min-elev=%d  systems=%s"
              % (rate, "+".join(ifaces), args.talker.upper(),
                 args.dynmodel, args.min_elev, ",".join(systems)))
        print_plan(base)
        return 0

    # --- find receivers -------------------------------------------------------
    if args.port:
        ports = [{"device": args.port}]
    else:
        ports = candidate_ports()
        if not ports:
            print("No USB serial ports found. Use --list to inspect, "
                  "or --port to target a specific port.")
            return 1
    print("Scanning ports: %s" % ", ".join(p["device"] for p in ports))
    found = []
    for p in ports:
        dev = p["device"]
        info, baud = detect(dev)
        if info:
            print("  [%s] u-blox gen %s  hw=%s fw=%s protver=%s (link baud %d)"
                  % (dev, info["gen"], info["hw"], info["sw"],
                     info["protver"], baud))
            found.append((dev, baud, info))
        else:
            print("  [%s] no u-blox response" % dev)
    if not found:
        print("No receivers found.")
        return 1

    print("\nProfile: rate=%dHz  talker=%s  dynmodel=%s  min-elev=%d"
          % (rate, args.talker.upper(), args.dynmodel, args.min_elev))
    print("         systems=%s  save=%s"
          % (",".join(systems),
             "Flash+BBR+RAM" if base.save else "RAM only"))
    if interactive:
        if input("Apply to all found receivers? [y/N] ").strip().lower() != "y":
            print("Aborted.")
            return 0

    # --- configure each receiver ----------------------------------------------
    failures = 0
    for dev, baud, info in found:
        proceed, gen_notes = valset_supported(info)
        print("\n=== %s (gen %s) ===" % (dev, info["gen"]))
        for n in gen_notes:
            print("  ! %s" % n)
        if not proceed:
            print("  Skipped.")
            continue

        kind = link_kind(dev)
        ifaces, uart_is_link = resolve_ifaces(args.iface, kind)
        prof = Profile(**{**base.__dict__, "ifaces": ifaces})
        print("  Link type           : %s  ->  configuring: %s%s"
              % (kind, "+".join(ifaces),
                 "  (link baud will change)" if uart_is_link else ""))
        print_load(prof)

        r = configure(dev, baud, info["gen"], prof, uart_is_link)
        for note in r["notes"]:
            print("  ! %s" % note)
        print("  systems applied     : %s" % ",".join(r["systems"]))
        for name, ok in r["results"].items():
            tag = "OK" if ok else ("NAK" if ok is False else "no-resp")
            print("  %-20s: %s" % (name, tag))
        if r["readback"]:
            print("  readback:")
            for k, v in r["readback"].items():
                print("    %-34s = %s" % (k, v))
        for m in r["mismatches"]:
            print("  MISMATCH: %s" % m)
        print("  link baud now       : %d" % r["baud"])
        if not r["clean"]:
            failures += 1
            print("  RESULT: NOT CLEAN (see NAK/no-resp/mismatch above)")
        else:
            print("  RESULT: OK")

        if not args.no_check:
            print("")
            capture_and_report(
                dev, r["baud"], 5.0, capacity_known=(kind != "usb"),
                header=make_header(dev, r["baud"], info=info, rate=rate),
                raw_samples=args.raw)

    return 2 if failures else 0


def main() -> int:
    try:
        args = build_argparser().parse_args()
    except SystemExit as exc:
        # argparse exits 2 on bad usage; our contract reserves 2 for
        # "receiver failed to apply", so map usage errors to 1.
        return 0 if exc.code in (0, None) else 1
    try:
        return _run(args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else 1


if __name__ == "__main__":
    sys.exit(main())
