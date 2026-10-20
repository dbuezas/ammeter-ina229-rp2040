#!/usr/bin/env python3
"""Capture every conversion from the ammeter as a binary stream, and check that none were lost.

    python3 host/stream.py --seconds 30 --out runs/idle.bin
    python3 host/stream.py --seconds 30 --shuntonly --vshct 50

The firmware sends the CURRENT register as the ADC produced it, 3 bytes per conversion, with
no averaging. All scaling happens here. At 3.5 kHz this is about 10 kB/s.

Samples come in packets:

  magic | seq u16 | count u16 | t_us u32 | q_raw 5B | samples 3B*count | crc16

A gap in `seq` shows a lost packet. `q_raw` is the chip's charge accumulator, which counts
every conversion even when the firmware does not read it. If the charge change and the sum of
the received samples do not match, the firmware missed conversions.

The `--out` file is written while the capture runs, and the mean so far is printed every 30 s.
If the capture is killed, the file keeps everything up to that point.
"""

import argparse
import glob
import os
import signal
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")

MAGIC = b"\xa5\x5a"
HDR = 15


def crc16_ccitt(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def s24(b):
    """The CURRENT register: 24 bits big-endian, 20 bits of signed data in [23:4]."""
    v = (b[0] << 16) | (b[1] << 8) | b[2]
    v <<= 8
    if v & 0x80000000:
        v -= 1 << 32
    return v >> 12


def find_port(explicit=None):
    if explicit:
        return explicit
    p = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not p:
        sys.exit("no /dev/cu.usbmodem* -- board in bootloader, or not attached")
    if len(p) > 1:
        # Other USB serial devices use the same name, so pick the one with the RP2040's
        # vendor ID (Raspberry Pi, 0x2E8A).
        from serial.tools import list_ports
        pico = sorted(c.device for c in list_ports.comports() if c.vid == RP2040_VID and c.device in p)
        if len(pico) == 1:
            return pico[0]
        sys.exit(f"several ports: {p}; pick one with --port")
    return p[0]


RP2040_VID = 0x2E8A


PROGRESS_S = 30.0        # seconds between progress lines
SCAN_B = 65536           # how many bytes at each end to search for a packet. A limit, so that a
                         # progress line on a long run does not take so long that packets get lost.


def _edge_packet(buf, last):
    """The first (or last) valid packet within SCAN_B bytes of that end, or None.

    The mean so far is (q_last - q_first) / (t_last - t_first), the same way the final summary
    calculates it."""
    window = bytes(buf[:SCAN_B]) if not last else bytes(buf[-SCAN_B:])
    found = None
    for pkt in parse_packets(window):
        if not last:
            return pkt
        found = pkt
    return found


_tail = {"prev": None, "wraps": 0}


def report_progress(buf, lsb, elapsed, total_s):
    """Print one progress line with the mean current so far.

    The last packet is parsed alone, so its time has no wraps. They are counted here instead:
    this runs every 30 s, far less than the 71.6 minutes between wraps."""
    first, last = _edge_packet(buf, False), _edge_packet(buf, True)
    if last:
        raw = last[1] & 0xFFFFFFFF
        if _tail["prev"] is not None and raw < _tail["prev"] - (1 << 31):
            _tail["wraps"] += 1
        _tail["prev"] = raw
        last = (last[0], raw + (_tail["wraps"] << 32)) + tuple(last[2:])
    head = f"  {elapsed:5.0f}/{total_s:.0f} s  {len(buf)/1024:7.1f} kB"
    if not first or not last or last[1] == first[1]:
        print(head + "   (no complete packet pair yet)", flush=True)
        return
    dq = (last[2] - first[2]) * lsb
    dt = (last[1] - first[1]) / 1e6
    print(head + f"   mean so far {dq/dt*1e6:8.2f} uA", flush=True)


def parse_packets(buf):
    """Go through the byte stream and yield (seq, t_us, q_raw, samples) per packet. A packet
    with a bad CRC is skipped, and parsing continues at the next magic.

    The timestamp is a 32-bit microsecond counter, so it wraps after 71.6 minutes. It is
    unwrapped here, so callers get times that keep growing. The unwrap starts at the first
    packet in `buf`, so when you parse only the end of a buffer, times start from there.
    """
    i = 0
    prev_raw = None
    wraps = 0
    while True:
        j = buf.find(MAGIC, i)
        if j < 0 or j + HDR + 2 > len(buf):
            break
        seq, count = struct.unpack_from("<HH", buf, j + 2)
        t_us, = struct.unpack_from("<I", buf, j + 6)
        end = j + HDR + 3 * count
        if count == 0 or count > 4096 or end + 2 > len(buf):
            i = j + 2
            continue
        got = struct.unpack_from("<H", buf, end)[0]
        ok = got == crc16_ccitt(buf[j:end])
        if not ok:
            i = j + 2
            continue
        q = int.from_bytes(buf[j + 10:j + 15], "big", signed=False)
        if q >= 1 << 39:
            q -= 1 << 40
        # A step back of more than half the range is a wrap. Only packets with a good CRC
        # count, so a corrupt timestamp cannot add a false wrap.
        if prev_raw is not None and t_us < prev_raw - (1 << 31):
            wraps += 1
        prev_raw = t_us
        body = buf[j + HDR:end]
        yield seq, t_us + (wraps << 32), q, [s24(body[k:k + 3]) for k in range(0, len(body), 3)]
        i = end + 2


def _sigterm(_sig, _frm):
    """Make `kill` act like Ctrl-C, so the cleanup in main() runs and stops the meter.
    A meter left in binary mode breaks the next run."""
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _sigterm)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--out")
    ap.add_argument("--port")
    # Shunt and range are always sent to the meter, so a capture does not use whatever the
    # last run left behind. A wrong shunt value gives wrong numbers with no error.
    ap.add_argument("--shunt", type=float, default=1.0, help="shunt resistance in ohms")
    ap.add_argument("--range", dest="adc_range", type=int, choices=(0, 1), default=0,
                    help="0 = coarse (+/-163.84 mV), 1 = fine")
    # 280 us is the fastest conversion time with full resolution (see the README). Shorter
    # times keep the mean right, but the trace gets much coarser.
    ap.add_argument("--vshct", type=int, default=280)
    ap.add_argument("--shuntonly", action="store_true", default=True)
    ap.add_argument("--all-channels", dest="shuntonly", action="store_false",
                    help="also convert bus and temperature (slower shunt sampling)")
    args = ap.parse_args()

    ser = serial.Serial(find_port(args.port), 115200, timeout=0.1)
    ser.dtr = True
    time.sleep(1.0)
    ser.write(b"stop\n"); ser.flush(); time.sleep(0.5)
    ser.reset_input_buffer()

    hdr = []
    for c in (f"set shunt {args.shunt}", f"set range {args.adc_range}", f"set vshct {args.vshct}",
              f"set shuntonly {1 if args.shuntonly else 0}", "cfg"):
        ser.write((c + "\n").encode()); ser.flush(); time.sleep(0.35)
        hdr.append(ser.read(4096).decode("utf-8", "replace"))
    text = "".join(hdr)
    lsb = 3.90625e-7
    for ln in text.splitlines():
        if "current_lsb=" in ln:
            lsb = float(ln.split("current_lsb=")[1].split()[0])
    print("\n".join(l for l in text.splitlines() if l.startswith("#")))

    ser.write(b"set bin 1\n"); ser.flush(); time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"start\n"); ser.flush()

    # Open the file before the first byte arrives and flush every chunk, so a killed run
    # keeps everything it got. parse_packets ignores a cut-off last packet.
    out = None
    if args.out:
        d = os.path.dirname(os.path.abspath(args.out))
        if d:
            os.makedirs(d, exist_ok=True)
        out = open(args.out, "wb")
        out.write(b"#ina229-bin1 current_lsb=%.12g\n" % lsb)
        out.flush()

    buf = bytearray()
    t0 = time.time()
    next_report = t0 + PROGRESS_S
    cut_short = False
    try:
        while time.time() - t0 < args.seconds:
            chunk = ser.read(262144)
            if chunk:
                buf += chunk
                if out:
                    out.write(chunk); out.flush()
            if time.time() >= next_report:
                next_report += PROGRESS_S
                report_progress(buf, lsb, time.time() - t0, args.seconds)
        ser.write(b"stop\n"); ser.flush()
        time.sleep(0.5)
        tail = ser.read(262144)
        buf += tail
        if out and tail:
            out.write(tail); out.flush()
    except KeyboardInterrupt:
        # Ctrl-C or `kill`: keep what arrived and summarise it.
        cut_short = True
        print(f"\ninterrupted after {time.time() - t0:.1f} s -- summarising what arrived", flush=True)
    finally:
        # Always stop the meter. If it keeps streaming binary, the next run reads its own
        # setup commands back as garbage packets.
        try:
            ser.write(b"stop\n"); ser.flush(); time.sleep(0.2)
            ser.write(b"set bin 0\n"); ser.flush(); time.sleep(0.3)
        except Exception:                                    # noqa: BLE001  a closed port, not a fault
            pass
        if out:
            out.close()
        ser.close()

    pkts = list(parse_packets(bytes(buf)))
    if not pkts:
        sys.exit(f"no packets decoded from {len(buf)} bytes")

    total = sum(len(p[3]) for p in pkts)
    span = (pkts[-1][1] - pkts[0][1]) / 1e6
    seqs = [p[0] for p in pkts]
    gaps = sum((b - a - 1) % 65536 for a, b in zip(seqs, seqs[1:]))

    print(f"\nbytes {len(buf):,}  packets {len(pkts):,}  samples {total:,}")
    print(f"span {span:.3f} s   sample rate {total/span:,.0f} /s   "
          f"link {len(buf)/(time.time()-t0)/1024:.1f} kB/s")
    print(f"dropped packets (seq gaps): {gaps}")

    # Did the firmware read every conversion? The chip's charge counter has them all.
    dq = (pkts[-1][2] - pkts[0][2]) * lsb
    tdt = (pkts[-1][1] - pkts[0][1]) / 1e6
    got = sum(sum(p[3]) for p in pkts[:-1]) * lsb
    period = tdt / max(1, total - len(pkts[-1][3]))
    print(f"\nself-check against the chip's accumulator:")
    print(f"  charge from CHARGE register : {dq*1e6:10.3f} uC over {tdt:.3f} s "
          f"= {dq/tdt*1e6:8.3f} uA")
    print(f"  charge from the samples     : {got*period*1e6:10.3f} uC "
          f"= {got*period/tdt*1e6:8.3f} uA")
    # An absolute difference, not a ratio: near zero current a ratio is meaningless.
    a, b = dq / tdt, got * period / tdt
    d = abs(b - a)
    print(f"  difference: {d*1e6:.3f} uA")
    if d < 0.3e-6:
        print("  -> the two agree; every conversion is accounted for")
    else:
        print("  -> they DISAGREE. Either the loop missed conversions (the samples would"
              " then read LOW), or the two paths carry different offsets.")
        print("     Re-run against a real load: a constant difference is an offset,"
              " a proportional one is missed conversions.")

    if args.out:
        print(f"\nraw stream -> {args.out} ({len(buf):,} bytes, written as it arrived"
              f"{'; CUT SHORT' if cut_short else ''})")


def write_capture(path, buf, lsb, marks=()):
    """Save a complete stream with its scale and marks, for a caller that already has the
    whole buffer. main() does not use this; it writes while it captures.

    Marks are text lines at the top of the file, because the packets have no place for
    them. The plotter draws them as labelled lines."""
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"#ina229-bin1 current_lsb=%.12g\n" % lsb)
        for t, label in marks:
            f.write(b"#mark %.3f %s\n" % (t, str(label).encode()))
        f.write(bytes(buf))


if __name__ == "__main__":
    main()
