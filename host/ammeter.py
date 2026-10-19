#!/usr/bin/env python3
"""Serial client for the ammeter: configure it, log CSV rows, summarise a run.

Use it from another script to measure something: start the meter, do the thing, add
marks, and read the total charge at the end.

    # one-off: is it wired up?
    python3 host/ammeter.py id

    # log ten minutes
    python3 host/ammeter.py log --seconds 600 --out runs/idle.csv

    # from another tool, bracketing whatever it is measuring
    from ammeter import Ammeter
    with Ammeter(out="runs/ab.csv") as amm:
        amm.mark("idle")
        time.sleep(120)
        amm.mark("radio on")
        ...
    print(amm.summary)

Take totals from the summary's charge, not from the mean of the rows. The chip counts
every conversion, so the charge is right even when rows were lost.
"""

import argparse
import glob
import os
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")


def find_port(explicit=None):
    """The meter's serial port. If there are several, ask for --port instead of guessing."""
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* -- is the board still in bootloader mode? "
                 "(a board in BOOTSEL is a disk, not a serial port)")
    if len(ports) > 1:
        sys.exit(f"several CDC ports: {ports}\npick one with --port")
    return ports[0]


class Ammeter:
    """A configured, streaming meter. Rows are appended to `out` as they arrive."""

    # Default: 1 ohm shunt, coarse range. A wrong shunt value gives wrong numbers with no error.
    def __init__(self, out=True, port=None, shunt=1.0, adc_range=0, avg=1,
                 vshct=1052, vbusct=150, rate=10.0, quiet=True):
        # Every line is logged and flushed as it arrives, so the per-row peaks can be
        # checked later. `out=True` picks a timestamped file in runs/.
        if out is True:
            d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
            os.makedirs(d, exist_ok=True)
            out = os.path.join(d, time.strftime("ammeter-%Y%m%d-%H%M%S.csv"))
        self.out_path = out
        self.port_name = find_port(port)
        self.ser = serial.Serial(self.port_name, 115200, timeout=0.2)
        # Throw away old output from the board, or a partial line in it causes an
        # "unknown command" error on the first exchange.
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.out = open(out, "w") if out else None
        self.rows = []
        self.summary = {}
        self.quiet = quiet
        self._thread = None          # the reader, alive only between start() and stop()
        self._stop_reader = True
        self._cfg = dict(shunt=shunt, adc_range=adc_range, avg=avg,
                         vshct=vshct, vbusct=vbusct, rate=rate)

    # -- low level ------------------------------------------------------
    def send(self, line):
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def drain(self, seconds=0.3):
        """Collect whatever the board says for a moment, routing rows to the log."""
        got, deadline = [], time.time() + seconds
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip()
            if not line:
                continue
            got.append(line)
            self._absorb(line)
        return got

    def _absorb(self, line):
        if self.out:
            self.out.write(line + "\n")
            self.out.flush()
        if line.startswith("S,"):
            self.rows.append(line)
        elif not self.quiet:
            print(line)

    # -- lifecycle ------------------------------------------------------
    def configure(self):
        c = self._cfg
        self.send("stop")
        self.drain(0.2)
        for cmd in (f"set shunt {c['shunt']}", f"set range {c['adc_range']}",
                    f"set avg {c['avg']}", f"set vshct {c['vshct']}",
                    f"set vbusct {c['vbusct']}", f"set rate {c['rate']}"):
            self.send(cmd)
            self.drain(0.15)

    def check_id(self):
        self.send("id")
        for line in self.drain(0.5):
            if line.startswith("# id "):
                return line
        return "# id: no reply from the board"

    def mark(self, text):
        self.send(f"mark {text}")

    def _reader(self):
        """Read the port all the time between start() and stop(), on its own thread.

        Without it, a caller that sleeps or runs something else inside `with Ammeter(...)`
        stops reading, the serial buffer overflows, and almost all rows and marks are lost
        with no error.
        """
        while not self._stop_reader:
            try:
                raw = self.ser.readline()
            except Exception:
                return
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip()
            if line:
                self._absorb(line)

    def start(self):
        self.send("start")
        self.drain(0.3)
        # Start the thread after drain() and stop it before the next drain(). If both read the
        # port at the same time, each gets only some of the lines.
        self._stop_reader = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_reader = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.send("stop")
        tail = self.drain(0.6)
        for line in tail:
            if line.startswith("# stat "):
                self.summary = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        return self.summary

    def run_for(self, seconds, on_tick=None):
        end = time.time() + seconds
        while time.time() < end:
            self.drain(0.5)
            if on_tick:
                on_tick(self)

    def close(self):
        try:
            self.ser.close()
        finally:
            if self.out:
                self.out.close()

    def __enter__(self):
        self.configure()
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        self.close()
        return False


def fmt_summary(s):
    if not s:
        return "no summary line came back"
    secs = float(s.get("elapsed", 0))
    q_c = float(s.get("charge", 0))
    e_j = float(s.get("energy", 0))
    mean = float(s.get("mean_current", 0))
    return (f"  elapsed      {secs:.1f} s\n"
            f"  mean current {mean * 1e6:.2f} uA        <- charge / elapsed\n"
            f"  charge       {q_c:.4f} C = {q_c / 3.6:.4f} mAh\n"
            f"  energy       {e_j:.4f} J = {e_j / 3.6:.4f} mWh\n"
            f"  flags        {s.get('flags', '?')}   (0000 = no accumulator or math overflow)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["id", "log", "shell"])
    ap.add_argument("--port")
    ap.add_argument("--out", default=True,
                    help="CSV file to write rows to (default: a timestamped file in runs/)")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--shunt", type=float, default=1.0, help="shunt resistance in ohms")
    ap.add_argument("--range", dest="adc_range", type=int, default=0, choices=[0, 1],
                    help="0 = +/-163.84 mV (164 mA full scale at 1 ohm), 1 = 4x finer")
    ap.add_argument("--avg", type=int, default=1, help="hardware averages per conversion")
    ap.add_argument("--vshct", type=int, default=1052, help="shunt conversion time, us")
    ap.add_argument("--vbusct", type=int, default=150, help="bus conversion time, us")
    ap.add_argument("--rate", type=float, default=10.0, help="CSV rows per second")
    ap.add_argument("--mark", action="append", default=[],
                    help="SECONDS:TEXT -- drop a mark that many seconds into the run")
    args = ap.parse_args()

    amm = Ammeter(out=args.out, port=args.port, shunt=args.shunt,
                  adc_range=args.adc_range, avg=args.avg, vshct=args.vshct,
                  vbusct=args.vbusct, rate=args.rate, quiet=(args.cmd == "log"))
    print(f"# port {amm.port_name}")

    if args.cmd == "id":
        amm.drain(0.4)
        print(amm.check_id())
        amm.close()
        return

    if args.cmd == "shell":
        amm.quiet = False
        amm.configure()
        print("# type firmware commands; ctrl-c to leave")
        try:
            import threading
            threading.Thread(target=lambda: [amm.drain(0.5) for _ in iter(int, 1)],
                             daemon=True).start()
            for line in sys.stdin:
                amm.send(line.rstrip())
        except KeyboardInterrupt:
            pass
        amm.close()
        return

    # log
    amm.configure()
    print(amm.check_id())
    pending = sorted((float(m.split(":", 1)[0]), m.split(":", 1)[1]) for m in args.mark)
    amm.start()
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            amm.drain(0.5)
            while pending and time.time() - t0 >= pending[0][0]:
                when, text = pending.pop(0)
                amm.mark(text)
                print(f"# mark @{when:.0f}s: {text}")
            n = len(amm.rows)
            print(f"\r# {time.time() - t0:6.1f}s  {n} rows", end="", flush=True)
    except KeyboardInterrupt:
        print("\n# interrupted")
    print()
    print(fmt_summary(amm.stop()))
    print(f"# {len(amm.rows)} sample rows -> {amm.out_path}")
    amm.close()


if __name__ == "__main__":
    main()
