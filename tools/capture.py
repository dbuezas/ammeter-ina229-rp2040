#!/usr/bin/env python3
"""Read ammeter captures and prepare them for plotting.

`plot_server.py` serves these functions together with the viewer, `plot.html`. Another
server can import them and serve the same page.

A capture holds two kinds of data:

  * CURRENT -- the sampled current readings. Fine resolution, so they show the shape of
    the current. They only cover the conversions the firmware read.
  * CHARGE -- the chip's own counter. One count is one CURRENT_LSB in coulombs (312.5 nC
    at 1 ohm, coarse), too coarse for a trace over a few milliseconds. But it never misses
    a conversion, so it is exact for totals.

So the plot is drawn from CURRENT, and totals come from CHARGE.
"""

import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# Only runs/, not /tmp: the OS cleans /tmp, and it is full of other files.
DEFAULT_DIRS = [os.path.join(HERE, "..", "runs")]

# The packet parser lives in host/stream.py, the tool that writes the files.
sys.path.insert(0, os.path.join(HERE, "..", "host"))


def parse_binary(path):
    """A '.bin' capture from host/stream.py: every conversion.

    Only packets have timestamps, so a packet's samples are spread evenly up to the next
    packet. The ADC runs at a fixed rate, so this is right. Every conversion is here, so
    no separate peak series is needed."""
    from stream import parse_packets

    raw = open(path, "rb").read()
    lsb = 3.90625e-7
    marks = []
    # The file starts with '#' text lines: the scale, then any marks.
    while raw.startswith(b"#"):
        nl = raw.find(b"\n")
        if nl < 0:
            break
        line = raw[:nl]
        m = re.search(rb"current_lsb=([0-9.eE+-]+)", line)
        if m:
            lsb = float(m.group(1))
        m = re.match(rb"#mark\s+([0-9.]+)\s+(.*)", line)
        if m:
            marks.append({"t": float(m.group(1)),
                          "label": m.group(2).decode("utf-8", "replace").strip()})
        raw = raw[nl + 1:]

    pk = list(parse_packets(raw))
    if not pk:
        return None

    t, i, q, qt = [], [], [], []
    for k, (_seq, t_us, q_raw, samples) in enumerate(pk):
        nxt = pk[k + 1][1] if k + 1 < len(pk) else t_us + (t_us - pk[k - 1][1] if k else 0)
        step = (nxt - t_us) / max(1, len(samples))
        for j, v in enumerate(samples):
            t.append((t_us + j * step) / 1e6)
            i.append(v * lsb)
        q.append(q_raw * lsb)
        qt.append(t_us / 1e6)
    return {
        "file": os.path.basename(path), "lsb": lsb,
        "t": t, "i": i, "imax": i,
        "q": q, "qt": qt,
        "sat": 0, "marks": marks, "n": len(t),
    }


def parse_capture(path):
    """Read a compact ('Q,') or full ('S,') capture into arrays.

    Returns t (seconds), i (amps, from the CURRENT register), imax (amps), q (charge in
    coulombs) with its own qt, the marks, and the current_lsb it was recorded at."""
    if path.endswith(".bin"):
        return parse_binary(path)
    lsb = None
    t_us, q_raw, imax_raw, sat = [], [], [], []
    i_mean = []
    marks = []
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln:
            continue
        if "current_lsb=" in ln:
            m = re.search(r"current_lsb=([0-9.eE+-]+)", ln)
            if m:
                lsb = float(m.group(1))
            continue
        p = ln.split(",")
        if ln.startswith("Q,") and len(p) == 7:
            # t_us, q_raw, i_sum, n, imax, sat
            try:
                t_us.append(int(p[1])); q_raw.append(int(p[2]))
                n = int(p[4])
                i_mean.append(int(p[3]) / n if n else 0.0)
                imax_raw.append(int(p[5])); sat.append(int(p[6]))
            except (ValueError, ZeroDivisionError):
                pass
        elif ln.startswith("Q,") and len(p) == 5:      # older captures, no CURRENT sum
            try:
                t_us.append(int(p[1])); q_raw.append(int(p[2]))
                i_mean.append(None)
                imax_raw.append(int(p[3])); sat.append(int(p[4]))
            except ValueError:
                pass
        elif ln.startswith("S,") and len(p) == 14:
            try:
                t_us.append(int(p[1]) * 1000); q_raw.append(int(p[4]))
                i_mean.append(float(p[3]) / (lsb or 3.90625e-7))
                imax_raw.append(int(round(float(p[8]) / (lsb or 3.90625e-7))))
                sat.append(int(p[10]))
            except ValueError:
                pass
        elif ln.startswith("M,") and len(p) >= 3:
            try:
                marks.append({"t": int(p[1]) / 1000.0, "label": ln.split(",", 2)[2]})
            except ValueError:
                pass

    L = lsb or 3.90625e-7
    t, i, mx = [], [], []
    for k in range(1, len(t_us)):
        dt = (t_us[k] - t_us[k - 1]) / 1e6
        if dt <= 0:
            continue
        t.append(t_us[k] / 1e6)
        # Use CURRENT. Older captures do not have it, so fall back to the charge difference.
        v = i_mean[k] if k < len(i_mean) and i_mean[k] is not None else None
        i.append(v * L if v is not None else (q_raw[k] - q_raw[k - 1]) * L / dt)
        mx.append(imax_raw[k] * L)
    return {
        "file": os.path.basename(path), "lsb": L,
        "t": t, "i": i, "imax": mx,
        "q": [v * L for v in q_raw[1:]],
        "qt": [v / 1e6 for v in t_us[1:]],
        "sat": sum(sat), "marks": marks,
        "n": len(t),
    }


_CACHE = {}


def load_cached(path):
    """Parse a capture once and keep it. The key includes the file's change time, so a
    capture that is still being written is read again."""
    key = (path, os.path.getmtime(path))
    if key not in _CACHE:
        _CACHE.clear()                     # one capture at a time; these are large
        _CACHE[key] = parse_capture(path)
    return _CACHE[key]


def decimate(d, t0, t1, n):
    """Reduce a time range to at most n buckets, each with its mean and its peak.

    The peak is the largest `i_max` in the bucket, and `i_max` is the largest single
    sample in a row, so short spikes still show.

    The stats come from the charge counter at both ends of the range, so they are exact
    however few points are drawn."""
    t, i, mx = d["t"], d["i"], d["imax"]
    lo, hi = 0, len(t) - 1
    while lo < hi and t[lo] < t0:
        lo += 1
    while hi > lo and t[hi] > t1:
        hi -= 1
    step = max(1e-9, t[hi] - t[lo]) / max(1, n)
    ot, oi, op = [], [], []
    bpk, bsum, bn = mx[lo], 0.0, 0
    b_end = t[lo] + step
    for k in range(lo, hi + 1):
        if mx[k] > bpk:
            bpk = mx[k]
        bsum += i[k]
        bn += 1
        if t[k] >= b_end or k == hi:
            ot.append(t[k]); oi.append(bsum / bn); op.append(bpk)
            bpk, bsum, bn = mx[k], 0.0, 0
            b_end = t[k] + step

    qt, q = d["qt"], d["q"]
    ia = next((k for k in range(len(qt)) if qt[k] >= t0), None)
    ib = next((k for k in range(len(qt) - 1, -1, -1) if qt[k] <= t1), None)
    stats = None
    if ia is not None and ib is not None and ib > ia:
        dq, dt = q[ib] - q[ia], qt[ib] - qt[ia]
        stats = {"charge": dq, "dt": dt, "mean": dq / dt if dt else 0,
                 "peak": max(mx[lo:hi + 1]) if hi >= lo else 0,
                 "n": hi - lo + 1}
    return {"t": ot, "i": oi, "imax": op, "stats": stats}


def list_captures(dirs):
    """Every capture file in dirs, newest first."""
    out = []
    for d in dirs:
        for pat in ("*.txt", "*.csv", "*.bin"):
            for f in glob.glob(os.path.join(d, pat)):
                try:
                    with open(f, "rb") as fh:
                        head = fh.read(4096)
                except OSError:
                    continue
                if head.startswith(b"#ina229-bin"):
                    out.append((os.path.getmtime(f), f))
                    continue
                txt = head.decode("utf-8", "replace")
                if "ina229" in txt or "\nQ," in txt or txt.startswith("Q,"):
                    out.append((os.path.getmtime(f), f))
    out.sort(reverse=True)
    return [f for _, f in out]


def read_meta(path):
    """Scale factor and marks from the text lines at the top of a binary capture. The
    browser decodes the packets itself, which is faster than doing it here."""
    lsb, marks, off = 3.90625e-7, [], 0
    with open(path, "rb") as f:
        head = f.read(8192)
    while off < len(head) and head[off:off + 1] == b"#":
        nl = head.find(b"\n", off)
        if nl < 0:
            break
        line = head[off:nl]
        m = re.search(rb"current_lsb=([0-9.eE+-]+)", line)
        if m:
            lsb = float(m.group(1))
        m = re.match(rb"#mark\s+([0-9.]+)\s+(.*)", line)
        if m:
            marks.append({"t": float(m.group(1)),
                          "label": m.group(2).decode("utf-8", "replace").strip()})
        off = nl + 1
    return {"lsb": lsb, "marks": marks, "offset": off,
            "bytes": os.path.getsize(path) - off}


def pick_capture(url, dirs=None, files=None):
    """The capture named by the request's `f=`, or the newest one. Only a listed capture is
    accepted, so the URL cannot be used to read other files."""
    from urllib.parse import parse_qs, urlparse
    files = files if files is not None else list_captures(dirs or DEFAULT_DIRS)
    want = parse_qs(urlparse(url).query).get("f", [None])[0]
    return want if want in files else (files[0] if files else None)


def serve_meta(url, dirs=None):
    """Everything the page needs before it fetches the bytes."""
    files = list_captures(dirs or DEFAULT_DIRS)
    path = pick_capture(url, dirs, files)
    if not path:
        return {"error": "no captures found"}
    if not path.endswith(".bin"):
        return {"binary": False, "path": path, "file": os.path.basename(path),
                "files": files}
    m = read_meta(path)
    m.update({"binary": True, "path": path, "file": os.path.basename(path),
              "files": files})
    return m


def raw_bytes(path):
    """The packet stream with its text preamble stripped."""
    return open(path, "rb").read()[read_meta(path)["offset"]:]


def serve_data(url, dirs=None):
    """Answer a /data request: pick the capture, reduce the range, add the stats."""
    from urllib.parse import parse_qs, urlparse
    qs = parse_qs(urlparse(url).query)
    files = list_captures(dirs or DEFAULT_DIRS)
    path = pick_capture(url, dirs, files)
    if not path:
        return {"error": "no captures found"}
    d = load_cached(path)
    if not d["t"]:
        return {"error": f"no samples in {os.path.basename(path)}"}
    full0, full1 = d["t"][0], d["t"][-1]
    t0 = max(full0, min(float(qs.get("t0", [full0])[0]), full1))
    t1 = max(t0 + 1e-6, min(float(qs.get("t1", [full1])[0]), full1))
    n = max(200, min(60000, int(qs.get("n", [3000])[0])))
    return {"file": os.path.basename(path), "path": path, "files": files,
            "lsb": d["lsb"], "n": d["n"], "sat": d["sat"], "marks": d["marks"],
            "full": [full0, full1], "range": [t0, t1], **decimate(d, t0, t1, n)}
