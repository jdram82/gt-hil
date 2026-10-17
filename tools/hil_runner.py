#!/usr/bin/env python3
"""
hil_runner.py - run the same JSON scenarios against the REAL-TIME rig.

The plant runs in plant_node (BeagleBone), the controller runs on the STM32 (or
in ctrl_node for the software-only rehearsal), and they talk Modbus RTU. This
runner plays operator and test supervisor: it talks to plant_node's local
command port, injects the events of the scenario, samples the state and judges
the same pass criteria as scenario_runner.py.

    # rehearsal on one machine: spawns a virtual serial cable, plant_node and ctrl_node
    python3 tools/hil_runner.py --spawn scenarios/startup_to_rated.json

    # real rig: plant_node is already running on the BeagleBone, STM32 is wired
    python3 tools/hil_runner.py --host 127.0.0.1 --cmd-port 5001 scenarios/*.json

Real time means a scenario takes as long as it says (plus the pre-roll), about
2 to 4 minutes each. Negative tests are NOT run here: removing a rule needs a
controller build with that rule disabled, which is done in the software tier.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import signal
import socket
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenario_runner as R                       # noqa: E402  (criteria, plots, reports)

ROOT = R.ROOT
SAMPLE_S = 0.05                                    # poll the plant at 20 Hz
RECORD_S = 0.10                                    # keep one sample in two
R.LOG_EVERY = 1                                    # the HIL data is already at 10 Hz

FAULT_MODE_NUM = R.FAULT_MODES


class Node:
    """Client of plant_node's command port."""

    def __init__(self, host, port, timeout=5.0):
        for _ in range(100):
            try:
                self.s = socket.create_connection((host, port), timeout=timeout)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError(f"cannot reach plant_node at {host}:{port}")
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.f = self.s.makefile("rw")
        n_doubles, rated = self.cmd("info").split()[1:3]
        if int(n_doubles) != len(R.OUT_FIELDS):
            raise RuntimeError("SimOut layout mismatch between plant_node and scenario_runner.py")
        self.rated_mw = float(rated)
        self.ramp = None

    def cmd(self, line):
        self.f.write(line + "\n")
        self.f.flush()
        return self.f.readline().strip()

    def get(self):
        parts = self.cmd("get").split()
        if parts[0] != "D":
            raise RuntimeError("bad reply to get: " + " ".join(parts[:3]))
        v = [float(x) for x in parts[1:]]
        return dict(zip(R.OUT_FIELDS, v))

    def apply(self, ev, t_now, events_after=()):
        a = ev["action"]
        if a == "start":            self.cmd("start 1")
        elif a == "stop":           self.cmd("start 0")
        elif a == "open_breaker":   self.cmd("breaker 0")
        elif a == "close_breaker":  self.cmd("breaker 1")
        elif a == "set_load":       self.cmd("load %.6f" % ev.get("mw", self.rated_mw * ev.get("pct", 0.0) / 100.0))
        elif a == "reset":          self.cmd("reset %d" % (1 if ev.get("value", 1) else 0))
        elif a == "droop":          self.cmd("droop %d" % (1 if ev.get("value", 1) else 0))
        elif a == "permissive":     self.cmd("perm %s %d" % (ev["name"], 1 if ev["ok"] else 0))
        elif a == "speed_fault":    self.cmd("sfault %d %d %g %g" % (ev["ch"], FAULT_MODE_NUM[ev["mode"]], ev.get("a", 0.0), ev.get("b", 0.0)))
        elif a == "tc_fault":       self.cmd("tfault %d %d %g %g" % (ev["ch"], FAULT_MODE_NUM[ev["mode"]], ev.get("a", 0.0), ev.get("b", 0.0)))
        elif a == "clear_faults":   self.cmd("clear")
        elif a == "ambient_ramp":   self.ramp = (t_now, t_now + ev["over_s"], 288.15, 288.15 + ev["delta_k"])
        elif a == "link":
            if not ev["up"]:
                # timed drop, exact to one plant step: find the matching link-up event
                t_up = next((e["t"] for e in events_after if e["action"] == "link" and e["up"]), None)
                if t_up is None:
                    self.cmd("link 0")
                else:
                    self.cmd("linkdrop %g" % ((t_up - ev["t"]) * 1000.0))
            # link-up events are implied by the timed drop; an explicit one with no drop is harmless
            elif ev.get("_explicit"):
                self.cmd("link 1")
        else:
            raise ValueError(f"unknown action {a}")

    def step_ambient(self, t_now):
        if self.ramp:
            t0, t1, k0, k1 = self.ramp
            f = min(1.0, max(0.0, (t_now - t0) / (t1 - t0)))
            self.cmd("ambient %.4f" % (k0 + (k1 - k0) * f))


def pace(period):
    """Generator yielding at a fixed wall-clock period without drift."""
    t = time.monotonic()
    while True:
        t += period
        d = t - time.monotonic()
        if d > 0:
            time.sleep(d)
        yield


def prepare(node, sc):
    """Fresh plant, controller back to IDLE with its latched flags cleared."""
    node.cmd("reset_sim %d %d" % (int(sc.get("seed", 1)), 1 if sc.get("noise", True) else 0))
    node.cmd("start 0")
    node.cmd("reset 0")
    time.sleep(0.3)
    node.cmd("reset 1")
    time.sleep(0.4)
    node.cmd("reset 0")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0:
        o = node.get()
        if int(o["ctrl_state"]) == 0 and o["tripped"] < 0.5:
            return
        time.sleep(0.1)
    raise RuntimeError("controller did not return to IDLE after reset (is the controller running?)")


def preroll(node, spec, log):
    log("preroll: starting the unit")
    node.cmd("start 1")
    t_band = 0.0
    t_begin = time.monotonic()
    for _ in pace(SAMPLE_S):
        o = node.get()
        if time.monotonic() - t_begin > 400.0:
            raise RuntimeError("preroll failed: unit did not reach RUN at rated speed in 400 s")
        if int(o["ctrl_state"]) == R.CS_RUN and abs(o["speed_rpm"] - R.RATED_RPM) < 25.0:
            t_band += SAMPLE_S
        else:
            t_band = 0.0
        if t_band >= 5.0:
            break
    log("preroll: at speed, closing breaker and loading")
    target = node.rated_mw * spec.get("load_pct", 0.0) / 100.0
    node.cmd("breaker 1")
    ramp_s, settle = 10.0, spec.get("settle_s", 30.0)
    n = int((ramp_s + settle) / SAMPLE_S)
    for i, _ in zip(range(n), pace(SAMPLE_S)):
        node.cmd("load %.6f" % (target * min(1.0, i * SAMPLE_S / ramp_s)))


def run(sc, node, log):
    prepare(node, sc)
    if sc.get("preroll"):
        preroll(node, sc["preroll"], log)
    t0 = node.get()["time_s"]
    events = sorted(sc.get("events", []), key=lambda e: e["t"])
    for i, e in enumerate(events):
        e["_i"] = i
    done = set()
    fired, rows, last_rec = [], [], -1.0
    log("scenario: running %.0f s in real time" % sc["duration_s"])
    for _ in pace(SAMPLE_S):
        o = node.get()
        t = o["time_s"] - t0
        for i, ev in enumerate(events):
            if ev["_i"] in done:
                continue
            if t >= ev["t"] - 1e-9:
                later = [e for e in events if e["t"] > ev["t"]]
                node.apply(ev, t, later)
                done.add(ev["_i"])
                if ev["action"] == "link" and not ev["up"]:        # the matching "up" is consumed
                    up = next((e for e in later if e["action"] == "link" and e["up"]), None)
                    if up is not None:
                        done.add(up["_i"])
                fired.append({"t": ev["t"], "action": ev["action"],
                              "detail": {k: v for k, v in ev.items() if k not in ("t", "action", "_i")}})
        node.step_ambient(t)
        if t - last_rec >= RECORD_S - 1e-9:
            last_rec = t
            rows.append([o[n] for n in R.OUT_FIELDS])
        if t >= sc["duration_s"]:
            break
    data = np.array(rows)
    data[:, 0] -= t0
    return {n: data[:, i] for i, n in enumerate(R.OUT_FIELDS)}, fired


class Rig:
    """Spawns vserial + plant_node + ctrl_node for the software rehearsal."""

    def __init__(self, cmd_port, seed_noise_args=()):
        self.procs, self.cmd_port = [], cmd_port
        self.tmp = tempfile.mkdtemp(prefix="gthil_")
        state = os.path.join(self.tmp, "ports")
        self.vs = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "vserial.py"), "--state", state],
                                   stdout=subprocess.DEVNULL)
        for _ in range(100):
            if os.path.exists(state) and open(state).read().strip():
                break
            time.sleep(0.05)
        a, b = open(state).read().split()
        self.plant = subprocess.Popen([os.path.join(ROOT, "build", "plant_node"), "--port", a,
                                       "--cmd-port", str(cmd_port)] + list(seed_noise_args),
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        time.sleep(0.4)
        self.ctrl = subprocess.Popen([os.path.join(ROOT, "build", "ctrl_node"), "--port", b],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def stop(self):
        out = {}
        for name, p in (("ctrl_node", self.ctrl), ("plant_node", self.plant)):
            p.send_signal(signal.SIGTERM)
            try:
                txt, _ = p.communicate(timeout=5)
                out[name] = json.loads(txt.strip().splitlines()[-1])
            except Exception:
                p.kill()
                out[name] = None
        self.vs.terminate()
        return out


def run_one(path, args, out_root):
    raw = open(path, "rb").read()
    sc = json.loads(raw)
    sha = hashlib.sha256(raw).hexdigest()[:12]
    t_start = time.monotonic()

    def log(msg):
        print(f"[{sc['id']} +{time.monotonic() - t_start:5.0f}s] {msg}", flush=True)

    rig = Rig(args.cmd_port) if args.spawn else None
    try:
        node = Node(args.host, args.cmd_port)
        node.cmd("clear_timing")
        data, fired = run(sc, node, log)
        timing = json.loads(node.cmd("timing"))
    finally:
        node_stats = rig.stop() if rig else {}

    hil = sc.get("hil", {})
    skipped = hil.get("skip", {})
    crit = []
    for c in sc["criteria"]:
        if c["id"] in skipped:
            continue
        crit.append(hil.get("replace", {}).get(c["id"], c))
    for rid, c in hil.get("replace", {}).items():          # replacements that add a new criterion
        if rid not in [x["id"] for x in sc["criteria"]]:
            crit.append(c)
    results = [R.evaluate(c, data) for c in crit]
    for r in results:
        r["passed"] = bool(r["passed"])
    mets = R.metrics(data)
    passed = all(r["passed"] for r in results)

    meta = {
        "mode": "real-time HIL" + (" (software rehearsal: virtual serial, controller on Linux)" if args.spawn
                                   else " (plant_node on the BeagleBone, controller on the STM32)"),
        "scenario file": os.path.relpath(path, ROOT),
        "scenario sha256 (first 12)": sha,
        "run at (UTC)": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
        "seed": sc.get("seed", 1),
        "sensor noise": sc.get("noise", True),
        "rated electrical power (MW)": f"{node.rated_mw:.2f}",
        "python": platform.python_version(),
        "host": platform.platform(),
        "negative tests": "not run in real time (see the software tier)",
    }
    if skipped:
        meta["criteria not checked in HIL"] = "; ".join(f"{k}: {v}" for k, v in skipped.items())
    if hil.get("note"):
        meta["HIL note"] = hil["note"]
    out = os.path.join(out_root, sc["id"])
    os.makedirs(out, exist_ok=True)
    R.write_csv(os.path.join(out, "data.csv"), data, meta)
    R.plot(os.path.join(out, "plot.png"), sc, data, fired, "  [real-time HIL]")
    R.write_report(os.path.join(out, "report.md"), sc, results, mets, fired, meta, passed)
    with open(os.path.join(out, "report.json"), "w") as f:
        json.dump({"scenario": sc["id"], "name": sc["name"], "passed": passed, "criteria": results,
                   "metrics": mets, "run": meta, "timing": timing, "controller_node": node_stats.get("ctrl_node"),
                   "plant_node_exit": node_stats.get("plant_node")}, f, indent=2,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(f"\n{'PASS' if passed else 'FAIL'}  {sc['name']}  (real-time HIL)")
    for r in results:
        print(f"   {'ok  ' if r['passed'] else 'FAIL'} {r['text']}  [{r['measured']}; limit {r['limit']}]")
    return {"id": sc["id"], "name": sc["name"], "passed": passed, "timing": timing}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="+")
    ap.add_argument("--spawn", action="store_true", help="start vserial, plant_node and ctrl_node (software rehearsal)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--cmd-port", type=int, default=5001)
    ap.add_argument("--out", default=os.path.join(ROOT, "results_hil"))
    args = ap.parse_args()
    summary = [run_one(p, args, args.out) for p in args.scenarios]
    n_ok = sum(1 for s in summary if s["passed"])
    os.makedirs(args.out, exist_ok=True)
    lines = ["# Real-time HIL scenario summary\n", f"{n_ok} of {len(summary)} scenarios passed.\n",
             "| Result | Scenario | Report |", "| --- | --- | --- |"]
    for s in summary:
        lines.append(f"| {'PASS' if s['passed'] else 'FAIL'} | {s['name']} | [{s['id']}/report.md]({s['id']}/report.md) |")
    with open(os.path.join(args.out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n=== {n_ok} of {len(summary)} scenarios passed (real-time HIL) ===")
    sys.exit(0 if n_ok == len(summary) else 1)


if __name__ == "__main__":
    main()
