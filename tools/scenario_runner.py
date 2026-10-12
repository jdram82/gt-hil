#!/usr/bin/env python3
"""
scenario_runner.py - run JSON scenarios against the plant + controller and
write evidence: report (Markdown + JSON), data (CSV) and a plot (PNG).

    python3 tools/scenario_runner.py scenarios/*.json
    python3 tools/scenario_runner.py scenarios/load_rejection.json --out results

Exit code 0 only if every criterion of every scenario passes.

The runner talks to the C library (build/libgtsim.so) through ctypes.  It
never re-implements physics: it only sets inputs, steps the simulation and
judges the recorded signals.  That is the same split the real rig will have
(runner on the host, plant on the BeagleBone Black).
"""
import argparse
import ctypes
import csv
import datetime as dt
import hashlib
import json
import os
import platform
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_PATH = os.path.join(ROOT, "build", "libgtsim.so")
STEP_S = 0.010                      # must match HIL_STEP_S in include/hil_constants.h
RATED_RPM = 3500.0
LOG_EVERY = 10                      # CSV / plot at 10 Hz

# Order must match struct SimOut in plant/sim.h (the library self-checks the count).
OUT_FIELDS = [
    "time_s", "speed_rpm", "speed1_rpm", "speed2_rpm", "speed3_rpm", "speed_voted_rpm",
    "n_pu", "exh_gas_c", "exh_tc_c", "tc1_c", "tc2_c", "tc3_c", "tc4_c",
    "exh_voted_c", "exh_spread_c", "tit_k", "press_bar_g", "power_mw", "load_mw",
    "fuel_kg_s", "valve_pos", "fsr_cmd", "fsr_gov", "fsr_acc", "fsr_temp", "fsr_start",
    "selected", "ctrl_state", "tripped", "trip_cause", "alarm_word", "perm_word",
    "sensor_bad_word", "flame", "starter_on", "breaker", "sync_permit", "link_up",
    "plant_wd", "missed_polls", "ke_mj", "p_gas_net_mw", "p_comp_mw", "p_turb_mw",
    "p_shaft_net_mw",
]
STATE_NAMES = ["IDLE", "CRANK", "IGNITE", "ACCEL", "WARMUP", "RUN", "COOLDOWN", "TRIP"]
TRIP_NAMES = ["none", "overspeed", "overtemperature", "vibration", "low lube oil", "e-stop",
              "flame-out", "fail to light", "exhaust spread", "sensor fault",
              "communication loss", "fuel supply"]
CS_RUN = 5

# Must match the FEAT_* enum in controller/protection.h and the hil_regs.h codes.
FEATURES = {"voting": 0, "sensor_diag": 1, "governor": 2, "accel_limiter": 3,
            "temp_limiter": 4, "breaker_preset": 5, "overspeed_trip": 6,
            "overtemp_trip": 7, "spread_trip": 8, "comms_watchdog": 9, "plant_watchdog": 10}
FEAT_ALL = (1 << 11) - 1
FAULT_MODES = {"drift": 1, "stuck": 2, "noisy": 3, "offset": 4}
FAULT_CODES = {n: i for i, n in enumerate(
    ["none", "overspeed", "overtemp", "vibration", "low_oil", "estop", "flameout",
     "fail_to_light", "exh_spread", "sensor_fault", "comms_loss", "fuel_supply"])}
PERMISSIVES = {"estop": 0, "lube": 1, "fuel": 2, "starter": 3}
ALARM_BITS = {"overspeed": 1, "overtemp": 2, "spread": 4, "speed1_bad": 8, "speed2_bad": 16,
              "speed3_bad": 32, "comms": 64, "start_block": 128}


class SimOut(ctypes.Structure):
    _fields_ = [(n, ctypes.c_double) for n in OUT_FIELDS]


def load_lib():
    if not os.path.exists(LIB_PATH):
        sys.exit(f"{LIB_PATH} not found - run 'make' first")
    lib = ctypes.CDLL(LIB_PATH)
    lib.sim_create.restype = ctypes.c_void_p
    lib.sim_destroy.argtypes = [ctypes.c_void_p]
    lib.sim_reset.argtypes = [ctypes.c_void_p, ctypes.c_ulonglong, ctypes.c_int]
    lib.sim_set_start.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_load.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.sim_set_breaker.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_reset.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_droop.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_link.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_permissive.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.sim_inject_speed_fault.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                                           ctypes.c_double, ctypes.c_double]
    lib.sim_inject_tc_fault.argtypes = lib.sim_inject_speed_fault.argtypes
    lib.sim_clear_faults.argtypes = [ctypes.c_void_p]
    lib.sim_set_ambient_k.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.sim_set_features.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.sim_step.argtypes = [ctypes.c_void_p]
    lib.sim_get.argtypes = [ctypes.c_void_p, ctypes.POINTER(SimOut)]
    lib.sim_rated_power_mw.argtypes = [ctypes.c_void_p]
    lib.sim_rated_power_mw.restype = ctypes.c_double
    lib.sim_out_doubles.restype = ctypes.c_int
    if lib.sim_out_doubles() != len(OUT_FIELDS):
        sys.exit("SimOut layout mismatch between sim.h and scenario_runner.py")
    return lib


class Sim:
    def __init__(self, lib, seed, noise, features=FEAT_ALL):
        self.lib = lib
        self.h = lib.sim_create()
        lib.sim_reset(self.h, seed, 1 if noise else 0)
        lib.sim_set_features(self.h, features)
        self.out = SimOut()
        self.ramp = None                       # (t_start, t_end, k0, k1) ambient ramp
        self.rated_mw = lib.sim_rated_power_mw(self.h)

    def close(self):
        self.lib.sim_destroy(self.h)

    def step(self):
        if self.ramp:
            t0, t1, k0, k1 = self.ramp
            f = min(1.0, max(0.0, (self.out.time_s - t0) / (t1 - t0)))
            self.lib.sim_set_ambient_k(self.h, k0 + (k1 - k0) * f)
        self.lib.sim_step(self.h)
        self.lib.sim_get(self.h, ctypes.byref(self.out))
        return self.out

    def start(self, on):       self.lib.sim_set_start(self.h, 1 if on else 0)
    def load(self, mw):        self.lib.sim_set_load(self.h, mw)
    def breaker(self, closed): self.lib.sim_set_breaker(self.h, 1 if closed else 0)

    def apply(self, ev):
        """Apply one scenario event. Supervisor-side faults only touch the plant."""
        a, L, h = ev["action"], self.lib, self.h
        if a == "start":            self.start(True)
        elif a == "stop":           self.start(False)
        elif a == "open_breaker":   self.breaker(False)
        elif a == "close_breaker":  self.breaker(True)
        elif a == "set_load":       self.load(ev.get("mw", self.rated_mw * ev.get("pct", 0.0) / 100.0))
        elif a == "reset":          L.sim_set_reset(h, 1 if ev.get("value", 1) else 0)
        elif a == "droop":          L.sim_set_droop(h, 1 if ev.get("value", 1) else 0)
        elif a == "link":           L.sim_set_link(h, 1 if ev["up"] else 0)
        elif a == "permissive":     L.sim_set_permissive(h, PERMISSIVES[ev["name"]], 1 if ev["ok"] else 0)
        elif a == "speed_fault":    L.sim_inject_speed_fault(h, ev["ch"], FAULT_MODES[ev["mode"]],
                                                             ev.get("a", 0.0), ev.get("b", 0.0))
        elif a == "tc_fault":       L.sim_inject_tc_fault(h, ev["ch"], FAULT_MODES[ev["mode"]],
                                                          ev.get("a", 0.0), ev.get("b", 0.0))
        elif a == "clear_faults":   L.sim_clear_faults(h)
        elif a == "ambient_ramp":
            t = self.out.time_s
            self.ramp = (t, t + ev["over_s"], 288.15, 288.15 + ev["delta_k"])
        else:
            raise ValueError(f"unknown action {a}")


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------
def preroll(sim, spec):
    """Bring the unit to rated speed and load it, without recording."""
    sim.start(True)
    o = sim.step()
    t_in_band = 0.0
    while True:
        o = sim.step()
        if o.time_s > 300.0:
            raise RuntimeError("preroll failed: unit did not reach rated speed in 300 s")
        if int(o.ctrl_state) == CS_RUN and abs(o.speed_rpm - RATED_RPM) < 25.0:
            t_in_band += STEP_S
        else:
            t_in_band = 0.0
        if t_in_band >= 5.0:
            break

    target_mw = sim.rated_mw * spec.get("load_pct", 0.0) / 100.0
    ramp_s = 10.0
    sim.breaker(True)
    steps = int((ramp_s + spec.get("settle_s", 30.0)) / STEP_S)
    for i in range(steps):
        frac = min(1.0, (i * STEP_S) / ramp_s)
        sim.load(target_mw * frac)
        sim.step()
    return target_mw


def run(sc, lib, features=FEAT_ALL):
    # The preroll always runs with every rule on; the rule under test is removed
    # only when the scenario proper begins (otherwise the unit would never get there).
    sim = Sim(lib, int(sc.get("seed", 1)), bool(sc.get("noise", True)),
              FEAT_ALL if sc.get("preroll") else features)
    try:
        if sc.get("preroll"):
            preroll(sim, sc["preroll"])
            lib.sim_set_features(sim.h, features)
        t0 = sim.out.time_s
        events = sorted(sc.get("events", []), key=lambda e: e["t"])
        ei = 0
        fired = []
        rows = []
        n_steps = int(sc["duration_s"] / STEP_S)
        for _ in range(n_steps):
            t = sim.out.time_s - t0
            while ei < len(events) and t >= events[ei]["t"] - 1e-9:
                ev = events[ei]
                sim.apply(ev)
                fired.append({"t": ev["t"], "action": ev["action"],
                              "detail": {k: v for k, v in ev.items() if k not in ("t", "action")}})
                ei += 1
            o = sim.step()
            rows.append([getattr(o, n) for n in OUT_FIELDS])
        data = np.array(rows)
        data[:, 0] -= t0                       # scenario time starts at 0
        return {n: data[:, i] for i, n in enumerate(OUT_FIELDS)}, fired, sim.rated_mw
    finally:
        sim.close()


# --------------------------------------------------------------------------
# Criteria
# --------------------------------------------------------------------------
def first_run(t, ok, start_t, hold_s):
    """Start time of the first run of True (at or after start_t) lasting >= hold_s."""
    run_start = None
    for i in range(len(t)):
        if t[i] < start_t:
            continue
        if ok[i]:
            if run_start is None:
                run_start = t[i]
            if t[i] - run_start >= hold_s - 1e-9:
                return run_start
        else:
            run_start = None
    return None


def win(t, crit):
    """Boolean mask of the samples inside [from_s, to_s]."""
    return (t >= crit.get("from_s", 0.0)) & (t <= crit.get("to_s", 1e12))


def evaluate(crit, d):
    t = d["time_s"]
    sig = d[crit["signal"]]
    ty = crit["type"]
    r = {"id": crit["id"], "text": crit.get("text", crit["id"]), "type": ty}

    if ty == "reach_band":
        ok = np.abs(sig - crit["center"]) <= crit["tolerance"]
        t_reach = first_run(t, ok, 0.0, crit["hold_s"])
        r["passed"] = t_reach is not None and t_reach <= crit["within_s"]
        r["measured"] = "not reached" if t_reach is None else f"reached at {t_reach:.1f} s"
        r["limit"] = f"<= {crit['within_s']:.0f} s"
    elif ty == "max_below":
        m = win(t, crit)
        peak = float(np.max(sig[m]))
        r["passed"] = peak < crit["limit"]
        r["measured"] = f"peak {peak:.1f}"
        r["limit"] = f"< {crit['limit']:.0f}"
    elif ty == "min_above":
        m = win(t, crit)
        low = float(np.min(sig[m]))
        r["passed"] = low > crit["limit"]
        r["measured"] = f"minimum {low:.1f}"
        r["limit"] = f"> {crit['limit']:.0f}"
    elif ty == "flag_false":
        m = win(t, crit)
        hit = (sig > 0.5) & m
        r["passed"] = not bool(np.any(hit))
        r["measured"] = "never set" if r["passed"] else f"set at {t[np.argmax(hit)]:.1f} s"
        r["limit"] = "never"
    elif ty == "flag_true":
        m = win(t, crit)
        hit = (sig > 0.5) & m
        r["passed"] = bool(np.any(hit))
        r["measured"] = "never set" if not r["passed"] else f"set at {t[np.argmax(hit)]:.1f} s"
        r["limit"] = f"between {crit.get('from_s', 0):.0f} and {crit.get('to_s', t[-1]):.0f} s"
    elif ty in ("bit_set", "bit_clear"):
        m = win(t, crit)
        mask = ALARM_BITS[crit["bit"]]
        hit = ((sig.astype(int) & mask) != 0) & m
        if ty == "bit_set":
            r["passed"] = bool(np.any(hit))
            r["measured"] = "never set" if not r["passed"] else f"set at {t[np.argmax(hit)]:.1f} s"
            r["limit"] = f"between {crit.get('from_s', 0):.0f} and {crit.get('to_s', t[-1]):.0f} s"
        else:
            r["passed"] = not bool(np.any(hit))
            r["measured"] = "never set" if r["passed"] else f"set at {t[np.argmax(hit)]:.1f} s"
            r["limit"] = "never"
    elif ty == "final_is":
        v = int(round(float(sig[-1])))
        r["passed"] = v == FAULT_CODES[crit["value"]]
        r["measured"] = TRIP_NAMES[v] if 0 <= v < len(TRIP_NAMES) else str(v)
        r["limit"] = crit["value"].replace("_", " ")
    elif ty == "value_in_band_at":
        i = int(np.searchsorted(t, crit["at_s"]))
        v = float(sig[min(i, len(sig) - 1)])
        r["passed"] = crit["lo"] <= v <= crit["hi"]
        r["measured"] = f"{v:.1f} at {crit['at_s']:.1f} s"
        r["limit"] = f"{crit['lo']:g} to {crit['hi']:g}"
    elif ty == "settles_in_band":
        ok = (sig >= crit["lo"]) & (sig <= crit["hi"])
        ts = first_run(t, ok, crit["after_s"], crit["hold_s"])
        r["passed"] = ts is not None and (ts - crit["after_s"]) <= crit["within_s"]
        r["measured"] = "never settled" if ts is None else f"settled {ts - crit['after_s']:.1f} s after event"
        r["limit"] = f"<= {crit['within_s']:.0f} s"
    else:
        raise ValueError(f"unknown criterion type {ty}")
    return r


def metrics(d):
    t = d["time_s"]
    return {
        "peak_speed_rpm": float(np.max(d["speed_rpm"])),
        "final_speed_rpm": float(d["speed_rpm"][-1]),
        "peak_exh_gas_c": float(np.max(d["exh_gas_c"])),
        "peak_exh_tc_c": float(np.max(d["exh_tc_c"])),
        "peak_fuel_kg_s": float(np.max(d["fuel_kg_s"])),
        "peak_power_mw": float(np.max(d["power_mw"])),
        "tripped": bool(np.any(d["tripped"] > 0.5)),
        "first_out": TRIP_NAMES[int(round(d["trip_cause"][-1]))],
        "peak_spread_c": float(np.max(d["exh_spread_c"])),
        "duration_s": float(t[-1]),
    }


# --------------------------------------------------------------------------
# Evidence output
# --------------------------------------------------------------------------
def git_commit():
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 and out.stdout.strip() else "not a git repository"
    except Exception:
        return "unavailable"


def write_csv(path, d, meta):
    idx = np.arange(0, len(d["time_s"]), LOG_EVERY)
    with open(path, "w", newline="") as f:
        for k, v in meta.items():
            f.write(f"# {k}: {v}\n")
        w = csv.writer(f)
        w.writerow(OUT_FIELDS)
        for i in idx:
            w.writerow([f"{d[n][i]:.6g}" for n in OUT_FIELDS])


def plot(path, sc, d, fired, title_suffix=""):
    t = d["time_s"]
    fig, ax = plt.subplots(6, 1, figsize=(10, 14), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2.4, 2.2, 1.8, 1.4, 1.4]})
    c = {"blue": "#1f77b4", "orange": "#e07b00", "green": "#2a9d4b", "red": "#d62728",
         "grey": "#777777", "purple": "#7b4fb3"}

    for k, col in (("speed1_rpm", "#9ecae1"), ("speed2_rpm", "#a1d99b"), ("speed3_rpm", "#fdae6b")):
        ax[0].plot(t, d[k], color=col, lw=0.8, label=k.replace("_rpm", " (meas.)"))
    ax[0].plot(t, d["speed_voted_rpm"], color=c["purple"], lw=1.0, ls="--", label="Voted")
    ax[0].plot(t, d["speed_rpm"], color=c["blue"], lw=1.6, label="Speed (true)")
    ax[0].axhspan(3450, 3550, color=c["green"], alpha=0.12, label="3450-3550 RPM band")
    ax[0].axhline(3850, color=c["red"], ls="--", lw=1, label="Overspeed trip 3850")
    ax[0].set_ylabel("Speed (RPM)")
    ax[0].legend(loc="center right", fontsize=7, ncol=2)

    for k in ("tc1_c", "tc2_c", "tc3_c", "tc4_c"):
        ax[1].plot(t, d[k], lw=0.7, alpha=0.7, label=k.replace("_c", ""))
    ax[1].plot(t, d["exh_gas_c"], color=c["orange"], lw=1.4, label="Exhaust gas (true)")
    ax[1].plot(t, d["exh_voted_c"], color="black", lw=1.0, ls="--", label="Median of 4")
    ax[1].axhline(620, color=c["red"], ls="--", lw=1, label="Trip 620")
    ax[1].axhline(600, color=c["grey"], ls=":", lw=1, label="Limiter 600")
    ax[1].set_ylabel("Exhaust temp (degC)")
    ax[1].legend(loc="center right", fontsize=7, ncol=2)

    ax[2].plot(t, d["fsr_cmd"], color="black", lw=2.0, label="FSR commanded (min-select)")
    ax[2].plot(t, np.clip(d["fsr_gov"], 0, 1.05), color=c["blue"], lw=0.9, alpha=0.8, label="Governor")
    ax[2].plot(t, np.clip(d["fsr_acc"], 0, 1.05), color=c["green"], lw=0.9, alpha=0.8, label="Acceleration")
    ax[2].plot(t, np.clip(d["fsr_temp"], 0, 1.05), color=c["orange"], lw=0.9, alpha=0.8, label="Temperature")
    ax[2].plot(t, np.clip(d["fsr_start"], 0, 1.05), color=c["grey"], lw=0.9, alpha=0.8, label="Start ramp")
    ax[2].set_ylim(-0.03, 1.08)
    ax[2].set_ylabel("Fuel demand (FSR)")
    ax[2].legend(loc="center right", fontsize=7, ncol=2)

    ax[3].plot(t, d["power_mw"], color=c["green"], lw=1.4, label="Electrical output")
    ax[3].plot(t, d["load_mw"], color=c["grey"], lw=1.0, ls="--", label="Load demand")
    ax[3].set_ylabel("Power (MW)")
    ax[3].legend(loc="center right", fontsize=8)

    ax[4].step(t, d["ctrl_state"], where="post", color="black", lw=1.4)
    ax[4].set_yticks(range(len(STATE_NAMES)))
    ax[4].set_yticklabels(STATE_NAMES, fontsize=6)
    ax[4].set_ylabel("Ctrl state")

    ax[5].step(t, d["link_up"] * 0.9 + 0.0, where="post", color=c["blue"], lw=1.0, label="link up")
    ax[5].step(t, d["tripped"] * 0.8 + 1.1, where="post", color=c["red"], lw=1.0, label="tripped")
    ax[5].step(t, d["plant_wd"] * 0.8 + 2.2, where="post", color=c["orange"], lw=1.0, label="plant watchdog")
    ax[5].step(t, d["breaker"] * 0.8 + 3.3, where="post", color=c["green"], lw=1.0, label="breaker")
    ax[5].set_yticks([])
    ax[5].legend(loc="center right", fontsize=7, ncol=2)
    ax[5].set_xlabel("Scenario time (s)")

    for a in ax:
        a.grid(alpha=0.25)
        for ev in fired:
            a.axvline(ev["t"], color=c["red"], lw=0.8, alpha=0.5)
    fo = int(round(d["trip_cause"][-1]))
    if fo:
        ax[4].text(0.01, 0.9, f"first-out: {TRIP_NAMES[fo]}", transform=ax[4].transAxes, fontsize=8,
                   color=c["red"], va="top")
    ax[0].set_title(sc["name"] + title_suffix, fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_report(path, sc, results, mets, fired, meta, passed, negatives=()):
    L = []
    L.append(f"# {sc['name']}\n")
    L.append(f"**Result: {'PASS' if passed else 'FAIL'}**\n")
    L.append(f"{sc.get('description', '')}\n")
    L.append("## Criteria\n")
    L.append("| | Criterion | Measured | Limit |")
    L.append("| --- | --- | --- | --- |")
    for r in results:
        L.append(f"| {'PASS' if r['passed'] else 'FAIL'} | {r['text']} | {r['measured']} | {r['limit']} |")
    L.append("\n## Key figures\n")
    L.append(f"- Peak speed: {mets['peak_speed_rpm']:.0f} RPM")
    L.append(f"- Final speed: {mets['final_speed_rpm']:.0f} RPM")
    L.append(f"- Peak exhaust gas temperature: {mets['peak_exh_gas_c']:.0f} degC")
    L.append(f"- Peak fuel flow: {mets['peak_fuel_kg_s']:.2f} kg/s")
    L.append(f"- Peak electrical output: {mets['peak_power_mw']:.1f} MW")
    L.append(f"- Tripped: {'yes' if mets['tripped'] else 'no'}"
             + (f" (first-out: {mets['first_out']})" if mets['tripped'] else ""))
    L.append(f"- Peak exhaust spread: {mets['peak_spread_c']:.0f} degC")
    if negatives:
        L.append("\n## Negative tests (the rule is removed; the scenario must FAIL)\n")
        L.append("| Outcome | Rule removed | Criteria that failed | Why this proves the rule |")
        L.append("| --- | --- | --- | --- |")
        for n in negatives:
            L.append(f"| {'FAILS AS INTENDED' if n['ok'] else 'UNEXPECTED PASS'} | {', '.join(n['disable'])} "
                     f"| {', '.join(n['failed']) or 'none'} | {n['text']} |")
    L.append("\n## Events\n")
    for e in fired:
        det = ", ".join(f"{k}={v}" for k, v in e.get("detail", {}).items())
        L.append(f"- t = {e['t']:.1f} s: {e['action']}" + (f" ({det})" if det else ""))
    L.append("\n## Plot\n")
    L.append("![scenario plot](plot.png)\n")
    L.append("## Run record\n")
    for k, v in meta.items():
        L.append(f"- {k}: {v}")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def feature_mask(disable):
    m = FEAT_ALL
    for name in disable:
        m &= ~(1 << FEATURES[name])
    return m


def run_one(path, lib, out_root):
    raw = open(path, "rb").read()
    sc = json.loads(raw)
    sha = hashlib.sha256(raw).hexdigest()[:12]
    data, fired, rated_mw = run(sc, lib)
    results = [evaluate(c, data) for c in sc["criteria"]]
    for r in results:
        r["passed"] = bool(r["passed"])
    mets = metrics(data)
    positive_ok = all(r["passed"] for r in results)

    # ---- negative tests: remove one rule, the scenario must now fail ----
    out = os.path.join(out_root, sc["id"])
    os.makedirs(out, exist_ok=True)
    negatives = []
    for neg in sc.get("negative_tests", []):
        nd, nf, _ = run(sc, lib, feature_mask(neg["disable"]))
        nres = [evaluate(c, nd) for c in sc["criteria"]]
        failed = [r["id"] for r in nres if not bool(r["passed"])]
        expect = neg.get("must_fail", [])
        ok = bool(failed) and all(e in failed for e in expect)
        nm = metrics(nd)
        negatives.append({"id": neg["id"], "disable": neg["disable"], "text": neg["text"],
                          "failed": failed, "ok": ok,
                          "metrics": {"peak_speed_rpm": nm["peak_speed_rpm"], "peak_exh_gas_c": nm["peak_exh_gas_c"],
                                      "tripped": nm["tripped"], "first_out": nm["first_out"],
                                      "final_speed_rpm": nm["final_speed_rpm"]}})
        plot(os.path.join(out, f"negative_{neg['id']}.png"), sc, nd, nf,
             f"  [NEGATIVE TEST: {', '.join(neg['disable'])} removed]")
    negatives_ok = all(n["ok"] for n in negatives)
    passed = positive_ok and negatives_ok

    meta = {
        "scenario file": os.path.relpath(path, ROOT),
        "scenario sha256 (first 12)": sha,
        "run at (UTC)": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
        "git commit": git_commit(),
        "seed": sc.get("seed", 1),
        "sensor noise": sc.get("noise", True),
        "rated electrical power (MW)": f"{rated_mw:.2f}",
        "python": platform.python_version(),
    }

    write_csv(os.path.join(out, "data.csv"), data, meta)
    plot(os.path.join(out, "plot.png"), sc, data, fired)
    write_report(os.path.join(out, "report.md"), sc, results, mets, fired, meta, passed, negatives)
    with open(os.path.join(out, "report.json"), "w") as f:
        json.dump({"scenario": sc["id"], "name": sc["name"], "passed": passed,
                   "positive_passed": positive_ok, "criteria": results, "metrics": mets,
                   "negative_tests": negatives, "run": meta}, f, indent=2,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))

    print(f"\n{'PASS' if passed else 'FAIL'}  {sc['name']}")
    for r in results:
        print(f"   {'ok  ' if r['passed'] else 'FAIL'} {r['text']}  [{r['measured']}; limit {r['limit']}]")
    for n in negatives:
        print(f"   {'neg ok ' if n['ok'] else 'NEG BAD'} without {', '.join(n['disable'])}: "
              f"scenario fails on {', '.join(n['failed']) or 'nothing (!)'}")
    return {"id": sc["id"], "name": sc["name"], "passed": passed, "results": results,
            "metrics": mets, "negatives": negatives}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="+", help="scenario JSON files")
    ap.add_argument("--out", default=os.path.join(ROOT, "results"), help="results folder")
    args = ap.parse_args()

    lib = load_lib()
    summary = [run_one(p, lib, args.out) for p in args.scenarios]

    os.makedirs(args.out, exist_ok=True)
    n_ok = sum(1 for s in summary if s["passed"])
    n_neg = sum(len(s["negatives"]) for s in summary)
    n_neg_ok = sum(1 for s in summary for n in s["negatives"] if n["ok"])
    lines = ["# Scenario summary\n",
             f"{n_ok} of {len(summary)} scenarios passed. "
             f"{n_neg_ok} of {n_neg} negative tests failed as intended.\n",
             "| Result | Scenario | Negative tests | Report |", "| --- | --- | --- | --- |"]
    for s in summary:
        nn = sum(1 for n in s["negatives"] if n["ok"])
        lines.append(f"| {'PASS' if s['passed'] else 'FAIL'} | {s['name']} | {nn} of {len(s['negatives'])} "
                     f"| [{s['id']}/report.md]({s['id']}/report.md) |")
    with open(os.path.join(args.out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"\n=== {n_ok} of {len(summary)} scenarios passed, "
          f"{n_neg_ok} of {n_neg} negative tests failed as intended ===")
    sys.exit(0 if n_ok == len(summary) else 1)


if __name__ == "__main__":
    main()
