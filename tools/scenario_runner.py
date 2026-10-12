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
    "time_s", "speed_rpm", "speed_meas_rpm", "n_pu", "exh_gas_c", "exh_tc_c",
    "exh_meas_c", "tit_k", "press_bar_g", "power_mw", "load_mw", "fuel_kg_s",
    "valve_pos", "fsr_cmd", "fsr_gov", "fsr_acc", "fsr_temp", "fsr_start",
    "selected", "ctrl_state", "tripped", "trip_cause", "flame", "starter_on",
    "breaker", "ke_mj", "p_gas_net_mw", "p_comp_mw", "p_turb_mw", "p_shaft_net_mw",
]
STATE_NAMES = ["IDLE", "CRANK", "IGNITE", "ACCEL", "RUN", "COOLDOWN", "TRIP"]
TRIP_NAMES = ["none", "overspeed", "overtemperature", "flame-out", "fail to light"]
CS_RUN = 4


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
    lib.sim_step.argtypes = [ctypes.c_void_p]
    lib.sim_get.argtypes = [ctypes.c_void_p, ctypes.POINTER(SimOut)]
    lib.sim_rated_power_mw.argtypes = [ctypes.c_void_p]
    lib.sim_rated_power_mw.restype = ctypes.c_double
    lib.sim_out_doubles.restype = ctypes.c_int
    if lib.sim_out_doubles() != len(OUT_FIELDS):
        sys.exit("SimOut layout mismatch between sim.h and scenario_runner.py")
    return lib


class Sim:
    def __init__(self, lib, seed, noise):
        self.lib = lib
        self.h = lib.sim_create()
        lib.sim_reset(self.h, seed, 1 if noise else 0)
        self.out = SimOut()
        self.rated_mw = lib.sim_rated_power_mw(self.h)

    def close(self):
        self.lib.sim_destroy(self.h)

    def step(self):
        self.lib.sim_step(self.h)
        self.lib.sim_get(self.h, ctypes.byref(self.out))
        return self.out

    def start(self, on):       self.lib.sim_set_start(self.h, 1 if on else 0)
    def load(self, mw):        self.lib.sim_set_load(self.h, mw)
    def breaker(self, closed): self.lib.sim_set_breaker(self.h, 1 if closed else 0)


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


def run(sc, lib):
    sim = Sim(lib, int(sc.get("seed", 1)), bool(sc.get("noise", True)))
    try:
        if sc.get("preroll"):
            preroll(sim, sc["preroll"])
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
                a = ev["action"]
                if a == "start":           sim.start(True)
                elif a == "stop":          sim.start(False)
                elif a == "open_breaker":  sim.breaker(False)
                elif a == "close_breaker": sim.breaker(True)
                elif a == "set_load":
                    mw = ev.get("mw", sim.rated_mw * ev.get("pct", 0.0) / 100.0)
                    sim.load(mw)
                else:
                    raise ValueError(f"unknown action {a}")
                fired.append({"t": ev["t"], "action": a})
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
        m = t >= crit.get("from_s", 0.0)
        peak = float(np.max(sig[m]))
        r["passed"] = peak < crit["limit"]
        r["measured"] = f"peak {peak:.1f}"
        r["limit"] = f"< {crit['limit']:.0f}"
    elif ty == "min_above":
        m = t >= crit.get("from_s", 0.0)
        low = float(np.min(sig[m]))
        r["passed"] = low > crit["limit"]
        r["measured"] = f"minimum {low:.1f}"
        r["limit"] = f"> {crit['limit']:.0f}"
    elif ty == "flag_false":
        r["passed"] = not bool(np.any(sig > 0.5))
        r["measured"] = "never set" if r["passed"] else f"set at {t[np.argmax(sig > 0.5)]:.1f} s"
        r["limit"] = "never"
    elif ty == "value_in_band_at":
        i = int(np.searchsorted(t, crit["at_s"]))
        v = float(sig[min(i, len(sig) - 1)])
        r["passed"] = crit["lo"] <= v <= crit["hi"]
        r["measured"] = f"{v:.1f} at {crit['at_s']:.1f} s"
        r["limit"] = f"{crit['lo']:.0f} to {crit['hi']:.0f}"
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


def plot(path, sc, d, fired):
    t = d["time_s"]
    fig, ax = plt.subplots(5, 1, figsize=(10, 12), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2.4, 2.4, 2, 1.4]})
    c = {"blue": "#1f77b4", "orange": "#e07b00", "green": "#2a9d4b", "red": "#d62728",
         "grey": "#777777", "purple": "#7b4fb3"}

    ax[0].plot(t, d["speed_rpm"], color=c["blue"], lw=1.6, label="Speed (true)")
    ax[0].axhspan(3450, 3550, color=c["green"], alpha=0.12, label="3450-3550 RPM band")
    ax[0].axhline(3850, color=c["red"], ls="--", lw=1, label="Overspeed trip 3850")
    ax[0].set_ylabel("Speed (RPM)")
    ax[0].legend(loc="center right", fontsize=8)

    ax[1].plot(t, d["exh_gas_c"], color=c["orange"], lw=1.4, label="Exhaust gas (true)")
    ax[1].plot(t, d["exh_tc_c"], color=c["purple"], lw=1.2, ls="-", label="Thermocouple (3 s lag)")
    ax[1].axhline(620, color=c["red"], ls="--", lw=1, label="Trip 620")
    ax[1].axhline(600, color=c["grey"], ls=":", lw=1, label="Limiter 600")
    ax[1].set_ylabel("Exhaust temp (degC)")
    ax[1].legend(loc="center right", fontsize=8)

    ax[2].plot(t, d["fsr_cmd"], color="black", lw=2.0, label="FSR commanded (min-select)")
    ax[2].plot(t, np.clip(d["fsr_gov"], 0, 1.05), color=c["blue"], lw=0.9, alpha=0.8, label="Governor")
    ax[2].plot(t, np.clip(d["fsr_acc"], 0, 1.05), color=c["green"], lw=0.9, alpha=0.8, label="Acceleration")
    ax[2].plot(t, np.clip(d["fsr_temp"], 0, 1.05), color=c["orange"], lw=0.9, alpha=0.8, label="Temperature")
    ax[2].plot(t, np.clip(d["fsr_start"], 0, 1.05), color=c["grey"], lw=0.9, alpha=0.8, label="Start ramp")
    ax[2].set_ylim(-0.03, 1.08)
    ax[2].set_ylabel("Fuel demand (FSR, 0-1)")
    ax[2].legend(loc="center right", fontsize=7, ncol=2)

    ax[3].plot(t, d["power_mw"], color=c["green"], lw=1.4, label="Electrical output")
    ax[3].plot(t, d["load_mw"], color=c["grey"], lw=1.0, ls="--", label="Load demand")
    ax[3].set_ylabel("Power (MW)")
    ax[3].legend(loc="center right", fontsize=8)

    ax[4].step(t, d["ctrl_state"], where="post", color="black", lw=1.4)
    ax[4].set_yticks(range(len(STATE_NAMES)))
    ax[4].set_yticklabels(STATE_NAMES, fontsize=7)
    ax[4].set_ylabel("Controller state")
    ax[4].set_xlabel("Scenario time (s)")

    for a in ax:
        a.grid(alpha=0.25)
        for ev in fired:
            a.axvline(ev["t"], color=c["red"], lw=0.8, alpha=0.5)
    ax[0].set_title(sc["name"], fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_report(path, sc, results, mets, fired, meta, passed):
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
    L.append(f"- Tripped: {'yes' if mets['tripped'] else 'no'}")
    L.append("\n## Events\n")
    for e in fired:
        L.append(f"- t = {e['t']:.1f} s: {e['action']}")
    L.append("\n## Plot\n")
    L.append("![scenario plot](plot.png)\n")
    L.append("## Run record\n")
    for k, v in meta.items():
        L.append(f"- {k}: {v}")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def run_one(path, lib, out_root):
    raw = open(path, "rb").read()
    sc = json.loads(raw)
    sha = hashlib.sha256(raw).hexdigest()[:12]
    data, fired, rated_mw = run(sc, lib)
    results = [evaluate(c, data) for c in sc["criteria"]]
    for r in results:
        r["passed"] = bool(r["passed"])
    mets = metrics(data)
    passed = all(r["passed"] for r in results)

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

    out = os.path.join(out_root, sc["id"])
    os.makedirs(out, exist_ok=True)
    write_csv(os.path.join(out, "data.csv"), data, meta)
    plot(os.path.join(out, "plot.png"), sc, data, fired)
    write_report(os.path.join(out, "report.md"), sc, results, mets, fired, meta, passed)
    with open(os.path.join(out, "report.json"), "w") as f:
        json.dump({"scenario": sc["id"], "name": sc["name"], "passed": passed,
                   "criteria": results, "metrics": mets, "run": meta}, f, indent=2,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))

    print(f"\n{'PASS' if passed else 'FAIL'}  {sc['name']}")
    for r in results:
        print(f"   {'ok  ' if r['passed'] else 'FAIL'} {r['text']}  [{r['measured']}; limit {r['limit']}]")
    return {"id": sc["id"], "name": sc["name"], "passed": passed, "results": results, "metrics": mets}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="+", help="scenario JSON files")
    ap.add_argument("--out", default=os.path.join(ROOT, "results"), help="results folder")
    args = ap.parse_args()

    lib = load_lib()
    summary = [run_one(p, lib, args.out) for p in args.scenarios]

    os.makedirs(args.out, exist_ok=True)
    n_ok = sum(1 for s in summary if s["passed"])
    lines = ["# Scenario summary\n",
             f"{n_ok} of {len(summary)} scenarios passed.\n",
             "| Result | Scenario | Report |", "| --- | --- | --- |"]
    for s in summary:
        lines.append(f"| {'PASS' if s['passed'] else 'FAIL'} | {s['name']} | [{s['id']}/report.md]({s['id']}/report.md) |")
    with open(os.path.join(args.out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"\n=== {n_ok} of {len(summary)} scenarios passed ===")
    sys.exit(0 if n_ok == len(summary) else 1)


if __name__ == "__main__":
    main()
