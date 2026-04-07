#!/usr/bin/env python3
"""Michael Karpov(2026)

python-based runner for FSM tests

Usage:
    python run_test.py <*.json>    -> Runs all tests
                       --no-plot   -> Skips plotter
    python run_test.py <*.csv>     -> Sim & plot, no test
"""

import argparse
import json
import os
import pathlib
import subprocess
import sys
from dataclasses import dataclass

import pandas as pd

from plot import plot, STATE_NAMES

STATE_TO_NUM = {name: i for i, name in enumerate(STATE_NAMES)}
NUM_TO_STATE = {i: name for i, name in enumerate(STATE_NAMES)}

SCRIPT_DIR = pathlib.Path(__file__).parent
FSM_TEST_DIR = SCRIPT_DIR.parent
PROJECT_ROOT = FSM_TEST_DIR.parent.parent


@dataclass
class ExpectedTransition:
    state: int
    time_ms: float
    tolerance_ms: float = 500.0

@dataclass
class ExpectedPyroFire:
    channel: int
    time_ms: float
    tolerance_ms: float = 500.0

@dataclass
class TestCase:
    name: str
    description: str
    input_csv: pathlib.Path
    expected_transitions: list[ExpectedTransition]
    expected_pyro_fires: list[ExpectedPyroFire]


def load_test_case(json_path):
    with open(json_path) as f:
        data = json.load(f)

    json_dir = json_path.parent
    return TestCase(
        name=data.get("name", json_path.stem),
        description=data.get("description", ""),
        input_csv=json_dir / data["input_csv"],
        expected_transitions=[
            ExpectedTransition(STATE_TO_NUM[t["state"]], t["time_ms"], t.get("tolerance_ms", 500.0))
            for t in data.get("transitions", [])
        ],
        expected_pyro_fires=[
            ExpectedPyroFire(p["channel"], p["time_ms"], p.get("tolerance_ms", 500.0))
            for p in data.get("pyro_fires", [])
        ],
    )


def detect_transitions(df):
    seen = set()
    transitions = []
    prev = None
    for _, row in df.iterrows():
        s = int(row["state"])
        if s != prev and s not in seen:
            seen.add(s)
            transitions.append((s, float(row["timestamp_ms"])))
        prev = s
    return transitions


def detect_pyro_fires(df):
    fires = []
    for ch in range(4):
        for col in [f"pyro_{ch}_firing", f"pyro_{ch}_consumed"]:
            if col not in df.columns:
                continue
            edges = df[col].astype(int).diff().fillna(0)
            rows = df[edges == 1]
            if len(rows) > 0:
                fires.append((ch, float(rows.iloc[0]["timestamp_ms"])))
                break
    return fires


def validate(transitions, pyro_fires, tc):
    ok = True
    msgs = []

    by_state = dict(transitions)
    if tc.expected_transitions:
        msgs.append("Transitions:")
        for exp in tc.expected_transitions:
            name = NUM_TO_STATE.get(exp.state, str(exp.state))
            actual = by_state.get(exp.state)
            if actual is None:
                msgs.append(f"FAIL: {name} -- never reached (expected ~{exp.time_ms:.0f}ms)")
                ok = False
            else:
                delta = actual - exp.time_ms
                status = "PASS" if abs(delta) <= exp.tolerance_ms else "FAIL"
                if status == "FAIL": ok = False
                msgs.append(f"{status}: {name} at {actual:.0f}ms (expected {exp.time_ms:.0f}ms, delta {delta:+.0f}ms, tol +/-{exp.tolerance_ms:.0f}ms)")

    by_ch = dict(pyro_fires)
    if tc.expected_pyro_fires:
        msgs.append("Pyro fires:")
        for exp in tc.expected_pyro_fires:
            actual = by_ch.get(exp.channel)
            if actual is None:
                msgs.append(f"FAIL: ch{exp.channel} -- never fired (expected ~{exp.time_ms:.0f}ms)")
                ok = False
            else:
                delta = actual - exp.time_ms
                status = "PASS" if abs(delta) <= exp.tolerance_ms else "FAIL"
                if status == "FAIL": ok = False
                msgs.append(f"{status}: ch{exp.channel} at {actual:.0f}ms (expected {exp.time_ms:.0f}ms, delta {delta:+.0f}ms, tol +/-{exp.tolerance_ms:.0f}ms)")

    return ok, msgs


def run_simulator(input_csv, output_csv):
    print("Building FSM simulator...")
    build = subprocess.run(["pio", "run", "-e", "fsm_silsim_test"], cwd=PROJECT_ROOT)
    if build.returncode != 0:
        print("Build failed.")
        return False

    exe_name = "program.exe" if os.name == "nt" else "program"
    exe = PROJECT_ROOT / ".pio" / "build" / "fsm_silsim_test" / exe_name
    print(f"Running: {input_csv} -> {output_csv}")
    return subprocess.run([str(exe), str(input_csv), str(output_csv)], cwd=PROJECT_ROOT).returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=pathlib.Path, help=".test.json (test mode) or .csv (plot mode)")
    parser.add_argument("-o", "--output", type=pathlib.Path, default=None)
    parser.add_argument("--skip-sim", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--save-plot", type=pathlib.Path, default=None, help="Save plot to PNG instead of showing")
    parser.add_argument("--title", type=str, default=None)
    args = parser.parse_args()

    if args.input.suffix == ".json":
        tc = load_test_case(args.input)
        output_csv = args.output or FSM_TEST_DIR / "output" / f"{tc.name}.csv"
        output_csv.parent.mkdir(parents=True, exist_ok=True)

        if not args.skip_sim and not run_simulator(tc.input_csv, output_csv):
            return 1
        if not output_csv.exists():
            print(f"Output not found: {output_csv}")
            return 1

        df = pd.read_csv(output_csv)
        passed, msgs = validate(detect_transitions(df), detect_pyro_fires(df), tc)

        print(f"[{'PASS' if passed else 'FAIL'}] {tc.name}")
        for msg in msgs:
            print(msg)

        if not args.no_plot:
            title = args.title or f"FSM SILSIM: {tc.name}"
            plot(df, title, save_path=args.save_plot)
        return 0 if passed else 1

    if args.skip_sim:
        output_csv = args.input
    else:
        output_csv = args.output or FSM_TEST_DIR / "output" / f"{args.input.stem}.csv"
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        if not run_simulator(args.input, output_csv):
            return 1

    if not output_csv.exists():
        print(f"Output not found: {output_csv}")
        return 1

    df = pd.read_csv(output_csv)
    if not args.no_plot:
        title = args.title or f"FSM SILSIM: {args.input.stem}"
        plot(df, title, save_path=args.save_plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
