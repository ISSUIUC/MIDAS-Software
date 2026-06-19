"""Michael Karpov (2026)

Arbitrary python data converter to silsim-capable csv.
Supports:
    - Telemega data
    - (that's it) 

Usage:
    python convert_csv.py <input.csv> <output.csv> --<format>
"""

import argparse
import csv
import math
import pathlib
import sys
from abc import ABC, abstractmethod

WINDOW = 16 # for averaging

def sensor_average(values, window):
    if not values:
        return 0.0
    recent = values[-window:]
    return sum(recent) / len(recent)


def split_half_derivative(values, times, window):
    if len(values) < window:
        return 0.0
    recent = values[-window:]
    recent_t = times[-window:]
    half = window // 2
    first_avg = sum(recent[:half]) / half
    first_t = sum(recent_t[:half]) / half
    second_avg = sum(recent[half:]) / half
    second_t = sum(recent_t[half:]) / half
    dt = second_t - first_t
    if dt == 0:
        return 0.0
    return (second_avg - first_avg) / dt


class FlightFormat(ABC):
    name: str
    description: str

    @abstractmethod
    def col_time(self) -> str: ...

    @abstractmethod
    def col_accel(self) -> str: ...

    @abstractmethod
    def col_altitude(self) -> str: ...

    @abstractmethod
    def col_speed(self) -> str: ...

    @abstractmethod
    def time_to_seconds(self, raw: float) -> float: ...

    @abstractmethod
    def accel_to_g(self, raw: float) -> float: ...

    def col_tilt(self) -> str | None:
        return None

    def tilt_to_deg(self, raw: float) -> float:
        return raw

    def parse_vx(self, row: dict) -> float:
        col = self.col_speed()
        return float(row.get(col))

    def required_columns(self) -> list[str]:
        return [self.col_time(), self.col_accel(), self.col_altitude()]


class TeleMegaFormat(FlightFormat):
    name = "tm"
    description = "TeleMega"

    def col_time(self): return "time"
    def col_accel(self): return "accel_x"
    def col_altitude(self): return "b_altitude"
    def col_speed(self): return "speed"
    def col_tilt(self): return "tilt"
    def time_to_seconds(self, raw): return raw
    def accel_to_g(self, raw): return raw / 9.81


FORMATS: dict[str, FlightFormat] = {}

def register_format(fmt: FlightFormat):
    FORMATS[fmt.name] = fmt

register_format(TeleMegaFormat())


def convert(input_path, output_path, fmt, extra):
    accel_buf, alt_buf, time_buf = [], [], []

    with open(input_path, newline="") as fin, open(output_path, "w", newline="") as fout:
        reader = csv.DictReader(fin)
        if reader.fieldnames is None:
            print("Error: empty CSV", file=sys.stderr)
            return 1
        for col in fmt.required_columns():
            if col not in reader.fieldnames:
                print(f"Error: column '{col}' not found. Available: {reader.fieldnames}", file=sys.stderr)
                return 1

        has_tilt = fmt.col_tilt() is not None and fmt.col_tilt() in reader.fieldnames
        writer = csv.writer(fout)
        header = ["timestamp_ms", "acceleration_g", "altitude_m", "vertical_speed_mps", "vx_mps"]
        if has_tilt:
            header.append("tilt_deg")
        writer.writerow(header)

        for row in reader:
            try:
                raw_time = float(row[fmt.col_time()])
            except (ValueError, KeyError):
                continue

            time_s = fmt.time_to_seconds(raw_time)
            time_ms = time_s * 1000.0

            try:
                mul = 1
                if "ainv" in extra:
                    if extra["ainv"]: mul = -1
                raw_accel = mul * float(row[fmt.col_accel()])
            except (ValueError, KeyError):
                continue
            accel_buf.append(fmt.accel_to_g(raw_accel))
            avg_accel = sensor_average(accel_buf, WINDOW)

            try:
                alt = float(row[fmt.col_altitude()])
            except (ValueError, KeyError):
                continue

            alt_buf.append(alt)
            time_buf.append(time_s)
            vspeed = split_half_derivative(alt_buf, time_buf, WINDOW)
            vx = fmt.parse_vx(row)

            out_row = [f"{time_ms:.3f}", f"{avg_accel:.6f}", f"{alt:.3f}", f"{vspeed:.4f}", f"{vx:.4f}"]
            if has_tilt:
                try:
                    tilt_deg = fmt.tilt_to_deg(float(row[fmt.col_tilt()]))
                except (ValueError, KeyError):
                    tilt_deg = 0.0
                out_row.append(f"{tilt_deg:.4f}")
            writer.writerow(out_row)

    print(f"Converted {input_path} -> {output_path}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    fmt_group = parser.add_mutually_exclusive_group(required=True)
    for fmt in FORMATS.values():
        fmt_group.add_argument(f"--{fmt.name}", action="store_const", const=fmt.name, dest="format", help=fmt.description)
    parser.add_argument("--ainv", action="store_true", help="Inverts the acceleration axis")
    args = parser.parse_args()
    return convert(args.input, args.output, FORMATS[args.format], {
        "ainv": args.ainv
    })


if __name__ == "__main__":
    sys.exit(main())
