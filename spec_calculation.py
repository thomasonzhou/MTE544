import argparse
import glob
import json
import math
import os
from typing import Dict, List, Optional, Tuple

from utilities import FileReader


def _normalize_time(stamps: List[float]) -> List[float]:
    if not stamps:
        return []
    # Convert to seconds if input looks like nanoseconds
    first = stamps[0]
    scale = 1e9 if first > 1e11 else 1.0
    t0 = first / scale
    return [(s / scale) - t0 for s in stamps]


def _compute_percent_overshoot(errors: List[float]) -> float:
    if not errors:
        return 0.0
    initial = errors[0]
    amplitude = abs(initial)
    if amplitude == 0.0:
        return 0.0

    if initial > 0:
        # Look for undershoot (negative peak)
        min_val = min(errors)
        overshoot_val = max(0.0, -min_val)
    elif initial < 0:
        # Look for positive overshoot
        max_val = max(errors)
        overshoot_val = max(0.0, max_val)
    else:
        return 0.0

    return 100.0 * (overshoot_val / amplitude)


def _find_first_zero_crossing_time(times: List[float], errors: List[float]) -> Optional[float]:
    if not times or not errors or len(times) != len(errors):
        return None
    e0 = errors[0]
    for i in range(1, len(errors)):
        if errors[i] == 0.0:
            return times[i]
        if (errors[i] > 0 and e0 < 0) or (errors[i] < 0 and e0 > 0):
            # Linear interpolate crossing between i-1 and i
            t1, t2 = times[i - 1], times[i]
            e1, e2 = errors[i - 1], errors[i]
            if e2 == e1:
                return t2
            alpha = abs(e1) / (abs(e1) + abs(e2))
            return t1 + alpha * (t2 - t1)
    return None


def _find_settling_time(times: List[float], errors: List[float], band: float) -> Optional[float]:
    if not times or not errors or len(times) != len(errors):
        return None
    abs_errors = [abs(e) for e in errors]
    # Find the earliest time index such that all subsequent values stay within band
    for i in range(len(abs_errors)):
        if all(val <= band for val in abs_errors[i:]):
            return times[i]
    return None


def _find_fall_time(times: List[float], errors: List[float], high_frac: float = 0.9, low_frac: float = 0.1) -> Optional[float]:
    if not times or not errors or len(times) != len(errors):
        return None
    amplitude = abs(errors[0])
    if amplitude == 0.0:
        return 0.0
    abs_errors = [abs(e) for e in errors]
    t_high: Optional[float] = None
    t_low: Optional[float] = None
    high_th = high_frac * amplitude
    low_th = low_frac * amplitude

    for i in range(len(abs_errors)):
        if t_high is None and abs_errors[i] <= high_th:
            t_high = times[i]
        if t_low is None and abs_errors[i] <= low_th:
            t_low = times[i]
        if t_high is not None and t_low is not None:
            break

    if t_high is None or t_low is None:
        return None
    return max(0.0, t_low - t_high)


def _rms(values: List[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _steady_state_stats(values: List[float], frac_tail: float = 0.1) -> Tuple[float, float, float]:
    if not values:
        return 0.0, 0.0, 0.0
    n = len(values)
    tail_start = int((1.0 - max(0.0, min(1.0, frac_tail))) * n)
    tail = values[tail_start:] if tail_start < n else values[-1:]
    mean = sum(tail) / len(tail)
    var = sum((v - mean) * (v - mean) for v in tail) / len(tail)
    std = math.sqrt(var)
    last_abs = abs(values[-1])
    return mean, std, last_abs


def compute_metrics_for_log(csv_file: str, tol_percent: float, abs_tol: Optional[float], tail_frac: float) -> Optional[Dict[str, float]]:
    headers, rows = FileReader(csv_file).read_file()
    if not headers or not rows:
        return None
    # Expecting error logs with at least: e, ..., stamp
    try:
        idx_e = headers.index("e")
        idx_stamp = len(headers) - 1  # last is stamp
    except ValueError:
        return None

    errors = [row[idx_e] for row in rows]
    stamps = [row[idx_stamp] for row in rows]
    times = _normalize_time(stamps)

    initial_amp = abs(errors[0]) if errors else 0.0
    band = abs_tol if (abs_tol is not None and abs_tol > 0.0) else tol_percent * initial_amp

    metrics: Dict[str, float] = {}
    metrics["initial_error"] = errors[0]
    metrics["final_error"] = errors[-1]
    metrics["max_abs_error"] = max((abs(e) for e in errors), default=0.0)
    metrics["rms_error"] = _rms(errors)
    metrics["percent_overshoot"] = _compute_percent_overshoot(errors)

    t_zero = _find_first_zero_crossing_time(times, errors)
    metrics["zero_crossing_time"] = t_zero if t_zero is not None else float("nan")

    t_settle = _find_settling_time(times, errors, band) if band is not None else None
    metrics["settling_time"] = t_settle if t_settle is not None else float("nan")

    t_fall = _find_fall_time(times, errors)
    metrics["fall_time_90_to_10"] = t_fall if t_fall is not None else float("nan")

    mean_tail, std_tail, last_abs = _steady_state_stats([abs(e) for e in errors], frac_tail=tail_frac)
    metrics["steady_state_abs_mean"] = mean_tail
    metrics["steady_state_abs_std"] = std_tail
    metrics["final_abs_error"] = last_abs
    metrics["tolerance_band"] = band

    # Also report duration and sample count
    metrics["duration"] = times[-1] if times else 0.0
    metrics["num_samples"] = float(len(errors))

    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute control performance specs from log CSVs (linear.csv/angular.csv)")
    parser.add_argument("--folder", required=True, help="Folder containing CSV log files (e.g., p_point_logs)")
    parser.add_argument("--tol", type=float, default=0.1, help="Settling band as fraction of initial error (default: 0.02 = 2%)")
    parser.add_argument("--abs-tol", dest="abs_tol", type=float, default=None, help="Absolute settling band (overrides --tol if provided)")
    parser.add_argument("--tail-frac", type=float, default=0.1, help="Tail fraction for steady-state stats (default: last 10%)")
    parser.add_argument("--output", type=str, default=None, help="Optional path to write metrics JSON")
    args = parser.parse_args()

    logs_dir = args.folder
    if not os.path.isdir(logs_dir):
        raise SystemExit(f"Input folder does not exist: {logs_dir}")

    csv_files = sorted(glob.glob(os.path.join(logs_dir, "*.csv")))
    if not csv_files:
        raise SystemExit(f"No CSV files found in {logs_dir}")

    summaries: Dict[str, Dict[str, float]] = {}

    for csv_file in csv_files:
        metrics = compute_metrics_for_log(csv_file, tol_percent=args.tol, abs_tol=args.abs_tol, tail_frac=args.tail_frac)
        base = os.path.splitext(os.path.basename(csv_file))[0]
        if metrics is None:
            # Likely a non-error CSV (e.g., robot_pose); skip
            continue
        summaries[base] = metrics

    if not summaries:
        raise SystemExit("No compatible error CSVs found (expected headers including 'e' and 'stamp').")

    # Pretty print to console
    print(f"Computed specs from {logs_dir} (tol={args.tol*100:.1f}% abs_tol={args.abs_tol})\n")
    for name, m in summaries.items():
        print(f"== {name} ==")
        print(f"  samples              : {int(m['num_samples'])}")
        print(f"  duration             : {m['duration']:.3f} s")
        print(f"  initial_error        : {m['initial_error']:.6f}")
        print(f"  final_error          : {m['final_error']:.6f}")
        print(f"  final_abs_error      : {m['final_abs_error']:.6f}")
        print(f"  max_abs_error        : {m['max_abs_error']:.6f}")
        print(f"  rms_error            : {m['rms_error']:.6f}")
        print(f"  percent_overshoot    : {m['percent_overshoot']:.3f} %")
        print(f"  zero_crossing_time   : {m['zero_crossing_time']:.3f} s")
        print(f"  settling_time        : {m['settling_time']:.3f} s (band={m['tolerance_band']:.6f})")
        print(f"  fall_time_90_to_10   : {m['fall_time_90_to_10']:.3f} s")
        print(f"  steady_state_abs_mean: {m['steady_state_abs_mean']:.6f}")
        print(f"  steady_state_abs_std : {m['steady_state_abs_std']:.6f}")
        print("")

    if args.output:
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(summaries, f, indent=2)
        print(f"Saved metrics to {args.output}")


if __name__ == "__main__":
    main()


