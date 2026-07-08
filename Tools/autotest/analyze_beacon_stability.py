#!/usr/bin/env python3
'''
Analyze BIN logs produced by the ArduCopter beacon-stability parameter sweep
(test.CopterBeaconSweep in autotest.py / AutoTestCopterBeaconStabilitySweep in
arducopter.py) and produce a report ranking each swept value by measured
attitude stability during the cruise portion of its flight.

Reads the logs/beacon_stability/<param>/<value>.json sidecars written by the
sweep test (which record which param/value produced which BIN file and
whether the flight completed, timed out, or errored), computes roll/pitch
stability metrics from each completed run's ATT messages, and writes a CSV of
all runs plus a Markdown report recommending the best value per parameter
group.

The sweep flies the first part of each leg on SRC1 (GPS) and switches the
active EKF3 source set to SRC3 (Beacon) partway through -- metrics are only
meaningful for the segment actually running on the noisy beacon source, so
the analysis window is: MODE == GUIDED AND XKFS.SS == 2 (tertiary/SRC3, per
AP_NavEKF3/LogStructure.h: "Source Set (primary=0/secondary=1/tertiary=2)").
XKFS is logged periodically per EKF3 core, giving a direct sample-by-sample
record of which source set was actually active -- more robust than inferring
the switch from a one-off status-text message.

Usage:
    python3 Tools/autotest/analyze_beacon_stability.py
    python3 Tools/autotest/analyze_beacon_stability.py --root logs/beacon_stability \
        --settle-s 2.0 --out-csv report.csv --out-md report.md
'''

import argparse
import glob
import json
import os

import numpy

from pymavlink import DFReader

GUIDED_MODE_NUM = 4  # ArduCopter Mode::Number::GUIDED
SRC3_SOURCE_SET = 2  # XKFS.SS: primary=0/secondary=1/tertiary(SRC3)=2

# Weighted-sum score components; lower score = more stable. Each metric is
# min-max normalized within its own param group before weighting, so the
# score is only meaningful for comparing values within one group, not across
# groups with different units. Not a validated ground truth -- a reasonable
# starting point to iterate on.
SCORE_WEIGHTS = {
    "roll_std_deg": 0.35,
    "pitch_std_deg": 0.35,
    "roll_rms_err_deg": 0.15,
    "pitch_rms_err_deg": 0.15,
}


def discover_runs(root):
    '''yield metadata dicts loaded from each *.json sidecar under root/<param>/'''
    for param_dir in sorted(glob.glob(os.path.join(root, "*"))):
        if not os.path.isdir(param_dir):
            continue
        param_dir_name = os.path.basename(param_dir)
        for meta_path in sorted(glob.glob(os.path.join(param_dir, "*.json"))):
            with open(meta_path) as f:
                meta = json.load(f)
            meta["param_dir"] = param_dir_name
            meta["meta_path"] = meta_path
            yield meta


def load_log(path):
    '''single pass over a BIN log, returning (att, modes, src_sets) sample lists.

    att: list of (time_us, roll, pitch, desroll, despitch)
    modes: list of (time_us, mode_num)
    src_sets: list of (time_us, source_set) from XKFS.SS, core 0 only
    '''
    dfreader = DFReader.DFReader_binary(path, zero_time_base=True)
    att = []
    modes = []
    src_sets = []
    while True:
        m = dfreader.recv_match(type=['ATT', 'MODE', 'XKFS'])
        if m is None:
            break
        mtype = m.get_type()
        if mtype == 'ATT':
            att.append((m.TimeUS, m.Roll, m.Pitch, m.DesRoll, m.DesPitch))
        elif mtype == 'MODE':
            modes.append((m.TimeUS, m.ModeNum))
        else:  # XKFS
            if m.C == 0:
                src_sets.append((m.TimeUS, m.SS))
    return att, modes, src_sets


def guided_src3_window(modes, src_sets, settle_s):
    '''bound the analysis segment to MODE==GUIDED intersected with the time
    the EKF3 was actually on the tertiary (SRC3/Beacon) source set, per
    XKFS.SS -- metrics from the initial SRC1 (GPS) portion of the leg, or
    from LAND, aren't representative of the parameter under test. settle_s
    skips a short buffer right after the source switch to avoid scoring the
    one-off transient of the EKF re-converging onto the new source.
    Raises ValueError if either segment can't be found or the intersection
    is empty/too short.'''
    if not modes:
        raise ValueError("no MODE samples in log")
    if not src_sets:
        raise ValueError("no XKFS samples in log")

    # MODE is logged on every mode-set call, including duplicate re-asserts of
    # the same mode number (not just on an actual mode change) -- so the real
    # exit from GUIDED is the first *different* mode_num after guided_start,
    # not simply the next MODE record.
    guided_start = None
    guided_end = None
    for t, mode_num in modes:
        if mode_num == GUIDED_MODE_NUM:
            if guided_start is None:
                guided_start = t
        elif guided_start is not None:
            guided_end = t
            break
    if guided_start is None:
        raise ValueError("vehicle never entered GUIDED mode")
    if guided_end is None:
        guided_end = max(modes[-1][0], src_sets[-1][0])

    src3_times = [t for t, ss in src_sets if ss == SRC3_SOURCE_SET]
    if not src3_times:
        raise ValueError("EKF3 never reported source set SRC3 (XKFS.SS==%d)" % SRC3_SOURCE_SET)
    src3_start = min(src3_times)
    src3_end = max(src3_times)

    t_start = max(guided_start, src3_start) + settle_s * 1e6
    t_end = min(guided_end, src3_end)
    if t_end <= t_start:
        raise ValueError(
            "empty GUIDED/SRC3 intersection (guided=[%d,%d], src3=[%d,%d])" %
            (guided_start, guided_end, src3_start, src3_end))
    return t_start, t_end


def zero_crossing_rate(x, t_us):
    '''oscillations/sec of x around its own mean over the given time base -- a
    diagnostic indicator of attitude "hunting" frequency, reported but not
    scored.'''
    centred = x - x.mean()
    crossings = numpy.sum(numpy.diff(numpy.sign(centred)) != 0)
    duration_s = (t_us[-1] - t_us[0]) * 1e-6
    return float(crossings / duration_s) if duration_s > 0 else 0.0


def metrics_for_run(att, t_start, t_end):
    window = [w for w in att if t_start <= w[0] <= t_end]
    if len(window) < 20:
        raise ValueError("too few ATT samples in cruise window (%d)" % len(window))
    t_us = numpy.array([w[0] for w in window], dtype=float)
    roll = numpy.array([w[1] for w in window])
    pitch = numpy.array([w[2] for w in window])
    roll_err = roll - numpy.array([w[3] for w in window])
    pitch_err = pitch - numpy.array([w[4] for w in window])
    return {
        "roll_std_deg": float(numpy.std(roll)),
        "pitch_std_deg": float(numpy.std(pitch)),
        "roll_rms_err_deg": float(numpy.sqrt(numpy.mean(roll_err ** 2))),
        "pitch_rms_err_deg": float(numpy.sqrt(numpy.mean(pitch_err ** 2))),
        "roll_zero_cross_hz": zero_crossing_rate(roll, t_us),
        "pitch_zero_cross_hz": zero_crossing_rate(pitch, t_us),
    }


def normalize(values):
    v = numpy.array(values, dtype=float)
    lo, hi = v.min(), v.max()
    if hi - lo < 1e-9:
        return numpy.zeros_like(v)
    return (v - lo) / (hi - lo)


def score_group(rows):
    '''rows: list of metrics dicts for one param group's scored (non-failed)
    runs. Returns a parallel list of scores, lower = more stable.'''
    norm_cols = {k: normalize([r[k] for r in rows]) for k in SCORE_WEIGHTS}
    return [
        sum(SCORE_WEIGHTS[k] * norm_cols[k][i] for k in SCORE_WEIGHTS)
        for i in range(len(rows))
    ]


def analyze(root, settle_s):
    '''returns (all_rows, groups) where all_rows is a flat list of per-run
    dicts (for the CSV) and groups is {param_dir: [row, ...]} for the
    Markdown report, each row annotated with metrics/score or an error.'''
    all_rows = []
    groups = {}

    for meta in discover_runs(root):
        row = {
            "param_dir": meta["param_dir"],
            "param_names": ",".join(meta["param_names"]),
            "value": meta["value"],
            "status": meta.get("status", "completed"),
            "log_path": meta["log_path"],
            "error": "",
        }
        for k in SCORE_WEIGHTS:
            row[k] = None
        row["roll_zero_cross_hz"] = None
        row["pitch_zero_cross_hz"] = None
        row["score"] = None

        # Score regardless of status: a "timed_out" run (never reached the
        # final target) still flew a real GUIDED+SRC3 segment up until the
        # timeout, and that segment is exactly what we want to measure. Only
        # "error" runs that never got the vehicle flying at all are likely to
        # come up empty here, and that'll surface naturally as an error below.
        try:
            att, modes, src_sets = load_log(meta["log_path"])
            t_start, t_end = guided_src3_window(modes, src_sets, settle_s)
            metrics = metrics_for_run(att, t_start, t_end)
            row.update(metrics)
        except Exception as e:
            row["error"] = str(e)

        all_rows.append(row)
        groups.setdefault(meta["param_dir"], []).append(row)

    for param_dir, rows in groups.items():
        scorable = [r for r in rows if not r["error"]]
        if not scorable:
            continue
        scores = score_group(scorable)
        for r, s in zip(scorable, scores):
            r["score"] = s

    return all_rows, groups


def write_csv(path, rows):
    fieldnames = [
        "param_dir", "param_names", "value", "status",
        "roll_std_deg", "pitch_std_deg", "roll_rms_err_deg", "pitch_rms_err_deg",
        "roll_zero_cross_hz", "pitch_zero_cross_hz", "score", "error", "log_path",
    ]
    with open(path, "w") as f:
        f.write(",".join(fieldnames) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(k, "")) for k in fieldnames) + "\n")


def write_markdown(path, groups):
    lines = ["# Beacon Stability Sweep Report", ""]
    lines.append(
        "Lower score = more stable (weighted, per-group-normalized combination of "
        "roll/pitch std-dev and RMS tracking error, measured only over the segment "
        "flying GUIDED on SRC3/Beacon; weights: %s). Runs that timed out, errored, "
        "or never reached a GUIDED+SRC3 segment are listed but excluded from "
        "scoring." % SCORE_WEIGHTS)
    lines.append("")

    summary = ["", "## Summary", "", "| Param | Recommended value | Score |", "|---|---|---|"]

    for param_dir in sorted(groups):
        rows = groups[param_dir]
        scored = sorted([r for r in rows if r["score"] is not None], key=lambda r: r["score"])
        unscored = [r for r in rows if r["score"] is None]

        lines.append("## %s" % param_dir)
        lines.append("")
        lines.append(
            "| Value | roll_std | pitch_std | roll_rms_err | pitch_rms_err | "
            "roll_zero_cross_hz | pitch_zero_cross_hz | score |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for i, r in enumerate(scored):
            marker = "**Recommended** " if i == 0 else ""
            lines.append("| %s%s | %.3f | %.3f | %.3f | %.3f | %.2f | %.2f | %.4f |" % (
                marker, r["value"], r["roll_std_deg"], r["pitch_std_deg"],
                r["roll_rms_err_deg"], r["pitch_rms_err_deg"],
                r["roll_zero_cross_hz"], r["pitch_zero_cross_hz"], r["score"]))
        for r in unscored:
            lines.append("| %s | - | - | - | - | - | - | (%s) |" % (r["value"], r["error"]))
        lines.append("")

        if scored:
            summary.append("| %s | %s | %.4f |" % (param_dir, scored[0]["value"], scored[0]["score"]))
        else:
            summary.append("| %s | (no scorable runs) | - |" % param_dir)

    lines.extend(summary)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=os.path.join("logs", "beacon_stability"))
    parser.add_argument(
        "--settle-s", type=float, default=2.0,
        help="seconds to skip right after the SRC1->SRC3 switch, to exclude the EKF's "
             "one-off re-convergence transient from the stability metrics")
    parser.add_argument("--out-csv", default="beacon_stability_report.csv")
    parser.add_argument("--out-md", default="beacon_stability_report.md")
    args = parser.parse_args()

    all_rows, groups = analyze(args.root, args.settle_s)
    if not all_rows:
        print("no runs found under %s" % args.root)
        return

    write_csv(args.out_csv, all_rows)
    write_markdown(args.out_md, groups)
    print("wrote %s (%d runs) and %s" % (args.out_csv, len(all_rows), args.out_md))


if __name__ == '__main__':
    main()
