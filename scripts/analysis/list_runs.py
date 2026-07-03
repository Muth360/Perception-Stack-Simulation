#!/usr/bin/env python3
"""
list_runs.py

Lists every run_id found in perception_sim's CSV instrumentation, along with
when it started and how many clusters it produced, to help you pick which
two run_ids to pass to compare_runs.py.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser(description="List run_ids available for comparison.")
    parser.add_argument("--csv-dir", default="data/csv",
                         help="Directory containing perception_sim's CSV instrumentation (default: data/csv)")
    args = parser.parse_args()

    csv_dir = Path(args.csv_dir)
    path = csv_dir / "cluster_centers.csv"
    if not path.exists():
        print(f"error: {path} not found. Run perception_sim_app at least once first.", file=sys.stderr)
        return 1

    df = pd.read_csv(path)
    summary = (
        df.groupby("run_id")
        .agg(first_seen_ms=("timestamp_ms", "min"),
             last_seen_ms=("timestamp_ms", "max"),
             num_clusters=("run_id", "count"))
        .reset_index()
        .sort_values("first_seen_ms")
    )
    summary["first_seen"] = pd.to_datetime(summary["first_seen_ms"], unit="ms")

    print(summary[["run_id", "first_seen", "num_clusters"]].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
