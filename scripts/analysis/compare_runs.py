#!/usr/bin/env python3
"""
compare_runs.py

Compares two perception_sim runs (e.g. one built against stock Eigen, one
against a modified Eigen) using the CSV instrumentation written to
data/csv/*.csv, and produces:

  - CSV summaries: per-cluster comparison, unmatched-cluster lists,
    aggregate (mean/max/std) metrics, cluster-count summary
  - Plots of numerical differences: per-cluster bar charts, error
    histograms, a centroid-vs-orientation-error scatter, and a headline
    metrics summary
  - A top-down visualization overlay of both runs' bounding boxes and
    centroids, with displacement lines between matched clusters

Metrics computed per matched cluster pair:
  - centroid error            (Euclidean distance between cluster centers)
  - bounding box error        (center distance, per-axis half-extent
                                 difference, volume difference)
  - orientation error         (geodesic angle between PCA/box axes, in
                                 degrees, after correcting for the
                                 sign-ambiguity inherent to eigenvectors)
  - cluster count differences (matched / unmatched counts between runs)

Clusters are matched between runs by nearest centroid (Hungarian
algorithm), not by index, since a numerical change could in principle
shift which points survive filtering/clustering upstream of PCA.

Usage:
    python3 compare_runs.py RUN_ID_A RUN_ID_B \\
        --csv-dir data/csv \\
        --output-dir data/comparison/RUN_A_vs_RUN_B

Find available run_ids first with:
    python3 list_runs.py --csv-dir data/csv
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import compute_metrics
import generate_report
import generate_plots
import generate_overlay


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare two perception_sim runs (e.g. stock Eigen vs. modified Eigen).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("run_id_a", help="run_id of the baseline run (e.g. stock Eigen)")
    parser.add_argument("run_id_b", help="run_id of the comparison run (e.g. modified Eigen)")
    parser.add_argument("--csv-dir", default="data/csv",
                         help="Directory containing perception_sim's CSV instrumentation (default: data/csv)")
    parser.add_argument("--output-dir", default=None,
                         help="Directory to write comparison outputs "
                              "(default: <csv-dir>/../comparison/<run_a>_vs_<run_b>)")
    args = parser.parse_args()

    csv_dir = Path(args.csv_dir)
    if not csv_dir.exists():
        print(f"error: csv-dir not found: {csv_dir}", file=sys.stderr)
        return 1

    output_dir = (
        Path(args.output_dir) if args.output_dir
        else csv_dir.parent / "comparison" / f"{args.run_id_a}_vs_{args.run_id_b}"
    )

    print(f"Comparing run_a={args.run_id_a!r} vs run_b={args.run_id_b!r}")
    print(f"Reading CSVs from: {csv_dir}")
    print(f"Writing outputs to: {output_dir}")

    result = compute_metrics.compare_runs(csv_dir, args.run_id_a, args.run_id_b)

    cc = result["cluster_count"]
    print(
        f"\nCluster counts: run_a={cc['num_clusters_a']}  run_b={cc['num_clusters_b']}  "
        f"diff={cc['cluster_count_diff']}  matched={cc['num_matched']}  "
        f"unmatched_a={cc['num_unmatched_a']}  unmatched_b={cc['num_unmatched_b']}"
    )

    generate_report.write_reports(result, output_dir)

    if result["per_cluster"].empty:
        print("\nNo matched clusters between the two runs -- skipping plots/overlay.")
        return 0

    generate_plots.generate_plots(result, output_dir)
    generate_overlay.generate_overlay(result, output_dir)

    print("\nAggregate metrics (across matched clusters):")
    for k, v in result["aggregate"].items():
        print(f"  {k}: {v:.6g}")

    print(f"\nDone. See {output_dir} for full outputs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
