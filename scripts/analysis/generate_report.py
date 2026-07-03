"""
generate_report.py

Writes CSV (and one JSON) summary files for a run comparison produced by
compute_metrics.compare_runs().
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd


def write_reports(result: dict, output_dir: Path) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Full per-cluster comparison: every metric, one row per matched cluster.
    per_cluster_path = output_dir / "per_cluster_comparison.csv"
    result["per_cluster"].to_csv(per_cluster_path, index=False)

    # Clusters that appeared in only one run (cluster count differences,
    # made concrete rather than just a single number).
    pd.DataFrame(result["unmatched_a"]).to_csv(output_dir / "unmatched_clusters_run_a.csv", index=False)
    pd.DataFrame(result["unmatched_b"]).to_csv(output_dir / "unmatched_clusters_run_b.csv", index=False)

    # Aggregate (mean/max/std) metrics across all matched clusters, in tidy
    # long format so it's easy to skim or plot directly.
    agg_rows = [{"metric": k, "value": v} for k, v in result["aggregate"].items()]
    pd.DataFrame(agg_rows).to_csv(output_dir / "aggregate_metrics.csv", index=False)

    # Cluster-count summary, as both CSV (for spreadsheet consumers) and
    # JSON (for scripts/CI consumers).
    cc = result["cluster_count"]
    pd.DataFrame([cc]).to_csv(output_dir / "cluster_count_summary.csv", index=False)
    with open(output_dir / "cluster_count_summary.json", "w") as f:
        json.dump(cc, f, indent=2)

    print(f"Wrote CSV summaries to {output_dir}")
