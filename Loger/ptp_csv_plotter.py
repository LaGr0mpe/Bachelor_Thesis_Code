#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PTP CSV Plotter

Post-processing script for logs produced by the STM32 PTP multislave logger.
It reads one or more CSV files, generates publication-friendly plots, and saves
them in vector formats such as SVG and PDF.

Typical usage:
    python ptp_csv_plotter.py ptp_multislave_log_20260425_221500.csv

    python ptp_csv_plotter.py "ptp_multislave_log_*.csv" --format svg pdf

    python ptp_csv_plotter.py logs/*.csv --out-dir figures --x time --show

Plot only a selected sample window:
    python ptp_csv_plotter.py log.csv --sample-from 5000 --sample-to 20000

Create several windows from the same log:
    python ptp_csv_plotter.py log.csv \
        --sample-range warmup=0:5000 \
        --sample-range steady=5000:30000 \
        --sample-range late=30000:
"""

from __future__ import annotations

import argparse
import glob
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt


# Keep text editable in SVG/PDF. This is useful for thesis/paper figures.
matplotlib.rcParams["svg.fonttype"] = "none"
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42

# A clean default look. No external style packages are required.
matplotlib.rcParams["axes.grid"] = True
matplotlib.rcParams["grid.alpha"] = 0.35
matplotlib.rcParams["axes.axisbelow"] = True
matplotlib.rcParams["figure.dpi"] = 110


PLOT_METRICS = [
    # column, title, y label, draw zero line
    ("offset_ns", "Raw offset", "Offset, ns", True),
    ("offset_avg_ns", "Filtered offset", "Offset, ns", True),
    ("mean_path_delay_ns", "Mean path delay", "Delay, ns", False),
    ("freq_err_ppb", "Frequency error", "Frequency error, ppb", True),
    ("current_addend", "Current addend", "Addend", False),
    ("pi_output_ppb", "PI output", "PI output, ppb", True),
]

REJECTION_METRICS = [
    ("last_sample_rejected", "Last sample rejected", "0/1", False),
    ("rejected_sample_count", "Rejected samples total", "count", False),
    ("reject_mpd_count", "Rejected by MPD", "count", False),
    ("reject_abs_offset_count", "Rejected by abs. offset", "count", False),
    ("reject_jump_count", "Rejected by jump", "count", False),
    ("delayreq_timeout_count", "DelayReq timeouts", "count", False),
    ("delayresp_ignored_count", "Ignored DelayResp", "count", False),
]

REQUIRED_COLUMNS = {
    "pc_time_iso",
    "slave",
    "sample",
    "offset_ns",
    "offset_avg_ns",
    "mean_path_delay_ns",
    "freq_err_ppb",
    "current_addend",
    "pi_output_ppb",
}


@dataclass(frozen=True)
class SampleWindow:
    """A sample interval for plotting. Bounds are inclusive."""

    label: str
    sample_from: int | None = None
    sample_to: int | None = None

    @property
    def is_full(self) -> bool:
        return self.sample_from is None and self.sample_to is None

    def title_suffix(self) -> str:
        if self.is_full:
            return "full log"
        left = "start" if self.sample_from is None else str(self.sample_from)
        right = "end" if self.sample_to is None else str(self.sample_to)
        return f"samples {left}..{right}"


def safe_name(text: str) -> str:
    """Make a string safe for file/folder names on Windows/Linux/macOS."""
    text = text.strip()
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text)
    return text.strip("_") or "window"


def make_window_label(sample_from: int | None, sample_to: int | None) -> str:
    left = "start" if sample_from is None else str(sample_from)
    right = "end" if sample_to is None else str(sample_to)
    return f"samples_{left}_{right}"


def parse_optional_int(value: str) -> int | None:
    value = value.strip()
    if value == "":
        return None
    return int(value)


def parse_sample_range(spec: str) -> SampleWindow:
    """
    Parse a sample window.

    Supported forms:
        5000:20000
        5000:
        :5000
        steady=5000:20000
        steady:5000:20000
    """
    spec = spec.strip()
    if not spec:
        raise ValueError("empty --sample-range value")

    label: str | None = None
    range_part = spec

    if "=" in spec:
        label, range_part = spec.split("=", 1)
    else:
        # Also support label:from:to, but keep plain from:to working.
        parts = spec.split(":")
        if len(parts) == 3:
            label = parts[0]
            range_part = f"{parts[1]}:{parts[2]}"

    if ":" not in range_part:
        raise ValueError(
            f"wrong --sample-range format: {spec!r}. "
            "Use FROM:TO, FROM:, :TO, or LABEL=FROM:TO."
        )

    left, right = range_part.split(":", 1)
    sample_from = parse_optional_int(left)
    sample_to = parse_optional_int(right)

    if sample_from is not None and sample_to is not None and sample_from > sample_to:
        raise ValueError(f"wrong --sample-range {spec!r}: FROM is greater than TO")

    if label is None or not label.strip():
        label = make_window_label(sample_from, sample_to)

    return SampleWindow(
        label=safe_name(label),
        sample_from=sample_from,
        sample_to=sample_to,
    )


def build_windows(args: argparse.Namespace) -> list[SampleWindow]:
    if args.sample_range:
        if args.sample_from is not None or args.sample_to is not None:
            raise ValueError(
                "Use either --sample-from/--sample-to or --sample-range, not both."
            )
        return [parse_sample_range(spec) for spec in args.sample_range]

    if args.sample_from is not None or args.sample_to is not None:
        if args.sample_from is not None and args.sample_to is not None and args.sample_from > args.sample_to:
            raise ValueError("--sample-from must not be greater than --sample-to")
        return [
            SampleWindow(
                label=make_window_label(args.sample_from, args.sample_to),
                sample_from=args.sample_from,
                sample_to=args.sample_to,
            )
        ]

    return [SampleWindow(label="full")]


def expand_input_files(patterns: Iterable[str]) -> list[Path]:
    """Expand file names and glob patterns in a platform-independent way."""
    files: list[Path] = []

    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            files.extend(Path(m) for m in matches)
        else:
            p = Path(pattern)
            if p.exists():
                files.append(p)
            else:
                print(f"Warning: no files matched: {pattern}", file=sys.stderr)

    # Remove duplicates while keeping order.
    unique: list[Path] = []
    seen: set[Path] = set()
    for p in files:
        p = p.resolve()
        if p not in seen:
            seen.add(p)
            unique.append(p)

    return unique


def load_log(path: Path) -> pd.DataFrame:
    """Load one CSV log and convert numeric/time columns."""
    df = pd.read_csv(path)

    missing = sorted(REQUIRED_COLUMNS - set(df.columns))
    if missing:
        raise ValueError(f"{path.name}: missing required columns: {', '.join(missing)}")

    df["pc_time_iso"] = pd.to_datetime(df["pc_time_iso"], errors="coerce")

    # Convert all non-string columns that may be present.
    for col in df.columns:
        if col not in {"pc_time_iso", "slave", "port"}:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # Drop rows where the essential fields were not parsed.
    df = df.dropna(subset=["slave", "sample"])
    df = df.sort_values(["slave", "sample"]).reset_index(drop=True)

    return df


def filter_by_sample_window(df: pd.DataFrame, window: SampleWindow) -> pd.DataFrame:
    """Return only rows inside the requested sample interval."""
    filtered = df

    if window.sample_from is not None:
        filtered = filtered[filtered["sample"] >= window.sample_from]

    if window.sample_to is not None:
        filtered = filtered[filtered["sample"] <= window.sample_to]

    return filtered.copy()


def make_x_values(group: pd.DataFrame, x_mode: str) -> tuple[pd.Series, str]:
    """Return x-axis values and label."""
    if x_mode == "time":
        if group["pc_time_iso"].notna().any():
            t0 = group["pc_time_iso"].dropna().iloc[0]
            elapsed_s = (group["pc_time_iso"] - t0).dt.total_seconds()
            return elapsed_s, "Elapsed time from window start, s"

        # Fallback if timestamps are invalid.
        return group["sample"], "Sample"

    return group["sample"], "Sample"


def set_reasonable_ylim(ax, y: pd.Series) -> None:
    """Set y limits with a small margin, ignoring NaN values."""
    y = y.dropna()
    if y.empty:
        return

    ymin = float(y.min())
    ymax = float(y.max())

    if math.isclose(ymin, ymax):
        margin = max(1.0, abs(ymin) * 0.1 + 1.0)
    else:
        margin = max((ymax - ymin) * 0.1, 1.0)

    ax.set_ylim(ymin - margin, ymax + margin)


def save_figure(fig, base_path: Path, formats: list[str], dpi: int) -> None:
    base_path.parent.mkdir(parents=True, exist_ok=True)

    for fmt in formats:
        out_path = base_path.with_suffix(f".{fmt}")
        if fmt.lower() in {"png", "jpg", "jpeg"}:
            fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        else:
            fig.savefig(out_path, bbox_inches="tight")

        print(f"Saved: {out_path}")


def plot_slave_dashboard(
    df: pd.DataFrame,
    slave: str,
    log_name: str,
    window: SampleWindow,
    out_base: Path,
    formats: list[str],
    x_mode: str,
    dpi: int,
) -> None:
    """Create one 2x3 dashboard for a single slave."""
    group = df[df["slave"] == slave].copy()
    if group.empty:
        return

    fig, axes = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
    axes_flat = axes.ravel()

    x, xlabel = make_x_values(group, x_mode)

    for ax, (col, title, ylabel, zero_line) in zip(axes_flat, PLOT_METRICS):
        if col not in group.columns:
            ax.set_visible(False)
            continue

        ax.plot(x, group[col], linewidth=1.1)
        if zero_line:
            ax.axhline(0, linewidth=0.9, linestyle="--")

        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        set_reasonable_ylim(ax, group[col])

    fig.suptitle(f"{log_name} | {slave} | {window.title_suffix()}", fontsize=14)
    save_figure(fig, out_base, formats, dpi)
    plt.close(fig)


def plot_comparison_metric(
    df: pd.DataFrame,
    metric: tuple[str, str, str, bool],
    log_name: str,
    window: SampleWindow,
    out_base: Path,
    formats: list[str],
    x_mode: str,
    dpi: int,
) -> None:
    """Create one figure comparing all slaves for one metric."""
    col, title, ylabel, zero_line = metric

    if col not in df.columns:
        return

    fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)

    any_data = False
    xlabel = "Sample"
    all_y: list[pd.Series] = []

    for slave, group in df.groupby("slave", sort=True):
        x, xlabel = make_x_values(group, x_mode)
        y = group[col]

        if y.dropna().empty:
            continue

        ax.plot(x, y, linewidth=1.1, label=str(slave))
        all_y.append(y)
        any_data = True

    if not any_data:
        plt.close(fig)
        return

    if zero_line:
        ax.axhline(0, linewidth=0.9, linestyle="--")

    ax.set_title(f"{log_name} | {title} | {window.title_suffix()}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()

    # Use a common y-scale based only on the selected window.
    set_reasonable_ylim(ax, pd.concat(all_y, ignore_index=True))

    save_figure(fig, out_base, formats, dpi)
    plt.close(fig)


def plot_rejection_dashboard(
    df: pd.DataFrame,
    log_name: str,
    window: SampleWindow,
    out_base: Path,
    formats: list[str],
    x_mode: str,
    dpi: int,
) -> None:
    """Create a compact dashboard for rejection counters."""
    available = [m for m in REJECTION_METRICS if m[0] in df.columns]
    if not available:
        return

    n = len(available)
    ncols = 2
    nrows = math.ceil(n / ncols)

    fig, axes = plt.subplots(nrows, ncols, figsize=(13, 3.4 * nrows), constrained_layout=True)
    axes_flat = list(axes.ravel()) if hasattr(axes, "ravel") else [axes]

    for ax, (col, title, ylabel, zero_line) in zip(axes_flat, available):
        for slave, group in df.groupby("slave", sort=True):
            x, xlabel = make_x_values(group, x_mode)
            ax.plot(x, group[col], linewidth=1.1, label=str(slave))

        if zero_line:
            ax.axhline(0, linewidth=0.9, linestyle="--")

        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.legend()

    for ax in axes_flat[len(available):]:
        ax.set_visible(False)

    fig.suptitle(f"{log_name} | Rejection counters | {window.title_suffix()}", fontsize=14)
    save_figure(fig, out_base, formats, dpi)
    plt.close(fig)


def calculate_stats(df: pd.DataFrame, log_name: str, window: SampleWindow) -> pd.DataFrame:
    """Calculate basic statistics per slave for the selected sample window."""
    rows = []

    for slave, group in df.groupby("slave", sort=True):
        row = {
            "log": log_name,
            "window": window.label,
            "window_sample_from_requested": window.sample_from,
            "window_sample_to_requested": window.sample_to,
            "slave": slave,
            "rows": len(group),
            "sample_min": group["sample"].min(),
            "sample_max": group["sample"].max(),
        }

        for col in [
            "rejected_sample_count",
            "reject_mpd_count",
            "reject_abs_offset_count",
            "reject_jump_count",
            "delayreq_timeout_count",
            "delayresp_ignored_count",
            "phase_step_count",
            "phase_capture_step_count",
            "freq_update_count",
            "sync_rx_ts_valid_count",
            "tx_ts_seen",
        ]:
            if col not in group.columns:
                continue

            s = group[col].dropna()
            if s.empty:
                continue

            row[f"{col}_mean"] = s.mean()
            row[f"{col}_mean_abs"] = s.abs().mean()
            row[f"{col}_std"] = s.std(ddof=1)
            row[f"{col}_min"] = s.min()
            row[f"{col}_max"] = s.max()

        for col in [
            "rejected_sample_count",
            "reject_mpd_count",
            "reject_abs_offset_count",
            "reject_jump_count",
            "phase_step_count",
            "phase_capture_step_count",
            "freq_update_count",
            "sync_rx_ts_valid_count",
            "tx_ts_seen",
        ]:
            if col not in group.columns:
                continue

            s = group[col].dropna()
            if s.empty:
                continue

            row[f"{col}_first"] = s.iloc[0]
            row[f"{col}_last"] = s.iloc[-1]
            row[f"{col}_delta"] = s.iloc[-1] - s.iloc[0]

        rows.append(row)

    return pd.DataFrame(rows)


def process_window(
    df_full: pd.DataFrame,
    csv_path: Path,
    out_dir: Path,
    formats: list[str],
    x_mode: str,
    dpi: int,
    show: bool,
    make_rejection_plots: bool,
    window: SampleWindow,
) -> None:
    log_name = csv_path.stem
    df = filter_by_sample_window(df_full, window)

    if df.empty:
        print(f"Warning: {csv_path.name}: no rows in window {window.title_suffix()}", file=sys.stderr)
        return

    file_out_dir = out_dir / log_name if window.is_full else out_dir / log_name / window.label
    file_out_dir.mkdir(parents=True, exist_ok=True)

    base_name = log_name if window.is_full else f"{log_name}_{window.label}"

    print(f"  Window: {window.title_suffix()} -> rows={len(df)}")

    # Statistics for the same selected window.
    stats = calculate_stats(df, log_name, window)
    stats_path = file_out_dir / f"{base_name}_summary_stats.csv"
    stats.to_csv(stats_path, index=False)
    print(f"Saved: {stats_path}")

    # One dashboard per slave.
    for slave in sorted(df["slave"].dropna().unique()):
        safe_slave = safe_name(str(slave))
        plot_slave_dashboard(
            df=df,
            slave=slave,
            log_name=log_name,
            window=window,
            out_base=file_out_dir / f"{base_name}_{safe_slave}_dashboard",
            formats=formats,
            x_mode=x_mode,
            dpi=dpi,
        )

    # One comparison plot per important metric.
    for metric in PLOT_METRICS:
        col = metric[0]
        plot_comparison_metric(
            df=df,
            metric=metric,
            log_name=log_name,
            window=window,
            out_base=file_out_dir / f"{base_name}_compare_{col}",
            formats=formats,
            x_mode=x_mode,
            dpi=dpi,
        )

    if make_rejection_plots:
        plot_rejection_dashboard(
            df=df,
            log_name=log_name,
            window=window,
            out_base=file_out_dir / f"{base_name}_rejections",
            formats=formats,
            x_mode=x_mode,
            dpi=dpi,
        )

    if show:
        # Re-open a compact preview window after saving.
        for slave in sorted(df["slave"].dropna().unique()):
            group = df[df["slave"] == slave]
            x, xlabel = make_x_values(group, x_mode)

            fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
            ax.plot(x, group["offset_ns"], linewidth=1.1, label="raw offset")
            ax.plot(x, group["offset_avg_ns"], linewidth=1.1, label="filtered offset")
            ax.axhline(0, linewidth=0.9, linestyle="--")
            ax.set_title(f"{log_name} | {slave} | Offset preview | {window.title_suffix()}")
            ax.set_xlabel(xlabel)
            ax.set_ylabel("Offset, ns")
            ax.legend()
            set_reasonable_ylim(ax, pd.concat([group["offset_ns"], group["offset_avg_ns"]], ignore_index=True))
            plt.show()


def process_file(
    csv_path: Path,
    out_dir: Path,
    formats: list[str],
    x_mode: str,
    dpi: int,
    show: bool,
    make_rejection_plots: bool,
    windows: list[SampleWindow],
) -> None:
    print(f"\nProcessing: {csv_path}")
    df_full = load_log(csv_path)

    for window in windows:
        process_window(
            df_full=df_full,
            csv_path=csv_path,
            out_dir=out_dir,
            formats=formats,
            x_mode=x_mode,
            dpi=dpi,
            show=show,
            make_rejection_plots=make_rejection_plots,
            window=window,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and save plots from STM32 PTP multislave CSV logs."
    )

    parser.add_argument(
        "csv_files",
        nargs="+",
        help="CSV file paths or glob patterns, for example: ptp_multislave_log_*.csv",
    )
    parser.add_argument(
        "--out-dir",
        default="ptp_figures",
        help="Output directory for figures and summary CSV files. Default: ptp_figures",
    )
    parser.add_argument(
        "--format",
        nargs="+",
        default=["svg", "pdf"],
        choices=["svg", "pdf", "png", "jpg", "jpeg"],
        help="Output formats. Default: svg pdf",
    )
    parser.add_argument(
        "--x",
        choices=["sample", "time"],
        default="sample",
        help="X axis mode. Default: sample",
    )
    parser.add_argument(
        "--sample-from",
        type=int,
        default=None,
        help="Plot only samples with sample >= this value. Example: --sample-from 5000",
    )
    parser.add_argument(
        "--sample-to",
        type=int,
        default=None,
        help="Plot only samples with sample <= this value. Example: --sample-to 30000",
    )
    parser.add_argument(
        "--sample-range",
        action="append",
        default=[],
        help=(
            "Named or unnamed sample window. Can be used multiple times. "
            "Examples: --sample-range 5000:30000, --sample-range steady=5000:30000, "
            "--sample-range late=30000:"
        ),
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="DPI for raster formats such as PNG/JPG. Default: 300",
    )
    parser.add_argument(
        "--no-rejections",
        action="store_true",
        help="Do not generate rejection counter plots.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show quick preview windows after saving figures.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        windows = build_windows(args)
    except Exception as exc:
        print(f"Argument error: {exc}", file=sys.stderr)
        return 2

    csv_files = expand_input_files(args.csv_files)
    if not csv_files:
        print("No CSV files found.", file=sys.stderr)
        return 1

    out_dir = Path(args.out_dir)

    for csv_path in csv_files:
        try:
            process_file(
                csv_path=csv_path,
                out_dir=out_dir,
                formats=args.format,
                x_mode=args.x,
                dpi=args.dpi,
                show=args.show,
                make_rejection_plots=not args.no_rejections,
                windows=windows,
            )
        except Exception as exc:
            print(f"Error while processing {csv_path}: {exc}", file=sys.stderr)

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
