import csv
import sys
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


FIELDNAMES = [
    "sample",
    "offset_ns",
    "raw_offset_ns",
    "offset_avg_ns",
    "mean_path_delay_ns",
    "freq_err_ppb",
    "current_addend",
    "last_addend_step",
    "phase_step_count",
    "phase_capture_step_count",
    "freq_update_count",
    "coarse_mode",
    "sync_rx_ts_valid_count",
    "tx_ts_seen",
    "delayreq_timeout_count",
    "delayresp_ignored_count",
    "pi_prop_ppb",
    "pi_integral_ppb",
    "pi_output_ppb",
    "last_sample_rejected",
    "rejected_sample_count",
    "reject_mpd_count",
    "reject_abs_offset_count",
    "reject_jump_count",
]


PLOTS = [
    ("Offset", "offset_ns", "offset_ns"),
    ("Raw Offset", "raw_offset_ns", "raw_offset_ns"),
    ("Offset Avg", "offset_avg_ns", "offset_avg_ns"),
    ("Mean Path Delay", "mean_path_delay_ns", "mean_path_delay_ns"),
    ("Freq Error", "freq_err_ppb", "freq_err_ppb"),
    ("Current Addend", "current_addend", "current_addend"),
    ("Last Addend Step", "last_addend_step", "last_addend_step"),
    ("Phase Step Count", "phase_step_count", "phase_step_count"),
    ("Phase Capture Step Count", "phase_capture_step_count", "phase_capture_step_count"),
    ("Freq Update Count", "freq_update_count", "freq_update_count"),
    ("Coarse Mode", "coarse_mode", "coarse_mode"),
    ("RX Timestamp Count", "sync_rx_ts_valid_count", "sync_rx_ts_valid_count"),
    ("TX Timestamp Count", "tx_ts_seen", "tx_ts_seen"),
    ("DelayReq Timeout Count", "delayreq_timeout_count", "delayreq_timeout_count"),
    ("Ignored DelayResp Count", "delayresp_ignored_count", "delayresp_ignored_count"),
    ("PI Prop", "pi_prop_ppb", "pi_prop_ppb"),
    ("PI Integral", "pi_integral_ppb", "pi_integral_ppb"),
    ("PI Output", "pi_output_ppb", "pi_output_ppb"),
    ("Last Sample Rejected", "last_sample_rejected", "last_sample_rejected"),
    ("Rejected Sample Count", "rejected_sample_count", "rejected_sample_count"),
    ("Reject MPD Count", "reject_mpd_count", "reject_mpd_count"),
    ("Reject Abs Offset Count", "reject_abs_offset_count", "reject_abs_offset_count"),
    ("Reject Jump Count", "reject_jump_count", "reject_jump_count"),
]


def get_int(row, key, default=0):
    value = row.get(key, "")

    if value is None or value == "":
        return default

    return int(value)


def load_csv(csv_file: str):
    data = {name: [] for name in FIELDNAMES}

    with open(csv_file, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            for name in FIELDNAMES:
                data[name].append(get_int(row, name, default=0))

    return data


def add_plot_tab(notebook: ttk.Notebook, title: str, x, y, ylabel: str):
    frame = ttk.Frame(notebook)
    notebook.add(frame, text=title)

    fig = Figure(figsize=(9, 5), dpi=100)
    ax = fig.add_subplot(111)

    ax.plot(x, y)
    ax.set_title(title)
    ax.set_xlabel("sample")
    ax.set_ylabel(ylabel)
    ax.grid(True)

    if "offset" in ylabel.lower() or "ppb" in ylabel.lower():
        ax.axhline(0, linewidth=0.9, linestyle="--")

    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.draw()
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    toolbar = NavigationToolbar2Tk(canvas, frame, pack_toolbar=False)
    toolbar.update()
    toolbar.pack(side=tk.TOP, fill=tk.X)


def add_summary_tab(notebook: ttk.Notebook, data):
    frame = ttk.Frame(notebook)
    notebook.add(frame, text="Summary")

    text = tk.Text(frame, wrap="word", height=30)
    text.pack(fill=tk.BOTH, expand=True)

    if not data["sample"]:
        text.insert("1.0", "No data loaded.\n")
        text.configure(state="disabled")
        return

    def minmax(arr):
        return min(arr), max(arr)

    summary_lines = []

    summary_lines.append("CSV summary\n")
    summary_lines.append(f"Samples: {len(data['sample'])}")
    summary_lines.append(f"First sample id: {data['sample'][0]}")
    summary_lines.append(f"Last sample id:  {data['sample'][-1]}")
    summary_lines.append("")

    important_minmax = [
        "offset_ns",
        "raw_offset_ns",
        "offset_avg_ns",
        "mean_path_delay_ns",
        "freq_err_ppb",
        "current_addend",
        "pi_prop_ppb",
        "pi_integral_ppb",
        "pi_output_ppb",
        "rejected_sample_count",
        "reject_mpd_count",
        "reject_abs_offset_count",
        "reject_jump_count",
        "delayreq_timeout_count",
        "delayresp_ignored_count",
    ]

    for key in important_minmax:
        mn, mx = minmax(data[key])
        summary_lines.append(f"{key}:")
        summary_lines.append(f"  min = {mn}")
        summary_lines.append(f"  max = {mx}")
        summary_lines.append("")

    summary_lines.append("Final values:")

    for key in FIELDNAMES:
        summary_lines.append(f"  {key} = {data[key][-1]}")

    summary = "\n".join(summary_lines)

    text.insert("1.0", summary)
    text.configure(state="disabled")


def main():
    if len(sys.argv) < 2:
        print("Usage: python ptp_plot_tabs.py <csv_file>")
        sys.exit(1)

    csv_file = sys.argv[1]
    data = load_csv(csv_file)

    root = tk.Tk()
    root.title(f"PTP Log Viewer - {csv_file}")
    root.geometry("1400x900")

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True)

    add_summary_tab(notebook, data)

    x = data["sample"]

    for title, key, ylabel in PLOTS:
        add_plot_tab(notebook, title, x, data[key], ylabel)

    root.mainloop()


if __name__ == "__main__":
    main()