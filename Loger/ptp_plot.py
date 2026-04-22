import csv
import sys
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


def get_int(row, key, default=0):
    value = row.get(key, "")
    if value is None or value == "":
        return default
    return int(value)


def load_csv(csv_file: str):
    data = {
        "sample": [],
        "offset_ns": [],
        "raw_offset_ns": [],
        "offset_avg_ns": [],
        "mean_path_delay_ns": [],
        "freq_err_ppb": [],
        "current_addend": [],
        "last_addend_step": [],
        "phase_step_count": [],
        "phase_capture_step_count": [],
        "freq_update_count": [],
        "coarse_mode": [],
        "sync_rx_ts_valid_count": [],
        "tx_ts_seen": [],
        "pi_prop_ppb": [],
        "pi_integral_ppb": [],
        "pi_output_ppb": [],
        "last_sample_rejected": [],
        "rejected_sample_count": [],
        "reject_mpd_count": [],
        "reject_abs_offset_count": [],
        "reject_jump_count": [],
    }

    with open(csv_file, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["sample"].append(get_int(row, "sample"))
            data["offset_ns"].append(get_int(row, "offset_ns"))
            data["raw_offset_ns"].append(get_int(row, "raw_offset_ns"))
            data["offset_avg_ns"].append(get_int(row, "offset_avg_ns"))
            data["mean_path_delay_ns"].append(get_int(row, "mean_path_delay_ns"))
            data["freq_err_ppb"].append(get_int(row, "freq_err_ppb"))
            data["current_addend"].append(get_int(row, "current_addend"))
            data["last_addend_step"].append(get_int(row, "last_addend_step"))
            data["phase_step_count"].append(get_int(row, "phase_step_count"))
            data["phase_capture_step_count"].append(get_int(row, "phase_capture_step_count"))
            data["freq_update_count"].append(get_int(row, "freq_update_count"))
            data["coarse_mode"].append(get_int(row, "coarse_mode"))
            data["sync_rx_ts_valid_count"].append(get_int(row, "sync_rx_ts_valid_count"))
            data["tx_ts_seen"].append(get_int(row, "tx_ts_seen"))
            data["pi_prop_ppb"].append(get_int(row, "pi_prop_ppb"))
            data["pi_integral_ppb"].append(get_int(row, "pi_integral_ppb"))
            data["pi_output_ppb"].append(get_int(row, "pi_output_ppb"))
            data["last_sample_rejected"].append(get_int(row, "last_sample_rejected"))
            data["rejected_sample_count"].append(get_int(row, "rejected_sample_count"))
            data["reject_mpd_count"].append(get_int(row, "reject_mpd_count"))
            data["reject_abs_offset_count"].append(get_int(row, "reject_abs_offset_count"))
            data["reject_jump_count"].append(get_int(row, "reject_jump_count"))

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

    offset_min, offset_max = minmax(data["offset_ns"])
    raw_offset_min, raw_offset_max = minmax(data["raw_offset_ns"])
    avg_min, avg_max = minmax(data["offset_avg_ns"])
    mpd_min, mpd_max = minmax(data["mean_path_delay_ns"])
    freq_min, freq_max = minmax(data["freq_err_ppb"])
    addend_min, addend_max = minmax(data["current_addend"])
    pi_prop_min, pi_prop_max = minmax(data["pi_prop_ppb"])
    pi_int_min, pi_int_max = minmax(data["pi_integral_ppb"])
    pi_out_min, pi_out_max = minmax(data["pi_output_ppb"])
    rej_total_min, rej_total_max = minmax(data["rejected_sample_count"])
    rej_mpd_min, rej_mpd_max = minmax(data["reject_mpd_count"])
    rej_abs_min, rej_abs_max = minmax(data["reject_abs_offset_count"])
    rej_jump_min, rej_jump_max = minmax(data["reject_jump_count"])

    summary = f"""CSV summary

Samples: {len(data["sample"])}
First sample id: {data["sample"][0]}
Last sample id:  {data["sample"][-1]}

offset_ns:
  min = {offset_min}
  max = {offset_max}

raw_offset_ns:
  min = {raw_offset_min}
  max = {raw_offset_max}

offset_avg_ns:
  min = {avg_min}
  max = {avg_max}

mean_path_delay_ns:
  min = {mpd_min}
  max = {mpd_max}

freq_err_ppb:
  min = {freq_min}
  max = {freq_max}

current_addend:
  min = {addend_min}
  max = {addend_max}

pi_prop_ppb:
  min = {pi_prop_min}
  max = {pi_prop_max}

pi_integral_ppb:
  min = {pi_int_min}
  max = {pi_int_max}

pi_output_ppb:
  min = {pi_out_min}
  max = {pi_out_max}

rejected_sample_count:
  min = {rej_total_min}
  max = {rej_total_max}

reject_mpd_count:
  min = {rej_mpd_min}
  max = {rej_mpd_max}

reject_abs_offset_count:
  min = {rej_abs_min}
  max = {rej_abs_max}

reject_jump_count:
  min = {rej_jump_min}
  max = {rej_jump_max}

Final values:
  offset_ns = {data["offset_ns"][-1]}
  raw_offset_ns = {data["raw_offset_ns"][-1]}
  offset_avg_ns = {data["offset_avg_ns"][-1]}
  mean_path_delay_ns = {data["mean_path_delay_ns"][-1]}
  freq_err_ppb = {data["freq_err_ppb"][-1]}
  current_addend = {data["current_addend"][-1]}
  last_addend_step = {data["last_addend_step"][-1]}
  phase_step_count = {data["phase_step_count"][-1]}
  phase_capture_step_count = {data["phase_capture_step_count"][-1]}
  freq_update_count = {data["freq_update_count"][-1]}
  coarse_mode = {data["coarse_mode"][-1]}
  sync_rx_ts_valid_count = {data["sync_rx_ts_valid_count"][-1]}
  tx_ts_seen = {data["tx_ts_seen"][-1]}
  pi_prop_ppb = {data["pi_prop_ppb"][-1]}
  pi_integral_ppb = {data["pi_integral_ppb"][-1]}
  pi_output_ppb = {data["pi_output_ppb"][-1]}
  last_sample_rejected = {data["last_sample_rejected"][-1]}
  rejected_sample_count = {data["rejected_sample_count"][-1]}
  reject_mpd_count = {data["reject_mpd_count"][-1]}
  reject_abs_offset_count = {data["reject_abs_offset_count"][-1]}
  reject_jump_count = {data["reject_jump_count"][-1]}
"""
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

    add_plot_tab(notebook, "Offset", x, data["offset_ns"], "offset_ns")
    add_plot_tab(notebook, "Raw Offset", x, data["raw_offset_ns"], "raw_offset_ns")
    add_plot_tab(notebook, "Offset Avg", x, data["offset_avg_ns"], "offset_avg_ns")
    add_plot_tab(notebook, "Mean Path Delay", x, data["mean_path_delay_ns"], "mean_path_delay_ns")
    add_plot_tab(notebook, "Freq Error", x, data["freq_err_ppb"], "freq_err_ppb")
    add_plot_tab(notebook, "Current Addend", x, data["current_addend"], "current_addend")
    add_plot_tab(notebook, "Last Addend Step", x, data["last_addend_step"], "last_addend_step")
    add_plot_tab(notebook, "Phase Step Count", x, data["phase_step_count"], "phase_step_count")
    add_plot_tab(notebook, "Phase Capture Step Count", x, data["phase_capture_step_count"], "phase_capture_step_count")
    add_plot_tab(notebook, "Freq Update Count", x, data["freq_update_count"], "freq_update_count")
    add_plot_tab(notebook, "Coarse Mode", x, data["coarse_mode"], "coarse_mode")
    add_plot_tab(notebook, "RX Timestamp Count", x, data["sync_rx_ts_valid_count"], "sync_rx_ts_valid_count")
    add_plot_tab(notebook, "TX Timestamp Count", x, data["tx_ts_seen"], "tx_ts_seen")
    add_plot_tab(notebook, "PI Prop", x, data["pi_prop_ppb"], "pi_prop_ppb")
    add_plot_tab(notebook, "PI Integral", x, data["pi_integral_ppb"], "pi_integral_ppb")
    add_plot_tab(notebook, "PI Output", x, data["pi_output_ppb"], "pi_output_ppb")
    add_plot_tab(notebook, "Last Sample Rejected", x, data["last_sample_rejected"], "last_sample_rejected")
    add_plot_tab(notebook, "Rejected Sample Count", x, data["rejected_sample_count"], "rejected_sample_count")
    add_plot_tab(notebook, "Reject MPD Count", x, data["reject_mpd_count"], "reject_mpd_count")
    add_plot_tab(notebook, "Reject Abs Offset Count", x, data["reject_abs_offset_count"], "reject_abs_offset_count")
    add_plot_tab(notebook, "Reject Jump Count", x, data["reject_jump_count"], "reject_jump_count")

    root.mainloop()


if __name__ == "__main__":
    main()