import csv
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# CONFIG
# =========================
PORT = "COM4"          # поменяй на свой COM
BAUD = 115200
WINDOW_POINTS = 400
CSV_PREFIX = "ptp_live_log"
PRINT_EVERY_N_SAMPLES = 20

# =========================
# DATA BUFFERS
# =========================
sample = deque(maxlen=WINDOW_POINTS)
offset_ns = deque(maxlen=WINDOW_POINTS)
raw_offset_ns = deque(maxlen=WINDOW_POINTS)
offset_avg_ns = deque(maxlen=WINDOW_POINTS)
freq_err_ppb = deque(maxlen=WINDOW_POINTS)
current_addend = deque(maxlen=WINDOW_POINTS)
pi_output_ppb = deque(maxlen=WINDOW_POINTS)
rejected_sample_count = deque(maxlen=WINDOW_POINTS)
phase_capture_step_count = deque(maxlen=WINDOW_POINTS)

all_count = 0
csv_writer = None
csv_file = None
ser = None

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
    "pi_prop_ppb",
    "pi_integral_ppb",
    "pi_output_ppb",
    "last_sample_rejected",
    "rejected_sample_count",
    "reject_mpd_count",
    "reject_abs_offset_count",
    "reject_jump_count",
]


def parse_line(line: str):
    if not line.startswith("DATA,"):
        return None

    parts = line.strip().split(",")

    # DATA + 22 поля = 23 элемента
    if len(parts) != 23:
        return None

    try:
        return {
            "sample": int(parts[1]),
            "offset_ns": int(parts[2]),
            "raw_offset_ns": int(parts[3]),
            "offset_avg_ns": int(parts[4]),
            "mean_path_delay_ns": int(parts[5]),
            "freq_err_ppb": int(parts[6]),
            "current_addend": int(parts[7]),
            "last_addend_step": int(parts[8]),
            "phase_step_count": int(parts[9]),
            "phase_capture_step_count": int(parts[10]),
            "freq_update_count": int(parts[11]),
            "coarse_mode": int(parts[12]),
            "sync_rx_ts_valid_count": int(parts[13]),
            "tx_ts_seen": int(parts[14]),
            "pi_prop_ppb": int(parts[15]),
            "pi_integral_ppb": int(parts[16]),
            "pi_output_ppb": int(parts[17]),
            "last_sample_rejected": int(parts[18]),
            "rejected_sample_count": int(parts[19]),
            "reject_mpd_count": int(parts[20]),
            "reject_abs_offset_count": int(parts[21]),
            "reject_jump_count": int(parts[22]),
        }
    except ValueError:
        return None


def update_axis(ax, xdata, ydata, title, ylabel):
    ax.clear()
    ax.plot(xdata, ydata)
    ax.set_title(title)
    ax.set_xlabel("sample")
    ax.set_ylabel(ylabel)
    ax.grid(True)

    if not xdata or not ydata:
        return

    xmin = min(xdata)
    xmax = max(xdata)
    if xmin == xmax:
        xmax = xmin + 1
    ax.set_xlim(xmin, xmax)

    ymin = min(ydata)
    ymax = max(ydata)

    if ymin == ymax:
        margin = max(1.0, abs(ymin) * 0.1 + 1.0)
        ax.set_ylim(ymin - margin, ymax + margin)
    else:
        span = ymax - ymin
        margin = max(span * 0.1, 1.0)
        ax.set_ylim(ymin - margin, ymax + margin)


def poll_serial():
    global all_count

    while True:
        raw = ser.readline()
        if not raw:
            break

        line = raw.decode(errors="ignore").strip()
        if not line:
            continue

        if line.startswith("FMT,"):
            print(line)
            continue

        row = parse_line(line)
        if row is None:
            continue

        csv_writer.writerow(row)
        csv_file.flush()

        sample.append(row["sample"])
        offset_ns.append(row["offset_ns"])
        raw_offset_ns.append(row["raw_offset_ns"])
        offset_avg_ns.append(row["offset_avg_ns"])
        freq_err_ppb.append(row["freq_err_ppb"])
        current_addend.append(row["current_addend"])
        pi_output_ppb.append(row["pi_output_ppb"])
        rejected_sample_count.append(row["rejected_sample_count"])
        phase_capture_step_count.append(row["phase_capture_step_count"])

        all_count += 1

        if all_count % PRINT_EVERY_N_SAMPLES == 0:
            print(
                f"samples={all_count:6d}  "
                f"sample_id={row['sample']:6d}  "
                f"offset_ns={row['offset_ns']:8d}  "
                f"raw_ns={row['raw_offset_ns']:8d}  "
                f"avg_ns={row['offset_avg_ns']:8d}  "
                f"freq_ppb={row['freq_err_ppb']:8d}  "
                f"pi_out={row['pi_output_ppb']:8d}  "
                f"coarse={row['coarse_mode']}  "
                f"phase_cap_steps={row['phase_capture_step_count']:4d}  "
                f"rej={row['last_sample_rejected']}  "
                f"rej_total={row['rejected_sample_count']:5d}  "
                f"addend={row['current_addend']}"
            )


def animate(_frame):
    poll_serial()

    x = list(sample)

    update_axis(ax1, x, list(offset_ns), "PTP Offset", "offset_ns")
    update_axis(ax2, x, list(raw_offset_ns), "Raw Offset", "raw_offset_ns")
    update_axis(ax3, x, list(offset_avg_ns), "Filtered Offset", "offset_avg_ns")
    update_axis(ax4, x, list(freq_err_ppb), "Frequency Error", "freq_err_ppb")
    update_axis(ax5, x, list(current_addend), "Current Addend", "current_addend")
    update_axis(ax6, x, list(pi_output_ppb), "PI Output", "pi_output_ppb")
    update_axis(ax7, x, list(rejected_sample_count), "Rejected Samples", "rejected_sample_count")
    update_axis(ax8, x, list(phase_capture_step_count), "Phase Capture Steps", "phase_capture_step_count")

    fig.tight_layout()


def on_close(_event):
    print("\nClosing...")
    try:
        if ser is not None and ser.is_open:
            ser.close()
    except Exception:
        pass

    try:
        if csv_file is not None:
            csv_file.close()
    except Exception:
        pass


def main():
    global csv_writer, csv_file, ser
    global fig, ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8

    csv_name = f"{CSV_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    print(f"Open {PORT} @ {BAUD}")
    print(f"Write CSV -> {csv_name}")
    print("Close plot window to stop.\n")

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    csv_file = open(csv_name, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    csv_writer.writeheader()

    fig, axes = plt.subplots(2, 4, figsize=(18, 8))
    ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8 = axes.flatten()

    fig.canvas.mpl_connect("close_event", on_close)

    anim = FuncAnimation(fig, animate, interval=200, cache_frame_data=False)

    plt.show()


if __name__ == "__main__":
    main()