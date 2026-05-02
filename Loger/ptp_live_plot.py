import csv
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# =========================
# CONFIG
# =========================

PORT = "COM5"          # поменяй на свой COM-порт
BAUD = 115200

WINDOW_POINTS = 500
PLOT_INTERVAL_MS = 200
PRINT_EVERY_N_SAMPLES = 20

CSV_PREFIX = "ptp_slave_live_log"
RAW_PREFIX = "ptp_slave_raw_log"

SERIAL_TIMEOUT = 0.02


# =========================
# CURRENT STM32 DATA FORMAT
# =========================

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

EXPECTED_PARTS = 1 + len(FIELDNAMES)  # DATA + fields


# =========================
# BUFFERS
# =========================

buffers = {
    "sample": deque(maxlen=WINDOW_POINTS),
    "offset_ns": deque(maxlen=WINDOW_POINTS),
    "raw_offset_ns": deque(maxlen=WINDOW_POINTS),
    "offset_avg_ns": deque(maxlen=WINDOW_POINTS),
    "mean_path_delay_ns": deque(maxlen=WINDOW_POINTS),
    "freq_err_ppb": deque(maxlen=WINDOW_POINTS),
    "current_addend": deque(maxlen=WINDOW_POINTS),
    "last_addend_step": deque(maxlen=WINDOW_POINTS),
    "pi_output_ppb": deque(maxlen=WINDOW_POINTS),
    "coarse_mode": deque(maxlen=WINDOW_POINTS),
    "last_sample_rejected": deque(maxlen=WINDOW_POINTS),
    "rejected_sample_count": deque(maxlen=WINDOW_POINTS),
}

ser = None
csv_file = None
raw_file = None
csv_writer = None

total_rows = 0
bad_rows = 0
last_print_time = time.time()


# =========================
# PARSING
# =========================

def parse_data_line(line: str):
    """
    Parses current STM32 line:

    DATA,sample,offset_ns,raw_offset_ns,offset_avg_ns,...
    """

    if not line.startswith("DATA,"):
        return None

    parts = line.strip().split(",")

    if len(parts) != EXPECTED_PARTS:
        raise ValueError(
            f"Wrong field count: got {len(parts)}, expected {EXPECTED_PARTS}. "
            f"Line: {line}"
        )

    row = {}

    for idx, name in enumerate(FIELDNAMES, start=1):
        row[name] = int(parts[idx])

    return row


def append_row_to_buffers(row: dict):
    for key in buffers:
        buffers[key].append(row[key])


# =========================
# SERIAL POLLING
# =========================

def poll_serial():
    global total_rows, bad_rows, last_print_time

    if ser is None:
        return

    lines_read = 0

    while True:
        raw = ser.readline()

        if not raw:
            break

        lines_read += 1

        line = raw.decode("utf-8", errors="replace").strip()

        if not line:
            continue

        raw_file.write(line + "\n")

        if line.startswith("FMT,"):
            print("\nReceived format from STM32:")
            print(line)
            continue

        if not line.startswith("DATA,"):
            continue

        try:
            row = parse_data_line(line)
        except Exception as exc:
            bad_rows += 1
            print(f"\nParse error #{bad_rows}: {exc}")
            continue

        csv_writer.writerow(row)
        csv_file.flush()
        raw_file.flush()

        append_row_to_buffers(row)
        total_rows += 1

        if total_rows % PRINT_EVERY_N_SAMPLES == 0:
            print_status(row)

    return lines_read


def print_status(row: dict):
    mode = "COARSE" if row["coarse_mode"] else "FINE"

    print(
        f"samples={total_rows:6d}  "
        f"id={row['sample']:6d}  "
        f"mode={mode:6s}  "
        f"offset={row['offset_ns']:9d} ns  "
        f"avg={row['offset_avg_ns']:9d} ns  "
        f"mpd={row['mean_path_delay_ns']:6d} ns  "
        f"freq={row['freq_err_ppb']:8d} ppb  "
        f"pi={row['pi_output_ppb']:8d} ppb  "
        f"addend={row['current_addend']}  "
        f"step={row['last_addend_step']:6d}  "
        f"rej={row['last_sample_rejected']}  "
        f"rej_total={row['rejected_sample_count']}"
    )


# =========================
# PLOTTING
# =========================

def update_axis(ax, x, y, title, ylabel, zero_line=False):
    ax.clear()

    if len(x) == 0 or len(y) == 0:
        ax.set_title(title)
        ax.set_xlabel("sample")
        ax.set_ylabel(ylabel)
        ax.grid(True)
        return

    ax.plot(x, y)

    if zero_line:
        ax.axhline(0, linewidth=1)

    ax.set_title(title)
    ax.set_xlabel("sample")
    ax.set_ylabel(ylabel)
    ax.grid(True)

    xmin = min(x)
    xmax = max(x)

    if xmin == xmax:
        xmax = xmin + 1

    ax.set_xlim(xmin, xmax)

    ymin = min(y)
    ymax = max(y)

    if ymin == ymax:
        margin = max(1.0, abs(ymin) * 0.1 + 1.0)
    else:
        margin = max((ymax - ymin) * 0.1, 1.0)

    ax.set_ylim(ymin - margin, ymax + margin)


def animate(_frame):
    poll_serial()

    x = list(buffers["sample"])

    update_axis(
        ax1,
        x,
        list(buffers["offset_ns"]),
        "PTP Offset",
        "offset_ns",
        zero_line=True,
    )

    update_axis(
        ax2,
        x,
        list(buffers["offset_avg_ns"]),
        "Filtered Offset / offset_avg",
        "offset_avg_ns",
        zero_line=True,
    )

    update_axis(
        ax3,
        x,
        list(buffers["mean_path_delay_ns"]),
        "Mean Path Delay",
        "mean_path_delay_ns",
        zero_line=False,
    )

    update_axis(
        ax4,
        x,
        list(buffers["freq_err_ppb"]),
        "Measured Frequency Error",
        "freq_err_ppb",
        zero_line=True,
    )

    update_axis(
        ax5,
        x,
        list(buffers["current_addend"]),
        "Current Addend",
        "addend",
        zero_line=False,
    )

    update_axis(
        ax6,
        x,
        list(buffers["pi_output_ppb"]),
        "PI Output",
        "pi_output_ppb",
        zero_line=True,
    )

    fig.suptitle(
        f"STM32 PTP Slave Live Monitor | rows={total_rows}, parse_errors={bad_rows}",
        fontsize=12,
    )

    fig.tight_layout()


# =========================
# CLEANUP
# =========================

def on_close(_event):
    print("\nClosing...")

    try:
        if ser is not None and ser.is_open:
            ser.close()
            print("Serial closed.")
    except Exception as exc:
        print(f"Serial close error: {exc}")

    try:
        if csv_file is not None:
            csv_file.close()
            print("CSV closed.")
    except Exception as exc:
        print(f"CSV close error: {exc}")

    try:
        if raw_file is not None:
            raw_file.close()
            print("Raw log closed.")
    except Exception as exc:
        print(f"Raw log close error: {exc}")


# =========================
# MAIN
# =========================

def main():
    global ser, csv_file, raw_file, csv_writer
    global fig, ax1, ax2, ax3, ax4, ax5, ax6

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    out_dir = Path(".")
    csv_name = out_dir / f"{CSV_PREFIX}_{timestamp}.csv"
    raw_name = out_dir / f"{RAW_PREFIX}_{timestamp}.txt"

    print(f"Opening serial: {PORT} @ {BAUD}")
    print(f"CSV log:       {csv_name}")
    print(f"Raw log:       {raw_name}")
    print("Close the plot window to stop.\n")

    ser = serial.Serial(PORT, BAUD, timeout=SERIAL_TIMEOUT)

    # Дать порту немного времени после открытия
    time.sleep(0.5)

    # Очистить старый мусор из буфера
    ser.reset_input_buffer()

    csv_file = open(csv_name, "w", newline="", encoding="utf-8")
    raw_file = open(raw_name, "w", newline="", encoding="utf-8")

    csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    csv_writer.writeheader()
    csv_file.flush()

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    ax1, ax2, ax3, ax4, ax5, ax6 = axes.flatten()

    fig.canvas.mpl_connect("close_event", on_close)

    _anim = FuncAnimation(
        fig,
        animate,
        interval=PLOT_INTERVAL_MS,
        cache_frame_data=False,
    )

    plt.show()


if __name__ == "__main__":
    main()