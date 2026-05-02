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

SLAVES = [
    {
        "name": "slave1",
        "port": "COM4",
        "baud": 115200,
    },
    {
        "name": "slave2",
        "port": "COM5",
        "baud": 115200,
    },
]

WINDOW_POINTS = 600
PLOT_INTERVAL_MS = 200
SERIAL_TIMEOUT = 0.01
PRINT_EVERY_N_ROWS = 50

CSV_PREFIX = "ptp_multislave_log"


# =========================
# STM32 DATA FORMAT
# =========================

DATA_FIELDNAMES = [
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
    "consecutive_reject_count",
    "resync_count",
]

EXPECTED_PARTS = 1 + len(DATA_FIELDNAMES)

CSV_FIELDNAMES = [
    "pc_time_iso",
    "slave",
    "port",
] + DATA_FIELDNAMES


# =========================
# STATE
# =========================

class SlaveState:
    def __init__(self, name, port, baud):
        self.name = name
        self.port = port
        self.baud = baud
        self.ser = None

        self.total_rows = 0
        self.last_printed_rows = 0

        self.buffers = {
            "sample": deque(maxlen=WINDOW_POINTS),
            "offset_ns": deque(maxlen=WINDOW_POINTS),
            "offset_avg_ns": deque(maxlen=WINDOW_POINTS),
            "mean_path_delay_ns": deque(maxlen=WINDOW_POINTS),
            "freq_err_ppb": deque(maxlen=WINDOW_POINTS),
            "current_addend": deque(maxlen=WINDOW_POINTS),
            "last_addend_step": deque(maxlen=WINDOW_POINTS),
            "pi_output_ppb": deque(maxlen=WINDOW_POINTS),
            "coarse_mode": deque(maxlen=WINDOW_POINTS),
            "last_sample_rejected": deque(maxlen=WINDOW_POINTS),
            "rejected_sample_count": deque(maxlen=WINDOW_POINTS),
            "reject_mpd_count": deque(maxlen=WINDOW_POINTS),
            "reject_abs_offset_count": deque(maxlen=WINDOW_POINTS),
            "reject_jump_count": deque(maxlen=WINDOW_POINTS),
            "delayreq_timeout_count": deque(maxlen=WINDOW_POINTS),
            "delayresp_ignored_count": deque(maxlen=WINDOW_POINTS),
        }


states = [SlaveState(s["name"], s["port"], s["baud"]) for s in SLAVES]

csv_file = None
csv_writer = None


# =========================
# PARSING
# =========================

def parse_data_line(line: str) -> dict | None:
    if not line.startswith("DATA,"):
        return None

    parts = line.strip().split(",")

    if len(parts) != EXPECTED_PARTS:
        raise ValueError(
            f"Wrong field count: got {len(parts)}, expected {EXPECTED_PARTS}. "
            f"Line: {line}"
        )

    row = {}

    for idx, name in enumerate(DATA_FIELDNAMES, start=1):
        row[name] = int(parts[idx])

    return row


def append_row_to_buffers(state: SlaveState, row: dict):
    for key in state.buffers:
        state.buffers[key].append(row[key])


def latest_value(state: SlaveState, key: str, default=0):
    buf = state.buffers.get(key)
    if not buf:
        return default
    if len(buf) == 0:
        return default
    return buf[-1]


# =========================
# SERIAL
# =========================

def open_serials():
    for state in states:
        print(f"Opening {state.name}: {state.port} @ {state.baud}")
        state.ser = serial.Serial(
            state.port,
            state.baud,
            timeout=SERIAL_TIMEOUT,
        )

        time.sleep(0.2)
        state.ser.reset_input_buffer()


def poll_one_slave(state: SlaveState):
    global csv_writer, csv_file

    if state.ser is None:
        return

    while True:
        raw = state.ser.readline()

        if not raw:
            break

        line = raw.decode("utf-8", errors="replace").strip()

        if not line:
            continue

        if line.startswith("FMT,"):
            print(f"\n[{state.name}] FMT received:")
            print(line)
            continue

        if not line.startswith("DATA,"):
            continue

        try:
            row = parse_data_line(line)
        except Exception as exc:
            print(f"[{state.name}] parse error: {exc}")
            continue

        full_row = {
            "pc_time_iso": datetime.now().isoformat(timespec="milliseconds"),
            "slave": state.name,
            "port": state.port,
            **row,
        }

        csv_writer.writerow(full_row)
        csv_file.flush()

        append_row_to_buffers(state, row)
        state.total_rows += 1

        if state.total_rows - state.last_printed_rows >= PRINT_EVERY_N_ROWS:
            state.last_printed_rows = state.total_rows
            print_status(state, row)


def poll_all_serials():
    for state in states:
        poll_one_slave(state)


def print_status(state: SlaveState, row: dict):
    mode = "COARSE" if row["coarse_mode"] else "FINE"

    print(
    f"[{state.name}] "
    f"rows={state.total_rows:6d}  "
    f"id={row['sample']:6d}  "
    f"mode={mode:6s}  "
    f"offset={row['offset_ns']:8d} ns  "
    f"avg={row['offset_avg_ns']:8d} ns  "
    f"mpd={row['mean_path_delay_ns']:6d} ns  "
    f"pi={row['pi_output_ppb']:8d} ppb  "
    f"addend={row['current_addend']}  "
    f"rej={row['last_sample_rejected']}  "
    f"rej_total={row['rejected_sample_count']}  "
    f"rej_mpd={row['reject_mpd_count']}  "
    f"to={row['delayreq_timeout_count']}  "
    f"ign={row['delayresp_ignored_count']}"
)


# =========================
# PLOTTING
# =========================

def update_axis(ax, state: SlaveState, y_key: str, title: str, ylabel: str, zero_line=False):
    ax.clear()

    x = list(state.buffers["sample"])
    y = list(state.buffers[y_key])

    ax.set_title(f"{state.name}: {title}")
    ax.set_xlabel("sample")
    ax.set_ylabel(ylabel)
    ax.grid(True)

    if len(x) == 0 or len(y) == 0:
        return

    ax.plot(x, y)

    if zero_line:
        ax.axhline(0, linewidth=1)

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


def update_slave_plots(state: SlaveState, axes_2x3):
    ax1, ax2, ax3, ax4, ax5, ax6 = axes_2x3

    update_axis(
        ax1,
        state,
        "offset_ns",
        "Offset",
        "ns",
        zero_line=True,
    )

    update_axis(
        ax2,
        state,
        "offset_avg_ns",
        "Filtered offset",
        "ns",
        zero_line=True,
    )

    update_axis(
        ax3,
        state,
        "mean_path_delay_ns",
        "Mean path delay",
        "ns",
        zero_line=False,
    )

    update_axis(
        ax4,
        state,
        "freq_err_ppb",
        "Frequency error",
        "ppb",
        zero_line=True,
    )

    update_axis(
        ax5,
        state,
        "current_addend",
        "Current addend",
        "addend",
        zero_line=False,
    )

    update_axis(
        ax6,
        state,
        "pi_output_ppb",
        "PI output",
        "ppb",
        zero_line=True,
    )


def slave_summary_for_title(state: SlaveState) -> str:
    rejected = latest_value(state, "rejected_sample_count")
    reject_mpd = latest_value(state, "reject_mpd_count")
    reject_abs = latest_value(state, "reject_abs_offset_count")
    reject_jump = latest_value(state, "reject_jump_count")
    timeout = latest_value(state, "delayreq_timeout_count")
    ignored = latest_value(state, "delayresp_ignored_count")

    return (
        f"{state.name}: rows={state.total_rows}, "
        f"rejected={rejected}, "
        f"mpd={reject_mpd}, "
        f"abs={reject_abs}, "
        f"jump={reject_jump}, "
        f"to={timeout}, "
        f"ign={ignored}"
    )


def animate(_frame):
    poll_all_serials()

    # rows 0-1: slave1, rows 2-3: slave2
    update_slave_plots(states[0], list(axes[0]) + list(axes[1]))
    update_slave_plots(states[1], list(axes[2]) + list(axes[3]))

    fig.suptitle(
        "STM32 PTP Multislave Monitor | "
        f"{slave_summary_for_title(states[0])} | "
        f"{slave_summary_for_title(states[1])}",
        fontsize=11,
    )

    fig.tight_layout()


# =========================
# CLEANUP
# =========================

def on_close(_event):
    print("\nClosing...")

    for state in states:
        try:
            if state.ser is not None and state.ser.is_open:
                state.ser.close()
                print(f"{state.name}: serial closed.")
        except Exception as exc:
            print(f"{state.name}: serial close error: {exc}")

    try:
        if csv_file is not None:
            csv_file.close()
            print("CSV closed.")
    except Exception as exc:
        print(f"CSV close error: {exc}")


# =========================
# MAIN
# =========================

def main():
    global csv_file, csv_writer
    global fig, axes

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name = Path(f"{CSV_PREFIX}_{timestamp}.csv")

    print("PTP multislave logger")
    print(f"CSV log: {csv_name}")
    print("Close plot window to stop.\n")

    open_serials()

    csv_file = open(csv_name, "w", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
    csv_writer.writeheader()
    csv_file.flush()

    fig, axes = plt.subplots(4, 3, figsize=(17, 11))
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