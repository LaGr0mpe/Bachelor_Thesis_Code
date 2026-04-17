import csv
import serial
import sys
from datetime import datetime

PORT = "COM4"        
BAUD = 115200         
OUTFILE = f"ptp_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

FIELDNAMES = [
    "sample",
    "offset_ns",
    "offset_avg_ns",
    "mean_path_delay_ns",
    "freq_err_ppb",
    "current_addend",
    "last_addend_step",
    "phase_step_count",
    "freq_update_count",
    "sync_rx_ts_valid_count",
    "tx_ts_seen",
]

def parse_line(line: str):
    if not line.startswith("DATA,"):
        return None

    parts = line.strip().split(",")
    if len(parts) != 12:
        return None

    try:
        return {
            "sample": int(parts[1]),
            "offset_ns": int(parts[2]),
            "offset_avg_ns": int(parts[3]),
            "mean_path_delay_ns": int(parts[4]),
            "freq_err_ppb": int(parts[5]),
            "current_addend": int(parts[6]),
            "last_addend_step": int(parts[7]),
            "phase_step_count": int(parts[8]),
            "freq_update_count": int(parts[9]),
            "sync_rx_ts_valid_count": int(parts[10]),
            "tx_ts_seen": int(parts[11]),
        }
    except ValueError:
        return None

def main():
    print(f"Open {PORT} @ {BAUD}")
    print(f"Write CSV -> {OUTFILE}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser, open(OUTFILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()

        count = 0
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()

                if line.startswith("FMT,"):
                    print(line)
                    continue

                row = parse_line(line)
                if row is None:
                    continue

                writer.writerow(row)
                f.flush()
                count += 1

                if count % 20 == 0:
                    print(
                        f"samples={count:6d}  "
                        f"sample_id={row['sample']:6d}  "
                        f"offset_ns={row['offset_ns']:8d}  "
                        f"avg_ns={row['offset_avg_ns']:8d}  "
                        f"freq_ppb={row['freq_err_ppb']:8d}  "
                        f"addend={row['current_addend']}"
                    )

        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()