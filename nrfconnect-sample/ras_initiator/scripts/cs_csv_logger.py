#!/usr/bin/env python3
"""Capture the CSV export of the ras_initiator sample into a .csv file.

The firmware prints one row per measured tone, prefixed with "CSV,"
(header prefixed with "CSVH,"), mixed with the regular log output.
This script keeps only those rows and writes them to a CSV file, adding a
host timestamp column.

Usage:
    python3 cs_csv_logger.py /dev/ttyACM1                 # live from the DK
    python3 cs_csv_logger.py /dev/ttyACM1 -o run1.csv
    python3 cs_csv_logger.py capture.log -o run1.csv      # from a saved log
    cat capture.log | python3 cs_csv_logger.py -          # from stdin

Live capture needs pyserial: pip install pyserial
"""

import argparse
import csv
import os
import sys
from datetime import datetime

# Must match CSV_HEADER in src/main.c (used if the boot header was missed).
DEFAULT_HEADER = [
    "timestamp_ms", "ranging_counter", "ap", "channel", "freq_mhz", "h_re", "h_im",
    "amplitude", "phase_deg", "n_iqs", "tone_quality", "rtt_count", "rtt_mean_ns",
    "tof_ns", "rssi_dbm", "dist_ifft_m", "dist_phase_slope_m", "dist_rtt_m",
]


def open_source(src, baud):
    if src == "-":
        return sys.stdin
    if os.path.exists(src) and not src.startswith("/dev/") and not src.upper().startswith("COM"):
        return open(src, encoding="utf-8", errors="replace")
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is required for serial capture: pip install pyserial")
    ser = serial.Serial(src, baud, timeout=1)
    return _SerialLines(ser)


class _SerialLines:
    def __init__(self, ser):
        self.ser = ser

    def __iter__(self):
        while True:
            raw = self.ser.readline()
            if raw:
                yield raw.decode("utf-8", errors="replace")

    def close(self):
        self.ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="serial port (e.g. /dev/ttyACM1, COM5), log file, or '-'")
    ap.add_argument("-o", "--output",
                    default=datetime.now().strftime("cs_data_%Y%m%d_%H%M%S.csv"),
                    help="output CSV file (default: cs_data_<date>_<time>.csv)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-q", "--quiet", action="store_true", help="do not echo non-CSV lines")
    args = ap.parse_args()

    header = list(DEFAULT_HEADER)
    n_rows = 0
    procedures = set()
    source = open_source(args.source, args.baud)

    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["host_time"] + header)
        try:
            for line in source:
                line = line.strip()
                # A log line may have been interleaved before the tag.
                idx = line.find("CSV")
                if idx < 0:
                    if line and not args.quiet:
                        print(line)
                    continue
                line = line[idx:]
                if line.startswith("CSVH,"):
                    fw_header = line[5:].split(",")
                    if fw_header != header:
                        print(f"warning: firmware header differs from script: {fw_header}",
                              file=sys.stderr)
                    continue
                if not line.startswith("CSV,"):
                    continue
                fields = line[4:].split(",")
                if len(fields) != len(header):
                    print(f"skipped malformed row: {line}", file=sys.stderr)
                    continue
                writer.writerow([datetime.now().isoformat(timespec="milliseconds")] + fields)
                n_rows += 1
                procedures.add((fields[1], fields[2]))
                if n_rows % 50 == 0:
                    f.flush()
                    print(f"[{args.output}] {n_rows} rows, {len(procedures)} procedures",
                          file=sys.stderr)
        except KeyboardInterrupt:
            pass
        finally:
            if hasattr(source, "close") and source is not sys.stdin:
                source.close()

    print(f"Saved {n_rows} rows ({len(procedures)} procedures) to {args.output}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
