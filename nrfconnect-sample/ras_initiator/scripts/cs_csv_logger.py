#!/usr/bin/env python3
"""Capture the CSV export of the ras_initiator sample into a .csv file.

The firmware prints one row per measured tone as 8 ';'-separated fields
(optionally prefixed with "CSV,"):

    initiator_quality;reflector_quality;channel;antenna;initiator_I;initiator_Q;reflector_I;reflector_Q

This matches the per-cell format of the reference dataset under scripts/data/
(Wieme et al. Bluetooth Channel Sounding IQ Dataset). Each measurement is
written to the output as a single line, fields separated by ';', with no
header row -- exactly like the cell layout of those files.

A log line may carry several tones joined by ',' (e.g. "CSV,0;0;75;0;...;...")
or a single tone; both are handled. Non-CSV log lines are echoed unless
--quiet is given.

Usage:
    python3 cs_csv_logger.py /dev/ttyACM1                 # live from the DK
    python3 cs_csv_logger.py /dev/ttyACM1 -o run1.csv     # + save to run1.csv
    python3 cs_csv_logger.py capture.log -o run1.csv      # from a saved log
    cat capture.log | python3 cs_csv_logger.py -          # from stdin

Live capture needs pyserial: pip install pyserial
"""

import argparse
import os
import sys
from datetime import datetime

# One tone = 8 ';'-separated fields, in output order.
FIELD_NAMES = [
    "initiator_quality", "reflector_quality", "channel", "antenna",
    "initiator_I", "initiator_Q", "reflector_I", "reflector_Q",
]
N_FIELDS = len(FIELD_NAMES)
PREFIX = "CSV,"


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

    n_rows = 0
    source = open_source(args.source, args.baud)

    try:
        with open(args.output, "w", newline="") as out:
            for line in source:
                line = line.strip()
                # Drop an optional "CSV," / interleaved-log prefix.
                idx = line.find(PREFIX)
                if idx >= 0:
                    line = line[idx + len(PREFIX):]
                # A line may hold several tones joined by ',': handle each.
                found = False
                for cell in line.split(","):
                    fields = cell.split(";")
                    if len(fields) == N_FIELDS:
                        out.write(";".join(fields) + "\n")
                        n_rows += 1
                        found = True
                    elif cell:
                        if not found and cell.strip() and not args.quiet:
                            print(cell)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(source, "close") and source is not sys.stdin:
            source.close()

    print(f"Saved {n_rows} tone rows to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
