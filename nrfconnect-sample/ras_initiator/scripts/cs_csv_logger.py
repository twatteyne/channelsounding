#!/usr/bin/env python3
"""Capture the CSV export of the ras_initiator sample into a .csv file.

The firmware prints two record types, mixed with the regular log output:
  - "CSV,"  one row per measured tone (procedure, antenna path, channel)
  - "CSVP," one row per (procedure, antenna path) with the values computed once
            per procedure: tone quality, RTT, ToF, RSSI, distance estimates.
Headers are prefixed with "CSVH," and "CSVPH,".

Tones go to the output file, procedures to <output>_procedures.csv; each gets
a host timestamp column. Join the two on (ranging_counter, ap).

Usage:
    python3 cs_csv_logger.py /dev/ttyACM1                 # live from the DK
    python3 cs_csv_logger.py /dev/ttyACM1 -o run1.csv     # + run1_procedures.csv
    python3 cs_csv_logger.py capture.log -o run1.csv      # from a saved log
    cat capture.log | python3 cs_csv_logger.py -          # from stdin

Live capture needs pyserial: pip install pyserial
"""

import argparse
import csv
import os
import sys
from datetime import datetime

# Must match CSV_HEADER / CSV_PROC_HEADER in src/main.c
# (used if the boot headers were missed).
TONE_HEADER = [
    "timestamp_ms", "ranging_counter", "ap", "channel", "freq_mhz", "h_re", "h_im",
    "amplitude", "phase_deg", "n_iqs",
]
PROC_HEADER = [
    "timestamp_ms", "ranging_counter", "ap", "tone_quality", "rtt_count", "rtt_mean_ns",
    "tof_ns", "rssi_dbm", "dist_ifft_m", "dist_phase_slope_m", "dist_rtt_m",
]

# tag -> (header tag, expected header, name)
RECORDS = {
    "CSV": ("CSVH", TONE_HEADER, "tones"),
    "CSVP": ("CSVPH", PROC_HEADER, "procedures"),
}


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

    root, ext = os.path.splitext(args.output)
    paths = {"CSV": args.output, "CSVP": f"{root}_procedures{ext or '.csv'}"}
    headers_by_tag = {RECORDS[t][0]: t for t in RECORDS}
    n_rows = {t: 0 for t in RECORDS}
    procedures = set()
    source = open_source(args.source, args.baud)

    files = {t: open(paths[t], "w", newline="") for t in RECORDS}
    writers = {t: csv.writer(f) for t, f in files.items()}
    for t, w in writers.items():
        w.writerow(["host_time"] + RECORDS[t][1])
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
            tag, _, rest = line.partition(",")
            fields = rest.split(",")
            if tag in headers_by_tag:
                expected = RECORDS[headers_by_tag[tag]][1]
                if fields != expected:
                    print(f"warning: firmware {tag} header differs from script: {fields}",
                          file=sys.stderr)
                continue
            if tag not in RECORDS:
                continue
            if len(fields) != len(RECORDS[tag][1]):
                print(f"skipped malformed row: {line}", file=sys.stderr)
                continue
            writers[tag].writerow([datetime.now().isoformat(timespec="milliseconds")] + fields)
            n_rows[tag] += 1
            if tag == "CSVP":
                procedures.add((fields[1], fields[2]))
                if len(procedures) % 10 == 0:
                    for f in files.values():
                        f.flush()
                    print(f"[{args.output}] {n_rows['CSV']} tone rows, "
                          f"{len(procedures)} procedures", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(source, "close") and source is not sys.stdin:
            source.close()
        for f in files.values():
            f.close()

    print(f"Saved {n_rows['CSV']} tone rows to {paths['CSV']}", file=sys.stderr)
    print(f"Saved {n_rows['CSVP']} procedure rows ({len(procedures)} procedures) "
          f"to {paths['CSVP']}", file=sys.stderr)

if __name__ == "__main__":
    main()
