"""
pc_udp_receiver.py
──────────────────
Receives UDP sensor data broadcast by the ESP32 sensor node (ESP32_Sensor_AP).
Run this script AFTER connecting your PC to the "ESP32_Sensor" WiFi hotspot.

Data field order (comma-separated, one packet per line):
  [0]  LocalTime        — seconds since sensor node started (s)
  [1]  AccelerationX    — X-axis acceleration (m/s²)
  [2]  AccelerationY    — Y-axis acceleration (m/s²)
  [3]  AccelerationZ    — Z-axis acceleration (m/s²)
  [4]  GyroscopeX       — X-axis angular velocity (rad/s)
  [5]  GyroscopeY       — Y-axis angular velocity (rad/s)
  [6]  GyroscopeZ       — Z-axis angular velocity (rad/s)
  [7]  MagnetX          — X-axis magnetic field (μT)
  [8]  MagnetY          — Y-axis magnetic field (μT)
  [9]  MagnetZ          — Z-axis magnetic field (μT)
  [10] ImuTemperature   — IMU die temperature (°C)

Usage:
  python pc_udp_receiver.py               # print to console
  python pc_udp_receiver.py --save        # also save to CSV file
  python pc_udp_receiver.py --save --out data.csv  # specify output filename
"""

import socket
import argparse
import csv
import sys
import time
from datetime import datetime

# ─── Configuration ────────────────────────────────────────────────────────────
UDP_IP   = ""       # Empty string = listen on all interfaces
UDP_PORT = 8888     # Must match the port defined in the ESP32 sketch
TIMEOUT  = 2.0      # Socket receive timeout in seconds

# Column headers matching the CSV data order
FIELD_NAMES = [
    "LocalTime",        # s
    "AccelerationX",    # m/s²
    "AccelerationY",    # m/s²
    "AccelerationZ",    # m/s²
    "GyroscopeX",       # rad/s
    "GyroscopeY",       # rad/s
    "GyroscopeZ",       # rad/s
    "MagnetX",          # μT
    "MagnetY",          # μT
    "MagnetZ",          # μT
    "ImuTemperature",   # °C
]
EXPECTED_FIELDS = len(FIELD_NAMES)

# ─── Argument Parsing ─────────────────────────────────────────────────────────
def parse_args():
    parser = argparse.ArgumentParser(description="ESP32 UDP sensor data receiver")
    parser.add_argument(
        "--save", action="store_true",
        help="Save received data to a CSV file"
    )
    parser.add_argument(
        "--out", type=str, default="",
        help="Output CSV filename (default: sensor_<timestamp>.csv)"
    )
    parser.add_argument(
        "--port", type=int, default=UDP_PORT,
        help=f"UDP port to listen on (default: {UDP_PORT})"
    )
    return parser.parse_args()

# ─── Packet Parser ────────────────────────────────────────────────────────────
def parse_packet(raw: str):
    """
    Parse a comma-separated sensor packet.
    Returns a dict of {field_name: float} on success, or None on error.
    """
    parts = raw.strip().split(",")
    if len(parts) != EXPECTED_FIELDS:
        return None
    try:
        values = [float(p) for p in parts]
        return dict(zip(FIELD_NAMES, values))
    except ValueError:
        return None

# ─── Pretty Printer ───────────────────────────────────────────────────────────
def print_packet(data: dict, packet_count: int):
    t = data["LocalTime"]
    print(
        f"[#{packet_count:06d} | t={t:8.3f}s] "
        f"Accel=({data['AccelerationX']:+7.3f}, {data['AccelerationY']:+7.3f}, {data['AccelerationZ']:+7.3f}) m/s²  "
        f"Gyro=({data['GyroscopeX']:+7.3f}, {data['GyroscopeY']:+7.3f}, {data['GyroscopeZ']:+7.3f}) rad/s  "
        f"Mag=({data['MagnetX']:+8.2f}, {data['MagnetY']:+8.2f}, {data['MagnetZ']:+8.2f}) μT  "
        f"Temp={data['ImuTemperature']:5.2f}°C"
    )

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()
    port = args.port

    # Determine CSV output path
    csv_file   = None
    csv_writer = None
    if args.save:
        filename = args.out if args.out else f"sensor_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        csv_file = open(filename, "w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=FIELD_NAMES)
        csv_writer.writeheader()
        print(f"[INFO] Saving data to: {filename}")

    # Open UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Allow receiving broadcast packets
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((UDP_IP, port))
    sock.settimeout(TIMEOUT)

    print(f"[INFO] Listening for UDP packets on port {port}...")
    print("[INFO] Make sure your PC is connected to the 'ESP32_Sensor' WiFi hotspot.")
    print("[INFO] Press Ctrl+C to stop.\n")
    print("─" * 120)

    packet_count  = 0
    error_count   = 0
    start_time    = time.time()

    try:
        while True:
            try:
                raw_bytes, addr = sock.recvfrom(512)
            except socket.timeout:
                # Print a keep-alive indicator so the user knows we're still listening
                elapsed = time.time() - start_time
                print(f"  [waiting... {elapsed:.0f}s elapsed, {packet_count} packets received]",
                      end="\r", flush=True)
                continue

            raw_str = raw_bytes.decode("utf-8", errors="replace")
            data    = parse_packet(raw_str)

            if data is None:
                error_count += 1
                print(f"[WARN] Could not parse packet from {addr}: {repr(raw_str)}")
                continue

            packet_count += 1
            print_packet(data, packet_count)

            if csv_writer is not None:
                csv_writer.writerow(data)
                csv_file.flush()   # flush after every row for safety

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n{'─' * 120}")
        print(f"[INFO] Stopped. Received {packet_count} packets in {elapsed:.1f}s "
              f"({packet_count / elapsed:.1f} pkt/s). Parse errors: {error_count}.")

    finally:
        sock.close()
        if csv_file is not None:
            csv_file.close()
            print(f"[INFO] CSV file closed.")


if __name__ == "__main__":
    main()
