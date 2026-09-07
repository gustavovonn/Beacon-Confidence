#!/usr/bin/env python3
"""
Convert VeReMi Benchmark Parquet traces to zero-dependency compact binary (.bin) format.
Matches SerializedBsmRecord layout defined in include/BinaryMessageFormat.h.
"""

import os
import sys
import struct
import argparse
import glob
import pandas as pd
import numpy as np

# Magic header: "V2XBSM01" -> 0x56325842534D3031
MAGIC = 0x56325842534D3031

# Struct layout: < d q q q 22d  (1 double, 3 int64, 22 doubles = 26 fields)
# 1. arrivalTime (d)
# 2. sender (q)
# 3. receiver_id (q)
# 4. receiver_pseudo (q)
# 5. px (d), py (d)
# 6. pnx (d), pny (d)
# 7. sx (d), sy (d)
# 8. snx (d), sny (d)
# 9. ax (d), ay (d)
# 10. anx (d), any (d)
# 11. hx (d), hy (d)
# 12. hnx (d), hny (d)
# 13. epx (d), epy (d)
# 14. ehx (d), ehy (d)
# 15. esx (d), esy (d)
RECORD_FORMAT = '<dqqq22d'
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

def convert_file(parquet_path, bin_path):
    print(f"[*] Converting: {os.path.basename(parquet_path)} -> {os.path.basename(bin_path)}")
    df = pd.read_parquet(parquet_path)

    # Ensure required columns exist, fill defaults if missing
    defaults = {
        'rcvTime': 0.0,
        'sender': -1,
        'receiver_id': -1,
        'receiver_pseudo': -1,
        'pos_x': 0.0, 'pos_y': 0.0,
        'pos_noise_x': 0.0, 'pos_noise_y': 0.0,
        'spd_x': 0.0, 'spd_y': 0.0,
        'spd_noise_x': 0.0, 'spd_noise_y': 0.0,
        'acl_x': 0.0, 'acl_y': 0.0,
        'acl_noise_x': 0.0, 'acl_noise_y': 0.0,
        'hed_x': 1.0, 'hed_y': 0.0,
        'hed_noise_x': 0.0, 'hed_noise_y': 0.0,
        'ego_pos_x': 0.0, 'ego_pos_y': 0.0,
        'ego_hed_x': 1.0, 'ego_hed_y': 0.0,
        'ego_spd_x': 0.0, 'ego_spd_y': 0.0
    }
    for col, default_val in defaults.items():
        if col not in df.columns:
            df[col] = default_val

    # Sort chronologically by rcvTime
    df = df.sort_values('rcvTime').reset_index(drop=True)
    num_records = len(df)

    # Prepare structured buffer
    values = zip(
        df['rcvTime'].to_numpy(dtype=np.float64),
        df['sender'].to_numpy(dtype=np.int64),
        df['receiver_id'].to_numpy(dtype=np.int64),
        df['receiver_pseudo'].to_numpy(dtype=np.int64),
        df['pos_x'].to_numpy(dtype=np.float64), df['pos_y'].to_numpy(dtype=np.float64),
        df['pos_noise_x'].to_numpy(dtype=np.float64), df['pos_noise_y'].to_numpy(dtype=np.float64),
        df['spd_x'].to_numpy(dtype=np.float64), df['spd_y'].to_numpy(dtype=np.float64),
        df['spd_noise_x'].to_numpy(dtype=np.float64), df['spd_noise_y'].to_numpy(dtype=np.float64),
        df['acl_x'].to_numpy(dtype=np.float64), df['acl_y'].to_numpy(dtype=np.float64),
        df['acl_noise_x'].to_numpy(dtype=np.float64), df['acl_noise_y'].to_numpy(dtype=np.float64),
        df['hed_x'].to_numpy(dtype=np.float64), df['hed_y'].to_numpy(dtype=np.float64),
        df['hed_noise_x'].to_numpy(dtype=np.float64), df['hed_noise_y'].to_numpy(dtype=np.float64),
        df['ego_pos_x'].to_numpy(dtype=np.float64), df['ego_pos_y'].to_numpy(dtype=np.float64),
        df['ego_hed_x'].to_numpy(dtype=np.float64), df['ego_hed_y'].to_numpy(dtype=np.float64),
        df['ego_spd_x'].to_numpy(dtype=np.float64), df['ego_spd_y'].to_numpy(dtype=np.float64),
    )

    os.makedirs(os.path.dirname(bin_path) or '.', exist_ok=True)
    with open(bin_path, 'wb') as f:
        # Write 8-byte magic + 8-byte count
        f.write(struct.pack('<QQ', MAGIC, num_records))
        packer = struct.Struct(RECORD_FORMAT)
        for row in values:
            f.write(packer.pack(*row))

    size_mb = os.path.getsize(bin_path) / (1024 * 1024)
    print(f"    [+] Wrote {num_records:,} records to {bin_path} ({size_mb:.2f} MB)")

def main():
    parser = argparse.ArgumentParser(description="Export Parquet traces to zero-dependency binary format")
    parser.add_argument("--input", "-i", type=str, help="Input parquet file or directory")
    parser.add_argument("--output-dir", "-o", type=str, default=None, help="Output directory for .bin files")
    args = parser.parse_args()

    input_path = args.input or "/home/gust/CP-Artery/cp-trustfulness/f2md/veremi"
    out_dir = args.output_dir or (os.path.dirname(input_path) if os.path.isfile(input_path) else input_path)

    if os.path.isfile(input_path):
        out_name = os.path.splitext(os.path.basename(input_path))[0] + ".bin"
        convert_file(input_path, os.path.join(out_dir, out_name))
    elif os.path.isdir(input_path):
        parquets = sorted(glob.glob(os.path.join(input_path, "*.parquet")))
        # Filter out symlinks or large non-50k traces
        valid_parquets = [p for p in parquets if os.path.getsize(p) > 1000 and "interpolated" not in os.path.basename(p)]
        print(f"Found {len(valid_parquets)} Parquet files in {input_path}")
        for p in valid_parquets:
            out_name = os.path.splitext(os.path.basename(p))[0] + ".bin"
            convert_file(p, os.path.join(out_dir, out_name))
    else:
        print(f"Error: Path not found: {input_path}")
        sys.exit(1)

if __name__ == "__main__":
    main()
