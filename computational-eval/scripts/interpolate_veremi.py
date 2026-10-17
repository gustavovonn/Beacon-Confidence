#!/usr/bin/env python3
"""
VeReMi Trace Benchmark Dataset Preparation Tool
================================================
Host-only ETL tool to prepare decoupled, lightweight Parquet datasets 
for standalone C++ benchmarking on workstation or SBC.

Features:
  - Multi-process parallel JSON parsing across available CPU cores.
  - Generates synchronized observer kinematics (1D spline interpolation).
  - Configurable message limit (--max-msgs, default: 50,000) for compact SBC transfer.
  - Supports individual profiles or batch processing of all 8 rush hour profiles.

Usage:
  python3 interpolate_veremi.py --all-profiles
  python3 interpolate_veremi.py --profile ConstPos_0709
  python3 interpolate_veremi.py --profile RandomPos_0709 --max-msgs 50000
"""

import os
import sys
import glob
import json
import time
import argparse
import multiprocessing as mp
import pandas as pd
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.interpolate import interp1d

RUSH_HOUR_PROFILES = [
    "ConstPos_0709",
    "ConstPosOffset_0709",
    "ConstSpeed_0709",
    "ConstSpeedOffset_0709",
    "RandomPos_0709",
    "RandomPosOffset_0709",
    "RandomSpeed_0709",
    "RandomSpeedOffset_0709"
]

def parse_single_trace_file(filepath):
    """
    Parses a single traceJSON file, extracts Type 2 (observer) and Type 3 (received BSM),
    and performs 1D spline interpolation of receiver state at each rcvTime.
    """
    type2_data = []
    type3_data = []
    
    filename = os.path.basename(filepath)
    parts = filename.split('-')
    receiver_id = int(parts[1]) if len(parts) >= 3 and parts[1].isdigit() else -1
    receiver_pseudo = int(parts[2]) if len(parts) >= 3 and parts[2].isdigit() else receiver_id

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
                rt = row.get('rcvTime', 0.0)
                if rt > 29100.0:
                    continue
                msg_type = row.get('type')
                if msg_type == 2:
                    type2_data.append({
                        'rcvTime': row['rcvTime'],
                        'ego_pos_x': row['pos'][0],
                        'ego_pos_y': row['pos'][1],
                        'ego_spd_x': row['spd'][0],
                        'ego_spd_y': row['spd'][1],
                        'ego_hed_x': row['hed'][0],
                        'ego_hed_y': row['hed'][1]
                    })
                elif msg_type == 3:
                    type3_data.append({
                        'rcvTime': row['rcvTime'],
                        'sender': row['sender'],
                        'receiver_id': receiver_id,
                        'receiver_pseudo': receiver_pseudo,
                        'pos_x': row['pos'][0],
                        'pos_y': row['pos'][1],
                        'pos_noise_x': row.get('pos_noise', [0, 0, 0])[0],
                        'pos_noise_y': row.get('pos_noise', [0, 0, 0])[1],
                        'spd_x': row['spd'][0],
                        'spd_y': row['spd'][1],
                        'spd_noise_x': row.get('spd_noise', [0, 0, 0])[0],
                        'spd_noise_y': row.get('spd_noise', [0, 0, 0])[1],
                        'acl_x': row['acl'][0],
                        'acl_y': row['acl'][1],
                        'acl_noise_x': row.get('acl_noise', [0, 0, 0])[0],
                        'acl_noise_y': row.get('acl_noise', [0, 0, 0])[1],
                        'hed_x': row['hed'][0],
                        'hed_y': row['hed'][1],
                        'hed_noise_x': row.get('hed_noise', [0, 0, 0])[0],
                        'hed_noise_y': row.get('hed_noise', [0, 0, 0])[1]
                    })
            except Exception:
                continue

    if not type3_data:
        return None

    df3 = pd.DataFrame(type3_data)

    if type2_data:
        df2 = pd.DataFrame(type2_data)
        df2 = df2.drop_duplicates(subset=['rcvTime']).sort_values('rcvTime')
        
        if len(df2) >= 2:
            f_px = interp1d(df2['rcvTime'], df2['ego_pos_x'], bounds_error=False, fill_value="extrapolate")
            f_py = interp1d(df2['rcvTime'], df2['ego_pos_y'], bounds_error=False, fill_value="extrapolate")
            f_sx = interp1d(df2['rcvTime'], df2['ego_spd_x'], bounds_error=False, fill_value="extrapolate")
            f_sy = interp1d(df2['rcvTime'], df2['ego_spd_y'], bounds_error=False, fill_value="extrapolate")
            f_hx = interp1d(df2['rcvTime'], df2['ego_hed_x'], bounds_error=False, fill_value="extrapolate")
            f_hy = interp1d(df2['rcvTime'], df2['ego_hed_y'], bounds_error=False, fill_value="extrapolate")

            df3['ego_pos_x'] = f_px(df3['rcvTime'])
            df3['ego_pos_y'] = f_py(df3['rcvTime'])
            df3['ego_spd_x'] = f_sx(df3['rcvTime'])
            df3['ego_spd_y'] = f_sy(df3['rcvTime'])
            df3['ego_hed_x'] = f_hx(df3['rcvTime'])
            df3['ego_hed_y'] = f_hy(df3['rcvTime'])
        else:
            df3['ego_pos_x'] = df2.iloc[0]['ego_pos_x']
            df3['ego_pos_y'] = df2.iloc[0]['ego_pos_y']
            df3['ego_spd_x'] = df2.iloc[0]['ego_spd_x']
            df3['ego_spd_y'] = df2.iloc[0]['ego_spd_y']
            df3['ego_hed_x'] = df2.iloc[0]['ego_hed_x']
            df3['ego_hed_y'] = df2.iloc[0]['ego_hed_y']
    else:
        df3['ego_pos_x'] = 0.0
        df3['ego_pos_y'] = 0.0
        df3['ego_spd_x'] = 0.0
        df3['ego_spd_y'] = 0.0
        df3['ego_hed_x'] = 1.0
        df3['ego_hed_y'] = 0.0

    return df3

def process_profile(profile_name, source_data_dir, output_dir, max_msgs, num_workers):
    """
    Finds the rush hour peak folder (28800_32400) for profile_name,
    processes all traceJSON files in parallel, sorts chronologically,
    and writes out a standardized parquet file.
    """
    profile_dir = os.path.join(source_data_dir, profile_name)
    if not os.path.isdir(profile_dir):
        print(f"[!] Error: Profile directory not found: {profile_dir}")
        return False

    # Look for peak rush hour directory (VeReMi_28800_32400_*)
    candidates = [d for d in glob.glob(os.path.join(profile_dir, "VeReMi_28800_32400_*")) if os.path.isdir(d)]
    if not candidates:
        # Fallback to any VeReMi_* directory
        candidates = [d for d in glob.glob(os.path.join(profile_dir, "VeReMi_*")) if os.path.isdir(d)]
    
    if not candidates:
        # Check if there is a .zip archive to extract
        zip_candidates = [z for z in glob.glob(os.path.join(profile_dir, "VeReMi_28800_32400_*.zip")) if os.path.isfile(z)]
        if not zip_candidates:
            zip_candidates = [z for z in glob.glob(os.path.join(profile_dir, "VeReMi_*.zip")) if os.path.isfile(z)]
        if zip_candidates:
            target_zip = sorted(zip_candidates)[0]
            print(f"[*] Extracting archive: {os.path.basename(target_zip)}...")
            import zipfile
            with zipfile.ZipFile(target_zip, 'r') as zip_ref:
                zip_ref.extractall(profile_dir)
            candidates = [d for d in glob.glob(os.path.join(profile_dir, "VeReMi_28800_32400_*")) if os.path.isdir(d)]
            if not candidates:
                candidates = [d for d in glob.glob(os.path.join(profile_dir, "VeReMi_*")) if os.path.isdir(d)]

    if not candidates:
        print(f"[!] Error: No extracted VeReMi trace directory found in {profile_dir}")
        return False

    target_dir = sorted(candidates)[0]
    json_files = glob.glob(os.path.join(target_dir, "traceJSON-*.json"))
    if not json_files:
        print(f"[!] Error: No traceJSON-*.json files found in {target_dir}")
        return False

    print(f"\n[*] Processing '{profile_name}' from: {os.path.basename(target_dir)}")
    print(f"    Found {len(json_files)} trace JSON files. Using {num_workers} parallel workers...")

    t0 = time.time()
    # Process files in parallel
    with mp.get_context("fork").Pool(num_workers) as pool:
        dfs = pool.map(parse_single_trace_file, json_files)
    t_parse = time.time() - t0

    valid_dfs = [d for d in dfs if d is not None]
    if not valid_dfs:
        print(f"[!] Error: No valid BSM records extracted for {profile_name}")
        return False

    print(f"    Parsed {len(valid_dfs)} files in {t_parse:.2f}s. Concatenating & sorting...")
    final_df = pd.concat(valid_dfs, ignore_index=True)
    final_df = final_df.sort_values('rcvTime').reset_index(drop=True)

    total_extracted = len(final_df)
    if max_msgs > 0 and len(final_df) > max_msgs:
        final_df = final_df.head(max_msgs)
        print(f"    Slicing first {max_msgs} messages (out of {total_extracted:,} total) for compact benchmark dataset.")
    else:
        print(f"    Preserving all {total_extracted:,} messages.")

    # Cast integer columns to int64 for schema consistency
    final_df['receiver_id'] = final_df['receiver_id'].astype('int64')
    final_df['receiver_pseudo'] = final_df['receiver_pseudo'].astype('int64')
    final_df['sender'] = final_df['sender'].astype('int64')
    final_df['type'] = 3
    final_df['type'] = final_df['type'].astype('int64')

    os.makedirs(output_dir, exist_ok=True)
    out_file = os.path.join(output_dir, f"{profile_name}.parquet")
    table = pa.Table.from_pandas(final_df)
    pq.write_table(table, out_file)

    size_mb = os.path.getsize(out_file) / (1024 * 1024)
    print(f"    [+] Successfully wrote: {out_file} ({len(final_df):,} msgs, {size_mb:.2f} MB)")
    
    # Explicit memory cleanup
    import gc
    del valid_dfs, dfs, final_df, table
    gc.collect()
    return True

def get_default_paths():
    """Dynamically determine repository and data directories from script location."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    eval_dir = os.path.abspath(os.path.join(script_dir, ".."))

    # Locate project root (where .git lives, else one level up from computational-eval)
    cur = script_dir
    repo_root = None
    while cur and cur != os.path.dirname(cur):
        if os.path.isdir(os.path.join(cur, ".git")):
            repo_root = cur
            break
        cur = os.path.dirname(cur)
    if not repo_root:
        repo_root = os.path.abspath(os.path.join(eval_dir, ".."))

    default_source = os.path.join(repo_root, "veremi-extension", "data")
    default_output = os.path.join(eval_dir, "veremi", "parquet")
    return default_source, default_output

DEFAULT_SOURCE_DIR, DEFAULT_OUTPUT_DIR = get_default_paths()

def main():
    parser = argparse.ArgumentParser(description="VeReMi Benchmark Dataset Preparation (Host-Only ETL)")
    parser.add_argument("--profile", type=str, help="Specific profile to process (e.g. ConstPos_0709)")
    parser.add_argument("--all-profiles", action="store_true", help="Process all 8 rush hour profiles")
    parser.add_argument("--source-dir", type=str, default=DEFAULT_SOURCE_DIR,
                        help=f"Path to unzipped VeReMi Extension raw datasets (default: {DEFAULT_SOURCE_DIR})")
    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR,
                        help=f"Output directory for generated Parquet files (default: {DEFAULT_OUTPUT_DIR})")
    parser.add_argument("--max-msgs", type=int, default=50000,
                        help="Maximum messages to retain per profile (default: 50000; 0 for full trace)")
    parser.add_argument("--jobs", "-j", type=int, default=min(4, os.cpu_count() or 2),
                        help="Number of parallel worker processes (default: 4 or CPU count)")

    args = parser.parse_args()

    args.source_dir = os.path.abspath(args.source_dir)
    args.output_dir = os.path.abspath(args.output_dir)

    # Default to all profiles if neither is specified
    if not args.profile and not args.all_profiles:
        args.all_profiles = True

    targets = RUSH_HOUR_PROFILES if args.all_profiles else [args.profile]

    print("================================================================")
    print(" VeReMi Benchmark Dataset Preparation (Host-Only ETL)")
    print(f" Target Profiles: {len(targets)}")
    print(f" Max Messages   : {args.max_msgs if args.max_msgs > 0 else 'Full'}")
    print(f" Output Dir     : {args.output_dir}")
    print(f" Parallel Jobs  : {args.jobs}")
    print("================================================================")

    start_all = time.time()
    success_count = 0

    for prof in targets:
        # Standardize name if user passed without _0709
        if not prof.endswith("_0709") and not prof.endswith("_1416"):
            prof = f"{prof}_0709"
        if process_profile(prof, args.source_dir, args.output_dir, args.max_msgs, args.jobs):
            success_count += 1

    total_time = time.time() - start_all
    print("\n================================================================")
    print(f" [DONE] Completed {success_count}/{len(targets)} profiles in {total_time:.2f}s.")
    print(" Parquet files are ready for decoupled C++ benchmarking on SBC or host.")
    print("================================================================")

if __name__ == "__main__":
    main()
