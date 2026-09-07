#!/usr/bin/env python3
"""
VeReMi Trace Interpolator & Dataset Completion Tool
===================================================
Converts raw VeReMi simulation JSON traces into synchronized Apache Parquet datasets.

Why this is needed:
  In raw VeReMi simulation traces, observer ego state (Type 2) and received neighbor
  BSMs (Type 3) are logged at different timestamps. F2MD plausibility checks require
  the exact observer spatial coordinates, velocity vector, and heading at the microsecond
  of each BSM arrival.
  
What this script does:
  1. Scans VeReMi simulation folders for traceJSON-*.json files.
  2. Extracts both the physical vehicle Node ID (receiver_id) and receiver pseudonym (receiver_pseudo).
  3. Uses 1D linear spline interpolation (scipy.interpolate.interp1d) to compute
     synchronized observer coordinates (ego_pos_x, ego_pos_y), velocities (ego_spd_x, ego_spd_y),
     and headings (ego_hed_x, ego_hed_y) at the exact arrival timestamp (rcvTime) of each received BSM.
  4. Chronologically sorts all messages globally by rcvTime on disk.
  5. Exports the completed, enriched dataset to high-performance Apache Parquet.
"""
import json
import os
import sys
import glob
import pandas as pd
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.interpolate import interp1d

def process_file(filepath):
    type2_data = []
    type3_data = []
    
    receiver_id = -1
    receiver_pseudo = -1
    # traceJSON-[nodeID]-[pseudonym]-[attackType]-[time]-[seed].json
    filename = os.path.basename(filepath)
    if filename.startswith("traceJSON-"):
        parts = filename.split('-')
        if len(parts) >= 3:
            receiver_id = int(parts[1])      # Physical Vehicle Node ID
            receiver_pseudo = int(parts[2])  # Receiver Pseudonym ID
    
    with open(filepath, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if row.get('type') == 2:
                    type2_data.append({
                        'rcvTime': row['rcvTime'],
                        'ego_pos_x': row['pos'][0],
                        'ego_pos_y': row['pos'][1],
                        'ego_spd_x': row['spd'][0],
                        'ego_spd_y': row['spd'][1],
                        'ego_hed_x': row['hed'][0],
                        'ego_hed_y': row['hed'][1]
                    })
                elif row.get('type') == 3:
                    type3_data.append({
                        'rcvTime': row['rcvTime'],
                        'sender': row['sender'],
                        'receiver_id': receiver_id,
                        'receiver_pseudo': receiver_pseudo,
                        'pos_x': row['pos'][0],
                        'pos_y': row['pos'][1],
                        'pos_noise_x': row.get('pos_noise', [0,0,0])[0],
                        'pos_noise_y': row.get('pos_noise', [0,0,0])[1],
                        'spd_x': row['spd'][0],
                        'spd_y': row['spd'][1],
                        'spd_noise_x': row.get('spd_noise', [0,0,0])[0],
                        'spd_noise_y': row.get('spd_noise', [0,0,0])[1],
                        'acl_x': row['acl'][0],
                        'acl_y': row['acl'][1],
                        'acl_noise_x': row.get('acl_noise', [0,0,0])[0],
                        'acl_noise_y': row.get('acl_noise', [0,0,0])[1],
                        'hed_x': row['hed'][0],
                        'hed_y': row['hed'][1],
                        'hed_noise_x': row.get('hed_noise', [0,0,0])[0],
                        'hed_noise_y': row.get('hed_noise', [0,0,0])[1]
                    })
            except Exception as e:
                pass
                
    if not type3_data:
        return None
        
    df3 = pd.DataFrame(type3_data)
    
    if type2_data:
        df2 = pd.DataFrame(type2_data)
        # Drop duplicates in rcvTime just in case
        df2 = df2.drop_duplicates(subset=['rcvTime']).sort_values('rcvTime')
        
        # If we have at least 2 points, interpolate
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
            # Fallback to the single point
            df3['ego_pos_x'] = df2.iloc[0]['ego_pos_x']
            df3['ego_pos_y'] = df2.iloc[0]['ego_pos_y']
            df3['ego_spd_x'] = df2.iloc[0]['ego_spd_x']
            df3['ego_spd_y'] = df2.iloc[0]['ego_spd_y']
            df3['ego_hed_x'] = df2.iloc[0]['ego_hed_x']
            df3['ego_hed_y'] = df2.iloc[0]['ego_hed_y']
    else:
        # No ego data at all! (Should be rare)
        df3['ego_pos_x'] = 0.0
        df3['ego_pos_y'] = 0.0
        df3['ego_spd_x'] = 0.0
        df3['ego_spd_y'] = 0.0
        df3['ego_hed_x'] = 1.0
        df3['ego_hed_y'] = 0.0
        
    return df3

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 interpolate_veremi_traces.py <input_folder> <output_parquet>")
        sys.exit(1)
        
    input_folder = sys.argv[1]
    output_parquet = sys.argv[2]
    
    print(f"Reading JSON traces from {input_folder}...")
    files = glob.glob(os.path.join(input_folder, "traceJSON-*.json"))
    
    all_dfs = []
    total = len(files)
    
    for i, filepath in enumerate(files):
        if i % 100 == 0:
            print(f"Processed {i}/{total} files...")
        df = process_file(filepath)
        if df is not None:
            all_dfs.append(df)
            
    if not all_dfs:
        print("No valid Type 3 messages found!")
        sys.exit(1)
        
    print("Concatenating dataframes...")
    final_df = pd.concat(all_dfs, ignore_index=True)
    
    # Global chronological sort by rcvTime on disk
    print("Sorting chronologically by rcvTime...")
    final_df = final_df.sort_values('rcvTime').reset_index(drop=True)
    
    # Cast types
    final_df['receiver_id'] = final_df['receiver_id'].astype('int64')
    final_df['receiver_pseudo'] = final_df['receiver_pseudo'].astype('int64')
    final_df['sender'] = final_df['sender'].astype('int64')
    final_df['type'] = 3
    final_df['type'] = final_df['type'].astype('int64')
    
    # Write to parquet
    print(f"Writing {len(final_df)} rows to {output_parquet}...")
    table = pa.Table.from_pandas(final_df)
    pq.write_table(table, output_parquet)
    print("Done!")

if __name__ == "__main__":
    main()
