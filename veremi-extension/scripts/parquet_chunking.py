import os
import json
import argparse
from pathlib import Path
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

def process_file(json_file_path):
    records = []
    with open(json_file_path, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            try:
                data = json.loads(line)
                
                # Only process Type 3 (received BSMs) or Type 4 (Ground truth)
                msg_type = data.get("type", -1)
                if msg_type not in (3, 4):
                    continue
                
                # Extract base fields
                record = {
                    "type": msg_type,
                    "rcvTime": data.get("rcvTime"),
                    "sendTime": data.get("sendTime"),
                    "sender": data.get("sender"),
                    "senderPseudo": data.get("senderPseudo"),
                    "messageID": data.get("messageID")
                }
                
                # Flatten arrays and ignore noise fields
                if "pos" in data and isinstance(data["pos"], list) and len(data["pos"]) >= 2:
                    record["pos_x"] = data["pos"][0]
                    record["pos_y"] = data["pos"][1]
                    if len(data["pos"]) > 2:
                        record["pos_z"] = data["pos"][2]
                        
                if "spd" in data and isinstance(data["spd"], list) and len(data["spd"]) >= 2:
                    record["spd_x"] = data["spd"][0]
                    record["spd_y"] = data["spd"][1]
                    
                if "acl" in data and isinstance(data["acl"], list) and len(data["acl"]) >= 2:
                    record["acl_x"] = data["acl"][0]
                    record["acl_y"] = data["acl"][1]
                    
                if "hed" in data and isinstance(data["hed"], list) and len(data["hed"]) >= 2:
                    record["hed_x"] = data["hed"][0]
                    record["hed_y"] = data["hed"][1]
                
                records.append(record)
            except json.JSONDecodeError:
                pass
                
    return pd.DataFrame(records)

def process_and_chunk_dataset(base_dir, output_dir):
    base_path = Path(base_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # We will search for all scenario directories, e.g., ConstPosOffset/ConstPosOffset_1416/VeReMi...
    # For simplicity, we process any directory that contains traceJSON or traceGroundTruthJSON files
    
    # Keep track of partitioned writes
    for json_file in base_path.rglob('*.json'):
        if not json_file.is_file():
            continue
            
        filename = json_file.name
        is_ground_truth = filename.startswith("traceGroundTruthJSON")
        is_trace = filename.startswith("traceJSON")
        
        if not (is_ground_truth or is_trace):
            continue
            
        print(f"Processing: {json_file.relative_to(base_path)}")
        df = process_file(json_file)
        
        if df.empty:
            continue
            
        # Maintain the full relative directory structure (e.g. ConstPosOffset/ConstPosOffset_1416/VeReMi_...)
        rel_dir = json_file.relative_to(base_path).parent
        partition_dir = output_path / rel_dir
        partition_dir.mkdir(parents=True, exist_ok=True)
        
        out_filename = filename.replace('.json', '.parquet')
        out_filepath = partition_dir / out_filename
        
        table = pa.Table.from_pandas(df)
        pq.write_table(table, out_filepath, compression='snappy')
        
    print("\nProcessing and chunking complete!")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert VeReMi JSON to Parquet.')
    parser.add_argument('--input', type=str, default='../data', 
                        help='Base directory of the unzipped dataset')
    parser.add_argument('--output', type=str, default='../parquet', 
                        help='Output directory for the Parquet datalake')
    
    args = parser.parse_args()
    
    input_dir = Path(__file__).parent / args.input
    output_dir = Path(__file__).parent / args.output
    
    process_and_chunk_dataset(input_dir, output_dir)
