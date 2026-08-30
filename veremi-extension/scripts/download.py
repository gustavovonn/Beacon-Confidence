import os
import sys
import requests
import argparse
import time
from urllib.parse import urlparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

# The script will only download files containing one of these keywords.
# Leave the list empty [] to download everything by default.
DEFAULT_FILTERS = [
    "ConstPos",
    "ConstPosOffset",
    "RandomPos",
    "RandomPosOffset",
    "ConstSpeed",
    "ConstSpeedOffset",
    "RandomSpeed",
    "RandomSpeedOffset"
]

# Thread-safe locks and shared state for custom progress tracking
progress_lock = threading.Lock()
shared_state = {
    'downloaded_bytes': 0,
    'total_bytes': 0,
    'last_print_time': 0.0
}

def download_file(url, dest_path, filename, expected_size):
    try:
        response = requests.get(url, stream=True, timeout=30)
        response.raise_for_status()
        block_size = 8192 * 4  # 32KB chunks for faster disk writes
        
        with open(dest_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=block_size):
                if chunk:
                    f.write(chunk)
                    
                    # Update global progress safely
                    with progress_lock:
                        shared_state['downloaded_bytes'] += len(chunk)
                        current_time = time.time()
                        
                        # Print once every 2 seconds to avoid any terminal issues
                        if current_time - shared_state['last_print_time'] > 2.0:
                            dl_mb = shared_state['downloaded_bytes'] / (1024 * 1024)
                            tot_mb = shared_state['total_bytes'] / (1024 * 1024)
                            percent = (shared_state['downloaded_bytes'] / shared_state['total_bytes']) * 100 if shared_state['total_bytes'] > 0 else 0
                            
                            print(f"Overall Progress: {percent:5.1f}% | {dl_mb:7.1f} MB / {tot_mb:7.1f} MB downloaded")
                            shared_state['last_print_time'] = current_time
                            
        return True, filename
    except Exception as e:
        # Clean up partial file
        if os.path.exists(dest_path):
            os.remove(dest_path)
        return False, filename

from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Download dataset from Zenodo concurrently")
    # Default Zenodo Record for VeReMi Extension dataset
    # Reference: https://zenodo.org/records/20090854
    parser.add_argument("record_id", nargs='?', default="20090854", help="Zenodo record ID (default: 20090854 - https://zenodo.org/records/20090854)")
    
    # Set default destination to the data directory (i.e., veremi-extension/data/)
    default_dest = Path(__file__).parent.parent / 'data'
    parser.add_argument("--dest", type=str, default=str(default_dest), help="Destination directory")
    
    parser.add_argument("--filter", nargs="+", help="Optional list of keywords to filter which files to download", default=DEFAULT_FILTERS)
    parser.add_argument("--workers", type=int, default=5, help="Number of concurrent downloads (default: 5)")
    args = parser.parse_args()

    record_id = args.record_id
    dest_dir = args.dest
    filters = args.filter
    max_workers = args.workers

    os.makedirs(dest_dir, exist_ok=True)

    api_url = f"https://zenodo.org/api/records/{record_id}"
    print(f"Fetching metadata from {api_url}...")
    
    response = requests.get(api_url)
    if response.status_code != 200:
        print(f"Failed to fetch record {record_id}. Status: {response.status_code}")
        print(response.text)
        sys.exit(1)
        
    data = response.json()
    files = data.get('files', [])
    
    if not files:
        print("No files found in this Zenodo record.")
        sys.exit(0)
        
    print(f"Found {len(files)} file(s) in the record.")
    
    download_tasks = []
    
    for f in files:
        filename = f.get('key')
        download_url = f.get('links', {}).get('self')
        expected_size = f.get('size', -1)
        
        if not filename or not download_url:
            continue
            
        # Apply filter if provided
        if filters:
            if not any(keyword.lower() in filename.lower() for keyword in filters):
                continue
                
        dest_path = os.path.join(dest_dir, filename)
        if os.path.exists(dest_path):
            actual_size = os.path.getsize(dest_path)
            if expected_size == -1 or expected_size == actual_size:
                print(f"File {filename} already exists and is complete. Skipping.")
                continue
            else:
                print(f"File {filename} is incomplete. Re-downloading.")
                
        download_tasks.append((download_url, dest_path, filename, expected_size))
        if expected_size > 0:
            shared_state['total_bytes'] += expected_size

    if not download_tasks:
        print("All requested files are already downloaded!")
        sys.exit(0)
        
    print(f"Queueing {len(download_tasks)} files for download using {max_workers} concurrent workers...\n")
    
    success_count = 0
    failed_files = []
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(download_file, url, dest, fname, expected_size): fname 
            for (url, dest, fname, expected_size) in download_tasks
        }
        
        for future in as_completed(futures):
            success, fname = future.result()
            if success:
                success_count += 1
            else:
                failed_files.append(fname)
                
    # Final flush to ensure 100% is printed
    dl_mb = shared_state['downloaded_bytes'] / (1024 * 1024)
    tot_mb = shared_state['total_bytes'] / (1024 * 1024)
    print(f"Overall Progress: 100.0% | {dl_mb:7.1f} MB / {tot_mb:7.1f} MB downloaded")
        
    print(f"\nAll downloads finished! ({success_count}/{len(download_tasks)} successful)")
    if failed_files:
        print("Failed to download the following files:")
        for fname in failed_files:
            print(f" - {fname}")

if __name__ == "__main__":
    main()
