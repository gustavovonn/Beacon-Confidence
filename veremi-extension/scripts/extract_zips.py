import os
import zipfile
import shutil
from pathlib import Path
import argparse

def extract_nested_zips(base_dir, delete_zips_after=False):
    """
    Recursively finds and extracts zip files. 
    If a zip file is found, it is extracted into a folder of the same name (minus .zip),
    and then the function searches inside the newly extracted folder for more zips.
    """
    base_path = Path(base_dir)
    
    # We will keep looping as long as we find new zip files to extract
    found_zips = True
    processed_zips = set()
    while found_zips:
        found_zips = False
        
        # Find all zip files currently in the directory tree
        for zip_path in base_path.rglob('*.zip'):
            # Skip if this is somehow a directory named .zip
            if not zip_path.is_file() or str(zip_path) in processed_zips:
                continue
                
            print(f"Extracting: {zip_path.name}...")
            
            # Extract to a temporary folder first
            temp_dir = zip_path.with_name(zip_path.name + ".tmp")
            temp_dir.mkdir(parents=True, exist_ok=True)
            
            try:
                with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                    zip_ref.extractall(temp_dir)
                    
                # Inspect the contents of the temp folder
                contents = list(temp_dir.iterdir())
                
                if len(contents) == 1 and contents[0].is_dir():
                    # It's a single wrapper folder (e.g. ConstPosOffset or VeReMi...13:54:3)
                    inner_dir = contents[0]
                    target_dir = zip_path.parent / inner_dir.name
                    print(f"Flattening wrapped folder '{inner_dir.name}'")
                    
                    if not target_dir.exists():
                        shutil.move(str(inner_dir), str(target_dir))
                    else:
                        # Merge if target already exists
                        for item in list(inner_dir.iterdir()):
                            sub_target = target_dir / item.name
                            if not sub_target.exists():
                                shutil.move(str(item), str(target_dir))
                    shutil.rmtree(temp_dir)
                else:
                    # Multiple files inside zip. Rename temp_dir to the zip's name (without .zip)
                    target_dir = zip_path.with_suffix('')
                    if not target_dir.exists():
                        temp_dir.rename(target_dir)
                    else:
                        # Merge
                        for item in list(temp_dir.iterdir()):
                            sub_target = target_dir / item.name
                            if not sub_target.exists():
                                shutil.move(str(item), str(target_dir))
                        shutil.rmtree(temp_dir)
                
                # Mark that we found and extracted a zip in this pass
                found_zips = True
                
                if delete_zips_after:
                    print(f"Deleting archive: {zip_path.name}")
                    zip_path.unlink()
                else:
                    processed_zips.add(str(zip_path))
                    
            except zipfile.BadZipFile:
                print(f"ERROR: Bad zip file {zip_path}")
                processed_zips.add(str(zip_path))
                continue
                
    print("\nExtraction complete! All nested zips have been unpacked.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Automate nested VeReMi dataset extraction.')
    parser.add_argument('--dir', type=str, default='../data', 
                        help='Base directory of the dataset zips (e.g. data)')
    parser.add_argument('--delete-zips', action='store_true', 
                        help='Delete zip files after extraction to save space')
    
    args = parser.parse_args()
    
    target_dir = Path(__file__).parent / args.dir
    target_dir = target_dir.resolve()
    
    if not target_dir.exists():
        print(f"Directory not found: {target_dir}")
        exit(1)
        
    print(f"Starting extraction process in: {target_dir}")
    extract_nested_zips(target_dir, args.delete_zips)
