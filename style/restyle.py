
#!/usr/bin/env python3
"""
Python script to restyle C/C++/Arduino source files using astyle
Windows compatible version
"""

import os
import subprocess
import sys
import argparse
from pathlib import Path

# Configuration Constants
# ======================
# Directories to process (relative to workspace root)
TARGET_DIRECTORIES = ['main', 'port']

# File patterns to process
CPP_FILE_PATTERNS = ['*.c', '*.h', '*.cpp']
ARDUINO_FILE_PATTERNS = ['*.ino']

# Paths to exclude (files containing these strings in their path will be skipped)
EXCLUDE_PATTERNS = ['api']

# Configuration files (relative to workspace root)
ASTYLE_CORE_CONFIG = 'style/astyle_core.conf'
ASTYLE_EXAMPLES_CONFIG = 'style/astyle_examples.conf'

def find_astyle_executable():
    """Find astyle executable on Windows"""
    # Try common astyle executable names
    astyle_names = ['astyle.exe', 'astyle']
    
    for name in astyle_names:
        try:
            subprocess.run([name, '--version'], 
                         capture_output=True, check=True)
            return name
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    
    return None

def find_files(directory, patterns, exclude_patterns=None):
    """Find files matching patterns, excluding those matching exclude_patterns"""
    files = []
    exclude_patterns = exclude_patterns or []
    
    # Convert to absolute path for Windows compatibility
    directory = Path(directory).resolve()
    
    for pattern in patterns:
        for file_path in directory.rglob(pattern):
            # Skip if file path matches any exclude pattern
            if any(exclude in str(file_path) for exclude in exclude_patterns):
                continue
            files.append(str(file_path))
    
    return files

def restyle_files(files, config_file, astyle_exe, dry_run=False, verbose=False):
    """Apply astyle formatting to files using the specified config"""
    if not files:
        return
    
    # Convert config file path to absolute path for Windows
    config_file = str(Path(config_file).resolve())
    
    for file_path in files:
        try:
            if dry_run:
                print(f"[DRY RUN] Would format: {file_path}")
                continue
                
            if verbose:
                print(f"Formatting: {file_path}")
            else:
                print(f"Formatting: {Path(file_path).name}")
                
            # Use absolute paths for Windows compatibility
            result = subprocess.run([
                astyle_exe,
                '--suffix=none',
                f'--options={config_file}',
                str(Path(file_path).resolve())
            ], capture_output=True, text=True, check=True)
            
            if result.stdout and verbose:
                print(f"  {result.stdout.strip()}")
                
        except subprocess.CalledProcessError as e:
            print(f"Error formatting {file_path}: {e}")
            if e.stderr:
                print(f"  stderr: {e.stderr}")
        except Exception as e:
            print(f"Unexpected error formatting {file_path}: {e}")
            return False
    
    return True

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='Restyle C/C++/Arduino source files using astyle',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python style/restyle.py                    # Use default directories (main, port)
  python style/restyle.py main               # Only process 'main' directory
  python style/restyle.py main port src     # Process 'main', 'port', and 'src' directories
  python style/restyle.py --all             # Process all directories
        '''
    )
    
    parser.add_argument(
        'directories',
        nargs='*',
        default=None,
        help='Directories to process (relative to workspace root). If not specified, uses default directories from TARGET_DIRECTORIES.'
    )
    
    parser.add_argument(
        '--all',
        action='store_true',
        help='Process all subdirectories in the workspace root'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what files would be processed without actually formatting them'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Show verbose output'
    )
    
    return parser.parse_args()

def get_target_directories(args):
    """Get the list of directories to process based on command line arguments"""
    if args.all:
        # Find all subdirectories in workspace root
        workspace_dirs = []
        for item in Path('.').iterdir():
            if item.is_dir() and not item.name.startswith('.') and item.name != 'build':
                workspace_dirs.append(item.name)
        return workspace_dirs
    elif args.directories:
        # Use directories specified on command line
        return args.directories
    else:
        # Use default directories
        return TARGET_DIRECTORIES

def main():
    """Main function to restyle source files"""
    # Parse command line arguments
    args = parse_arguments()
    
    print("W55RP20-S2E Code Formatter (Windows)")
    print("=" * 40)
    
    if args.dry_run:
        print("DRY RUN MODE - No files will be modified")
        print("-" * 40)
    
    # Find astyle executable
    if not args.dry_run:
        astyle_exe = find_astyle_executable()
        if not astyle_exe:
            print("Error: astyle not found!")
            print("Please install astyle and ensure it's in your PATH.")
            print("You can download astyle from: http://astyle.sourceforge.net/")
            sys.exit(1)
        
        print(f"Using astyle executable: {astyle_exe}")
    else:
        astyle_exe = "astyle"  # Dummy value for dry run
    
    # Get script directory and change to workspace root
    script_dir = Path(__file__).parent
    workspace_root = script_dir.parent
    os.chdir(workspace_root)
    
    print(f"Working directory: {workspace_root}")
    
    # Get target directories
    target_directories = get_target_directories(args)
    print(f"Target directories: {', '.join(target_directories)}")
    
    # Configuration files (relative to workspace root)
    core_config = Path(ASTYLE_CORE_CONFIG)
    examples_config = Path(ASTYLE_EXAMPLES_CONFIG)
    
    # Check if config files exist
    if not core_config.exists():
        print(f"Warning: {core_config} not found!")
    if not examples_config.exists():
        print(f"Warning: {examples_config} not found!")
    
    total_files = 0
    
    for directory in target_directories:
        if not Path(directory).exists():
            print(f"Warning: Directory {directory} not found, skipping...")
            continue
            
        print(f"\nProcessing directory: {directory}")
        
        # Find C/C++ files (excluding *api* paths)
        cpp_files = find_files(
            directory, 
            CPP_FILE_PATTERNS, 
            exclude_patterns=EXCLUDE_PATTERNS
        )
        
        if cpp_files:
            print(f"Found {len(cpp_files)} C/C++ files")
            if core_config.exists():
                restyle_files(cpp_files, str(core_config), astyle_exe, args.dry_run, args.verbose)
                total_files += len(cpp_files)
            else:
                print("Skipping C/C++ files (config not found)")
        
        # Find Arduino files
        ino_files = find_files(directory, ARDUINO_FILE_PATTERNS)
        
        if ino_files:
            print(f"Found {len(ino_files)} Arduino files")
            if examples_config.exists():
                restyle_files(ino_files, str(examples_config), astyle_exe, args.dry_run, args.verbose)
                total_files += len(ino_files)
            else:
                print("Skipping Arduino files (config not found)")
    
    if args.dry_run:
        print(f"\n[DRY RUN] Would process {total_files} files")
    else:
        print(f"\nFormatting completed! Total files processed: {total_files}")

if __name__ == '__main__':
    main()
