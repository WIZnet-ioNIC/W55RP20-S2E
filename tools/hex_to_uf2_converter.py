#!/usr/bin/env python3
"""
Hex to UF2 Converter

This program converts the merged hex file (Boot-App_linker_Merged.hex) 
from bin_files folder to binary format and then converts it to UF2 format
for RP2040 flashing.

Usage:
    python hex_to_uf2_converter.py
"""

import subprocess
import os
import sys
import shutil

# Define paths
BIN_FILES_DIR = "bin_files"
TOOLS_DIR = "tools"
INPUT_HEX = "Boot-App_linker_Merged.hex"
OUTPUT_BIN = "Boot-App_linker_Merged.bin"
OUTPUT_UF2 = "Boot-App_linker_Merged.uf2"

# RP2040 specific settings
BASE_ADDRESS = "0x10000000"
FAMILY_ID = "RP2040"

def check_file_exists(file_path, description):
    """Check if a file exists and print appropriate message."""
    if not os.path.exists(file_path):
        print(f"[ERROR] {description} not found: {file_path}")
        return False
    print(f"[INFO] Found {description}: {file_path}")
    return True

def run_command(cmd, description):
    """Run a command and handle errors."""
    print(f"[INFO] {description}...")
    print(f"[CMD] {' '.join(cmd)}")
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        if result.stdout:
            print(f"[OUTPUT] {result.stdout.strip()}")
        print(f"[SUCCESS] {description} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] {description} failed:")
        print(f"[ERROR] Return code: {e.returncode}")
        if e.stdout:
            print(f"[ERROR] stdout: {e.stdout}")
        if e.stderr:
            print(f"[ERROR] stderr: {e.stderr}")
        return False
    except FileNotFoundError:
        print(f"[ERROR] Command not found. Make sure the required tools are installed.")
        return False

def main():
    """Main conversion process."""
    print("=" * 60)
    print("HEX to UF2 Converter for RP2040")
    print("=" * 60)
    
    # Change to workspace directory (parent of tools)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(script_dir)
    os.chdir(workspace_dir)
    print(f"[INFO] Working directory: {workspace_dir}")
    
    # Define full paths
    input_hex_path = os.path.join(BIN_FILES_DIR, INPUT_HEX)
    output_bin_path = os.path.join(BIN_FILES_DIR, OUTPUT_BIN)
    output_uf2_path = os.path.join(BIN_FILES_DIR, OUTPUT_UF2)
    uf2conv_script = os.path.join(TOOLS_DIR, "uf2conv.py")
    
    # Check if input files exist
    if not check_file_exists(input_hex_path, "Input HEX file"):
        print(f"[INFO] Please make sure merge_hex.py has been run first to generate {INPUT_HEX}")
        sys.exit(1)
    
    if not check_file_exists(uf2conv_script, "uf2conv.py script"):
        sys.exit(1)
    
    # Step 1: Convert HEX to BIN using arm-none-eabi-objcopy
    print("\n" + "-" * 40)
    print("Step 1: Converting HEX to BIN")
    print("-" * 40)
    
    hex_to_bin_cmd = [
        "arm-none-eabi-objcopy",
        "-I", "ihex",
        "-O", "binary",
        input_hex_path,
        output_bin_path
    ]
    
    if not run_command(hex_to_bin_cmd, "HEX to BIN conversion"):
        sys.exit(1)
    
    # Verify BIN file was created
    if not check_file_exists(output_bin_path, "Generated BIN file"):
        sys.exit(1)
    
    # Get file size info
    bin_size = os.path.getsize(output_bin_path)
    print(f"[INFO] Generated BIN file size: {bin_size} bytes ({bin_size/1024:.2f} KB)")
    
    # Step 2: Convert BIN to UF2 using uf2conv.py
    print("\n" + "-" * 40)
    print("Step 2: Converting BIN to UF2")
    print("-" * 40)
    
    bin_to_uf2_cmd = [
        sys.executable,  # Use current Python interpreter
        uf2conv_script,
        output_bin_path,
        "--base", BASE_ADDRESS,
        "--family", FAMILY_ID,
        "--convert",
        "--output", output_uf2_path
    ]
    
    if not run_command(bin_to_uf2_cmd, "BIN to UF2 conversion"):
        sys.exit(1)
    
    # Verify UF2 file was created
    if not check_file_exists(output_uf2_path, "Generated UF2 file"):
        sys.exit(1)
    
    # Get file size info
    uf2_size = os.path.getsize(output_uf2_path)
    print(f"[INFO] Generated UF2 file size: {uf2_size} bytes ({uf2_size/1024:.2f} KB)")
    
    # Summary
    print("\n" + "=" * 60)
    print("CONVERSION COMPLETED SUCCESSFULLY!")
    print("=" * 60)
    print(f"Input:  {input_hex_path}")
    print(f"Binary: {output_bin_path} ({bin_size} bytes)")
    print(f"UF2:    {output_uf2_path} ({uf2_size} bytes)")
    print("\nYou can now flash the UF2 file to your RP2040 device.")
    print("=" * 60)

if __name__ == "__main__":
    main()