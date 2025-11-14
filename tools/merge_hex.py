import subprocess
import os
import shutil

# Define paths
BOOT_DIR = "build/main/Boot"
APP_DIR = "build/main/App"
OUTPUT_DIR = "bin_files"

# Create output directory if it doesn't exist
try:
    if os.path.exists(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)
except PermissionError:
    print(f"Warning: Could not remove {OUTPUT_DIR}. Directory may be in use.")
    pass  # Continue if directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Input ELF and BIN paths
boot_elf = os.path.join(BOOT_DIR, "Boot.elf")
boot_bin_src = os.path.join(BOOT_DIR, "Boot.bin")
boot_u2f = os.path.join(BOOT_DIR, "Boot.uf2")

app_elf = os.path.join(APP_DIR, "App_linker.elf")
app_bin_src = os.path.join(APP_DIR, "App_linker.bin")
app_u2f = os.path.join(APP_DIR, "App_linker.uf2")

# Output HEX and BIN paths
boot_hex = os.path.join(OUTPUT_DIR, "Boot.hex")
app_hex = os.path.join(OUTPUT_DIR, "App_linker.hex")
boot_bin_dst = os.path.join(OUTPUT_DIR, "Boot.bin")
app_bin_dst = os.path.join(OUTPUT_DIR, "App_linker.bin")
boot_u2f_dst = os.path.join(OUTPUT_DIR, "Boot.uf2")
app_u2f_dst = os.path.join(OUTPUT_DIR, "App_linker.uf2")
merged_hex = os.path.join(OUTPUT_DIR, "Boot-App_linker_Merged.hex")

# Convert ELF to HEX
def elf_to_hex(elf_file, hex_file):
    subprocess.run(["arm-none-eabi-objcopy", "-O", "ihex", elf_file, hex_file], check=True)
    print(f"Converted {elf_file} -> {hex_file}")

# Move existing BIN files to output directory
def move_bin_file(src, dst):
    if os.path.exists(src):
        shutil.copy2(src, dst)
        print(f"Moved {src} -> {dst}")
    else:
        print(f"[Warning] BIN file not found: {src}")

# Merge HEX files using srec_cat
def merge_hex_files(hex1, hex2, output_hex):
    subprocess.run(["srec_cat", hex1, "-Intel", hex2, "-Intel", "-o", output_hex, "-Intel"], check=True)
    print(f"Merged {hex1} + {hex2} -> {output_hex}")

def main():
    # Convert ELF files to HEX
    elf_to_hex(boot_elf, boot_hex)
    elf_to_hex(app_elf, app_hex)

    # Move BIN files to bin_files/
    move_bin_file(boot_bin_src, boot_bin_dst)
    move_bin_file(app_bin_src, app_bin_dst)
    move_bin_file(boot_u2f, boot_u2f_dst)
    move_bin_file(app_u2f, app_u2f_dst)

    # Merge HEX files
    merge_hex_files(boot_hex, app_hex, merged_hex)

if __name__ == "__main__":
    main()
