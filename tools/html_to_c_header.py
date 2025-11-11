#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HTML to C Header Converter
Converts Web_page.html to Web_page.h with static const unsigned char array
"""

import os

def html_to_c_header():
    """
    Converts HTML file to C header file.
    Web_page.html -> Web_page.h
    """
    
    # Set file paths
    base_path = "port/app/html_file"
    html_file = os.path.join(base_path, "Web_page.html")
    header_file = os.path.join(base_path, "Web_page.h")
    
    try:
        # Read HTML file
        with open(html_file, 'rb') as f:
            html_data = f.read()
        
        # HTML data size (including null terminator)
        data_size = len(html_data) + 1
        
        print(f"HTML file size: {len(html_data)} bytes")
        print(f"Array size (including null terminator): {data_size} bytes")
        
        # Generate C header file
        with open(header_file, 'w', encoding='utf-8') as f:
            # Get current date
            from datetime import datetime
            current_date = datetime.now().strftime("%Y-%m-%d")
            
            # Write header comment
            f.write("/*\n")
            f.write("    file: Web_page.h\n")
            f.write("    description: Web_Page Constant header file\n")
            f.write("    author: Mason\n")
            f.write("    company: WIZnet\n")
            f.write(f"    data: {current_date}\n")
            f.write("*/\n")
            f.write("\n")
            
            # Add header guard
            f.write("#ifndef __WEB_PAGE_H__\n")
            f.write("#define __WEB_PAGE_H__\n")
            f.write("\n")
            
            # Start array declaration
            f.write(f"static const unsigned char _acWeb_page[0x{data_size:X}] = {{\n")
            
            # Output hexadecimal data, 16 bytes per line
            bytes_per_line = 16
            for i in range(0, len(html_data), bytes_per_line):
                line_data = html_data[i:i + bytes_per_line]
                hex_values = [f"0x{byte:02X}" for byte in line_data]
                f.write("    " + ", ".join(hex_values))
                
                # Add comma if not the last line
                if i + bytes_per_line < len(html_data):
                    f.write(",\n")
                else:
                    # Add null terminator at the last line
                    f.write(",\n    0x00\n")
            
            # End array declaration
            f.write("};\n")
            f.write("\n")
            f.write("#endif /* __WEB_PAGE_H__ */\n")
        
        print(f"[SUCCESS] C header file generated successfully: {header_file}")
        print(f"[INFO] Array name: _acWeb_page[0x{data_size:X}]")
        
    except FileNotFoundError as e:
        print(f"[ERROR] File not found: {e}")
    except Exception as e:
        print(f"[ERROR] Error occurred: {e}")

if __name__ == "__main__":
    print("HTML to C Header Converter")
    print("=" * 50)
    html_to_c_header()