#!/usr/bin/env python3

import sys
import subprocess
import glob

def main():
    if len(sys.argv) != 4:
        print(f"Usage: python {sys.argv[0]} <address> <user> <password>", file=sys.stderr)
        sys.exit(1)

    camera_host = sys.argv[1]
    username = sys.argv[2]
    password = sys.argv[3]

    # Find the first .eap file in the current directory
    eap_files = sorted(glob.glob("*.eap"))
    if not eap_files:
        print("ERROR: No .eap file found in the current directory", file=sys.stderr)
        sys.exit(1)

    eap_file = eap_files[0]
    print(f"Installing {eap_file} to {camera_host}...")

    # Upload using curl with digest authentication
    result = subprocess.run([
        "curl", "--digest",
        "-u", f"{username}:{password}",
        "-F", f"packfil=@{eap_file};type=application/octet-stream",
        f"http://{camera_host}/axis-cgi/applications/upload.cgi"
    ])

    if result.returncode != 0:
        print("ERROR: Upload failed", file=sys.stderr)
        sys.exit(result.returncode)

    print("\nDone.")

if __name__ == "__main__":
    main()
