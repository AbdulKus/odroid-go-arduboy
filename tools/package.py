#!/usr/bin/env python3
"""Package a built ESP32 app for the ODROID-GO SD installer and serial flash."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import zlib


def tile():
    data = bytearray()
    for y in range(48):
        for x in range(86):
            color = 0x10C4
            if 27 <= x < 59 and 7 <= y < 41:
                color = 0x7739
            if 31 <= x < 55 and 11 <= y < 25:
                color = 0x10C4
                if (x - 48) ** 2 + (y - 17) ** 2 in range(12, 20):
                    color = 0x7739
            if (33 <= x < 43 and y == 32) or (x == 37 and 28 <= y < 37):
                color = 0x10C4
            if (x - 49) ** 2 + (y - 32) ** 2 <= 2 or (x - 53) ** 2 + (y - 30) ** 2 <= 2:
                color = 0x10C4
            data.extend(struct.pack("<H", color))
    return data


def write_firmware(path, app, version):
    if app[0] != 0xE9:
        raise ValueError("Expected an ESP32 executable image")
    partition_size = (len(app) + 0xFFFF) & ~0xFFFF
    description = f"Arduboy Go {version}".encode("ascii")
    header = struct.pack("<24s40s8256s", b"ODROIDGO_FIRMWARE_V00_01", description, tile())
    partition = struct.pack("<BBxx16sIII", 0, 0x10, b"arduboy-go", 0, partition_size, len(app))
    payload = header + partition + app
    path.write_bytes(payload + struct.pack("<I", zlib.crc32(payload)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--out", type=Path, default=Path("dist"))
    parser.add_argument("--version", default="dev")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9._-]{1,28}", args.version):
        parser.error("version must contain 1–28 letters, digits, dots, underscores or hyphens")
    build, out = args.build.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    info = json.loads((build / "flasher_args.json").read_text())
    name = f"arduboy-go-{args.version}-odroid-go"
    fw = out / (name + ".fw")
    img = out / (name + ".img")
    write_firmware(fw, (build / info["app"]["file"]).read_bytes(), args.version)
    subprocess.run(
        [sys.executable, "-m", "esptool", "--chip", "esp32", "merge_bin",
         "-o", str(img), *info["write_flash_args"],
         *[item for entry in info["flash_files"].items() for item in entry]],
        cwd=build, check=True,
    )
    with (out / "SHA256SUMS").open("w", encoding="ascii") as checksums:
        for path in (fw, img):
            checksums.write(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n")


if __name__ == "__main__":
    main()
