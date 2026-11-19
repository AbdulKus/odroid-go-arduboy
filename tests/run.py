#!/usr/bin/env python3
"""Create isolated SD fixtures and run the desktop checks."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import zipfile
import zlib


def preview():
    def chunk(name, data):
        return struct.pack(">I", len(data)) + name + data + struct.pack(">I", zlib.crc32(name + data))

    data = bytearray()
    for y in range(64):
        data.append(0)
        for x in range(128):
            d = (x - 86) ** 2 + (y - 27) ** 2
            color = (12, 29, 44)
            if 230 < d <= 310 or (d < 230 and (x + 2 * y) % 5 == 0):
                color = (104, 219, 211)
            if (x * 13 + y * 39 + (x ^ y) * 7) % 113 == 0 and 6 < y < 47 and d > 310:
                color = (180, 207, 206)
            if (9 <= x <= 31 and abs(y - 38) == (x - 9) // 6) or (8 <= x <= 23 and y == 38):
                color = (249, 180, 93)
            if 37 < x < 61 and y == 42 + x // 12:
                color = (77, 162, 167)
            if y == 53 and 5 < x < 121:
                color = (44, 89, 104)
            data.extend(color)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 128, 64, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(data)) + chunk(b"IEND", b""))


def setup(root, hex_file):
    for kind in ("core", "large", "ui"):
        (root / kind / "roms/arduboy").mkdir(parents=True)
    for kind in ("core", "ui"):
        games = root / kind / "roms/arduboy"
        with zipfile.ZipFile(games / ("Orbit Signal.arduboy" if kind == "core" else "00 Orbit Signal.arduboy"),
                             "w", compression=zipfile.ZIP_DEFLATED) as package:
            package.writestr("info.json", json.dumps({
                "title": "Orbit Signal: Flight Test" if kind == "core" else "Orbit Signal",
                "banner": "banner.png",
                "binaries": [{"filename": "flight.hex"}],
            }))
            package.write(hex_file, "flight.hex")
            package.writestr("banner.png", preview())
    (root / "core/roms/arduboy/Broken.hex").write_text(":00000001FF\n")
    for name in ("Cinder Run", "Glass Circuit", "Lunar Relay", "Night Survey",
                 "Pinball Lab", "Starfield", "Vector Drift"):
        (root / "ui/roms/arduboy" / (name + ".hex")).write_bytes(hex_file.read_bytes())
    for page in range(10):
        group = root / f"large/roms/arduboy/Pack-{page}"
        group.mkdir()
        for i in range(500):
            (group / f"Game-{page * 500 + i:04}.HEX").touch()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("check", type=Path)
    parser.add_argument("hex_file", type=Path)
    parser.add_argument("--capture", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="ardugo-test-") as path:
        root = Path(path)
        setup(root, args.hex_file)
        subprocess.run([str(args.check.resolve()), str(root),
                        str(args.capture.resolve()) if args.capture else "-"], check=True)


if __name__ == "__main__":
    main()
