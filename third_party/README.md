# Third-party code and assets

| Component | Source and revision | License |
| --- | --- | --- |
| Ardens core | [Ardens e811cc6](https://github.com/tiberiusbrown/Ardens/tree/e811cc6be72c8a802bd3265a84daf4165c177b93) | [MIT](ardens/LICENSE.txt) |
| Cathy3K bootloader | [Mr.Blinky / Arduboy Homemade Package](https://github.com/MrBlinky/Arduboy-homemade-package), via Ardens | [MIT](ardens/src/boot/cathy3k_LICENSE) |
| bitsery, LZAV, yyjson | The copies from the pinned Ardens revision | [MIT](ardens/bitsery/LICENSE), [MIT](ardens/lzav/LICENSE), [MIT](ardens/yyjson/LICENSE) |
| miniz | [miniz ae07a36](https://github.com/richgel999/miniz/tree/ae07a3615925f252f54fecb84aee2e3cfd4f0676) | [License](miniz/LICENSE) |
| LodePNG | [LodePNG 7b2893e](https://github.com/lvandeve/lodepng/tree/7b2893e35891c0c85bee5595caafaf3c2044b409) | [zlib](lodepng/LICENSE) |
| Liberation Sans bitmap glyphs | Rasterized from the font shipped by Debian, using [make-font.py](../tools/make-font.py) | [SIL OFL 1.1](Liberation-fonts.txt) |

The Ardens subset builds without LLVM, desktop UI, profiler or full snapshots. The port moves instruction tables out of the main emulator allocation, makes game hashes from the ROM and initial FX content, and limits emulated flash writes to the declared save area. It uses Ardens' device-only save state serialization inside Arduboy Go's checksummed save format.

The ODROID-GO pin assignments and LCD setup follow [Hardkernel's ODROID-GO library](https://github.com/hardkernel/ODROID-GO) and [Retro-Go's ODROID-GO target](https://github.com/ducalex/retro-go/blob/4ced120669750ca7228fd0414211430c1d923166/components/retro-go/targets/odroid-go/config.h). The [SDSPI workaround](../tools/idf-sdspi.patch) adapts a [Retro-Go patch](https://github.com/ducalex/retro-go/blob/4ced120669750ca7228fd0414211430c1d923166/tools/patches/sdcard-fix%20(esp-idf%205).diff) to ESP-IDF 5.4.3.
