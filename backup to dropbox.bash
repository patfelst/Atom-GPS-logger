#!/bin/sh
# Ensure make executable with chmod +x myscript.bash
# Command backs up the folder to dropbox, excluding the ".pio" hidden directory which contains "libdeps" (library dependencies) and "build" (compiled binaries)
# Use --dry-run for a preview of what rsync will do
# rsync --dry-run --verbose --archive --update --include 'platformio.ini' --exclude '.pio/build' --exclude 'M5GFX' --exclude 'M5Unified' --exclude 'TFT_eSPI' --exclude '.DS_Store' "$PWD" "/Users/patrickfelstead/Dropbox/Patrick's Files/Electronic Design/PlatformIO"
rsync --verbose --archive --update --include 'platformio.ini' --exclude '.pio/build' --exclude 'M5GFX' --exclude 'M5Unified' --exclude 'TFT_eSPI' --exclude '.DS_Store' "$PWD" "/Users/patrickfelstead/Dropbox/Patrick's Files/Electronic Design/PlatformIO"