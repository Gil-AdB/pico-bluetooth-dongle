#!/bin/bash
echo "Starting build..." > build_log.txt
export PICO_SDK_PATH=/Users/gil-ad/work/pico-bluetooth-dongle/pico-sdk
cmake -S . -B build >> build_log.txt 2>&1
make -j8 -C build >> build_log.txt 2>&1
echo "Build finished." >> build_log.txt
