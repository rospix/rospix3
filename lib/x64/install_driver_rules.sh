#!/bin/bash

# Install script for Linux distributions
# This is a basic installer that merely copies the include files and
# libraries to the system-wide directories.

# Copy the udev rules file and reload all rules
cp ./60-opalkelly.rules /etc/udev/rules.d
cp ./60-pixet.rules /etc/udev/rules.d
udevadm control --reload-rules

# create symlink to libudev.0
LIBUDEV=`ldconfig -p | grep libudev | grep -oP "/.*" | sed -n 1p`
LIBDIR=$(dirname "${LIBUDEV}")

if [ ! -f "$LIBDIR"/libudev.so.0 ]; then
    sudo ln -s "$LIBUDEV" "$LIBDIR"/libudev.so.0
fi
