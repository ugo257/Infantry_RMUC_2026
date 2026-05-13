#!/bin/sh
set -eu

if command -v arm-none-eabi-gdb >/dev/null 2>&1; then
    exec arm-none-eabi-gdb "$@"
fi

for candidate in \
    "/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb" \
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeCLT/bin/arm-none-eabi-gdb"
do
    if [ -x "$candidate" ]; then
        exec "$candidate" "$@"
    fi
done

echo "arm-none-eabi-gdb not found in PATH." >&2
echo "Install STM32CubeCLT or add arm-none-eabi-gdb to PATH." >&2
exit 127
