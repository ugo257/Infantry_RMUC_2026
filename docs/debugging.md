# Debugging Setup

## OpenOCD + CMSIS-DAP

This repository uses a project-local OpenOCD install, so the VS Code launch
configuration does not depend on machine-specific absolute paths.

Install the local OpenOCD tool once:

```sh
./tools/install_openocd.sh
```

The debugger launch configuration will then use:

- `tools/debug/openocd-wrapper.sh`
- `tools/debug/arm-gdb-wrapper.sh`

`openocd-wrapper.sh` prefers the repository-local install under `.tools/`,
and falls back to a system `openocd` if one is already available in `PATH`.

For the `jixin.pro` `CMSIS-DAP_LU` probe used with this project, the OpenOCD
config pins:

- `cmsis-dap backend hid`
- `cmsis-dap vid_pid 0xc251 0xf001`
- `adapter serial LU_2022_8888`

That probe exposes CDC/DCI helper interfaces in addition to the real debug
interface, and OpenOCD autodetection can otherwise pick the wrong one.

If OpenOCD reports `CMSIS-DAP command CMD_INFO failed`, the probe is usually
enumerated but not responding to HID commands. In that case, unplug/replug the
probe directly to the Mac and retry the `Cortex Debug (OpenOCD, CMSIS-DAP,
STM32F407)` launch configuration.

## Live Watch

Use the `Cortex Debug (OpenOCD, CMSIS-DAP, STM32F407)` launch configuration in
VS Code. After the session starts, open `Cortex Live Watch` from the debug
sidebar and add global/static variables to monitor them in real time.
