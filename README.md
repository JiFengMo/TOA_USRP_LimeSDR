# NR-TOA USRP/LimeSDR

This repository is a C/C++ prototype for 5G NR time-of-arrival positioning.
It combines SDR radio backends, SSB/PRS synchronization and TOA estimation
logic, offline replay tools, and a first FPGA-provider scaffold for moving SSB
timing work into programmable logic.

## Current Scope

- UE-side NR TOA capture and measurement using USRP B210 or LimeSDR-style radio
  backends.
- Anchor-side SSB beacon generation and scheduled transmission.
- SSB/PBCH synchronization, channel estimation, CIR construction, integer and
  fractional TOA estimation, and measurement association.
- PRS extraction and TOA estimation test scaffolding.
- Offline IQ replay for reproducing SSB/PBCH near-miss and success cases.
- Initial ZedBoard/FMCOMMS3 FPGA boundary for a future SSB TOA accelerator.

The project is currently a research and bring-up tree. Some generated build
outputs and captured IQ data may exist in local snapshots, but the main source
interfaces are under `radio/`, `openair1/PHY/NR_POSITIONING/`, `executables/`,
`tools/`, `targets/`, and `fpga/`.

## Directory Map

```text
.
|-- CMakeLists.txt
|-- common/                         # Shared utility headers from the OAI-style tree
|-- executables/
|   |-- nr-toa-uesoftmodem.c         # UE receiver entry point
|   `-- nr-ssb-anchor.c              # Anchor / SSB beacon entry point
|-- radio/
|   |-- COMMON/common_lib.h          # Common radio-device contract
|   |-- USRP/usrp_lib.cpp            # UHD/USRP backend
|   `-- LIME/lime_lib.cpp            # LimeSDR backend placeholder/path
|-- openair1/PHY/NR_POSITIONING/
|   |-- nr_pos_api.h                 # Public module API aggregation
|   |-- nr_pos_provider_if.h         # Provider interface
|   |-- common/                      # IQ ring, clock, config, solver, radio adapter
|   |-- ssb/                         # SSB/PSS/SSS/PBCH sync and TOA estimation
|   |-- prs/                         # PRS processing path
|   |-- fpga/                        # Software provider boundary for PL acceleration
|   `-- tests/                       # Unit-style test executables
|-- targets/PROJECTS/NR-TOA/CONF/    # UE, anchor, SDR, sweep, and anchor DB configs
|-- tools/                           # Replay, plotting, export, and run helpers
|-- verification_logs/               # Historical logs for comparison/debugging
|-- verify_queue_20260409_204055/    # Reusable IQ replay captures and metadata
`-- fpga/                            # Vivado/IP scaffold for future hardware offload
```

## Build Requirements

The CMake build expects a Linux-like development environment with:

- CMake 3.16 or newer
- C and C++ compilers with C11/C++17 support
- UHD development package discoverable through `pkg-config`
- pthread and libm

Example package names vary by distribution, but the important dependency is
that `pkg-config --modversion uhd` succeeds before running CMake.

## Build

Use an out-of-tree build directory:

```bash
cmake -S . -B build
cmake --build build -j
```

Expected primary outputs:

- `build/nr-toa-uesoftmodem`
- `build/nr-ssb-anchor`
- `build/replay_toa_iq`
- `build/test_ssb_sync`
- `build/test_ssb_toa`
- `build/test_prs_toa`
- `build/test_solver`
- `build/test_fpga_provider_select`

## Run Tests

After building, run the available test binaries from the build directory:

```bash
./build/test_ssb_sync
./build/test_ssb_toa
./build/test_prs_toa
./build/test_solver
./build/test_fpga_provider_select
```

These tests exercise the local positioning primitives. They do not replace
hardware-in-the-loop validation with a synchronized SDR clock source.

## UE Receiver

Typical UE configurations live in `targets/PROJECTS/NR-TOA/CONF/`.

Example:

```bash
./build/nr-toa-uesoftmodem \
  -O targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.usrpb210.n1_2114.fixed.conf
```

Important configuration fields include:

- `sdr`: selects the backend, for example `usrp`.
- `sdr_addrs`: UHD device selector such as a serial number.
- `clock_source` and `time_source`: `internal` for standalone tests, `external`
  for synchronized timing experiments.
- `center_freq_hz`, `sample_rate_hz`, `rx_gain_db`, `tx_gain_db`.
- `mode`: currently used for `SSB_TOA` or `PRS_TOA` paths.
- `anchor_db_path`: CSV anchor database for measurement association/solving.
- `target_pci`, `ssb_scs_khz`, and sweep-related fields for SSB search control.

## SSB Anchor

Anchor configurations also live under `targets/PROJECTS/NR-TOA/CONF/`.

Example:

```bash
./build/nr-ssb-anchor \
  -O targets/PROJECTS/NR-TOA/CONF/anchor0.ssb.usrpb210.conf
```

For timing experiments, the anchor should use a validated external clock/time
source and an anchor database that matches the transmitted PCI, SSB index, and
physical anchor coordinates.

## Offline Replay

The replay tool is intended for deterministic debugging of captured IQ blocks:

```bash
./build/replay_toa_iq <capture.c16> <capture.meta.txt>
```

The `verify_queue_20260409_204055/` tree contains reusable `.c16` captures and
matching metadata for SSB/PBCH investigation. Historical logs in
`verification_logs/` are useful comparison points when checking PBCH decode and
DMRS scoring behavior.

## FPGA Scaffold

The FPGA material is intentionally a boundary scaffold, not a finished hardware
accelerator. The current split is:

- `fpga/`: Vivado/IP packaging scaffold for a ZedBoard/FMCOMMS3 target.
- `fpga/ip/ssb_toa/rtl/ssb_toa_axis_stub.v`: AXI-Stream placeholder IP.
- `fpga/sw/nr_fpga_ssb_regs.h`: software-visible register contract.
- `openair1/PHY/NR_POSITIONING/fpga/nr_fpga_ssb_provider.c`: provider hook that
  preserves the existing software provider contract.

See `fpga/README.md` and `openair1/PHY/NR_POSITIONING/fpga/README.md` before
changing the hardware boundary.

## Notes For Bring-Up

- Treat SDR timing and clock status as first-class validation items. A clean
  software decode does not prove external time alignment.
- Prefer offline replay for algorithm changes before running live RF tests.
- Keep generated build directories out of new commits unless a snapshot branch
  deliberately records the full local state.
- The `snapshot/current-version` branch is intended for preserving a known local
  state, while feature branches should keep reviewable source changes smaller.
