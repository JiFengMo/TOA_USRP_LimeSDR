# NR-TOA FPGA Bring-Up Scaffold

This directory is the first-pass hardware boundary for the SSB TOA FPGA port.
The software-facing contract stays aligned with the existing C provider model:

```text
AD9361 RX IQ -> axi_ad9361 -> AXI-Stream -> ssb_toa IP -> result FIFO/regs -> PS provider
```

## Target

- Board: ZedBoard, `xc7z020clg484-1`
- RF card: AD-FMCOMMS3-EBZ / AD9361
- Tool version used for this scaffold: Vivado 2022.2
- Runtime: Linux with ADI/libiio for AD9361 bring-up

## Files

- `ip/ssb_toa/rtl/ssb_toa_axis_stub.v`: AXI-Stream placeholder for the future SSB TOA accelerator.
- `ip/ssb_toa/package_ip.tcl`: packages the placeholder as a local Vivado IP.
- `sw/nr_fpga_ssb_regs.h`: PS-side register contract shared by the C provider and future UIO/DMA code.
- `adi_base/create_zed_fmcomms3_project.tcl`: validates and opens the ADI `fmcomms2_zed` project flow.
- `sim_vectors/README.md`: golden-vector workflow from existing `.c16` replay captures.

## Commands

Check and emit the local placeholder IP scaffold:

```bash
vivado -mode batch -source fpga/ip/ssb_toa/package_ip.tcl -tclargs fpga/ip/ssb_toa
```

Expected output:

- `fpga/ip/ssb_toa/packaged/component.xml`
- `fpga/ip/ssb_toa/packaged/hdl/ssb_toa_axis_stub.v`
- `fpga/ip/ssb_toa/packaged/reports/ssb_toa_elab.rpt`

Create the ADI ZedBoard/FMCOMMS3 base project after cloning ADI HDL:

```bash
vivado -mode batch -source fpga/adi_base/create_zed_fmcomms3_project.tcl -tclargs /path/to/adi/hdl
```

Expected output:

- ADI-generated `fmcomms2_zed` Vivado project under the ADI HDL tree.

Next step after this scaffold is to replace `ssb_toa_axis_stub.v` internals with
PSS search, SSB grid extraction, PBCH DMRS scoring, CIR, and TOA peak logic.
