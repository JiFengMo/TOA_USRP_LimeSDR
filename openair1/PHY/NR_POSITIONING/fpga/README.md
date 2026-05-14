# FPGA Provider Boundary

`nr_fpga_ssb_provider.c` is the software-side insertion point for the future PL
SSB TOA accelerator.

Current first-pass behavior:

- `provider = fpga_ssb` selects `fpga_ssb_provider`.
- Default backend is software-emulated and delegates to `nr_ssb_provider`, so
  existing IQ replay and golden tests still work before hardware is present.
- Set `NR_TOA_FPGA_BACKEND=uio` later to require a real UIO/DMA backend. That
  path intentionally fails today until the register and DMA binding is added.

The provider must preserve the existing contract:

```text
nr_iq_block_t -> acquire/track/extract_meas -> nr_sync_state_t + nr_toa_meas_t
```

This keeps PS-side control, logging, PBCH/PDCCH decode, and positioning solve in
software while the PL path gradually replaces high-throughput SSB timing work.
