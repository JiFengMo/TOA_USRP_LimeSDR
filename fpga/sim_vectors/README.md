# FPGA Simulation Vectors

Use existing `.c16` captures as the shared truth source for software and PL.

Generate a manifest plus raw IQ binary:

```bash
python tools/export_fpga_vectors.py \
  --iq verify_queue_20260409_204055/iq/pbch_nearmiss_seq00_ts43599106_cf2114550000_fs30720000_pci84_ssb1_pss229_dmrs152.c16 \
  --out fpga/sim_vectors/pbch_nearmiss_seq00
```

Expected outputs:

- `fpga/sim_vectors/pbch_nearmiss_seq00/iq.c16`
- `fpga/sim_vectors/pbch_nearmiss_seq00/manifest.json`

The manifest records sample rate, center frequency, PCI hints, and capture
metadata. Software golden outputs should be added beside it as
`golden_sync.json` after running `replay_toa_iq`.
