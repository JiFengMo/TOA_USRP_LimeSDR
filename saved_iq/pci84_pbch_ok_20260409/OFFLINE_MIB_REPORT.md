# Offline MIB Report

This report summarizes the two saved `PBCH OK` IQ captures for
`n1 / ARFCN 422910 / PCI 84`.

## Result

Both saved captures contain a successful PBCH decode and a valid MIB result in
their companion metadata files.

## Capture A

- IQ: `pbch_ok_seq01_ts44786797_cf2114550000_fs30720000_pci84_ssb3_pss301_dmrs771.c16`
- Meta: `pbch_ok_seq01_ts44786797_cf2114550000_fs30720000_pci84_ssb3_pss301_dmrs771.meta.txt`
- RX gain: `61.0 dB`
- PCI: `84`
- SSB index: `3`
- CFO: `2157.42 Hz`
- SNR: `2.25 dB`
- PSS metric: `0.301030`
- PBCH metric: `0.770502`
- PBCH second metric: `0.768809`
- PBCH OK: `1`
- MIB OK: `1`
- MIB payload: `0x642b15`
- Source success log: `precise_trials_run_20260409_113022/pci84_gain61.log`
- Source success line: `pbch_bch: ok pci=84 ssb=3 ... mib=0x642b15`

Decoded fields from `0x642b15`:

- `systemFrameNumber[9:4]` = `25`
- Full `SFN` from source success log = `410`
- `subCarrierSpacingCommon` = `0` (`scs15or60`, for this FR1 run interpreted as `15 kHz`)
- `ssb-SubcarrierOffset[3:0]` = `1`
- `dmrs-TypeA-Position` = `0` (`pos2`)
- `pdcch-ConfigSIB1` = `177` (`0xB1`)
- `controlResourceSetZero` = `11`
- `searchSpaceZero` = `1`
- `cellBarred` = `0` (`barred`, raw decoded value)
- `intraFreqReselection` = `1` (`notAllowed`)
- Remaining payload tail bit = implementation-side extra or padding bit in this 24-bit representation

## Capture B

- IQ: `pbch_ok_seq00_ts43508292_cf2114550000_fs30720000_pci84_ssb3_pss253_dmrs667.c16`
- Meta: `pbch_ok_seq00_ts43508292_cf2114550000_fs30720000_pci84_ssb3_pss253_dmrs667.meta.txt`
- RX gain: `64.0 dB`
- PCI: `84`
- SSB index: `3`
- CFO: `1111.28 Hz`
- SNR: `1.93 dB`
- PSS metric: `0.252712`
- PBCH metric: `0.667194`
- PBCH second metric: `0.661772`
- PBCH OK: `1`
- MIB OK: `1`
- MIB payload: `0x8c2b15`
- Source success log: `precise_trials_run_20260409_113022/pci84_gain64.log`
- Source success line: `pbch_bch: ok pci=84 ssb=3 ... mib=0x8c2b15`

Decoded fields from `0x8c2b15`:

- `systemFrameNumber[9:4]` = `35`
- Full `SFN` from source success log = `570`
- `subCarrierSpacingCommon` = `0` (`scs15or60`, for this FR1 run interpreted as `15 kHz`)
- `ssb-SubcarrierOffset[3:0]` = `1`
- `dmrs-TypeA-Position` = `0` (`pos2`)
- `pdcch-ConfigSIB1` = `177` (`0xB1`)
- `controlResourceSetZero` = `11`
- `searchSpaceZero` = `1`
- `cellBarred` = `0` (`barred`, raw decoded value)
- `intraFreqReselection` = `1` (`notAllowed`)
- Remaining payload tail bit = implementation-side extra or padding bit in this 24-bit representation

## Replay Note

The standalone `replay_toa_iq` tool was run offline against both `.c16` files.
Its generic free search, and a metadata-seeded replay attempt, did not
deterministically re-lock the same candidate in every run. However, these IQ
files were saved only when the live pipeline had already completed `pbch_ok=1`
and `mib_ok=1`, and that successful decode state is preserved in the saved
metadata above.

## Field Mapping Note

The `24-bit` `mib_payload` stored by this positioning workflow is not exposed as
a fully expanded ASN.1 object. The split above follows the standard NR MIB field
layout for the named fields, while the last remaining payload bit is treated as
an implementation-side extra or padding bit in this saved representation.
