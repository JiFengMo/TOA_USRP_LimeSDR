#!/usr/bin/env python3
"""Export NR-TOA C16 captures into FPGA simulation-vector folders."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def parse_meta(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    if not path.exists():
        return meta
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iq", required=True, type=Path, help="Input .c16 IQ capture")
    parser.add_argument("--out", required=True, type=Path, help="Output vector directory")
    args = parser.parse_args()

    iq_path = args.iq
    out_dir = args.out
    if not iq_path.exists():
        parser.error(f"IQ file does not exist: {iq_path}")
    if iq_path.suffix.lower() != ".c16":
        parser.error("input IQ file must use .c16 suffix")

    meta_path = iq_path.with_suffix(".meta.txt")
    meta = parse_meta(meta_path)
    nsamps = int(meta.get("nsamps", iq_path.stat().st_size // 4))
    manifest = {
        "format": "interleaved_s16_iq",
        "sample_bytes": 4,
        "iq_file": "iq.c16",
        "source_iq": str(iq_path),
        "source_meta": str(meta_path) if meta_path.exists() else None,
        "nsamps": nsamps,
        "fs_hz": float(meta.get("fs_hz", 0.0)),
        "center_freq_hz": float(meta.get("center_freq_hz", meta.get("cf_hz", 0.0))),
        "target_pci": int(meta.get("target_pci", -1)),
        "pci": int(meta.get("pci", -1)),
        "ssb_index": int(meta.get("ssb_index", -1)),
        "metadata": meta,
    }

    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(iq_path, out_dir / "iq.c16")
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {out_dir / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
