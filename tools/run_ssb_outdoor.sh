#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$ROOT_DIR/build/nr-toa-uesoftmodem"
DEFAULT_CFG="$ROOT_DIR/targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.usrpb210.n1_422910_pci84.precise.conf"

CFG="${1:-$DEFAULT_CFG}"
LOG_PATH="${2:-/tmp/nr_toa_outdoor_$(date +%Y%m%d_%H%M%S).log}"
IQ_DIR_DEFAULT="/tmp/nr_toa_iq_outdoor_$(date +%Y%m%d_%H%M%S)"
export NR_TOA_IQ_DUMP_DIR="${NR_TOA_IQ_DUMP_DIR:-$IQ_DIR_DEFAULT}"

if [[ ! -x "$BIN" ]]; then
  echo "missing executable: $BIN"
  echo "build first: cmake --build $ROOT_DIR/build -j4"
  exit 1
fi

if [[ ! -f "$CFG" ]]; then
  echo "missing config: $CFG"
  exit 1
fi

mkdir -p "$(dirname "$LOG_PATH")" "$NR_TOA_IQ_DUMP_DIR"

echo "Outdoor SSB test"
echo "bin: $BIN"
echo "cfg: $CFG"
echo "log: $LOG_PATH"
echo "iq_dump_dir: $NR_TOA_IQ_DUMP_DIR"
echo
echo "Live success markers:"
echo "  SSB candidate seen:   pss_hits:"
echo "  PBCH tried:           pbch_cand: / pbch_bch:"
echo "  PBCH+MIB success:     SSB_LOCK_EVENT:"
echo "  Final per-cycle line: sync_result:"
echo
echo "Stop with Ctrl+C."
echo

stdbuf -oL -eL "$BIN" "$CFG" | tee "$LOG_PATH"
