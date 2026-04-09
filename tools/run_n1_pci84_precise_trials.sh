#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_CFG="$ROOT_DIR/targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.usrpb210.n1_422910_pci84.precise.conf"
BIN="$ROOT_DIR/build/nr-toa-uesoftmodem"
TRIAL_SECS="${TRIAL_SECS:-30}"
GAINS=("${@:-58 61 64}")
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$ROOT_DIR/precise_trials_$RUN_TAG}"

mkdir -p "$OUT_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "missing executable: $BIN"
  exit 1
fi

if [[ ! -f "$BASE_CFG" ]]; then
  echo "missing base config: $BASE_CFG"
  exit 1
fi

echo "Precise PCI84 trials"
echo "base_cfg: $BASE_CFG"
echo "out_dir: $OUT_DIR"
echo "trial_secs: $TRIAL_SECS"
echo

for gain in "${GAINS[@]}"; do
  cfg="$OUT_DIR/pci84_gain${gain}.conf"
  log="$OUT_DIR/pci84_gain${gain}.log"
  iq_dir="$OUT_DIR/pci84_gain${gain}_iq"
  mkdir -p "$iq_dir"

  awk -v gain="$gain" '
    /^rx_gain_db[[:space:]]*=/ { print "rx_gain_db = " gain; next }
    /^gain_sweep_enable[[:space:]]*=/ { print "gain_sweep_enable = 0"; next }
    { print }
  ' "$BASE_CFG" > "$cfg"

  echo "=== gain ${gain} dB ==="
  echo "cfg: $cfg"
  echo "log: $log"
  echo "iq:  $iq_dir"

  timeout "${TRIAL_SECS}s" env NR_TOA_IQ_DUMP_DIR="$iq_dir" \
    "$BIN" "$cfg" | tee "$log" || true

  echo
done
