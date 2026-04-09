#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <logfile>"
  exit 1
fi

LOG_PATH="$1"

if [[ ! -f "$LOG_PATH" ]]; then
  echo "missing log: $LOG_PATH"
  exit 1
fi

echo "Relevant lines from $LOG_PATH"
echo
grep -E "SSB_LOCK_EVENT:|sync_result:|pbch_bch: ok|pbch_bch: crc-fail|pbch_cand\\[|pss_hits:|pss_search: not-detected" "$LOG_PATH" || true
echo

if grep -q "SSB_LOCK_EVENT:" "$LOG_PATH"; then
  echo "RESULT: detected SSB and decoded PBCH/MIB successfully."
elif grep -q "pbch_bch: ok" "$LOG_PATH"; then
  echo "RESULT: PBCH decode succeeded in at least one attempt."
elif grep -q "pbch_bch: crc-fail" "$LOG_PATH"; then
  echo "RESULT: SSB candidate was found and PBCH was attempted, but BCH CRC failed."
elif grep -q "pss_hits:" "$LOG_PATH"; then
  echo "RESULT: PSS/SSB candidate was seen, but PBCH did not complete."
else
  echo "RESULT: no usable SSB evidence in this log."
fi

