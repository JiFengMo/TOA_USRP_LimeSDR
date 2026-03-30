#include "nr/mib.h"
#include "common/error.h"

#include <string.h>

int nr_mib_unpack(const nr_pbch_decode_result_t *pbch, nr_mib_t *mib)
{
  if (!pbch || !mib)
    return TOA_ERR_INVALID_ARG;

  memset(mib, 0, sizeof(*mib));
  if (!pbch->crc_ok || pbch->payload_bits < 24)
    return TOA_OK;

  /* Sprint3 scaffold mapping from placeholder payload */
  mib->valid = 1;
  mib->sfn = (uint16_t)(pbch->payload[0] | ((pbch->payload[1] & 0x03) << 8));
  mib->scs_common = (uint8_t)(pbch->payload[2] & 0x01);
  mib->dmrs_type_a_pos = (uint8_t)((pbch->payload[2] >> 1) & 0x01);
  mib->pdcch_cfg_sib1 = (uint8_t)((pbch->payload[2] >> 2) & 0x0f);
  mib->cell_barred = 0;
  mib->intra_freq_reselection = 1;
  return TOA_OK;
}
