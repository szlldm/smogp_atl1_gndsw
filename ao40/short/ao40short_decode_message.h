#ifndef AO40SHORT_DEC_REF_H
#define AO40SHORT_DEC_REF_H

#include <stdint.h>
#include "ao40short_spiral-vit_scalar_1280.h"
#include "ao40short_decode_rs.h"

#define AO40SHORT_DEBUG

#define AO40SHORT_INTERLEAVER_STEP_SIZE    51
#define AO40SHORT_INTERLEAVER_PILOT_BITS   80

#define AO40SHORT_RAW_SIZE      2652 // 51*52
#define AO40SHORT_CONV_SIZE     2572

#define AO40SHORT_RS_SIZE        160
#define AO40SHORT_DATA_SIZE      128
#define AO40SHORT_FRAME_BITS    1280
#define AO40SHORT_RS_BLOCK_SIZE  160

#if !defined(AO40SHORT_NULL)
#define AO40SHORT_NULL ((void *)0)
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern const uint8_t ao40short_Scrambler[320];

void ao40short_decode_data(uint8_t raw[AO40SHORT_RAW_SIZE], uint8_t data[AO40SHORT_DATA_SIZE], int8_t *error);

#ifdef AO40SHORT_DEBUG
void ao40short_decode_data_debug(uint8_t raw[AO40SHORT_RAW_SIZE], uint8_t data[AO40SHORT_DATA_SIZE], int8_t *error, uint8_t conv[AO40SHORT_CONV_SIZE], uint8_t dec_data[AO40SHORT_RS_SIZE], uint8_t rs[AO40SHORT_RS_BLOCK_SIZE]);
#endif

#ifdef __cplusplus
}
#endif // __cplusplus

#endif
