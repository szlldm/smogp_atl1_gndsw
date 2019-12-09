#ifndef AO40_DEC_REF_H
#define AO40_DEC_REF_H

#include <stdint.h>
#include "ao40_spiral-vit_scalar.h"
#include "ao40_decode_rs.h"

#define AO40_DEBUG

#define AO40_RAW_SIZE      5200
#define AO40_CONV_SIZE     5132

#define AO40_RS_SIZE        320
#define AO40_DATA_SIZE      256
#define AO40_CODE_LENGTH    650
#define AO40_FRAME_BITS    2560
#define AO40_RS_BLOCK_SIZE  160

#if !defined(AO40_NULL)
#define AO40_NULL ((void *)0)
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern const uint8_t ao40_Scrambler[320];

void ao40_decode_data(uint8_t raw[AO40_RAW_SIZE], uint8_t data[AO40_DATA_SIZE], int8_t error[2]);

#ifdef AO40_DEBUG
void ao40_decode_data_debug(uint8_t raw[AO40_RAW_SIZE], uint8_t data[AO40_DATA_SIZE], int8_t  error[2], uint8_t conv[AO40_CONV_SIZE], uint8_t dec_data[AO40_RS_SIZE], uint8_t rs[2][AO40_RS_BLOCK_SIZE]);
#endif

#ifdef __cplusplus
}
#endif // __cplusplus

#endif
