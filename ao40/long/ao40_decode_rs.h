#ifndef AO40_DECODE_RS_H
#define AO40_DECODE_RS_H

#include <stdint.h> // because of uint8_t

#define AO40_NN     255  // GF(2^8-1)
#define AO40_NROOTS  32
#define AO40_FCR    112
#define AO40_PRIM    11
#define AO40_IPRIM  116
#define AO40_PAD     95

#include <stdio.h>

extern const uint8_t AO40_ALPHA_TO[];
extern const uint8_t AO40_INDEX_OF[];

#define AO40_MODNN(x) ao40_mod255(x)

static inline int ao40_mod255(int x){
  while (x >= 255) {
    x -= 255;
    x = (x >> 8) + (x & 255);
  }
  return x;
}

int8_t ao40_decode_rs_8(uint8_t *data, int *eras_pots, int no_eras);

#endif