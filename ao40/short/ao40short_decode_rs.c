#include "ao40short_decode_rs.h"

#include <stdint.h>
#include <string.h>

#if !defined(AO40SHORT_NROOTS)
#error "AO40SHORT_NROOTS not defined"
#endif

#if !defined(AO40SHORT_NN)
#error "AO40SHORT_NN not defined"
#endif

#if !defined(AO40SHORT_PAD)
#error "AO40SHORT_PAD not defined"
#endif

#if !defined(AO40SHORT_MODNN)
#error "AO40SHORT_MODNN not defined"
#endif

#if !defined(AO40SHORT_FCR)
#error "AO40SHORT_FCR not defined"
#endif

#if !defined(AO40SHORT_PRIM)
#error "AO40SHORT_PRIM not defined"
#endif

#if !defined(AO40SHORT_NULL)
#define AO40SHORT_NULL ((void *)0)
#endif

#undef AO40SHORT_MIN
#define AO40SHORT_MIN(a,b) ((a) < (b) ? (a) : (b))
#undef AO40SHORT_A0
#define AO40SHORT_A0 (AO40SHORT_NN)

const uint8_t AO40SHORT_INDEX_OF[] = {
   0xff, 0x00, 0x01, 0x63, 0x02, 0xc6, 0x64, 0x6a, 0x03, 0xcd, 0xc7, 0xbc, 0x65, 0x7e, 0x6b, 0x2a,
   0x04, 0x8d, 0xce, 0x4e, 0xc8, 0xd4, 0xbd, 0xe1, 0x66, 0xdd, 0x7f, 0x31, 0x6c, 0x20, 0x2b, 0xf3,
   0x05, 0x57, 0x8e, 0xe8, 0xcf, 0xac, 0x4f, 0x83, 0xc9, 0xd9, 0xd5, 0x41, 0xbe, 0x94, 0xe2, 0xb4,
   0x67, 0x27, 0xde, 0xf0, 0x80, 0xb1, 0x32, 0x35, 0x6d, 0x45, 0x21, 0x12, 0x2c, 0x0d, 0xf4, 0x38,
   0x06, 0x9b, 0x58, 0x1a, 0x8f, 0x79, 0xe9, 0x70, 0xd0, 0xc2, 0xad, 0xa8, 0x50, 0x75, 0x84, 0x48,
   0xca, 0xfc, 0xda, 0x8a, 0xd6, 0x54, 0x42, 0x24, 0xbf, 0x98, 0x95, 0xf9, 0xe3, 0x5e, 0xb5, 0x15,
   0x68, 0x61, 0x28, 0xba, 0xdf, 0x4c, 0xf1, 0x2f, 0x81, 0xe6, 0xb2, 0x3f, 0x33, 0xee, 0x36, 0x10,
   0x6e, 0x18, 0x46, 0xa6, 0x22, 0x88, 0x13, 0xf7, 0x2d, 0xb8, 0x0e, 0x3d, 0xf5, 0xa4, 0x39, 0x3b,
   0x07, 0x9e, 0x9c, 0x9d, 0x59, 0x9f, 0x1b, 0x08, 0x90, 0x09, 0x7a, 0x1c, 0xea, 0xa0, 0x71, 0x5a,
   0xd1, 0x1d, 0xc3, 0x7b, 0xae, 0x0a, 0xa9, 0x91, 0x51, 0x5b, 0x76, 0x72, 0x85, 0xa1, 0x49, 0xeb,
   0xcb, 0x7c, 0xfd, 0xc4, 0xdb, 0x1e, 0x8b, 0xd2, 0xd7, 0x92, 0x55, 0xaa, 0x43, 0x0b, 0x25, 0xaf,
   0xc0, 0x73, 0x99, 0x77, 0x96, 0x5c, 0xfa, 0x52, 0xe4, 0xec, 0x5f, 0x4a, 0xb6, 0xa2, 0x16, 0x86,
   0x69, 0xc5, 0x62, 0xfe, 0x29, 0x7d, 0xbb, 0xcc, 0xe0, 0xd3, 0x4d, 0x8c, 0xf2, 0x1f, 0x30, 0xdc,
   0x82, 0xab, 0xe7, 0x56, 0xb3, 0x93, 0x40, 0xd8, 0x34, 0xb0, 0xef, 0x26, 0x37, 0x0c, 0x11, 0x44,
   0x6f, 0x78, 0x19, 0x9a, 0x47, 0x74, 0xa7, 0xc1, 0x23, 0x53, 0x89, 0xfb, 0x14, 0x5d, 0xf8, 0x97,
   0x2e, 0x4b, 0xb9, 0x60, 0x0f, 0xed, 0x3e, 0xe5, 0xf6, 0x87, 0xa5, 0x17, 0x3a, 0xa3, 0x3c, 0xb7,
};

const uint8_t AO40SHORT_ALPHA_TO[] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x87, 0x89, 0x95, 0xad, 0xdd, 0x3d, 0x7a, 0xf4,
    0x6f, 0xde, 0x3b, 0x76, 0xec, 0x5f, 0xbe, 0xfb, 0x71, 0xe2, 0x43, 0x86, 0x8b, 0x91, 0xa5, 0xcd,
    0x1d, 0x3a, 0x74, 0xe8, 0x57, 0xae, 0xdb, 0x31, 0x62, 0xc4, 0x0f, 0x1e, 0x3c, 0x78, 0xf0, 0x67,
    0xce, 0x1b, 0x36, 0x6c, 0xd8, 0x37, 0x6e, 0xdc, 0x3f, 0x7e, 0xfc, 0x7f, 0xfe, 0x7b, 0xf6, 0x6b,
    0xd6, 0x2b, 0x56, 0xac, 0xdf, 0x39, 0x72, 0xe4, 0x4f, 0x9e, 0xbb, 0xf1, 0x65, 0xca, 0x13, 0x26,
    0x4c, 0x98, 0xb7, 0xe9, 0x55, 0xaa, 0xd3, 0x21, 0x42, 0x84, 0x8f, 0x99, 0xb5, 0xed, 0x5d, 0xba,
    0xf3, 0x61, 0xc2, 0x03, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x07, 0x0e, 0x1c, 0x38, 0x70, 0xe0,
    0x47, 0x8e, 0x9b, 0xb1, 0xe5, 0x4d, 0x9a, 0xb3, 0xe1, 0x45, 0x8a, 0x93, 0xa1, 0xc5, 0x0d, 0x1a,
    0x34, 0x68, 0xd0, 0x27, 0x4e, 0x9c, 0xbf, 0xf9, 0x75, 0xea, 0x53, 0xa6, 0xcb, 0x11, 0x22, 0x44,
    0x88, 0x97, 0xa9, 0xd5, 0x2d, 0x5a, 0xb4, 0xef, 0x59, 0xb2, 0xe3, 0x41, 0x82, 0x83, 0x81, 0x85,
    0x8d, 0x9d, 0xbd, 0xfd, 0x7d, 0xfa, 0x73, 0xe6, 0x4b, 0x96, 0xab, 0xd1, 0x25, 0x4a, 0x94, 0xaf,
    0xd9, 0x35, 0x6a, 0xd4, 0x2f, 0x5e, 0xbc, 0xff, 0x79, 0xf2, 0x63, 0xc6, 0x0b, 0x16, 0x2c, 0x58,
    0xb0, 0xe7, 0x49, 0x92, 0xa3, 0xc1, 0x05, 0x0a, 0x14, 0x28, 0x50, 0xa0, 0xc7, 0x09, 0x12, 0x24,
    0x48, 0x90, 0xa7, 0xc9, 0x15, 0x2a, 0x54, 0xa8, 0xd7, 0x29, 0x52, 0xa4, 0xcf, 0x19, 0x32, 0x64,
    0xc8, 0x17, 0x2e, 0x5c, 0xb8, 0xf7, 0x69, 0xd2, 0x23, 0x46, 0x8c, 0x9f, 0xb9, 0xf5, 0x6d, 0xda,
    0x33, 0x66, 0xcc, 0x1f, 0x3e, 0x7c, 0xf8, 0x77, 0xee, 0x5b, 0xb6, 0xeb, 0x51, 0xa2, 0xc3, 0x00,
};

int8_t ao40short_decode_rs_8(uint8_t *data, int *eras_pos, int no_eras) {
  int deg_lambda, el, deg_omega;
  int i, j, r,k;
  uint8_t u,q,tmp,num1,num2,den,discr_r;
  uint8_t lambda[AO40SHORT_NROOTS+1], s[AO40SHORT_NROOTS];        /* Err+Eras Locator poly and syndrome poly */
  uint8_t b[AO40SHORT_NROOTS+1], t[AO40SHORT_NROOTS+1], omega[AO40SHORT_NROOTS+1];
  uint8_t root[AO40SHORT_NROOTS], reg[AO40SHORT_NROOTS+1], loc[AO40SHORT_NROOTS];
  int syn_error, count;

  /* form the syndromes; i.e., evaluate data(x) at roots of g(x) */
  for (i=0;i<AO40SHORT_NROOTS;i++)
    s[i] = data[0];

  for (j=1;j<AO40SHORT_NN-AO40SHORT_PAD;j++) {
    for (i=0;i<AO40SHORT_NROOTS;i++) {
      if (s[i] == 0) {
        s[i] = data[j];
      } else {
        s[i] = data[j] ^ AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(AO40SHORT_INDEX_OF[s[i]] + (AO40SHORT_FCR+i)*AO40SHORT_PRIM)];
      }
    }
  }

  /* Convert syndromes to index form, checking for nonzero condition */
  syn_error = 0;
  for (i=0;i<AO40SHORT_NROOTS;i++) {
    syn_error |= s[i];
    s[i] = AO40SHORT_INDEX_OF[s[i]];
  }

  if (!syn_error) {
    /* if syndrome is zero, data[] is a codeword and there are no
     * errors to correct. So return data[] unmodified
     */
    count = 0;
    goto finish;
  }
  memset(&lambda[1],0,AO40SHORT_NROOTS*sizeof(lambda[0]));
  lambda[0] = 1;

  if (no_eras > 0) {
    /* Init lambda to be the erasure locator polynomial */
    lambda[1] = AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(AO40SHORT_PRIM*(AO40SHORT_NN-1-eras_pos[0]))];
    for (i = 1; i < no_eras; i++) {
      u = AO40SHORT_MODNN(AO40SHORT_PRIM*(AO40SHORT_NN-1-eras_pos[i]));
      for (j = i+1; j > 0; j--) {
        tmp = AO40SHORT_INDEX_OF[lambda[j - 1]];
        if (tmp != AO40SHORT_A0)
          lambda[j] ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(u + tmp)];
      }
    }

#if DEBUG >= 1
    /* Test code that verifies the erasure locator polynomial just constructed
       Needed only for decoder debugging. */

    /* find roots of the erasure location polynomial */
    for (i=1;i<=no_eras;i++)
      reg[i] = AO40SHORT_INDEX_OF[lambda[i]];

    count = 0;
    for (i = 1,k=AO40SHORT_IPRIM-1; i <= AO40SHORT_NN; i++,k = AO40SHORT_MODNN(k+AO40SHORT_IPRIM)) {
      q = 1;
      for (j = 1; j <= no_eras; j++)
        if (reg[j] != AO40SHORT_A0) {
          reg[j] = AO40SHORT_MODNN(reg[j] + j);
          q ^= AO40SHORT_ALPHA_TO[reg[j]];
        }
      if (q != 0)
        continue;
      /* store root and error location number indices */
      root[count] = i;
      loc[count] = k;
      count++;
    }
    if (count != no_eras) {
      printf("count = %d no_eras = %d\n lambda(x) is WRONG\n",count,no_eras);
      count = -1;
      goto finish;
    }
#if DEBUG >= 2
    printf("\n Erasure positions as determined by roots of Eras Loc Poly:\n");
    for (i = 0; i < count; i++)
      printf("%d ", loc[i]);
    printf("\n");
#endif
#endif
  }
  for (i=0;i<AO40SHORT_NROOTS+1;i++)
    b[i] = AO40SHORT_INDEX_OF[lambda[i]];

  /*
   * Begin Berlekamp-Massey algorithm to determine error+erasure
   * locator polynomial
   */
  r = no_eras;
  el = no_eras;
  while (++r <= AO40SHORT_NROOTS) {        /* r is the step number */
    /* Compute discrepancy at the r-th step in poly-form */
    discr_r = 0;
    for (i = 0; i < r; i++) {
      if ((lambda[i] != 0) && (s[r-i-1] != AO40SHORT_A0)) {
        discr_r ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(AO40SHORT_INDEX_OF[lambda[i]] + s[r-i-1])];
      }
    }
    discr_r = AO40SHORT_INDEX_OF[discr_r];        /* Index form */
    if (discr_r == AO40SHORT_A0) {
      /* 2 lines below: B(x) <-- x*B(x) */
      memmove(&b[1],b,AO40SHORT_NROOTS*sizeof(b[0]));
      b[0] = AO40SHORT_A0;
    } else {
      /* 7 lines below: T(x) <-- lambda(x) - discr_r*x*b(x) */
      t[0] = lambda[0];
      for (i = 0 ; i < AO40SHORT_NROOTS; i++) {
        if (b[i] != AO40SHORT_A0)
          t[i+1] = lambda[i+1] ^ AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(discr_r + b[i])];
        else
          t[i+1] = lambda[i+1];
      }
      if (2 * el <= r + no_eras - 1) {
        el = r + no_eras - el;
        /*
         * 2 lines below: B(x) <-- inv(discr_r) *
         * lambda(x)
         */
        for (i = 0; i <= AO40SHORT_NROOTS; i++)
          b[i] = (lambda[i] == 0) ? AO40SHORT_A0 : AO40SHORT_MODNN(AO40SHORT_INDEX_OF[lambda[i]] - discr_r + AO40SHORT_NN);
      } else {
        /* 2 lines below: B(x) <-- x*B(x) */
        memmove(&b[1],b,AO40SHORT_NROOTS*sizeof(b[0]));
        b[0] = AO40SHORT_A0;
      }
      memcpy(lambda,t,(AO40SHORT_NROOTS+1)*sizeof(t[0]));
    }
  }

  /* Convert lambda to index form and compute deg(lambda(x)) */
  deg_lambda = 0;
  for (i=0;i<AO40SHORT_NROOTS+1;i++) {
    lambda[i] = AO40SHORT_INDEX_OF[lambda[i]];
    if (lambda[i] != AO40SHORT_A0)
      deg_lambda = i;
  }
  /* Find roots of the error+erasure locator polynomial by Chien search */
  memcpy(&reg[1],&lambda[1],AO40SHORT_NROOTS*sizeof(reg[0]));
  count = 0;                /* Number of roots of lambda(x) */
  for (i = 1,k=AO40SHORT_IPRIM-1; i <= AO40SHORT_NN; i++,k = AO40SHORT_MODNN(k+AO40SHORT_IPRIM)) {
    q = 1; /* lambda[0] is always 0 */
    for (j = deg_lambda; j > 0; j--) {
      if (reg[j] != AO40SHORT_A0) {
        reg[j] = AO40SHORT_MODNN(reg[j] + j);
        q ^= AO40SHORT_ALPHA_TO[reg[j]];
      }
    }
    if (q != 0)
      continue; /* Not a root */
    /* store root (index-form) and error location number */
#if DEBUG>=2
    printf("count %d root %d loc %d\n",count,i,k);
#endif
    root[count] = i;
    loc[count] = k;
    /* If we've already found max possible roots,
     * abort the search to save time
     */
    if (++count == deg_lambda)
      break;
  }
  if (deg_lambda != count) {
    /*
     * deg(lambda) unequal to number of roots => uncorrectable
     * error detected
     */
#if (DEBUG >= 1)
    printf("deg_lambda != count \n");
#endif
    count = -1;
    goto finish;
  }
  /*
   * Compute err+eras evaluator poly omega(x) = s(x)*lambda(x) (modulo
   * x**AO40SHORT_NROOTS). in index form. Also find deg(omega).
   */
  deg_omega = deg_lambda-1;
  for (i = 0; i <= deg_omega;i++) {
    tmp = 0;
    for (j=i;j >= 0; j--) {
      if ((s[i - j] != AO40SHORT_A0) && (lambda[j] != AO40SHORT_A0))
        tmp ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(s[i - j] + lambda[j])];
    }
    omega[i] = AO40SHORT_INDEX_OF[tmp];
  }

  /*
   * Compute error values in poly-form. num1 = omega(inv(X(l))), num2 =
   * inv(X(l))**(AO40SHORT_FCR-1) and den = lambda_pr(inv(X(l))) all in poly-form
   */
  for (j = count-1; j >=0; j--) {
    num1 = 0;
    for (i = deg_omega; i >= 0; i--) {
      if (omega[i] != AO40SHORT_A0)
        num1  ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(omega[i] + i * root[j])];
    }
    num2 = AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(root[j] * (AO40SHORT_FCR - 1) + AO40SHORT_NN)];
    den = 0;

    /* lambda[i+1] for i even is the formal derivative lambda_pr of lambda[i] */
    for (i = AO40SHORT_MIN(deg_lambda,AO40SHORT_NROOTS-1) & ~1; i >= 0; i -=2) {
      if (lambda[i+1] != AO40SHORT_A0)
        den ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(lambda[i+1] + i * root[j])];
    }
#if DEBUG >= 1
    if (den == 0) {
      printf("\n ERROR: denominator = 0\n");
      count = -1;
      goto finish;
    }
#endif
    /* Apply error to data */
    if (num1 != 0 && loc[j] >= AO40SHORT_PAD) {
      data[loc[j]-AO40SHORT_PAD] ^= AO40SHORT_ALPHA_TO[AO40SHORT_MODNN(AO40SHORT_INDEX_OF[num1] + AO40SHORT_INDEX_OF[num2] + AO40SHORT_NN - AO40SHORT_INDEX_OF[den])];
    }
  }

finish:
  if (eras_pos != AO40SHORT_NULL) {
    for (i=0;i<count;i++)
      eras_pos[i] = loc[i];
  }

  return count;
}
