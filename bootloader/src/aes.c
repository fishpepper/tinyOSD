/*
    Copyright 2017 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/ or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#include "main.h"
#include "aes.h"


#include <stdint.h>
#include <string.h>


#define AES_NB              4
#define AES_NK              8
#define AES_NR             14
#define AES_KEYLEN         32
#define AES_KEY_EXP_SIZE  240
#define AES_BLOCKLEN       16

uint8_t *aes_key;
uint8_t aes_iv[AES_BLOCKLEN];

uint8_t aes_roundkey[AES_KEY_EXP_SIZE];

typedef uint8_t aes_state_t[4][4];
static aes_state_t* aes_state;

// aes_sboxes (will be filled during init)
static uint8_t aes_sbox[256];
static uint8_t aes_rsbox[256];

// The round constant word array, aes_rcon[i], contains the values given by
// x to th e power (i-1) being powers of x (x is denoted as {02}) in the field GF(2^8)
static const uint8_t aes_rcon[11] = {
    0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};


#define ROTL8(x, shift) ((uint8_t) ((x) << (shift)) | ((x) >> (8 - (shift))))


static void aes_init_sbox(void) {
    uint8_t p = 1, q = 1;

    // loop invariant p * q == 1 in galois field
    do {
        // multiply p by 2
        p = p ^ (p << 1) ^ (p & 0x80 ? 0x1B : 0);

        // divide q by 2
        q ^= q << 1;
        q ^= q << 2;
        q ^= q << 4;
        q ^= q & 0x80 ? 0x09 : 0;

        // compute affine transformation
        uint8_t xformed = q ^ ROTL8(q, 1)
                            ^ ROTL8(q, 2)
                            ^ ROTL8(q, 3)
                            ^ ROTL8(q, 4);

        // store value
        aes_sbox[p] = xformed ^ 0x63;
    } while (p != 1);

    // zero is a special case since it has no inverse
    aes_sbox[0] = 0x63;
}

static void aes_init_rsbox(void) {
    // use sboix to generate inverse aes_sbox
    for (uint16_t i = 0; i < 256; i++) {
        aes_rsbox[aes_sbox[i]] = i;
    }
}

void aes_init(void) {
    aes_init_sbox();
    aes_init_rsbox();
}

static void aes_key_expansion(void) {
    uint32_t i, k;
    // used for the column/row operations
    uint8_t tempa[4];

    // The first round key is the key itself.
    for (i = 0; i < AES_NK; ++i) {
      aes_roundkey[(i * 4) + 0] = aes_key[(i * 4) + 0];
      aes_roundkey[(i * 4) + 1] = aes_key[(i * 4) + 1];
      aes_roundkey[(i * 4) + 2] = aes_key[(i * 4) + 2];
      aes_roundkey[(i * 4) + 3] = aes_key[(i * 4) + 3];
    }

    // all other round keys are found from the previous round keys
    // i == Nk
    for (; i < AES_NB * (AES_NR + 1); ++i) {
        {
            tempa[0] = aes_roundkey[(i-1) * 4 + 0];
            tempa[1] = aes_roundkey[(i-1) * 4 + 1];
            tempa[2] = aes_roundkey[(i-1) * 4 + 2];
            tempa[3] = aes_roundkey[(i-1) * 4 + 3];
        }
        if (i % AES_NK == 0) {
            // This function shifts the 4 bytes in a word to the left once.
            // [a0,a1,a2,a3] becomes [a1,a2,a3,a0]

            // ROTWORD
            {
                k = tempa[0];
                tempa[0] = tempa[1];
                tempa[1] = tempa[2];
                tempa[2] = tempa[3];
                tempa[3] = k;
            }


            // SUBWORD takes a four-byte input word and
            // applies the S-box to each of the four bytes to produce an output word.
            {
                tempa[0] = aes_sbox[tempa[0]];
                tempa[1] = aes_sbox[tempa[1]];
                tempa[2] = aes_sbox[tempa[2]];
                tempa[3] = aes_sbox[tempa[3]];
            }

            tempa[0] =  tempa[0] ^ aes_rcon[i/AES_NK];
        }

        if (i % AES_NK == 4) {
            {
                tempa[0] = aes_sbox[tempa[0]];
                tempa[1] = aes_sbox[tempa[1]];
                tempa[2] = aes_sbox[tempa[2]];
                tempa[3] = aes_sbox[tempa[3]];
            }
        }


        aes_roundkey[i * 4 + 0] = aes_roundkey[(i - AES_NK) * 4 + 0] ^ tempa[0];
        aes_roundkey[i * 4 + 1] = aes_roundkey[(i - AES_NK) * 4 + 1] ^ tempa[1];
        aes_roundkey[i * 4 + 2] = aes_roundkey[(i - AES_NK) * 4 + 2] ^ tempa[2];
        aes_roundkey[i * 4 + 3] = aes_roundkey[(i - AES_NK) * 4 + 3] ^ tempa[3];
    }
}

static void aes_xor_with_iv(uint8_t *buf) {
    uint8_t i;
    for (i = 0; i < AES_BLOCKLEN; ++i) {
        buf[i] ^= aes_iv[i];
    }
}

// This function adds the round key to state.
// The round key is added to the state by an XOR function.
static void aes_add_round_key(uint8_t round) {
    uint8_t i, j;
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            (*aes_state)[i][j] ^= aes_roundkey[round * AES_NB * 4 + i * AES_NB + j];
        }
    }
}

// The SubBytes Function Substitutes the values in the
// state matrix with values in an S-box.
static void aes_sub_bytes(void) {
    uint8_t i, j;
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            (*aes_state)[j][i] = aes_sbox[(*aes_state)[j][i]];
        }
    }
}

// The SubBytes Function Substitutes the values in the
// state matrix with values in an S-box.
static void aes_inv_sub_byte(void) {
    uint8_t i, j;
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            (*aes_state)[j][i] = aes_rsbox[(*aes_state)[j][i]];
        }
    }
}


// The ShiftRows() function shifts the rows in the state to the left.
// Each row is shifted with different offset.
// Offset = Row number. So the first row is not shifted.
static void aes_shift_rows(void) {
    uint8_t temp;

    // Rotate first row 1 columns to left
    temp               = (*aes_state)[0][1];
    (*aes_state)[0][1] = (*aes_state)[1][1];
    (*aes_state)[1][1] = (*aes_state)[2][1];
    (*aes_state)[2][1] = (*aes_state)[3][1];
    (*aes_state)[3][1] = temp;

    // Rotate second row 2 columns to left
    temp               = (*aes_state)[0][2];
    (*aes_state)[0][2] = (*aes_state)[2][2];
    (*aes_state)[2][2] = temp;

    temp               = (*aes_state)[1][2];
    (*aes_state)[1][2] = (*aes_state)[3][2];
    (*aes_state)[3][2] = temp;

    // Rotate third row 3 columns to left
    temp               = (*aes_state)[0][3];
    (*aes_state)[0][3] = (*aes_state)[3][3];
    (*aes_state)[3][3] = (*aes_state)[2][3];
    (*aes_state)[2][3] = (*aes_state)[1][3];
    (*aes_state)[1][3] = temp;
}

static void aes_inv_shift_rows(void) {
    uint8_t temp;

    // Rotate first row 1 columns to right
    temp = (*aes_state)[3][1];
    (*aes_state)[3][1] = (*aes_state)[2][1];
    (*aes_state)[2][1] = (*aes_state)[1][1];
    (*aes_state)[1][1] = (*aes_state)[0][1];
    (*aes_state)[0][1] = temp;

    // Rotate second row 2 columns to right
    temp = (*aes_state)[0][2];
    (*aes_state)[0][2] = (*aes_state)[2][2];
    (*aes_state)[2][2] = temp;

    temp = (*aes_state)[1][2];
    (*aes_state)[1][2] = (*aes_state)[3][2];
    (*aes_state)[3][2] = temp;

    // Rotate third row 3 columns to right
    temp = (*aes_state)[0][3];
    (*aes_state)[0][3] = (*aes_state)[1][3];
    (*aes_state)[1][3] = (*aes_state)[2][3];
    (*aes_state)[2][3] = (*aes_state)[3][3];
    (*aes_state)[3][3] = temp;
}




#define AES_XTIME(__x) (((__x) << 1) ^ ((((__x) >> 7) & 1) * 0x1b))

// MixColumns function mixes the columns of the state matrix
static void aes_mix_columns(void) {
    uint8_t i;
    uint8_t Tmp, Tm, t;
    for (i = 0; i < 4; ++i) {
        t   = (*aes_state)[i][0];
        Tmp = (*aes_state)[i][0] ^ (*aes_state)[i][1] ^ (*aes_state)[i][2] ^ (*aes_state)[i][3];
        Tm  = (*aes_state)[i][0] ^ (*aes_state)[i][1]; Tm = AES_XTIME(Tm);  (*aes_state)[i][0] ^= Tm ^ Tmp;
        Tm  = (*aes_state)[i][1] ^ (*aes_state)[i][2]; Tm = AES_XTIME(Tm);  (*aes_state)[i][1] ^= Tm ^ Tmp;
        Tm  = (*aes_state)[i][2] ^ (*aes_state)[i][3]; Tm = AES_XTIME(Tm);  (*aes_state)[i][2] ^= Tm ^ Tmp;
        Tm  = (*aes_state)[i][3] ^ t;                  Tm = AES_XTIME(Tm);  (*aes_state)[i][3] ^= Tm ^ Tmp;
    }
}


// Multiply is used to multiply numbers in the field GF(2^8)
static uint8_t aes_multiply(uint8_t x, uint8_t y) {
    return (((y & 1) * x) ^
       ((y>>1 & 1) * AES_XTIME(x)) ^
       ((y>>2 & 1) * AES_XTIME(AES_XTIME(x))) ^
       ((y>>3 & 1) * AES_XTIME(AES_XTIME(AES_XTIME(x)))) ^
       ((y>>4 & 1) * AES_XTIME(AES_XTIME(AES_XTIME(AES_XTIME(x))))));
}
/*
#define aes_multiply(x, y)                                \
      (  ((y & 1) * x) ^                              \
      ((y>>1 & 1) * AES_XTIME(x)) ^                       \
      ((y>>2 & 1) * AES_XTIME(AES_XTIME(x))) ^                \
      ((y>>3 & 1) * AES_XTIME(AES_XTIME(AES_XTIME(x)))) ^         \
      ((y>>4 & 1) * AES_XTIME(AES_XTIME(AES_XTIME(AES_XTIME(x))))))   \
*/

// MixColumns function mixes the columns of the state matrix.
// The method used to multiply may be difficult to understand for the inexperienced.
// Please use the references to gain more information.
static void aes_inv_mix_columns(void) {
    int i;
    uint8_t a, b, c, d;
    for (i = 0; i < 4; ++i) {
        a = (*aes_state)[i][0];
        b = (*aes_state)[i][1];
        c = (*aes_state)[i][2];
        d = (*aes_state)[i][3];

        (*aes_state)[i][0] = aes_multiply(a, 0x0e) ^
                             aes_multiply(b, 0x0b) ^
                             aes_multiply(c, 0x0d) ^
                             aes_multiply(d, 0x09);
        (*aes_state)[i][1] = aes_multiply(a, 0x09) ^
                             aes_multiply(b, 0x0e) ^
                             aes_multiply(c, 0x0b) ^
                             aes_multiply(d, 0x0d);
        (*aes_state)[i][2] = aes_multiply(a, 0x0d) ^
                             aes_multiply(b, 0x09) ^
                             aes_multiply(c, 0x0e) ^
                             aes_multiply(d, 0x0b);
        (*aes_state)[i][3] = aes_multiply(a, 0x0b) ^
                             aes_multiply(b, 0x0d) ^
                             aes_multiply(c, 0x09) ^
                             aes_multiply(d, 0x0e);
    }
}


static void aes_cipher(void) {
    uint8_t round = 0;

    // add the First round key to the state before starting the rounds.
    aes_add_round_key(0);

    // There will be Nr rounds.
    // The first Nr-1 rounds are identical.
    // These Nr-1 rounds are executed in the loop below.
    for (round = 1; round < AES_NR; ++round) {
        aes_sub_bytes();
        aes_shift_rows();
        aes_mix_columns();
        aes_add_round_key(round);
    }

    // The last round is given below.
    // The MixColumns function is not here in the last round.
    aes_sub_bytes();
    aes_shift_rows();

    aes_add_round_key(AES_NR);
}

static void aes_inv_cipher(void) {
    uint8_t round = 0;

    // Add the First round key to the state before starting the rounds.
    aes_add_round_key(AES_NR);

    // There will be Nr rounds.
    // The first Nr-1 rounds are identical.
    // These Nr-1 rounds are executed in the loop below.
    for (round = (AES_NR - 1); round > 0; --round) {
        aes_inv_shift_rows();
        aes_inv_sub_byte();
        aes_add_round_key(round);
        aes_inv_mix_columns();
    }

    // The last round is given below.
    // The MixColumns function is not here in the last round.
    aes_inv_shift_rows();
    aes_inv_sub_byte();
    aes_add_round_key(0);
}

// 603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4


void aes_cbc_encrypt(uint8_t* output, uint8_t* input, uint32_t length, const uint8_t* key, const uint8_t* iv) {
    uintptr_t i;
    uint8_t extra = length % AES_BLOCKLEN;  // remaining bytes in the last non-full block

    // skip the key expansion if key is passed as 0
    if (0 != key) {
        aes_key = (uint8_t *)key;
        aes_key_expansion();
    }

    // If iv is passed as 0, we continue to encrypt without re-setting the Iv
    if (iv != 0) {
        for (int j = 0; j < AES_BLOCKLEN; j++) {
            aes_iv[j] = *(iv + j);
        }
    }

    for (i = 0; i < length; i += AES_BLOCKLEN) {
        aes_xor_with_iv(input);

        memcpy(output, input, AES_BLOCKLEN);

        aes_state = (aes_state_t*)output;

        aes_cipher();

        for (int j = 0; j < AES_BLOCKLEN; j++) {
            aes_iv[j] = *(output + j);
        }

        input  += AES_BLOCKLEN;
        output += AES_BLOCKLEN;
        // printf("Step %d - %d", i/16, i);
    }

    if (extra) {
        memcpy(output, input, extra);
        aes_state = (aes_state_t*)output;
        aes_cipher();
    }
}


void aes_cbc_decrypt(uint8_t* output, uint8_t* input, uint32_t length, const uint8_t* key, const uint8_t* iv) {
    uintptr_t i;
    uint8_t extra = length % AES_BLOCKLEN; /* Remaining bytes in the last non-full block */

    // skip the key expansion if key is passed as 0
    if (0 != key) {
        aes_key = key;
        aes_key_expansion();
    }

    // If iv is passed as 0, we continue to encrypt without re-setting the Iv
    if (iv != 0) {
        for (int j = 0; j < AES_BLOCKLEN; j++) {
            aes_iv[j] = *(iv + j);
        }
    }

    for (i = 0; i < length; i += AES_BLOCKLEN) {
        memcpy(output, input, AES_BLOCKLEN);
        aes_state = (aes_state_t*)output;
        aes_inv_cipher();
        aes_xor_with_iv(output);
        for (int j = 0; j < AES_BLOCKLEN; j++) {
            aes_iv[j] = *(input + j);
        }
        input  += AES_BLOCKLEN;
        output += AES_BLOCKLEN;
    }

    if (extra) {
        memcpy(output, input, extra);
        aes_state = (aes_state_t*)output;
        aes_inv_cipher();
    }
}
