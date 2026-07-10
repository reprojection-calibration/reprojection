#include "tagCustom36h11.h"

#include <stdlib.h>

static uint64_t codedata[106] = {
    0x0000000d2168aaebUL, 0x00000002d9603b28UL, 0x000000014292a6a3UL, 0x00000005292d9c96UL, 0x0000000ae1252cd3UL,
    0x0000000061a94b29UL, 0x000000016af928e9UL, 0x0000000b0d2436acUL, 0x0000000b5a58b3a5UL, 0x00000008d94f080cUL,
    0x00000005493b4aacUL, 0x000000096a654664UL, 0x0000000a551e512dUL, 0x00000004a0bf4883UL, 0x000000059b0236a2UL,
    0x0000000698ddac54UL, 0x0000000a5aaaa794UL, 0x0000000034d1925bUL, 0x000000082436d8cbUL, 0x0000000a556d6742UL,
    0x000000047270b496UL, 0x00000001263eb895UL, 0x0000000c6b28b6a0UL, 0x0000000122f54717UL, 0x00000000e2ac0d4bUL,
    0x00000000552e5686UL, 0x00000004392aac50UL, 0x0000000392d0569bUL, 0x0000000c9a926a6cUL, 0x0000000701aea0dbUL,
    0x000000005d594d16UL, 0x0000000698d10729UL, 0x00000006316a4896UL, 0x000000052232592fUL, 0x0000000aeaadb42cUL,
    0x00000002994ccd1bUL, 0x0000000434acd09dUL, 0x0000000325a2bed6UL, 0x0000000583554adbUL, 0x00000005087d0b4aUL,
    0x000000057d32b669UL, 0x0000000aa1756825UL, 0x00000008f2482865UL, 0x00000006994943d2UL, 0x0000000220a6163cUL,
    0x00000000a0659693UL, 0x000000050d6e6c9aUL, 0x0000000362adac56UL, 0x00000002dbcac6acUL, 0x0000000421ec8f1aUL,
    0x00000002d2994387UL, 0x0000000cbb59461cUL, 0x0000000ac876a2a4UL, 0x000000012af29728UL, 0x00000005a41aa692UL,
    0x0000000ab292df24UL, 0x00000002f8694ae4UL, 0x0000000e22d52853UL, 0x000000025e0b7624UL, 0x00000004e2a7724bUL,
    0x000000074970b726UL, 0x0000000a5c69872dUL, 0x00000005a26ea517UL, 0x00000005ab4c923cUL, 0x000000056a194d44UL,
    0x000000025da8a949UL, 0x0000000cb2dd4d2aUL, 0x0000000bca135a29UL, 0x0000000258a69125UL, 0x000000070ae54b3cUL,
    0x0000000ae2b4b0c1UL, 0x00000009269476d6UL, 0x000000035d69ca99UL, 0x000000095dd14268UL, 0x000000034b0eb64aUL,
    0x00000001a16d297cUL, 0x00000008da2a1d24UL, 0x0000000165213635UL, 0x0000000b24b7091bUL, 0x0000000964eea84aUL,
    0x00000005625e8169UL, 0x000000068a28b493UL, 0x00000004b9a0b795UL, 0x00000004aa380cb4UL, 0x00000004e154b752UL,
    0x000000051d68c324UL, 0x00000006921d4971UL, 0x000000009b91533eUL, 0x0000000b41ad94a3UL, 0x00000002da974564UL,
    0x0000000434b53d43UL, 0x0000000d1e92d32bUL, 0x00000003d8b9493dUL, 0x0000000562743a8bUL, 0x00000001525d6453UL,
    0x0000000499198a59UL, 0x0000000454653ca5UL, 0x00000002c56c885eUL, 0x000000052176c694UL, 0x00000005a4e56a94UL,
    0x0000000b4db63869UL, 0x00000004da4d05a6UL, 0x000000016b64d829UL, 0x0000000322795893UL, 0x00000000b34a875eUL,
    0x0000000e5daab22cUL,
};
apriltag_family_t* tagCustom36h11_create() {
    apriltag_family_t* tf = calloc(1, sizeof(apriltag_family_t));
    tf->name = strdup("tagCustom36h11");
    tf->h = 11;
    tf->ncodes = 106;
    tf->codes = codedata;
    tf->nbits = 36;
    tf->bit_x = calloc(36, sizeof(uint32_t));
    tf->bit_y = calloc(36, sizeof(uint32_t));
    tf->bit_x[0] = 1;
    tf->bit_y[0] = 1;
    tf->bit_x[1] = 2;
    tf->bit_y[1] = 1;
    tf->bit_x[2] = 3;
    tf->bit_y[2] = 1;
    tf->bit_x[3] = 4;
    tf->bit_y[3] = 1;
    tf->bit_x[4] = 5;
    tf->bit_y[4] = 1;
    tf->bit_x[5] = 2;
    tf->bit_y[5] = 2;
    tf->bit_x[6] = 3;
    tf->bit_y[6] = 2;
    tf->bit_x[7] = 4;
    tf->bit_y[7] = 2;
    tf->bit_x[8] = 3;
    tf->bit_y[8] = 3;
    tf->bit_x[9] = 6;
    tf->bit_y[9] = 1;
    tf->bit_x[10] = 6;
    tf->bit_y[10] = 2;
    tf->bit_x[11] = 6;
    tf->bit_y[11] = 3;
    tf->bit_x[12] = 6;
    tf->bit_y[12] = 4;
    tf->bit_x[13] = 6;
    tf->bit_y[13] = 5;
    tf->bit_x[14] = 5;
    tf->bit_y[14] = 2;
    tf->bit_x[15] = 5;
    tf->bit_y[15] = 3;
    tf->bit_x[16] = 5;
    tf->bit_y[16] = 4;
    tf->bit_x[17] = 4;
    tf->bit_y[17] = 3;
    tf->bit_x[18] = 6;
    tf->bit_y[18] = 6;
    tf->bit_x[19] = 5;
    tf->bit_y[19] = 6;
    tf->bit_x[20] = 4;
    tf->bit_y[20] = 6;
    tf->bit_x[21] = 3;
    tf->bit_y[21] = 6;
    tf->bit_x[22] = 2;
    tf->bit_y[22] = 6;
    tf->bit_x[23] = 5;
    tf->bit_y[23] = 5;
    tf->bit_x[24] = 4;
    tf->bit_y[24] = 5;
    tf->bit_x[25] = 3;
    tf->bit_y[25] = 5;
    tf->bit_x[26] = 4;
    tf->bit_y[26] = 4;
    tf->bit_x[27] = 1;
    tf->bit_y[27] = 6;
    tf->bit_x[28] = 1;
    tf->bit_y[28] = 5;
    tf->bit_x[29] = 1;
    tf->bit_y[29] = 4;
    tf->bit_x[30] = 1;
    tf->bit_y[30] = 3;
    tf->bit_x[31] = 1;
    tf->bit_y[31] = 2;
    tf->bit_x[32] = 2;
    tf->bit_y[32] = 5;
    tf->bit_x[33] = 2;
    tf->bit_y[33] = 4;
    tf->bit_x[34] = 2;
    tf->bit_y[34] = 3;
    tf->bit_x[35] = 3;
    tf->bit_y[35] = 4;
    tf->width_at_border = 8;
    tf->total_width = 16;
    tf->reversed_border = true;
    return tf;
}

void tagCustom36h11_destroy(apriltag_family_t* tf) {
    free(tf->bit_x);
    free(tf->bit_y);
    free(tf->name);
    free(tf);
}
