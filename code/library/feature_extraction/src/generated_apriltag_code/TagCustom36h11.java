package april.tag;

/** Tag family with 106 distinct codes.
    bits: 36,  minimum hamming: 11

    Max bits corrected       False positive rate
            0                  0.00000015 %
            1                  0.00000571 %
            2                  0.00010288 %
            3                  0.00120423 %
            4                  0.01029035 %
            5                  0.06844148 %

    Generation time: 1800.155000 s

    Hamming distance between pairs of codes (accounting for rotation):

       0  0
       1  0
       2  0
       3  0
       4  0
       5  0
       6  0
       7  0
       8  0
       9  0
      10  0
      11  585
      12  875
      13  853
      14  977
      15  848
      16  647
      17  376
      18  207
      19  112
      20  46
      21  31
      22  4
      23  3
      24  1
      25  0
      26  0
      27  0
      28  0
      29  0
      30  0
      31  0
      32  0
      33  0
      34  0
      35  0
      36  0
**/
public class TagCustom36h11 extends TagFamily
{
        private static class ConstructCodes0 {
                private static long[] constructCodes() {
                        return new long[] { 0xd2168aaebL, 0x2d9603b28L, 0x14292a6a3L, 0x5292d9c96L, 0xae1252cd3L, 0x061a94b29L, 0x16af928e9L, 0xb0d2436acL, 0xb5a58b3a5L, 0x8d94f080cL, 0x5493b4aacL, 0x96a654664L, 0xa551e512dL, 0x4a0bf4883L, 0x59b0236a2L, 0x698ddac54L, 0xa5aaaa794L, 0x034d1925bL, 0x82436d8cbL, 0xa556d6742L, 0x47270b496L, 0x1263eb895L, 0xc6b28b6a0L, 0x122f54717L, 0x0e2ac0d4bL, 0x0552e5686L, 0x4392aac50L, 0x392d0569bL, 0xc9a926a6cL, 0x701aea0dbL, 0x05d594d16L, 0x698d10729L, 0x6316a4896L, 0x52232592fL, 0xaeaadb42cL, 0x2994ccd1bL, 0x434acd09dL, 0x325a2bed6L, 0x583554adbL, 0x5087d0b4aL, 0x57d32b669L, 0xaa1756825L, 0x8f2482865L, 0x6994943d2L, 0x220a6163cL, 0x0a0659693L, 0x50d6e6c9aL, 0x362adac56L, 0x2dbcac6acL, 0x421ec8f1aL, 0x2d2994387L, 0xcbb59461cL, 0xac876a2a4L, 0x12af29728L, 0x5a41aa692L, 0xab292df24L, 0x2f8694ae4L, 0xe22d52853L, 0x25e0b7624L, 0x4e2a7724bL, 0x74970b726L, 0xa5c69872dL, 0x5a26ea517L, 0x5ab4c923cL, 0x56a194d44L, 0x25da8a949L, 0xcb2dd4d2aL, 0xbca135a29L, 0x258a69125L, 0x70ae54b3cL, 0xae2b4b0c1L, 0x9269476d6L, 0x35d69ca99L, 0x95dd14268L, 0x34b0eb64aL, 0x1a16d297cL, 0x8da2a1d24L, 0x165213635L, 0xb24b7091bL, 0x964eea84aL, 0x5625e8169L, 0x68a28b493L, 0x4b9a0b795L, 0x4aa380cb4L, 0x4e154b752L, 0x51d68c324L, 0x6921d4971L, 0x09b91533eL, 0xb41ad94a3L, 0x2da974564L, 0x434b53d43L, 0xd1e92d32bL, 0x3d8b9493dL, 0x562743a8bL, 0x1525d6453L, 0x499198a59L, 0x454653ca5L, 0x2c56c885eL, 0x52176c694L, 0x5a4e56a94L, 0xb4db63869L, 0x4da4d05a6L, 0x16b64d829L, 0x322795893L, 0x0b34a875eL, 0xe5daab22cL };
                }
        }

        private static long[] constructCodes() {
                long[] codes = new long[106];
                System.arraycopy(ConstructCodes0.constructCodes(), 0, codes, 0, 106);
                return codes;
        }

        public TagCustom36h11()
        {
                super(ImageLayout.Factory.createFromString("Custom", "bbwwwwwwwwwwwwbbbbwwwwwwwwwwwwbbwwbbbbbbbbbbbbwwwwbbbbbbbbbbbbwwwwbbwwwwwwwwbbwwwwbbwddddddwbbwwwwbbwddddddwbbwwwwbbwddddddwbbwwwwbbwddddddwbbwwwwbbwddddddwbbwwwwbbwddddddwbbwwwwbbwwwwwwwwbbwwwwbbbbbbbbbbbbwwwwbbbbbbbbbbbbwwbbwwwwwwwwwwwwbbbbwwwwwwwwwwwwbb"), 11, constructCodes());
        }
}