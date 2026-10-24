#include "Ngs2Test.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct QueueCase {
    const char* source;
    std::uint32_t rack;
    std::uint32_t rate;
    std::uint32_t systemRate;
    std::uint32_t grain;
    std::uint32_t renders;
    const char* ops;
    std::uint64_t out;
    const char* flags;
    const char* decoded;
    const char* decodedBytes;
};

static const QueueCase queueCases[] = {
    {"-d256=-32768", 0x1000, 48000, 48000, 64, 7, "0:a0[0,256,0,0,256] 0:p", 0x4e7c7ced4e8ecf83ull, "3 3 3 3 0 0 0", "64 128 192 256 0 0 0", "128 256 384 512 0 0 0"},
    {"-d2047=-32768", 0x1000, 48000, 48000, 512, 7, "0:a0[0,2047,0,0,2047] 0:p", 0xea1c5d0ca812e4eaull, "3 3 3 0 0 0 0", "512 1024 1536 0 0 0 0", "1024 2048 3072 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,201,0,1,200] 0:p", 0x28d42ec5ad272155ull, "0 0 0 0 0", "0 0 0 0 0", "0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,200,0,300,200] 0:p", 0x2526204e81a09e3full, "0 0 0 0 0", "0 0 0 0 0", "0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,300,0,10,0] 0:p", 0xf5cc11e39439ef19ull, "0 0 0 0 0", "0 0 0 0 0", "0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,210,3,10,200] 0:p", 0xd5113cb184d883e8ull, "3 3 0 0 0", "236 482 0 0 0", "512 1024 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a1[0,210,0,10,200] 0:p", 0x2c2d95791d43fde8ull, "23 23 23 23 23", "200 200 200 200 200", "420 420 420 420 420"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,110,0,10,100;1000,110,0,10,100] 0:p", 0x76aee5574da16a1aull, "0 0 0 0 0", "0 0 0 0 0", "0 0 0 0 0"},
    {"-i400@9=-32768", 0x1000, 44100, 48000, 256, 3, "0:a0[0,210,0,10,200] 0:p", 0xa2049831ea9fab67ull, "0 0 0", "0 0 0", "0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 4:R", 0xc676b0b2868f4751ull, "3 3 5 7 3 3 3 0", "256 512 768 1024 1280 1536 1792 0", "512 1024 1536 2048 2560 3072 3584 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 3:p", 0x726f189acec56ae7ull, "3 3 5 3 3 3 3 3", "256 512 768 256 512 768 1024 1280", "512 1024 1536 512 1024 1536 2048 2560"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 2:R", 0x8d62f37aa57c3bb8ull, "3 3 3 3 3 3 3 0", "256 512 768 1024 1280 1536 1792 0", "512 1024 1536 2048 2560 3072 3584 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:R", 0x8d62f37aa57c3bb8ull, "3 3 3 3 3 3 3 0", "256 512 768 1024 1280 1536 1792 0", "512 1024 1536 2048 2560 3072 3584 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 0:P 2:R", 0x7750f9c749b3b2f7ull, "7 7 3 3 3 3 3 3", "256 256 512 768 1024 1280 1536 1792", "512 512 1024 1536 2048 2560 3072 3584"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:P 0:p 2:R", 0x8d62f37aa57c3bb8ull, "3 3 3 3 3 3 3 0", "256 512 768 1024 1280 1536 1792 0", "512 1024 1536 2048 2560 3072 3584 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 3:f3f000000 4:R", 0x6392520c38e97fe9ull, "3 3 5 7 3 3 3 3", "256 512 768 896 1024 1152 1280 1408", "512 1024 1536 1792 2048 2304 2560 2816"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:P 3:R", 0x371673015d7755d9ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", "512 0 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 1:P 2:p", 0x3dd87fd258dcfec4ull, "3 5 3 3 3 3 3 3", "256 512 256 512 768 1024 1280 1536", "512 1024 512 1024 1536 2048 2560 3072"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a0[1000,300,0,0,300]=804a8054", 0x371673015d7755d9ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", "512 0 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a1[1000,300,0,0,300]=804a8054 1:p", 0xdcc73603f1e76413ull, "3 3 0 0 0 0 0 0", "256 256 0 0 0 0 0 0", "512 512 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:a2[1000,300,0,0,300]=804a8054 0:p", 0x371673015d7755d9ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", "512 0 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a4[1000,300,0,0,300]", 0x52d8c691a430b464ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a5[1000,300,0,0,300]", 0x52d8c691a430b464ull, "3 23 23 23 23 23 23 23", "256 300 300 300 300 300 300 300", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a8[1000,300,0,0,300]=804a8054", 0x371673015d7755d9ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", "512 0 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300] 0:p 1:a10[1000,300,0,0,300]=804a8054", 0x371673015d7755d9ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", "512 0 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a1[0,300,0,0,300] 0:p 3:a0[1000,300,0,0,300]", 0x2cb2c26245f49b9dull, "3 23 23 3 0 0 0 0", "256 300 300 560 0 0 0 0", "512 600 600 1120 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a1[0,300,0,0,300] 0:p 3:a2[1000,300,0,0,300]", 0x371673015d7755d9ull, "3 23 23 0 0 0 0 0", "256 300 300 0 0 0 0 0", "512 600 600 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a1[0,300,0,0,300] 0:p 3:a4[1000,300,0,0,300]", 0x2cb2c26245f49b9dull, "3 23 23 3 0 0 0 0", "256 300 300 560 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300;1000,300,0,0,300] 0:p 4:p", 0x7a78a7471c6244fbull, "3 3 0 0 3 3 0 0", "256 512 0 0 256 512 0 0", "512 1024 0 0 512 1024 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a0[0,300,0,0,300] 0:p 3:a4[] 3:p", 0xa9f58d0f447435d9ull, "3 0 0 1 1 1", "256 0 0 0 0 0", "512 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,100,0,0,100] 0:a1[200,100,0,0,100] 0:a1[400,100,0,0,100] 0:a1[600,100,0,0,100] 0:a1[800,100,0,0,100]=804a8054 0:p", 0x02c256a2580d6aedull, "3 23 23 23 23 23", "256 400 400 400 400 400", "512 800 800 800 800 800"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a0[0,100,0,0,100;200,100,0,0,100;400,100,0,0,100;600,100,0,0,100;800,100,0,0,100]=804a8054 0:p", 0x26c8ba1e58b62383ull, "1 1 1 1 1 1", "0 0 0 0 0 0", "0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,100,0,0,100;200,100,0,0,100;400,100,0,0,100;600,100,0,0,100] 0:p 2:a1[800,100,0,0,100;1000,100,0,0,100]", 0x1b14c5b9261c4c71ull, "3 23 23 23 23 23", "256 400 600 600 600 600", "512 800 1200 1200 1200 1200"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[] 0:a0[0,300,0,0,300] 0:p", 0xa9f58d0f447435d9ull, "3 0 0 0 0 0", "256 0 0 0 0 0", "512 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 9, "0:a1[0,300,0,0,300] 0:p 3:a0[1000,300,0,0,300] 6:p", 0x51a3100918ab15dfull, "3 23 23 3 0 0 3 3 0", "256 300 300 560 0 0 256 512 0", "512 600 600 1120 0 0 512 1024 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a1[0,100,0,0,100;200,100,0,0,100;400,100,0,0,100;600,100,0,0,100] 0:p 2:a0[800,100,0,0,100;1000,100,0,0,100] 5:p", 0x67df1cd4edffaf4bull, "3 23 0 0 0 0 0 0", "256 400 0 0 0 0 0 0", "512 800 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 7, "0:a1[0,100,0,0,100;200,100,0,0,100;400,100,0,0,100;600,100,0,0,100] 0:p 1:a0[800,100,0,0,100] 4:p", 0x7065d185b70e8dcdull, "3 0 0 0 0 0 0", "256 0 0 0 0 0 0", "512 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 7, "0:a1[0,100,0,0,100;200,100,0,0,100;400,100,0,0,100;600,100,0,0,100] 0:p 1:a0[800,100,0,0,100;1000,100,0,0,100;1200,100,0,0,100]=804a8054", 0xea0d8164f0b03aedull, "3 23 23 23 23 23 23", "256 400 400 400 400 400 400", "512 800 800 800 800 800 800"},
    {"t4000", 0x1000, 44100, 48000, 256, 9, "0:a1[0,300,0,0,300] 0:p 2:a0[1000,300,0,0,300]", 0xe477e6ecc04c8472ull, "3 23 3 0 0 0 0 0 0", "235 300 539 0 0 0 0 0 0", "470 600 1078 0 0 0 0 0 0"},
    {"t4000", 0x1000, 44100, 48000, 256, 9, "0:f3faf5c29 0:a1[0,300,0,0,300] 0:p 3:a0[1000,300,0,0,300]", 0xae39b869992cd1afull, "23 23 23 0 0 0 0 0 0", "300 300 300 0 0 0 0 0 0", "600 600 600 0 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,1000,0,0,1000] 0:p 2:a4[2000,1000,0,0,1000] 2:p", 0xca92f6a5e90836c2ull, "3 3 3 3 3 0 0 0", "256 512 256 512 768 0 0 0", "512 1024 512 1024 1536 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,300,0,0,300]", 0xf8ee439949c02d19ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,1000,0,0,1000] 0:p 1:a5[2000,300,0,0,300]", 0xf8ee439949c02d19ull, "3 23 23 23 23 23 23 23", "256 300 300 300 300 300 300 300", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,200,0,0,200;500,300,0,0,300] 0:p 1:a4[2000,1000,0,0,1000]", 0xaa1c8de6c08381e8ull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,2,0,300] 0:p 2:a4[2000,1000,0,0,1000]", 0xd9c3f3254c877f50ull, "3 3 0 0 0 0 0 0", "256 512 0 0 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,100,4294967295,0,100] 0:p 1:a4[2000,1000,0,0,1000]", 0x32dd4e753871f59bull, "3 0 0 0 0 0 0 0", "256 0 0 0 0 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,1000] 0:p", 0xa9f58d0f447435d9ull, "3 23 23 23 23 23", "256 300 300 300 300 300", "512 600 600 600 600 600"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,1000] 0:p 1:a2[300,300,0,0,0]", 0x65b0546f6c9fb424ull, "3 3 0 0 0 0", "256 512 0 0 0 0", "512 1024 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a2[0,300,0,0,0] 0:p", 0xb43a063055adc383ull, "0 0 0 0", "0 0 0 0", "0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a3[0,300,0,0,0] 0:a0[0,300,0,0,300] 0:p", 0xd152fe468f7115d9ull, "3 0 0 0", "256 0 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,0,0,0,300] 0:p", 0xb43a063055adc383ull, "0 0 0 0", "0 0 0 0", "0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a10[0,0,0,0,300] 0:p", 0xb43a063055adc383ull, "0 0 0 0", "0 0 0 0", "0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,1000] 0:p 1:a0[2000,300,0,0,0]", 0xc2eec7ddad901bbbull, "3 3 0 0 0 0", "256 512 0 0 0 0", "512 1024 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,1000] 0:p 2:a0[2000,300,0,0,0]", 0xbde530c8997f3717ull, "3 23 3 0 0 0", "256 300 560 0 0 0", "512 600 1120 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,400] 0:p 1:a0[2000,300,0,0,0]", 0x1fc2542f3152a24full, "3 0 0 0 0 0", "256 0 0 0 0 0", "512 0 0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a1[0,300,0,0,1000] 0:p 1:a2[1000,300,0,0,300]", 0xcddbfe1678a93179ull, "3 3 0 0 0 0", "256 512 0 0 0 0", "512 1024 0 0 0 0"},
    {"t4000", 0x4001, 48000, 48000, 256, 8, "0:a0[0,300,0,0,300;1000,300,2,0,300] 0:p", 0x636ed8260d6b5499ull, "3 3 3 0 0 0 0 0", "256 512 768 0 0 0 0 0", ""},
    {"t4000", 0x4001, 48000, 48000, 256, 6, "0:a1[0,300,0,0,300] 0:p 3:a0[1000,300,0,0,300]", 0xf5893ee70818fb9dull, "3 23 23 3 0 0", "256 300 300 560 0 0", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,64,2,0,64] 0:p", 0x8879fffa001104c3ull, "0 0 0 0", "0 0 0 0", "0 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,64,4294967295,0,64] 0:p", 0x10ff0f0c85068a03ull, "3 3 3 3", "256 512 768 1024", "512 1024 1536 2048"},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,64,2,0,64;1000,64,0,0,64;2000,64,3,0,64] 0:p", 0x959f600cc8c0e3c4ull, "3 0 0 0", "256 0 0 0", "512 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a1[0,64,4294967295,0,64] 0:p 1:a0[1000,64,2,0,64]", 0xa47eee2a6a921983ull, "3 3 3 3 3", "256 512 768 1024 1280", "512 1024 1536 2048 2560"},
};

static const QueueCase stopCases[] = {
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 3:S 4:p", 0, "3 3 5 0 3 3 3 3", "256 512 768 0 256 512 768 1024", "512 1024 1536 0 512 1024 1536 2048"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a0[0,300,0,0,300] 0:p 1:S 2:p", 0, "3 0 3 0 0 0", "256 0 256 0 0 0", "512 0 512 0 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a0[0,300,0,0,300] 0:p 1:s 3:p", 0, "3 0 0 3 0 0", "256 0 0 256 0 0", "512 0 0 512 0 0"},
    {"t4000", 0x1000, 48000, 48000, 256, 6, "0:a0[0,300,0,0,300] 0:p 1:k 2:p", 0, "3 0 3 0 0 0", "256 0 256 0 0 0", "512 0 512 0 0 0"},
};

static const QueueCase unsupportedCases[] = {
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,2000,0,0,2000] 0:p 2:P 3:a4[3000,300,0,0,300] 4:R", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,1000,0,0,1000] 0:p 2:a4[2000,100,0,0,100]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 8, "0:a0[0,300,3,0,300] 0:p 1:a4[2000,100,2,0,100]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,1000,0,0,1000;3000,1000,0,0,1000]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,1000,0,10,990]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,1000,0,0,0]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,0,0,0,1000]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a5[2000,1000,0,0,1000] 1:a3[3000,100,0,0,100]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,256,0,0,256]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a1[0,100,0,0,100] 0:p 1:a4[2000,0,0,0,1000]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,2000,0,0,2000] 0:p 2:P 2:a4[2000,1000,0,0,1000]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 5, "0:a0[0,2000,0,0,2000] 0:p 2:P 3:a4[2000,1000,0,0,1000]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a0[0,1000,0,0,1000] 0:p 1:a4[2000,1000,0,0,1000] 1:P", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,1000,0,0,1000] 0:p 1:a5[2000,1000,0,0,1000] 2:a3[3000,100,0,0,100]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 3, "0:a1[0,10,0,0,10;10,10,0,0,10;20,10,0,0,10] 0:p 1:a1[30,10,0,0,10;40,10,0,0,10]", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 2, "0:a0[0,0,4294967295,0,10] 0:p", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 32, 4, "0:a0[0,1000,0,0,1000] 0:p 1:P", 0, "", "", ""},
    {"t4000", 0x1000, 48000, 48000, 256, 4, "0:a0[0,256,0,0,256] 0:p 1:a4[2000,1000,0,0,1000]", 0, "", "", ""},
};

struct QueueOp {
    std::uint32_t render;
    char kind;
    std::uint32_t value;
    std::vector<Ngs2WaveformBlock> blocks;
    int result;
};

static std::vector<std::int16_t> Source(std::string spec) {
    if (spec[0] == '-') spec.erase(0, 1);
    unsigned frames = 0;
    unsigned at = 0;
    int value = 0;
    if (spec[0] == 't') {
        frames = static_cast<unsigned>(std::stoul(spec.substr(1)));
        std::vector<std::int16_t> pcm(frames);
        for (unsigned frame = 0; frame < frames; frame++) pcm[frame] = static_cast<std::int16_t>(8 * (frame + 1));
        return pcm;
    }
    if (spec[0] == 'i') {
        Require(std::sscanf(spec.c_str() + 1, "%u@%u=%d", &frames, &at, &value) == 3);
        std::vector<std::int16_t> pcm(frames, 0);
        pcm[at] = static_cast<std::int16_t>(value);
        return pcm;
    }
    Require(spec[0] == 'd' && std::sscanf(spec.c_str() + 1, "%u=%d", &frames, &value) == 2);
    return std::vector<std::int16_t>(frames, static_cast<std::int16_t>(value));
}

static std::vector<QueueOp> Ops(const char* text) {
    std::vector<QueueOp> ops;
    std::istringstream tokens(text);
    std::string token;
    while (tokens >> token) {
        QueueOp op{};
        const auto colon = token.find(':');
        op.render = static_cast<std::uint32_t>(std::stoul(token.substr(0, colon)));
        const auto equals = token.find('=');
        if (equals != std::string::npos) op.result = static_cast<int>(std::stoul(token.substr(equals + 1), nullptr, 16));
        const auto body = token.substr(colon + 1, equals == std::string::npos ? std::string::npos : equals - colon - 1);
        op.kind = body[0];
        if (op.kind == 'f') op.value = static_cast<std::uint32_t>(std::stoul(body.substr(1), nullptr, 16));
        if (op.kind == 'a') {
            const auto open = body.find('[');
            op.value = static_cast<std::uint32_t>(std::stoul(body.substr(1, open - 1), nullptr, 16));
            std::istringstream blocks(body.substr(open + 1, body.size() - open - 2));
            std::string block;
            while (std::getline(blocks, block, ';')) {
                unsigned offset = 0;
                unsigned size = 0;
                unsigned repeats = 0;
                unsigned skip = 0;
                unsigned samples = 0;
                Require(std::sscanf(block.c_str(), "%u,%u,%u,%u,%u", &offset, &size, &repeats, &skip, &samples) == 5);
                op.blocks.push_back({offset * sizeof(std::int16_t), size * sizeof(std::int16_t), repeats, skip, samples, 0, 0});
            }
        }
        ops.push_back(op);
    }
    return ops;
}

static std::string Join(const std::vector<std::uint64_t>& values, bool hex) {
    std::string text;
    for (const auto value : values) {
        char number[32];
        std::snprintf(number, sizeof(number), hex ? "%llx" : "%llu", static_cast<unsigned long long>(value));
        text += (text.empty() ? "" : " ") + std::string(number);
    }
    return text;
}

static uintptr_t CustomSampler(uintptr_t system, std::uint32_t blocks) {
    Ngs2CustomSamplerRackOption option{};
    auto& common = option.custom_rack_option.rack_option;
    common.size = sizeof(option);
    common.max_grain_samples = 512;
    common.max_voices = 1;
    common.max_matrices = 1;
    common.max_ports = 1;
    option.custom_rack_option.num_buffers = 1;
    option.max_channel_works = 1;
    option.max_waveform_blocks = blocks;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_CUSTOM_SAMPLER, &common, &query) == SCE_NGS2_OK);
    const auto info = Buffer(query);
    uintptr_t rack = 0;
    Require(sceNgs2RackCreate(system, SCE_NGS2_RACK_ID_CUSTOM_SAMPLER, &common, &info, &rack) == SCE_NGS2_OK);
    return Voice(rack);
}

static int Send(uintptr_t voice, bool custom, const QueueOp& op, const std::vector<std::int16_t>& pcm) {
    if (op.kind == 'a') {
        Ngs2SamplerVoiceWaveformBlocksParam param{{}, pcm.data(), op.value, static_cast<std::uint32_t>(op.blocks.size()), op.blocks.data()};
        param.header = {sizeof(param), 0, custom ? SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS : SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS};
        return sceNgs2VoiceControl(voice, &param.header);
    }
    if (op.kind == 'f') {
        float ratio;
        std::memcpy(&ratio, &op.value, sizeof(ratio));
        Ngs2SamplerVoicePitchParam param{{sizeof(Ngs2SamplerVoicePitchParam), 0, custom ? SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_PITCH : SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH}, ratio};
        return sceNgs2VoiceControl(voice, &param.header);
    }
    const std::uint32_t event = op.kind == 'p' ? SCE_NGS2_VOICE_EVENT_PLAY : op.kind == 'P' ? SCE_NGS2_VOICE_EVENT_PAUSE : op.kind == 's' ? SCE_NGS2_VOICE_EVENT_STOP
                               : op.kind == 'S' ? SCE_NGS2_VOICE_EVENT_STOP_IMM
                               : op.kind == 'k' ? SCE_NGS2_VOICE_EVENT_KILL
                                                : SCE_NGS2_VOICE_EVENT_RESUME;
    Ngs2VoiceEventParam param{{sizeof(Ngs2VoiceEventParam), 0, SCE_NGS2_VOICE_PARAM_EVENT}, event};
    return sceNgs2VoiceControl(voice, &param.header);
}

static std::uint64_t Hash(const std::vector<float>& out) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const float sample : out) {
        std::uint32_t bits;
        std::memcpy(&bits, &sample, sizeof(bits));
        for (int i = 0; i < 4; i++) {
            hash ^= (bits >> (8 * i)) & 0xff;
            hash *= 1099511628211ull;
        }
    }
    return hash;
}

struct QueueSystem {
    std::size_t firstBuffer = usedBuffers;
    uintptr_t handle = CreateSystem();

    ~QueueSystem() {
        sceNgs2SystemDestroy(handle, nullptr);
        usedBuffers = firstBuffer;
    }
};

static bool RunCase(const QueueCase& queueCase) {
    const auto pcm = Source(queueCase.source);
    const bool negated = queueCase.source[0] == '-';
    const bool custom = queueCase.rack == SCE_NGS2_RACK_ID_CUSTOM_SAMPLER;
    const QueueSystem system;
    Require(sceNgs2SystemSetGrainSamples(system.handle, queueCase.grain) == SCE_NGS2_OK && sceNgs2SystemSetSampleRate(system.handle, queueCase.systemRate) == SCE_NGS2_OK);
    const auto master = Mastering(system.handle, 1);
    const auto voice = custom ? CustomSampler(system.handle, 4) : Voice(CreateRack(system.handle, SCE_NGS2_RACK_ID_SAMPLER));
    Ngs2SamplerVoiceSetupParam setup{{sizeof(Ngs2SamplerVoiceSetupParam), 0, custom ? SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_SETUP : SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP},
                                     {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, queueCase.rate, 0, 0, 0}};
    Require(sceNgs2VoiceControl(voice, &setup.header) == SCE_NGS2_OK);
    Patch(voice, master);
    const auto ops = Ops(queueCase.ops);
    std::istringstream expectedFlags(queueCase.flags);
    std::istringstream expectedDecoded(queueCase.decoded);
    bool results = true;
    std::vector<float> out;
    std::vector<std::uint64_t> flags;
    std::vector<std::uint64_t> decoded;
    std::vector<std::uint64_t> decodedBytes;
    std::vector<float> grain(queueCase.grain);
    for (std::uint32_t render = 0; render < queueCase.renders; render++) {
        for (const auto& op : ops) {
            if (op.render == render) results = Send(voice, custom, op, pcm) == op.result && results;
        }
        const Ngs2RenderBufferInfo info{grain.data(), grain.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 1};
        Require(sceNgs2SystemRender(system.handle, &info, 1) == SCE_NGS2_OK);
        for (const float sample : grain) out.push_back(negated ? 0.0f - sample : sample);
        flags.push_back(Flags(voice));
        std::string expectedFlag;
        std::uint64_t expectedCount = 0;
        expectedFlags >> expectedFlag;
        expectedDecoded >> expectedCount;
        if (custom) {
            Ngs2CustomSamplerVoiceState state{};
            Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
            decoded.push_back(expectedFlag == "0" ? expectedCount : state.num_decoded_samples);
        } else {
            Ngs2SamplerVoiceState state{};
            Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
            decoded.push_back(state.num_decoded_samples);
            decodedBytes.push_back(state.decoded_data_size);
        }
    }
    const bool bytes = queueCase.decodedBytes[0] == '\0' || Join(decodedBytes, false) == queueCase.decodedBytes;
    return results && (queueCase.out == 0 || Hash(out) == queueCase.out) && Join(flags, true) == queueCase.flags && Join(decoded, false) == queueCase.decoded && bytes;
}

static void TestMeasuredQueues() {
    int failed = 0;
    for (std::size_t i = 0; i < std::size(queueCases); i++) {
        if (RunCase(queueCases[i])) continue;
        std::fprintf(stderr, "queue case %zu differs from the PS5 module\n", i);
        failed++;
    }
    Require(failed == 0);
}

static void TestStopFlags() {
    for (const auto& queueCase : stopCases) Require(RunCase(queueCase));
}

static void TestUnsupportedQueues() {
    for (const auto& queueCase : unsupportedCases) {
        bool unsupported = false;
        try {
            RunCase(queueCase);
        } catch (const std::runtime_error&) {
            unsupported = true;
        }
        Require(unsupported);
    }
}

static void TestNoWaveformBlocks() {
    const QueueSystem system;
    const auto voice = CustomSampler(system.handle, 0);
    Ngs2SamplerVoiceSetupParam setup{{sizeof(Ngs2SamplerVoiceSetupParam), 0, SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_SETUP}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}};
    Require(sceNgs2VoiceControl(voice, &setup.header) == SCE_NGS2_OK);
    const auto ops = Ops("0:a0[0,100,0,0,100]");
    const auto pcm = Source("t100");
    bool unsupported = false;
    try {
        Send(voice, true, ops[0], pcm);
    } catch (const std::runtime_error&) {
        unsupported = true;
    }
    Require(unsupported);
}

static void TestPartialFrame() {
    const QueueSystem system;
    const auto voice = Voice(CreateRack(system.handle, SCE_NGS2_RACK_ID_SAMPLER));
    Ngs2SamplerVoiceSetupParam setup{{sizeof(Ngs2SamplerVoiceSetupParam), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}};
    Require(sceNgs2VoiceControl(voice, &setup.header) == SCE_NGS2_OK);
    const auto pcm = Source("t4");
    const Ngs2WaveformBlock block{0, 3, 0, 0, 1, 0, 0};
    Ngs2SamplerVoiceWaveformBlocksParam param{{sizeof(Ngs2SamplerVoiceWaveformBlocksParam), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS}, pcm.data(), 0, 1, &block};
    bool unsupported = false;
    try {
        sceNgs2VoiceControl(voice, &param.header);
    } catch (const std::runtime_error&) {
        unsupported = true;
    }
    Require(unsupported);
}

int main() {
    TestMeasuredQueues();
    TestStopFlags();
    TestUnsupportedQueues();
    TestNoWaveformBlocks();
    TestPartialFrame();
    return 0;
}
