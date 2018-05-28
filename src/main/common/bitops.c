/************************************************************************************************
* @file    bitops.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   bit 操作
************************************************************************************************/

#include "bitops.h"

//https://www.cnblogs.com/qiyuexin/p/9095161.html

// 返回输入参数的最高有效bit位（从低位往左数最后的有效bit位）的序号，该序号与常规0起始序号不同，它是1起始的（当没有有效位时返回0）。


//
//// __ffs 用于找到一个int或者long行最高哪位是1，例如0x8000，就会返回15.代表从第8个bit开始不为0.其源码如下;
//static __always_inline unsigned long __ffs(unsigned long word)
//{
//int num = 0;
//
//if ((word & 0xffff) == 0) {
//num += 16;
//word >>= 16;
//}
//if ((word & 0xff) == 0) {
//num += 8;
//word >>= 8;
//}
//if ((word & 0xf) == 0) {
//num += 4;
//word >>= 4;
//}
//if ((word & 0x3) == 0) {
//num += 2;
//word >>= 2;
//}
//if ((word & 0x1) == 0)
//num += 1;
//return num;
//}

