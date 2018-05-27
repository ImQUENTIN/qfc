/************************************************************************************************
* @file    bitops.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   MDK 没有的一些 Linux 内核实现
************************************************************************************************/


#pragma once

/** 
 * fls - find last (most-significant) bit set 
 * @x: the word to search 
 * 
 * This is defined the same way as ffs. 
 * Note fls(0) = 0, fls(1) = 1, fls(0x80000000) = 32. 
 */  
static inline int constant_fls(int x)
{
    int r = 32;

    if (!x)
        return 0;
    if (!(x & 0xffff0000u)) {
        x <<= 16;
        r -= 16;
    }
    if (!(x & 0xff000000u)) {
        x <<= 8;
        r -= 8;
    }
    if (!(x & 0xf0000000u)) {
        x <<= 4;
        r -= 4;
    }
    if (!(x & 0xc0000000u)) {
        x <<= 2;
        r -= 2;
    }
    if (!(x & 0x80000000u)) {
        x <<= 1;
        r -= 1;
    }
    return r;
}


static inline int fls(int x)
{
#if 0
    return constant_fls(x);
#else
    int ret;
    __asm{
        //CLZ Rd, Rm // Counts number of leading zero bits in Rm store result in Rd. Rd is 32 if Rm.
        CLZ     ret, x  
    }
    ret = 32 - ret;
    return ret;
#endif
}

//https://www.cnblogs.com/qiyuexin/p/9095161.html
static inline int ffs( int x) 
{
    return fls(x & -x); 
}


