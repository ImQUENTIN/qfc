/************************************************************************************************
* @file    atomic.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    Apr. 23th, 2018.
* @brief   None
************************************************************************************************/
#pragma once

#include <stdint.h>

#include "platform.h"


// restore BASEPRI (called as cleanup function), with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}

// set BASEPRI_MAX, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
		uint8_t atomic_BASEPRI = __get_BASEPRI();
	    if(prio && (atomic_BASEPRI == 0 || atomic_BASEPRI > prio)) {
        __set_BASEPRI(prio);
    }
    return 1;
}

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save  = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __set_BASEPRI(__basepri_save), __ToDo = 0 )

// Run block with elevated BASEPRI (using BASEPRI_MAX), but do not create memory barrier.
// Be careful when using this, you must use some method to prevent optimizer from breaking things
// - lto is used for Cleanflight compilation, so function call is not memory barrier
// - use ATOMIC_BARRIER or volatile to protect used variables
// - gcc 4.8.4 does write all values in registers to memory before 'asm volatile', so this optimization does not help much
// - gcc 5 and later works as intended, generating quite optimal code
//#define ATOMIC_BLOCK_NB(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestore), __unused__)) = __get_BASEPRI(), \
//                                    __ToDo = __basepriSetRetVal(prio); __ToDo ; __ToDo = 0 ) \

// ATOMIC_BARRIER
// Create memory barrier
// - at the beginning of containing block (value of parameter must be reread from memory)
// - at exit of block (all exit paths) (parameter value if written into memory, but may be cached in register for subsequent use)
// On gcc 5 and higher, this protects only memory passed as parameter (any type can be used)
// this macro can be used only ONCE PER LINE, but multiple uses per block are fine

#if (__GNUC__ > 7)
# warning "Please verify that ATOMIC_BARRIER works as intended"
// increment version number if BARRIER works
// TODO - use flag to disable ATOMIC_BARRIER and use full barrier instead
// you should check that local variable scope with cleanup spans entire block
#endif

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif

#define ATOMIC_BARRIER_ENTER(dataPtr, refStr)                              \
    __asm__ volatile ("\t# barrier (" refStr ") enter\n" : "+m" (*(dataPtr)))

#define ATOMIC_BARRIER_LEAVE(dataPtr, refStr)                              \
    __asm__ volatile ("\t# barrier (" refStr ") leave\n" : "m" (*(dataPtr)))

#if defined(__clang__)
// CLang version, using Objective C-style block
// based on https://stackoverflow.com/questions/24959440/rewrite-gcc-cleanup-macro-with-nested-function-for-clang
typedef void (^__cleanup_block)(void);
static inline void __do_cleanup(__cleanup_block * b) { (*b)(); }

#define ATOMIC_BARRIER(data)                                            \
    typeof(data) *__UNIQL(__barrier) = &data;                           \
    ATOMIC_BARRIER_ENTER(__UNIQL(__barrier), #data);                    \
    __cleanup_block __attribute__((cleanup(__do_cleanup) __unused__)) __UNIQL(__cleanup) = \
        ^{  ATOMIC_BARRIER_LEAVE(__UNIQL(__barrier), #data); };         \
    do {} while(0)                                                      \
/**/
#else
// gcc version, uses local function for cleanup.
#define ATOMIC_BARRIER(data)                                            \
    __extension__ void  __UNIQL(__barrierEnd)(typeof(data) **__d) {     \
         ATOMIC_BARRIER_LEAVE(*__d, #data);                             \
    }                                                                   \
    typeof(data) __attribute__((__cleanup__(__UNIQL(__barrierEnd)))) *__UNIQL(__barrier) = &data; \
    ATOMIC_BARRIER_ENTER(__UNIQL(__barrier), #data);                    \
    do {} while(0)                                                      \
/**/
#endif

// define these wrappers for atomic operations, using gcc builtins
#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
