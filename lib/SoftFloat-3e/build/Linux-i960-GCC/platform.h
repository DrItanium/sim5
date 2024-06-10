//
// Created by jwscoggins on 6/9/24.
//

#ifndef SIM960_PLATFORM_H
#define SIM960_PLATFORM_H
#ifdef __ORDER_LITTLE_ENDIAN__
    #define LITTLEENDIAN 1
#endif

#ifdef __GNUC_STDC_INLINE__
#define INLINE inline
#else
#define INLINE extern inline
#endif

#define THREAD_LOCAL _Thread_local
#define SOFTFLOAT_BUILTIN_CLZ 1
#define SOFTFLOAT_INTRINSIC_INT128 1
#endif //SIM960_PLATFORM_H
