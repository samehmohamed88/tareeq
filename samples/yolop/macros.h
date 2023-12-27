#ifndef __MACROS_H
#define __MACROS_H

#define NV_TENSORRT_MAJOR 8

#if NV_TENSORRT_MAJOR >= 8
#define TRT_NOEXCEPT noexcept
#define TRT_CONST_ENQUEUE const
#else
#define TRT_NOEXCEPT
#define TRT_CONST_ENQUEUE
#endif

#endif  // __MACROS_H