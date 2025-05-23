#pragma once

//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! @brief
//! Math helpers

#ifdef __cplusplus
extern "C" {
#endif

#define MEMFAULT_ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define MEMFAULT_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MEMFAULT_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MEMFAULT_FLOOR(a, align) (((a) / (align)) * (align))
#define MEMFAULT_ABS(a) (((a) < 0) ? -(a) : (a))
#define MEMFAULT_CEIL_DIV(x, y) (((x) + (y) - 1) / (y))

#ifdef __cplusplus
}
#endif
