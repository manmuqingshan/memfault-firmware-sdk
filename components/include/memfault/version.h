#pragma once

//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! Contains Memfault SDK version information.

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
} sMfltSdkVersion;

#define MEMFAULT_SDK_VERSION \
  { .major = 1, .minor = 28, .patch = 0 }
#define MEMFAULT_SDK_VERSION_STR "1.28.0"

#ifdef __cplusplus
}
#endif
