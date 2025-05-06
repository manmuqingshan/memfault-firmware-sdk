//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! Hooks for collecting coredumps from failure paths in FreeRTOS kernel

#include "FreeRTOS.h"
#include "memfault/core/compiler.h"
#include "memfault/panics/assert.h"
#include "task.h"

//! Collect a coredump when a FreeRTOS assert takes place
//!
//! Implementation for assert function present in FreeRTOSConfig.h:
//!  #define configASSERT(x) if ((x) == 0) vAssertCalled( __FILE__, __LINE__ )
void vAssertCalled(MEMFAULT_UNUSED const char *file, MEMFAULT_UNUSED int line) {
  MEMFAULT_ASSERT(0);
}

//! Collects a coredump when a stack overflow takes place in FreeRTOS kernel
//!
//! @note: Depends on FreeRTOS kernel being compiled with
//!  configCHECK_FOR_STACK_OVERFLOW != 0
void vApplicationStackOverflowHook(MEMFAULT_UNUSED TaskHandle_t xTask,
                                   MEMFAULT_UNUSED char *pcTaskName) {
  MEMFAULT_ASSERT_WITH_REASON(0, kMfltRebootReason_StackOverflow);
}
