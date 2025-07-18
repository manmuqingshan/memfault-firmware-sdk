#pragma once

//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! @brief
//! Stores serialized event information that is ready to be sent up to the Memfault cloud (i.e
//! Heartbeat Metrics, Reboot Reasons & Trace events). Must be initialized on system boot.
//!
//! @note If calls to data_packetizer.c are made on a different task than the one
//! MemfaultPlatformTimerCallback is invoked on, memfault_lock() & memfault_unlock() should also
//! be implemented by the platform
//!
//! @note Recorded events are always written into RAM for minimal latency. Users of the API can
//! (optionally) implement the non-volatile event storage platform API and periodically flush
//! events to a non-volatile storage medium. More details can be found in
//! "memfault/core/platform/nonvolatile_event_storage.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "memfault/config.h"

// The unity test framework (http://www.throwtheswitch.org/unity) fails to generate mocks when
// opaque pointers are used in a header.  To work around, the problem, we pull in the full internal
// definition for "sMemfaultEventStorageImpl" when the unity framework is being used.
#if defined(UNITY_INCLUDE_CONFIG_H)
  #include "memfault/core/event_storage_implementation.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MemfaultEventStorageImpl sMemfaultEventStorageImpl;

//! Must be called by the customer on boot to setup event storage.
//!
//! This is where serialized event data like heartbeat metrics, reboot tracking info, and
//! trace event data is stored as it waits to be drained (via call's
//! to memfault_packetizer_get_next()).
//!
//! For any module using the event store the worst case size needed can be computed using the
//! exported APIs:
//!   memfault_reboot_tracking_compute_worst_case_storage_size()
//!   memfault_metrics_heartbeat_compute_worst_case_storage_size()
//!   memfault_trace_event_compute_worst_case_storage_size()
//!
//! For example, if a device connects to the Internet once/hour and collects a heartbeat at this
//! interval as well, the buffer size to allocate can easily be computed as:
//!  buf_len = memfault_metrics_heartbeat_compute_worst_case_storage_size() * 1
//!
//! When a module using the event store is initialized a WARNING will print on boot and an error
//! code will return if it is not appropriately sized to hold at least one event.
//!
//! @param buf The buffer to use for event storage
//! @param buf_len The length of the buffer to use for event storage
//!
//! @return a handle to the event storage implementation on success & false on failure.
//!  This handle will need to be provided to modules which use the event store on initialization
const sMemfaultEventStorageImpl *memfault_events_storage_boot(void *buf, size_t buf_len);

typedef struct MemfaultEventStorageInfo {
  size_t bytes_used;
  size_t bytes_free;
} sMemfaultEventStorageInfo;

#if MEMFAULT_EVENT_STORAGE_NV_SUPPORT_ENABLED
typedef struct MemfaultEventStoragePersistCbStatus {
  //! Summarizes the utilization of the RAM buffer passed in memfault_events_storage_boot()
  sMemfaultEventStorageInfo volatile_storage;
} sMemfaultEventStoragePersistCbStatus;

//! Invoked to request that events be saved to non-volatile storage
//!
//! @param status Summarizes the current state of event storage. The user of the SDK
//!  can optionally use this information to decide whether or not to persist events to
//!  non-volatile storage.
//!
//! @note By default this is a weak function which behaves as a no-op.
//!
//! @note The intended use of the API is to provide platforms which implement non-volatile event
//! storage a "hook" to help them determine when to flush events from volatile to non-volatile
//! storage. This function is invoked each time an event:
//!  - is saved into the RAM-backed buffer
//!  - is consumed from non-volatile storage _and_ there are still events in
//!    volatile storage. (i.e If non-volatile storage is full, reading events will free up space
//!    and then events residing in RAM can be persisted into the new freed up space)
//!
//! @note It is safe to call "memfault_event_storage_persist()" both synchronously and
//!  asynchronously from this callback
void memfault_event_storage_request_persist_callback(
  const sMemfaultEventStoragePersistCbStatus *status);

//! Saves events which have been collected into non-volatile storage
//!
//! @return number of events saved or <0 for unexpected errors
int memfault_event_storage_persist(void);

#endif

//! Retrieve the number of bytes used in the allocated event storage buffer.
//!
//! @note This function must not be called from an ISR context.
//!
//! @return zero if the storage has not been allocated.
size_t memfault_event_storage_bytes_used(void);

//! Retrieve the number of bytes free (unused) in the allocated event storage
//! buffer.
//!
//! @note This function must not be called from an ISR context.
//!
//! @return zero if the storage has not been allocated.
size_t memfault_event_storage_bytes_free(void);

//! Check if event storage component has booted
//!
//! @note This function must not be called from an ISR context.
//!
//! @returns true if event storage booted or false if not
bool memfault_event_storage_booted(void);

typedef struct {
  // Event storage uses 2 separate buffers, one for context and one for the
  // circular buffer storage. The context size is fixed, but the storage
  // size is user-configurable.
  void *context;
  size_t context_len;
  void *storage;
  size_t storage_len;
} sMfltEventStorageSaveState;
//! Provide a define for the size of the context storage state. This can be used
//! to appropriately size the full state storage buffer.
#if (INTPTR_MAX == 0x7fffffff)
  #define MEMFAULT_EVENT_STORAGE_STATE_SIZE_BYTES 44
#elif (INTPTR_MAX == 0x7fffffffffffffff)
  #define MEMFAULT_EVENT_STORAGE_STATE_SIZE_BYTES 80
#endif

//! Return a pointer to the event storage state. This is used to save the state
//! prior to loss of system power, if the platform needs to restore the state
//! after restart.
//!
//! @note If event storage is accessed after this function is called, the event
//! storage state may be undefined. This function should only be called during a
//! planned shutdown or deep sleep entry, when the rest of the system is
//! inactive.
sMfltEventStorageSaveState memfault_event_storage_get_state(void);

//! Restore the event storage state. This is used to restore the state after
//! system restarts, and is called from memfault_events_storage_boot. A platform
//! can implement this function to restore the event storage state after a deep
//! sleep or power loss.
//!
//! Must be implemented if MEMFAULT_EVENT_STORAGE_RESTORE_STATE is enabled.
//!
//! @param state The state to restore- should match the state returned from
//!   memfault_event_storage_get_state. This function assumes the state is valid
//!   (uncorrupted).
//!
//! @return true if the state was restored successfully, false otherwise.
extern bool memfault_event_storage_restore_state(sMfltEventStorageSaveState *state);

#ifdef __cplusplus
}
#endif
