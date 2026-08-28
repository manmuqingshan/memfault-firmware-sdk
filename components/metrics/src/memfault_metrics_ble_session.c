//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! @brief BLE disconnect session metrics implementation.

#include "memfault/metrics/ble_session.h"

// non-module includes below

#include "memfault/metrics/metrics.h"

#if MEMFAULT_METRICS_BLE_SESSION

void memfault_metrics_ble_session_connected(void) {
  (void)MEMFAULT_METRICS_SESSION_START(bt_conn);
}

void memfault_metrics_ble_session_disconnected(uint8_t reason_code) {
  (void)MEMFAULT_METRIC_SESSION_SET_UNSIGNED(bt_conn_disconnect_reason_code, bt_conn, reason_code);
  (void)MEMFAULT_METRICS_SESSION_END(bt_conn);
}

#endif  // MEMFAULT_METRICS_BLE_SESSION
