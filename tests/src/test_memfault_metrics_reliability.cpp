

#include "CppUTest/MemoryLeakDetectorMallocMacros.h"
#include "CppUTest/MemoryLeakDetectorNewMacros.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "comparators/comparator_memfault_metric_ids.hpp"
#include "memfault/core/platform/core.h"
#include "memfault/metrics/metrics.h"
#include "memfault/metrics/reliability.h"

static MemfaultMetricIdsComparator s_metric_id_comparator;

// These have to have global scope, so the test teardown can access them
static MemfaultMetricId operational_hours_key = MEMFAULT_METRICS_KEY(operational_hours);
static MemfaultMetricId operational_crashfree_hours_key =
  MEMFAULT_METRICS_KEY(operational_crashfree_hours);

extern "C" {
static uint64_t s_fake_time_ms = 0;
uint64_t memfault_platform_get_time_since_boot_ms(void) {
  return s_fake_time_ms;
}

static void prv_fake_time_incr(uint64_t fake_time_delta_ms) {
  s_fake_time_ms += fake_time_delta_ms;
}
}

// clang-format off
TEST_GROUP(MemfaultMetricsReliability){
  void setup() {
    s_fake_time_ms = 0;
    mock().strictOrder();
    mock().installComparator("MemfaultMetricId", s_metric_id_comparator);

    // zero state
    sMemfaultMetricsReliabilityCtx ctx = { 0 };
    memfault_metrics_reliability_boot(&ctx);
  }
  void teardown() {
    mock().checkExpectations();
    mock().removeAllComparatorsAndCopiers();
    mock().clear();
  }
};
// clang-format on

// Note: this is kept in one big test case, because the internal state of the
// function under test is not touched.
TEST(MemfaultMetricsReliability, Test_OperationalHours) {
  // 1 ms less than 1 hr
  prv_fake_time_incr(60 * 60 * 1000 - 1);
  // no mocks should be called
  memfault_metrics_reliability_collect();

  // 1 ms more (1 hr total)
  prv_fake_time_incr(1);
  bool unexpected_reboot = true;
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", 1)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_reboot_tracking_get_unexpected_reboot_occurred")
    .withOutputParameterReturning("unexpected_reboot_occurred", &unexpected_reboot,
                                  sizeof(unexpected_reboot))
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", 0)
    .andReturnValue(0);

  memfault_metrics_reliability_collect();

  // 1 hr more
  prv_fake_time_incr(60 * 60 * 1000);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", 1)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", 1)
    .andReturnValue(0);

  memfault_metrics_reliability_collect();

  // 1 ms more (2hr 1ms total)
  prv_fake_time_incr(1);
  // no mocks should be called
  memfault_metrics_reliability_collect();

  // Now test rollover scenarios
  // advance to UINT32_MAX - 1; this represents a little over 49 days long
  // heartbeat, so we won't track uptime
  const uint32_t elapsed_hours = (UINT32_MAX - 1 - s_fake_time_ms) / 1000 / 3600;
  prv_fake_time_incr(UINT32_MAX - 1 - s_fake_time_ms);
  // plenty of crashfree hours!
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", elapsed_hours)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", elapsed_hours)
    .andReturnValue(0);
  memfault_metrics_reliability_collect();

  // advance 1 ms (UINT32_MAX total)
  prv_fake_time_incr(1);
  // no mocks should be called
  memfault_metrics_reliability_collect();

  // 1 ms more (UINT32_MAX + 1 total)
  prv_fake_time_incr(1);
  // no mocks should be called
  memfault_metrics_reliability_collect();

  // advance exactly 1 hour
  prv_fake_time_incr(60 * 60 * 1000);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", 1)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", 1)
    .andReturnValue(0);

  memfault_metrics_reliability_collect();

  // advance to UINT64_MAX - 1. the amount of accumulated hours wraps at
  // UINT32_MAX due to the implementation only using uint32_t for tracking
  // heartbeat_ms duration- this should be ok, 49 days is a long heartbeat.
  const uint32_t elapsed_hours_2 = (uint32_t)((UINT32_MAX - 1 - s_fake_time_ms)) / 1000 / 3600;

  prv_fake_time_incr(UINT64_MAX - 1 - s_fake_time_ms);

  // plenty of crashfree hours!
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", elapsed_hours_2)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", elapsed_hours_2)
    .andReturnValue(0);
  memfault_metrics_reliability_collect();

  // advance 1 ms (UINT64_MAX total)
  prv_fake_time_incr(1);
  // no mocks should be called
  memfault_metrics_reliability_collect();

  // 1 ms more (UINT64_MAX + 1 total)
  prv_fake_time_incr(1);
  // no mocks should be called
  memfault_metrics_reliability_collect();
}

//! Test that pre-seeding with memfault_metrics_reliability_boot() yields
//! correct operational time
TEST(MemfaultMetricsReliability, Test_Boot) {
  sMemfaultMetricsReliabilityCtx ctx = {
    .last_heartbeat_ms = 1000u * 60u * 60u * 1000u,  // 1k hours
    .operational_ms = 1000,                          // accumulated 1 second of operational time
    .counted_unexpected_reboot = 1,
  };
  memfault_metrics_reliability_boot(&ctx);

  prv_fake_time_incr(ctx.last_heartbeat_ms);

  // advance by 1 hr - 999 ms
  uint32_t operational_hours = 1;
  prv_fake_time_incr(operational_hours * 60 * 60 * 1000 - 999);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", operational_hours)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", operational_hours)
    .andReturnValue(0);

  memfault_metrics_reliability_collect();
}

TEST(MemfaultMetricsReliability, Test_BootTimeSinceBootNonZero) {
  // test cold booting with time-since-boot non-zero (i.e. rtc backed)
  prv_fake_time_incr(1000u * 60u * 60u * 1000u);  // 1k hours
  memfault_metrics_reliability_boot(NULL);

  // advance by 1 hr
  uint32_t operational_hours = 1;
  prv_fake_time_incr(operational_hours * 60 * 60 * 1000);
  bool unexpected_reboot = false;

  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_hours_key)
    .withParameter("amount", operational_hours)
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_reboot_tracking_get_unexpected_reboot_occurred")
    .withOutputParameterReturning("unexpected_reboot_occurred", &unexpected_reboot,
                                  sizeof(unexpected_reboot))
    .andReturnValue(0);
  mock()
    .expectOneCall("memfault_metrics_heartbeat_add")
    .withParameterOfType("MemfaultMetricId", "key", &operational_crashfree_hours_key)
    .withParameter("amount", operational_hours)
    .andReturnValue(0);

  memfault_metrics_reliability_collect();
}
