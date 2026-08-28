//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! MRAM backed coredump storage implementation. To make use of this, be sure to
//! add a fixed partition named "memfault_coredump_partition" to your device
//! tree or use partition manager to define one.
//!
//! For example, if using partition manager, you might add an entry like this to
//! the application's pm_static.yml file:
//!
//!  memfault_coredump_partition:
//!    address: 0x155000
//!    end_address: 0x165000
//!    placement:
//!      align:
//!        start: 0x1000
//!      before:
//!      - end
//!    region: flash_primary
//!    size: 0x10000
//!
//! If using device tree specified partitions, you might add something like this
//! to your board's dts file/overlay:
//!
//!  &mram1x {
//!    partitions {
//!      memfault_coredump_partition: partition@1a9000 {
//!        compatible = "zephyr,mapped-partition";
//!        reg = <0x1a9000 DT_SIZE_K(20)>;
//!      };
//!    };
//!  };

#include <memfault/components.h>
#include <memfault/ports/zephyr/version.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/barrier.h>

//! nRF54H20 MRAM has a 16-byte write block size
#define MEMFAULT_COREDUMP_STORAGE_WRITE_SIZE (16)
#include <memfault/ports/buffered_coredump_storage.h>

//! Bound on how many times an MRAM word write is re-issued while waiting for the
//! MRAMC to commit it. Mirrors CONFIG_NRF_MRAM_MAX_RETRIES in soc_flash_nrf_mram.c.
#define MEMFAULT_COREDUMP_MRAM_WRITE_RETRIES (20)

// PARTITION_* macros were introduced in Zephyr 4.4.0, replacing
// FIXED_PARTITION_* (and supporting mapped-partition). Provide a fallback for
// previous Zephyr versions (fixed-partition)
#if MEMFAULT_ZEPHYR_VERSION_GTE_STRICT(4, 4)
  #define MEMFAULT_PARTITION_EXISTS(label) PARTITION_EXISTS(label)
  #define MEMFAULT_PARTITION_OFFSET(label) PARTITION_OFFSET(label)
  #define MEMFAULT_PARTITION_SIZE(label) PARTITION_SIZE(label)
  #define MEMFAULT_PARTITION_DEVICE(label) PARTITION_DEVICE(label)
#else
  #define MEMFAULT_PARTITION_EXISTS(label) FIXED_PARTITION_EXISTS(label)
  #define MEMFAULT_PARTITION_OFFSET(label) FIXED_PARTITION_OFFSET(label)
  #define MEMFAULT_PARTITION_SIZE(label) FIXED_PARTITION_SIZE(label)
  #define MEMFAULT_PARTITION_DEVICE(label) FIXED_PARTITION_DEVICE(label)
#endif

// Ensure the memfault_coredump_partition entry exists
#if !MEMFAULT_PARTITION_EXISTS(memfault_coredump_partition)
  #error "Be sure to add a fixed partition named 'memfault_coredump_partition'!"
#endif

#if defined(CONFIG_SOC_NRF54H20) || defined(CONFIG_SOC_SERIES_NRF92)
  // DT_CHOSEN(zephyr_flash) (mram1x) is the addressable memory range, not the driver-backed
  // device (mram1x_controller, its DT parent) - resolve the device via the partition instead.
  #define MEMFAULT_COREDUMP_FLASH_DEVICE MEMFAULT_PARTITION_DEVICE(memfault_coredump_partition)
  #define MEMFAULT_COREDUMP_PARTITION_OFFSET MEMFAULT_PARTITION_OFFSET(memfault_coredump_partition)
  #define MEMFAULT_COREDUMP_PARTITION_SIZE MEMFAULT_PARTITION_SIZE(memfault_coredump_partition)
  // MRAM is memory mapped, so the coredump partition can also be written to directly at this
  // absolute address (see memfault_platform_coredump_storage_buffered_write() below).
  #define MEMFAULT_COREDUMP_STORAGE_ADDR \
    (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + MEMFAULT_COREDUMP_PARTITION_OFFSET)
#else
  #error "Unknown SOC, please contact support: https://mflt.io/contact-support"
#endif

MEMFAULT_STATIC_ASSERT(MEMFAULT_COREDUMP_PARTITION_SIZE % MEMFAULT_COREDUMP_STORAGE_WRITE_SIZE == 0,
                       "Storage size must be a multiple of (16 bytes)");

void memfault_platform_coredump_storage_get_info(sMfltCoredumpStorageInfo *info) {
  *info = (sMfltCoredumpStorageInfo){
    .size = MEMFAULT_COREDUMP_PARTITION_SIZE,
  };
}

static bool prv_op_within_flash_bounds(uint32_t offset, size_t data_len) {
  sMfltCoredumpStorageInfo info = { 0 };

  memfault_platform_coredump_storage_get_info(&info);
  return (offset + data_len) <= info.size;
}

bool memfault_platform_coredump_storage_read(uint32_t offset, void *data, size_t read_len) {
  if (!prv_op_within_flash_bounds(offset, read_len)) {
    return false;
  }

  // special case: if the first word is 0, the coredump is cleared, and reads
  // should return all zeros
  uint32_t first_word = 0xffffffff;
  (int)flash_read(MEMFAULT_COREDUMP_FLASH_DEVICE, MEMFAULT_COREDUMP_PARTITION_OFFSET, &first_word,
                  sizeof(first_word));
  if (first_word == 0) {
    memset(data, 0, read_len);
    return true;
  }

  // read the data
  (int)flash_read(MEMFAULT_COREDUMP_FLASH_DEVICE, MEMFAULT_COREDUMP_PARTITION_OFFSET + offset, data,
                  read_len);
  return true;
}

bool memfault_platform_coredump_storage_erase(uint32_t offset, size_t erase_size) {
  // Erase is only ever called just prior to writing a new coredump or in debug
  // testing, so we only need to wipe the first 32 bits. We'll overwrite the
  // first unit of write size for simplicity.
  uint8_t erase_buf[sizeof(((sCoredumpWorkingBuffer *)0)->data)] = { 0 };

  return memfault_platform_coredump_storage_write(offset, erase_buf, sizeof(erase_buf));
}

//! Writes 16-byte-aligned MRAM words directly, bypassing flash_write()/the
//! nrf_mram driver.
//!
//! Zephyr's nrf_mram_write()/nrf_mram_erase() now take such a mutex (see
//! https://github.com/zephyrproject-rtos/zephyr/commit/f2df5fe20d9b3295a38720cc55ffb102ed14b4ae),
//! so flash_write() unfortunately cannot be used here. MRAM is memory mapped,
//! so write directly instead.
//!
//! A word write is internally an erase followed by a program, so read the word
//! back and retry until it matches: a reset landing mid-commit would otherwise
//! leave a stale/erased word behind. Bounded so a stuck controller cannot hang
//! the fault handler.
static bool prv_mram_write_words(uintptr_t addr, const void *data, size_t len) {
  volatile uint32_t *dst = (volatile uint32_t *)addr;
  const uint32_t *src = (const uint32_t *)data;
  const size_t words = len / sizeof(uint32_t);

  for (unsigned int retry = 0; retry < MEMFAULT_COREDUMP_MRAM_WRITE_RETRIES; retry++) {
    bool committed = true;

    for (size_t i = 0; i < words; i++) {
      dst[i] = src[i];
    }

    barrier_dsync_fence_full();

    for (size_t i = 0; i < words; i++) {
      if (dst[i] != src[i]) {
        committed = false;
        break;
      }
    }

    if (committed) {
      return true;
    }
  }

  return false;
}

bool memfault_platform_coredump_storage_buffered_write(sCoredumpWorkingBuffer *blk) {
  if (!prv_op_within_flash_bounds(blk->write_offset, sizeof(blk->data))) {
    return false;
  }

  return prv_mram_write_words(MEMFAULT_COREDUMP_STORAGE_ADDR + blk->write_offset, blk->data,
                              sizeof(blk->data));
}

//! Primarily used for debug. Call erase to wipe out the coredump magic.
//!
//! Note: unlike memfault_platform_coredump_storage_erase(), this runs from
//! normal thread context (after a successful upload), so it's safe - and
//! preferable, for the mutex serialization and bus-fault verification it
//! provides - to go through flash_write() here rather than the direct MRAM
//! write used by memfault_platform_coredump_storage_buffered_write().
void memfault_platform_coredump_storage_clear(void) {
  uint8_t erase_buf[sizeof(((sCoredumpWorkingBuffer *)0)->data)] = { 0 };

  (int)flash_write(MEMFAULT_COREDUMP_FLASH_DEVICE, MEMFAULT_COREDUMP_PARTITION_OFFSET, erase_buf,
                   sizeof(erase_buf));
}
