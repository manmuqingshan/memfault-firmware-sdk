zephyr_library_sources(memfault_platform_debug_log.c)
zephyr_library_sources(memfault_platform_core.c)
zephyr_library_sources(memfault_zephyr_ram_regions.c)
zephyr_library_sources(memfault_platform_lock.c)
zephyr_library_sources(memfault_platform_coredump_regions.c)
zephyr_library_sources(memfault_platform_metrics.c)
zephyr_library_sources(memfault_platform_system_time.c)

zephyr_library_sources_ifdef(CONFIG_MEMFAULT_HTTP_ENABLE memfault_platform_http.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_HTTP_ENABLE memfault_platform_fota.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_METRICS_THREADS memfault_platform_thread_metrics.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_RAM_BACKED_COREDUMP memfault_platform_ram_backed_coredump.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_ROOT_CERT_STORAGE_TLS_CREDENTIAL_STORAGE memfault_tls_root_cert_storage.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_SHELL memfault_demo_cli.c)
zephyr_library_sources_ifdef(CONFIG_MEMFAULT_SHELL_SELF_TEST memfault_self_test_platform.c)

if (NOT CONFIG_MEMFAULT_SOFTWARE_WATCHDOG_CUSTOM)
  zephyr_library_sources(memfault_software_watchdog.c)
endif()

if (CONFIG_MEMFAULT_CACHE_FAULT_REGS)
  # We need to wrap z_arm_fault() so we can get a copy
  # of the fault registers before Zephyr consumes and
  # clears the CFSR. See ports/zephyr/common/memfault_platform_core.c.
  zephyr_ld_options(-Wl,--wrap=z_arm_fault)
endif()

if (CONFIG_MEMFAULT_METRICS_BOOT_TIME)
  zephyr_ld_options(-Wl,--wrap=main)
endif()

# Allow users to Kconfig memfault logging as a backend as needed. See
# memfault/sdk/embedded/ports/zephyr/Kconfig and adjust MEMFAULT_LOGGING_XXX.
if (CONFIG_MEMFAULT_LOGGING_ENABLE)
  # Version 2 logging was first introduced in
  #  https://github.com/zephyrproject-rtos/zephyr/commit/f6a40ae183ed0781ffcdeac4ba9263116247a750
  #  and launched as part of the Zephyr v2.6 release
  #    https://github.com/zephyrproject-rtos/zephyr/pull/31535
  # It fully replaced version 1 after the 3.1 release: https://github.com/zephyrproject-rtos/zephyr/issues/46500
  # and the option to enable logging v1 via CONFIG_LOG1 was entirely removed
  if(CONFIG_LOG_MODE_MINIMAL)
    target_link_libraries(app INTERFACE "-Wl,--wrap=z_log_minimal_printk")
    zephyr_library_sources(memfault_logging_minimal.c)
  elseif(${KERNEL_VERSION_MAJOR}.${KERNEL_VERSION_MINOR}.${KERNEL_PATCHLEVEL} VERSION_GREATER_EQUAL "3.1.99" OR CONFIG_LOG2)
    zephyr_library_sources(memfault_logging.c)
  else()
    zephyr_library_sources(memfault_logging_legacy.c)
  endif()
endif()

zephyr_library_sources_ifdef(CONFIG_MEMFAULT_HTTP_PERIODIC_UPLOAD memfault_http_periodic_upload.c)

# Make sure RAM based storage regions are not initialized on boot
# by placing them in special linker sections
zephyr_linker_sources(NOINIT memfault-no-init.ld)

if (CONFIG_MEMFAULT_GNU_BUILD_ID_SOURCE_BUILTIN)
  zephyr_linker_sources(SECTIONS memfault-build-id.ld)
  # Override the default Zephyr setting which disables the GNU Build ID
  #   https://github.com/zephyrproject-rtos/zephyr/blob/d7ee114106eab485688223d97a49813d33b4cf21/cmake/linker/ld/target_base.cmake#L16
  zephyr_ld_options("-Wl,--build-id")
endif()

if (CONFIG_MEMFAULT_COMPACT_LOG)
  # Enable support for compact logs by adding the required linker section
  zephyr_linker_sources(SECTIONS memfault-compact-log.ld)
  # Compact logs requires a minimum C standard of gnu11, due to use of GNU
  # extensions and C11 features. Zephyr 3.7.0+ adds a Kconfig-native setting to
  # enable this. If unset, fallback to deprecated global CSTD property (for
  # pre-3.7.0).
  if (NOT (CONFIG_REQUIRES_STD_C11 AND CONFIG_GNU_C_EXTENSIONS))
    set_property(GLOBAL PROPERTY CSTD gnu11)
  endif()
endif()

if(CONFIG_MEMFAULT_HEAP_STATS AND CONFIG_HEAP_MEM_POOL_SIZE GREATER 0)
  zephyr_ld_options(-Wl,--wrap=k_malloc)
  zephyr_ld_options(-Wl,--wrap=k_free)
endif()

if(CONFIG_MEMFAULT_SOC_FAMILY_ESP32)
  zephyr_linker_sources(SECTIONS memfault-rtc-noinit-region.ld)
endif()

zephyr_include_directories(.)
