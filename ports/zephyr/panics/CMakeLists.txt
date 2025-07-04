if (CONFIG_RISCV)
    zephyr_library_sources(memfault_fault_handler_riscv.c)
elseif(CONFIG_ARM)
    zephyr_library_sources(memfault_fault_handler.c)
elseif(CONFIG_XTENSA)
    zephyr_library_sources(memfault_fault_handler_xtensa.c)

    # Due to variable stack frame widths, Zephyr does not give a definition for
    # z_arch_esf_t/struct arch_esf (stack information passed to Zephyr's fault
    # handler) for Xtensa, unlike the other architectures our Zephyr port
    # supports. Therefore, we have to add an include path to the folder where
    # the header defining xtensa stack frames lives.
    zephyr_include_directories(${ZEPHYR_BASE}/arch/xtensa/include)
elseif(CONFIG_ARCH_POSIX)
    zephyr_library_sources(memfault_fault_handler_posix.c)
else()
    # Unsupported configuration
    message(FATAL_ERROR "Unsupported chip architecture")
endif()

zephyr_include_directories(.)

# Zephyr fatals for ARM Cortex-M's take the following path:
#
# frame 3 k_sys_fatal_error_handler    nrf/lib/fatal_error/fatal_error.c
#                                      (overrides weak impl in zephyr/kernel/fatal.c)
#  frame 2 z_fatal_error               zephyr/kernel/fatal.c
#   frame 1 z_arm_fault                zephyr/arch/arm/core/aarch32/cortex_m/fault.c
#    frame 0 z_arm_hard_fault          zephyr/arch/arm/core/aarch32/cortex_m/fault_s.S
#
# The nrf-connect-sdk does not expose a mechanism to override "k_sys_fatal_error_handler"
# and simply reboots the system so instead we intercept the frame above and install the
# Memfault fault handler which will collect a coredump.
target_link_libraries(app INTERFACE "-Wl,--wrap=z_fatal_error")

# We trace task creation and deletion so the task TCBs and stacks can be collected at the time of a
# crash and all thread backtraces can be made available in the Memfault UI.
#
# To do this we wrap the arch_new_thread() function call that is made from thread.c
#
# A nicer way to do this would be to make use of the "sys_trace_thread_create" macro.
# Unfortunately, to override the macro, one must patch the Zephyr RTOS today.
#   https://github.com/zephyrproject-rtos/zephyr/blob/390537b/include/tracing/tracing.h#L57-L61
target_link_libraries(app INTERFACE "-Wl,--wrap=arch_new_thread")
target_link_libraries(app INTERFACE "-Wl,--wrap=z_thread_abort")
