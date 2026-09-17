# SPDX-License-Identifier: Apache-2.0

if(NOT (CONFIG_BOARD_TT_MMK_TT_KERAUNOS_SMC OR CONFIG_BOARD_TT_MMK_TT_MIMIR_SMC))
  return()
endif()

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UART_TT_VIRT src/vuart.c)
