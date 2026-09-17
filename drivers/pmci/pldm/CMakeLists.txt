# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PMCI_PLDM_PLATFORM pldm_platform.c)
zephyr_library_sources_ifdef(CONFIG_PMCI_PLDM_BASE pldm_base.c)
zephyr_library_sources_ifdef(CONFIG_PMCI_PLDM_PLATFORM pldm_pdr.c)
zephyr_library_sources_ifdef(CONFIG_TT_PMCI_PLDM_SHELL tt_pldm_oem.c)

zephyr_syscall_header(
  ${ZEPHYR_TT_SYSTEM_FIRMWARE_MODULE_DIR}/include/zephyr/drivers/pmci/pldm/pldm_oem_handler.h)
