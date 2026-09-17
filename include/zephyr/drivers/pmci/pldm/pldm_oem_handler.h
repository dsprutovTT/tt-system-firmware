/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PLDM_OEM_HANDLER_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PLDM_OEM_HANDLER_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <libpldm/base.h>

#ifdef __cplusplus
extern "C" {
#endif

/** PLDM OEM type number (0x3F per DSP0240). */
#define PLDM_OEM_TYPE 0x3FU

/**
 * @brief Driver API for PLDM OEM command handlers.
 *
 * A driver that registers with compatible "tenstorrent,pldm-oem-handler"
 * implements this API so that the PLDM responder and base layers can route
 * OEM-type messages without a compile-time dependency on the concrete handler.
 */
__subsystem struct pldm_oem_handler_driver_api {
	/**
	 * Return the supported OEM version table.
	 *
	 * @param dev          Driver instance.
	 * @param versions_size Set to the byte length of the returned array.
	 * @return Pointer to the version array, or NULL on error.
	 */
	const ver32_t *(*get_versions)(const struct device *dev, size_t *versions_size);

	/**
	 * Return the supported OEM command bitmap (32-byte array of bitfield8_t).
	 *
	 * @param dev Driver instance.
	 * @return Pointer to the commands bitmap, or NULL on error.
	 */
	const bitfield8_t *(*get_commands)(const struct device *dev);

	/**
	 * Build a PLDM response for an OEM-type request.
	 *
	 * @param dev            Driver instance.
	 * @param req_hdr        Decoded request header.
	 * @param req_msg        Raw request message (including header).
	 * @param req_payload_len Payload length (bytes after the PLDM header).
	 * @param resp_msg       Output buffer for the response.
	 * @param resp_pldm_len  Set to the response length on success.
	 * @return 0 on success, negative errno on failure.
	 */
	int (*build_response)(const struct device *dev, const struct pldm_header_info *req_hdr,
			      const struct pldm_msg *req_msg, size_t req_payload_len,
			      struct pldm_msg *resp_msg, size_t *resp_pldm_len);
};

static inline const ver32_t *pldm_oem_handler_get_versions(const struct device *dev,
							   size_t *versions_size)
{
	if (!device_is_ready(dev)) {
		return NULL;
	}

	const struct pldm_oem_handler_driver_api *api = DEVICE_API_GET(pldm_oem_handler, dev);

	if (api->get_versions == NULL) {
		return NULL;
	}

	return api->get_versions(dev, versions_size);
}

static inline const bitfield8_t *pldm_oem_handler_get_commands(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		return NULL;
	}

	const struct pldm_oem_handler_driver_api *api = DEVICE_API_GET(pldm_oem_handler, dev);

	if (api->get_commands == NULL) {
		return NULL;
	}

	return api->get_commands(dev);
}

static inline int pldm_oem_handler_build_response(const struct device *dev,
						  const struct pldm_header_info *req_hdr,
						  const struct pldm_msg *req_msg,
						  size_t req_payload_len, struct pldm_msg *resp_msg,
						  size_t *resp_pldm_len)
{
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	const struct pldm_oem_handler_driver_api *api = DEVICE_API_GET(pldm_oem_handler, dev);

	if (api->build_response == NULL) {
		return -ENOSYS;
	}

	return api->build_response(dev, req_hdr, req_msg, req_payload_len, resp_msg, resp_pldm_len);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_PLDM_OEM_HANDLER_H_ */
