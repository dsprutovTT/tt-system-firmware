/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tenstorrent_pldm_oem_handler

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pmci/pldm/pldm_oem_handler.h>

#include <errno.h>
#include <libpldm/base.h>
#include <string.h>

#include "pldm_base.h"

#define TT_PLDM_OEM_CMD_SHELL_EXEC       0x01U
#define TT_PLDM_OEM_CMD_SHELL_GET_RESULT 0x02U
#define TT_PLDM_OEM_CMD_SHELL_CANCEL     0x03U

enum tt_pldm_shell_state {
	TT_PLDM_SHELL_STATE_IDLE = 0,
	TT_PLDM_SHELL_STATE_QUEUED = 1,
	TT_PLDM_SHELL_STATE_RUNNING = 2,
	TT_PLDM_SHELL_STATE_DONE = 3,
	TT_PLDM_SHELL_STATE_FAILED = 4,
	TT_PLDM_SHELL_STATE_CANCELED = 5,
};

#define TT_PLDM_SHELL_WORKQ_STACK_SIZE 1024U
#define TT_PLDM_SHELL_MAX_CMD_LEN      96U
#define TT_PLDM_SHELL_MAX_OUTPUT_LEN   256U
#define TT_PLDM_SHELL_MAX_CHUNK_LEN    40U

struct tt_pldm_shell_context {
	struct k_mutex lock;
	struct k_work_q work_q;
	struct k_work work;

	K_KERNEL_STACK_MEMBER(work_q_stack, TT_PLDM_SHELL_WORKQ_STACK_SIZE);
	enum tt_pldm_shell_state state;
	bool cancel_requested;
	uint16_t request_id;
	char cmd[TT_PLDM_SHELL_MAX_CMD_LEN + 1U];
	size_t cmd_len;
	int32_t shell_rc;
	uint8_t output[TT_PLDM_SHELL_MAX_OUTPUT_LEN];
	size_t output_len;
};

static const ver32_t tt_pldm_oem_versions[] = {
	/* TT PLDM OEM version 1.0.0 + CRC32 over version entries. */
	{.alpha = 0x00, .update = 0xf0, .minor = 0xf0, .major = 0xf1},
	{.alpha = 0x2b, .update = 0x76, .minor = 0x8e, .major = 0x9a},
};

static const bitfield8_t tt_pldm_oem_commands[32] = {
	[TT_PLDM_OEM_CMD_SHELL_EXEC / 8] = {.byte = BIT(TT_PLDM_OEM_CMD_SHELL_EXEC % 8) |
						    BIT(TT_PLDM_OEM_CMD_SHELL_GET_RESULT % 8) |
						    BIT(TT_PLDM_OEM_CMD_SHELL_CANCEL % 8)},
};

static int tt_pldm_shell_encode_resp_header(const struct pldm_header_info *req_hdr,
					    struct pldm_msg *resp_msg)
{
	struct pldm_header_info resp_hdr = {
		.msg_type = PLDM_RESPONSE,
		.instance = req_hdr->instance,
		.pldm_type = req_hdr->pldm_type,
		.command = req_hdr->command,
	};
	uint8_t cc;

	cc = pack_pldm_header(&resp_hdr, &resp_msg->hdr);
	if (cc != PLDM_SUCCESS) {
		return -EINVAL;
	}

	return 0;
}

static void tt_pldm_shell_store_result(struct tt_pldm_shell_context *ctx,
				       enum tt_pldm_shell_state state, int shell_rc,
				       const char *output, size_t output_len)
{
	ctx->state = state;
	ctx->shell_rc = shell_rc;
	ctx->output_len = MIN(output_len, TT_PLDM_SHELL_MAX_OUTPUT_LEN);
	if (ctx->output_len > 0U && output != NULL) {
		memcpy(ctx->output, output, ctx->output_len);
	}
}

static void tt_pldm_shell_execute_work(struct k_work *work)
{
	struct tt_pldm_shell_context *ctx = CONTAINER_OF(work, struct tt_pldm_shell_context, work);
	char cmd[TT_PLDM_SHELL_MAX_CMD_LEN + 1U];
	const struct shell *sh;
	const char *output;
	size_t output_len = 0U;
	int rc;

	k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ctx->state != TT_PLDM_SHELL_STATE_QUEUED) {
		k_mutex_unlock(&ctx->lock);
		return;
	}

	if (ctx->cancel_requested) {
		tt_pldm_shell_store_result(ctx, TT_PLDM_SHELL_STATE_CANCELED, -ECANCELED, NULL, 0U);
		k_mutex_unlock(&ctx->lock);
		return;
	}

	ctx->state = TT_PLDM_SHELL_STATE_RUNNING;
	memcpy(cmd, ctx->cmd, ctx->cmd_len + 1U);
	k_mutex_unlock(&ctx->lock);

	sh = shell_backend_dummy_get_ptr();
	if (sh == NULL) {
		rc = -ENODEV;
		output = "shell backend unavailable\n";
		output_len = strlen(output);
	} else {
		shell_backend_dummy_clear_output(sh);
		rc = shell_execute_cmd(sh, cmd);
		output = shell_backend_dummy_get_output(sh, &output_len);
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ctx->cancel_requested) {
		tt_pldm_shell_store_result(ctx, TT_PLDM_SHELL_STATE_CANCELED, -ECANCELED, NULL, 0U);
	} else if (rc == 0) {
		tt_pldm_shell_store_result(ctx, TT_PLDM_SHELL_STATE_DONE, rc, output, output_len);
	} else {
		tt_pldm_shell_store_result(ctx, TT_PLDM_SHELL_STATE_FAILED, rc, output, output_len);
	}
	k_mutex_unlock(&ctx->lock);
}

static const ver32_t *tt_pldm_shell_get_versions(const struct device *dev, size_t *versions_size)
{
	ARG_UNUSED(dev);

	if (versions_size != NULL) {
		*versions_size = sizeof(tt_pldm_oem_versions);
	}

	return tt_pldm_oem_versions;
}

static const bitfield8_t *tt_pldm_shell_get_commands(const struct device *dev)
{
	ARG_UNUSED(dev);

	return tt_pldm_oem_commands;
}

static int tt_pldm_shell_handle_exec(struct tt_pldm_shell_context *ctx,
				     const struct pldm_header_info *req_hdr,
				     const struct pldm_msg *req_msg, size_t req_payload_len,
				     struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	const uint8_t *payload = req_msg->payload;
	uint8_t *resp_payload = resp_msg->payload;
	uint16_t request_id;
	uint8_t cmd_len;
	uint8_t cc = PLDM_SUCCESS;
	int rc;

	if (req_payload_len < 3U) {
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_LENGTH, resp_msg,
					     resp_pldm_len);
	}

	request_id = sys_get_le16(payload);
	cmd_len = payload[2];
	if (cmd_len == 0U || cmd_len > TT_PLDM_SHELL_MAX_CMD_LEN ||
	    req_payload_len != (size_t)(3U + cmd_len)) {
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_LENGTH, resp_msg,
					     resp_pldm_len);
	}

	rc = tt_pldm_shell_encode_resp_header(req_hdr, resp_msg);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ctx->state == TT_PLDM_SHELL_STATE_QUEUED || ctx->state == TT_PLDM_SHELL_STATE_RUNNING) {
		cc = PLDM_ERROR_NOT_READY;
	} else {
		ctx->request_id = request_id;
		ctx->cmd_len = cmd_len;
		memcpy(ctx->cmd, &payload[3], cmd_len);
		ctx->cmd[cmd_len] = '\0';
		ctx->cancel_requested = false;
		ctx->output_len = 0U;
		ctx->shell_rc = 0;
		ctx->state = TT_PLDM_SHELL_STATE_QUEUED;

		if (k_work_submit_to_queue(&ctx->work_q, &ctx->work) <= 0) {
			ctx->state = TT_PLDM_SHELL_STATE_IDLE;
			cc = PLDM_ERROR;
		}
	}

	resp_payload[0] = cc;
	sys_put_le16(request_id, &resp_payload[1]);
	resp_payload[3] = (uint8_t)ctx->state;
	k_mutex_unlock(&ctx->lock);

	*resp_pldm_len = sizeof(struct pldm_msg_hdr) + 4U;
	return 0;
}

static int tt_pldm_shell_handle_get_result(struct tt_pldm_shell_context *ctx,
					   const struct pldm_header_info *req_hdr,
					   const struct pldm_msg *req_msg, size_t req_payload_len,
					   struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	const uint8_t *payload = req_msg->payload;
	uint8_t *resp_payload = resp_msg->payload;
	uint16_t request_id;
	uint16_t offset;
	uint8_t max_len;
	enum tt_pldm_shell_state state;
	uint16_t total_len;
	uint8_t chunk_len = 0U;
	uint8_t more = 0U;
	int rc;

	if (req_payload_len != 5U) {
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_LENGTH, resp_msg,
					     resp_pldm_len);
	}

	request_id = sys_get_le16(payload);
	offset = sys_get_le16(&payload[2]);
	max_len = payload[4];

	rc = tt_pldm_shell_encode_resp_header(req_hdr, resp_msg);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ctx->state == TT_PLDM_SHELL_STATE_IDLE || ctx->request_id != request_id) {
		k_mutex_unlock(&ctx->lock);
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_DATA, resp_msg,
					     resp_pldm_len);
	}

	state = ctx->state;
	total_len = (uint16_t)MIN(ctx->output_len, UINT16_MAX);
	if (offset > total_len) {
		k_mutex_unlock(&ctx->lock);
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_DATA, resp_msg,
					     resp_pldm_len);
	}

	resp_payload[0] = PLDM_SUCCESS;
	sys_put_le16(request_id, &resp_payload[1]);
	resp_payload[3] = (uint8_t)state;
	sys_put_le32((uint32_t)ctx->shell_rc, &resp_payload[4]);
	sys_put_le16(total_len, &resp_payload[8]);
	sys_put_le16(offset, &resp_payload[10]);

	if (state == TT_PLDM_SHELL_STATE_DONE || state == TT_PLDM_SHELL_STATE_FAILED) {
		size_t allowed = TT_PLDM_SHELL_MAX_CHUNK_LEN;
		size_t remaining = total_len - offset;

		if (max_len != 0U) {
			allowed = MIN(allowed, max_len);
		}
		chunk_len = (uint8_t)MIN(remaining, allowed);
		if (chunk_len > 0U) {
			memcpy(&resp_payload[13], &ctx->output[offset], chunk_len);
		}
		more = (offset + chunk_len) < total_len ? 1U : 0U;
	} else if (state == TT_PLDM_SHELL_STATE_QUEUED || state == TT_PLDM_SHELL_STATE_RUNNING) {
		more = 1U;
	}

	resp_payload[12] = chunk_len;
	resp_payload[13 + chunk_len] = more;
	k_mutex_unlock(&ctx->lock);

	*resp_pldm_len = sizeof(struct pldm_msg_hdr) + 14U + chunk_len;
	return 0;
}

static int tt_pldm_shell_handle_cancel(struct tt_pldm_shell_context *ctx,
				       const struct pldm_header_info *req_hdr,
				       const struct pldm_msg *req_msg, size_t req_payload_len,
				       struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	const uint8_t *payload = req_msg->payload;
	uint8_t *resp_payload = resp_msg->payload;
	uint16_t request_id;
	uint8_t cc = PLDM_SUCCESS;
	int rc;

	if (req_payload_len != 2U) {
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_LENGTH, resp_msg,
					     resp_pldm_len);
	}

	request_id = sys_get_le16(payload);

	rc = tt_pldm_shell_encode_resp_header(req_hdr, resp_msg);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ctx->state == TT_PLDM_SHELL_STATE_IDLE || ctx->request_id != request_id) {
		k_mutex_unlock(&ctx->lock);
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_INVALID_DATA, resp_msg,
					     resp_pldm_len);
	}

	if (ctx->state == TT_PLDM_SHELL_STATE_QUEUED) {
		ctx->cancel_requested = true;
		tt_pldm_shell_store_result(ctx, TT_PLDM_SHELL_STATE_CANCELED, -ECANCELED, NULL, 0U);
	} else if (ctx->state == TT_PLDM_SHELL_STATE_RUNNING) {
		ctx->cancel_requested = true;
		cc = PLDM_ERROR_NOT_READY;
	}

	resp_payload[0] = cc;
	sys_put_le16(request_id, &resp_payload[1]);
	resp_payload[3] = (uint8_t)ctx->state;
	k_mutex_unlock(&ctx->lock);

	*resp_pldm_len = sizeof(struct pldm_msg_hdr) + 4U;
	return 0;
}

static int tt_pldm_shell_build_response(const struct device *dev,
					const struct pldm_header_info *req_hdr,
					const struct pldm_msg *req_msg, size_t req_payload_len,
					struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	struct tt_pldm_shell_context *ctx = dev->data;

	switch (req_hdr->command) {
	case TT_PLDM_OEM_CMD_SHELL_EXEC:
		return tt_pldm_shell_handle_exec(ctx, req_hdr, req_msg, req_payload_len, resp_msg,
						 resp_pldm_len);

	case TT_PLDM_OEM_CMD_SHELL_GET_RESULT:
		return tt_pldm_shell_handle_get_result(ctx, req_hdr, req_msg, req_payload_len,
						       resp_msg, resp_pldm_len);

	case TT_PLDM_OEM_CMD_SHELL_CANCEL:
		return tt_pldm_shell_handle_cancel(ctx, req_hdr, req_msg, req_payload_len, resp_msg,
						   resp_pldm_len);

	default:
		return pldm_cc_only_response(req_hdr, PLDM_ERROR_UNSUPPORTED_PLDM_CMD, resp_msg,
					     resp_pldm_len);
	}
}

static DEVICE_API(pldm_oem_handler, tt_pldm_shell_oem_api) = {
	.get_versions = tt_pldm_shell_get_versions,
	.get_commands = tt_pldm_shell_get_commands,
	.build_response = tt_pldm_shell_build_response,
};

static int tt_pldm_shell_driver_init(const struct device *dev)
{
	struct tt_pldm_shell_context *ctx = dev->data;

	k_mutex_init(&ctx->lock);
	k_work_init(&ctx->work, tt_pldm_shell_execute_work);
	k_work_queue_start(&ctx->work_q, ctx->work_q_stack,
			   K_KERNEL_STACK_SIZEOF(ctx->work_q_stack), K_PRIO_PREEMPT(1), NULL);
	ctx->state = TT_PLDM_SHELL_STATE_IDLE;

	return 0;
}

#define TT_PLDM_SHELL_DEFINE(inst)                                                                 \
	static struct tt_pldm_shell_context tt_pldm_shell_ctx_##inst;                              \
	DEVICE_DT_INST_DEFINE(inst, tt_pldm_shell_driver_init, NULL, &tt_pldm_shell_ctx_##inst,    \
			      NULL, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,                 \
			      &tt_pldm_shell_oem_api)

DT_INST_FOREACH_STATUS_OKAY(TT_PLDM_SHELL_DEFINE)
