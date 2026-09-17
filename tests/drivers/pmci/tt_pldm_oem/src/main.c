/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <errno.h>
#include <libpldm/base.h>
#include <zephyr/drivers/pmci/pldm/pldm_oem_handler.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RESP_BUF_PAYLOAD_MAX 320U

#define TT_PLDM_OEM_CMD_SHELL_EXEC       0x01U
#define TT_PLDM_OEM_CMD_SHELL_GET_RESULT 0x02U
#define TT_PLDM_OEM_CMD_SHELL_CANCEL     0x03U

#define TEST_OEM_NODE DT_NODELABEL(pldm_oem_test)

#define TT_PLDM_SHELL_STATE_IDLE     0U
#define TT_PLDM_SHELL_STATE_QUEUED   1U
#define TT_PLDM_SHELL_STATE_RUNNING  2U
#define TT_PLDM_SHELL_STATE_DONE     3U
#define TT_PLDM_SHELL_STATE_FAILED   4U
#define TT_PLDM_SHELL_STATE_CANCELED 5U

struct result_response {
	uint8_t cc;
	uint16_t request_id;
	uint8_t state;
	int32_t shell_rc;
	uint16_t total_len;
	uint16_t offset;
	uint8_t chunk_len;
	uint8_t more;
	const uint8_t *chunk;
};

static const struct device *const oem_dev = DEVICE_DT_GET(TEST_OEM_NODE);

int pldm_cc_only_response(const struct pldm_header_info *req_hdr, uint8_t cc,
			  struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	cc = encode_cc_only_resp(req_hdr->instance, req_hdr->pldm_type, req_hdr->command, cc,
				 resp_msg);
	if (cc != PLDM_SUCCESS) {
		return -EINVAL;
	}

	*resp_pldm_len = sizeof(struct pldm_msg_hdr) + 1U;
	return 0;
}

static void *suite_setup(void)
{
	zassert_true(device_is_ready(oem_dev));
	return NULL;
}

static struct pldm_header_info oem_req_hdr(uint8_t instance, uint8_t command)
{
	return (struct pldm_header_info){
		.msg_type = PLDM_REQUEST,
		.instance = instance,
		.pldm_type = PLDM_OEM_TYPE,
		.command = command,
	};
}

static void assert_response_header(const struct pldm_msg *resp_msg, uint8_t instance,
				   uint8_t command)
{
	struct pldm_header_info hdr;
	int rc;

	rc = unpack_pldm_header(&resp_msg->hdr, &hdr);
	zassert_equal(rc, PLDM_SUCCESS);
	zassert_equal(hdr.msg_type, PLDM_RESPONSE);
	zassert_equal(hdr.instance, instance);
	zassert_equal(hdr.pldm_type, PLDM_OEM_TYPE);
	zassert_equal(hdr.command, command);
}

static int send_exec_request(uint8_t instance, uint16_t request_id, const char *cmd,
			     uint8_t cmd_len, struct pldm_msg *resp_msg, size_t *resp_pldm_len)
{
	PLDM_MSG_BUFFER(req_buf, 3U + 96U);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, TT_PLDM_OEM_CMD_SHELL_EXEC);
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE,
				     TT_PLDM_OEM_CMD_SHELL_EXEC, req_msg);
	if (rc != PLDM_SUCCESS) {
		return -EINVAL;
	}

	sys_put_le16(request_id, &req_msg->payload[0]);
	req_msg->payload[2] = cmd_len;
	if (cmd_len > 0U && cmd != NULL) {
		memcpy(&req_msg->payload[3], cmd, cmd_len);
	}

	return pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 3U + cmd_len, resp_msg,
					       resp_pldm_len);
}

static int send_get_result_request(uint8_t instance, uint16_t request_id, uint16_t offset,
				   uint8_t max_len, struct pldm_msg *resp_msg,
				   size_t *resp_pldm_len)
{
	PLDM_MSG_BUFFER(req_buf, 5U);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE,
				     TT_PLDM_OEM_CMD_SHELL_GET_RESULT, req_msg);
	if (rc != PLDM_SUCCESS) {
		return -EINVAL;
	}

	sys_put_le16(request_id, &req_msg->payload[0]);
	sys_put_le16(offset, &req_msg->payload[2]);
	req_msg->payload[4] = max_len;

	return pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 5U, resp_msg,
					       resp_pldm_len);
}

static int parse_result_response(const struct pldm_msg *resp_msg, size_t resp_pldm_len,
				 struct result_response *decoded)
{
	size_t payload_len;

	if (resp_pldm_len < sizeof(struct pldm_msg_hdr) + 14U) {
		return -EINVAL;
	}

	payload_len = resp_pldm_len - sizeof(struct pldm_msg_hdr);

	decoded->cc = resp_msg->payload[0];
	decoded->request_id = sys_get_le16(&resp_msg->payload[1]);
	decoded->state = resp_msg->payload[3];
	decoded->shell_rc = (int32_t)sys_get_le32(&resp_msg->payload[4]);
	decoded->total_len = sys_get_le16(&resp_msg->payload[8]);
	decoded->offset = sys_get_le16(&resp_msg->payload[10]);
	decoded->chunk_len = resp_msg->payload[12];

	if (payload_len != (size_t)(14U + decoded->chunk_len)) {
		return -EINVAL;
	}

	decoded->chunk = &resp_msg->payload[13];
	decoded->more = resp_msg->payload[13U + decoded->chunk_len];

	return 0;
}

ZTEST(tt_pldm_oem, test_versions_get_has_expected_values)
{
	size_t versions_size = 0U;
	const ver32_t *versions = pldm_oem_handler_get_versions(oem_dev, &versions_size);

	zassert_not_null(versions);
	zassert_equal(versions_size, 2U * sizeof(ver32_t));
	zassert_equal(versions[0].major, 0xf1U);
	zassert_equal(versions[0].minor, 0xf0U);
	zassert_equal(versions[0].update, 0xf0U);
	zassert_equal(versions[0].alpha, 0x00U);
}

ZTEST(tt_pldm_oem, test_commands_get_contains_shell_commands)
{
	const bitfield8_t *commands = pldm_oem_handler_get_commands(oem_dev);

	zassert_not_null(commands);
	zassert_true((commands[TT_PLDM_OEM_CMD_SHELL_EXEC / 8].byte &
		      BIT(TT_PLDM_OEM_CMD_SHELL_EXEC % 8)) != 0U);
	zassert_true((commands[TT_PLDM_OEM_CMD_SHELL_GET_RESULT / 8].byte &
		      BIT(TT_PLDM_OEM_CMD_SHELL_GET_RESULT % 8)) != 0U);
	zassert_true((commands[TT_PLDM_OEM_CMD_SHELL_CANCEL / 8].byte &
		      BIT(TT_PLDM_OEM_CMD_SHELL_CANCEL % 8)) != 0U);
}

ZTEST(tt_pldm_oem, test_exec_rejects_invalid_length)
{
	PLDM_MSG_BUFFER(req_buf, 2U);
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 1U;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, TT_PLDM_OEM_CMD_SHELL_EXEC);
	size_t resp_pldm_len;
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE,
				     TT_PLDM_OEM_CMD_SHELL_EXEC, req_msg);
	zassert_equal(rc, PLDM_SUCCESS);

	rc = pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 2U, resp_msg,
					     &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_EXEC);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_LENGTH);
}

ZTEST(tt_pldm_oem, test_get_result_rejects_invalid_length)
{
	PLDM_MSG_BUFFER(req_buf, 5U);
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 6U;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	size_t resp_pldm_len;
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE,
				     TT_PLDM_OEM_CMD_SHELL_GET_RESULT, req_msg);
	zassert_equal(rc, PLDM_SUCCESS);

	rc = pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 4U, resp_msg,
					     &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_LENGTH);
}

ZTEST(tt_pldm_oem, test_get_result_when_idle_returns_invalid_data)
{
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 7U;
	size_t resp_pldm_len;
	int rc;

	rc = send_get_result_request(instance, 0x2222U, 0U, 40U, resp_msg, &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_DATA);
}

ZTEST(tt_pldm_oem, test_get_result_rejects_mismatched_request_id)
{
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 8U;
	const uint16_t request_id = 0x3333U;
	const char *cmd = "help";
	const uint8_t cmd_len = 4U;
	size_t resp_pldm_len;
	int rc;

	rc = send_exec_request(instance, request_id, cmd, cmd_len, resp_msg, &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_msg->payload[0], PLDM_SUCCESS);

	rc = send_get_result_request(instance, request_id + 1U, 0U, 40U, resp_msg, &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_DATA);
}

ZTEST(tt_pldm_oem, test_cancel_when_idle_returns_invalid_data)
{
	PLDM_MSG_BUFFER(req_buf, 2U);
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 2U;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, TT_PLDM_OEM_CMD_SHELL_CANCEL);
	size_t resp_pldm_len;
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE,
				     TT_PLDM_OEM_CMD_SHELL_CANCEL, req_msg);
	zassert_equal(rc, PLDM_SUCCESS);
	sys_put_le16(0x1111U, &req_msg->payload[0]);

	rc = pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 2U, resp_msg,
					     &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_CANCEL);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_DATA);
}

ZTEST(tt_pldm_oem, test_exec_and_get_result_success)
{
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 3U;
	const uint16_t request_id = 0x4040U;
	const char *cmd = "help";
	const uint8_t cmd_len = 4U;
	uint8_t output[RESP_BUF_PAYLOAD_MAX] = {0};
	struct result_response decoded = {0};
	size_t total_copied = 0U;
	size_t resp_pldm_len;
	uint16_t offset = 0U;
	int rc;

	rc = send_exec_request(instance, request_id, cmd, cmd_len, resp_msg, &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 4U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_EXEC);
	zassert_equal(resp_msg->payload[0], PLDM_SUCCESS);
	zassert_equal(sys_get_le16(&resp_msg->payload[1]), request_id);

	for (int i = 0; i < 100; ++i) {
		rc = send_get_result_request(instance, request_id, offset, 40U, resp_msg,
					     &resp_pldm_len);
		zassert_equal(rc, 0);
		assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);

		rc = parse_result_response(resp_msg, resp_pldm_len, &decoded);
		zassert_equal(rc, 0);
		zassert_equal(decoded.cc, PLDM_SUCCESS);
		zassert_equal(decoded.request_id, request_id);

		if (decoded.state == TT_PLDM_SHELL_STATE_DONE ||
		    decoded.state == TT_PLDM_SHELL_STATE_FAILED ||
		    decoded.state == TT_PLDM_SHELL_STATE_CANCELED) {
			if (decoded.chunk_len > 0U) {
				size_t to_copy = MIN((size_t)decoded.chunk_len,
						     sizeof(output) - total_copied);

				memcpy(&output[total_copied], decoded.chunk, to_copy);
				total_copied += to_copy;
				offset += decoded.chunk_len;
			}

			if (decoded.more == 0U) {
				break;
			}
		} else {
			k_msleep(10);
		}
	}

	zassert_equal(decoded.state, TT_PLDM_SHELL_STATE_DONE);
	zassert_equal(decoded.shell_rc, 0);
	zassert_equal(total_copied, decoded.total_len);
	zassert_true(decoded.total_len > 0U);
	zassert_true(total_copied < sizeof(output));
	zassert_equal(decoded.more, 0U);
}

ZTEST(tt_pldm_oem, test_get_result_rejects_invalid_offset)
{
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 4U;
	const uint16_t request_id = 0x5151U;
	const char *cmd = "help";
	const uint8_t cmd_len = 4U;
	struct result_response decoded = {0};
	size_t resp_pldm_len;
	uint16_t total_len = 0U;
	int rc;

	rc = send_exec_request(instance, request_id, cmd, cmd_len, resp_msg, &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_msg->payload[0], PLDM_SUCCESS);

	for (int i = 0; i < 100; ++i) {
		rc = send_get_result_request(instance, request_id, 0U, 40U, resp_msg,
					     &resp_pldm_len);
		zassert_equal(rc, 0);

		rc = parse_result_response(resp_msg, resp_pldm_len, &decoded);
		zassert_equal(rc, 0);
		total_len = decoded.total_len;
		if (decoded.state == TT_PLDM_SHELL_STATE_DONE ||
		    decoded.state == TT_PLDM_SHELL_STATE_FAILED ||
		    decoded.state == TT_PLDM_SHELL_STATE_CANCELED) {
			break;
		}
		k_msleep(10);
	}

	rc = send_get_result_request(instance, request_id, total_len + 1U, 40U, resp_msg,
				     &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, TT_PLDM_OEM_CMD_SHELL_GET_RESULT);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_INVALID_DATA);
}

ZTEST(tt_pldm_oem, test_unsupported_command_returns_error)
{
	PLDM_MSG_BUFFER(req_buf, 0U);
	PLDM_MSG_BUFFER(resp_buf, RESP_BUF_PAYLOAD_MAX);
	struct pldm_msg *req_msg = (struct pldm_msg *)req_buf;
	struct pldm_msg *resp_msg = (struct pldm_msg *)resp_buf;
	const uint8_t instance = 5U;
	const uint8_t unsupported_cmd = 0x7fU;
	struct pldm_header_info req_hdr = oem_req_hdr(instance, unsupported_cmd);
	size_t resp_pldm_len;
	int rc;

	rc = encode_pldm_header_only(PLDM_REQUEST, instance, PLDM_OEM_TYPE, unsupported_cmd,
				     req_msg);
	zassert_equal(rc, PLDM_SUCCESS);

	rc = pldm_oem_handler_build_response(oem_dev, &req_hdr, req_msg, 0U, resp_msg,
					     &resp_pldm_len);
	zassert_equal(rc, 0);
	zassert_equal(resp_pldm_len, sizeof(struct pldm_msg_hdr) + 1U);

	assert_response_header(resp_msg, instance, unsupported_cmd);
	zassert_equal(resp_msg->payload[0], PLDM_ERROR_UNSUPPORTED_PLDM_CMD);
}

ZTEST_SUITE(tt_pldm_oem, NULL, suite_setup, NULL, NULL, NULL);
