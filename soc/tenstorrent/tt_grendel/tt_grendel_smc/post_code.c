/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <tenstorrent/post_code.h>
#include <tc_util_user_override.h>

void SetPostCode(uint8_t fw_id, uint16_t post_code)
{
	WRITE_SCRATCH(0, (POST_CODE_PREFIX << 16) | (fw_id << 14) | (post_code & 0x3FFF));
}
