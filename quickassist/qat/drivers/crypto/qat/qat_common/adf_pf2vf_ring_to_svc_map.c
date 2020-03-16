/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 * Copyright(c) 2019 Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * qat-linux@intel.com
 *
 * BSD LICENSE
 * Copyright(c) 2019 Intel Corporation.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "adf_common_drv.h"
#include "adf_pf2vf_msg.h"
#include "adf_cfg.h"

#define ADF_VF2PF_RING_TO_SVC_VERSION 1
#define ADF_VF2PF_RING_TO_SVC_LENGTH  2

int adf_pf_ring_to_svc_msg_provider(struct adf_accel_dev *accel_dev,
				    u8 **buffer, u8 *length,
				    u8 *block_version, u8 compatibility,
				    u8 byte_num)
{
	static u8 data[ADF_VF2PF_RING_TO_SVC_LENGTH] = {0};
	struct adf_hw_device_data *hw_data = GET_HW_DATA(accel_dev);
	u16 ring_to_svc_map = hw_data->ring_to_svc_map;
	u16 byte = 0;

	for (byte = 0; byte < ADF_VF2PF_RING_TO_SVC_LENGTH; byte++) {
		data[byte] =
			(ring_to_svc_map >> (byte * ADF_PFVF_DATA_SHIFT))
				& ADF_PFVF_DATA_MASK;
	}

	*length = ADF_VF2PF_RING_TO_SVC_LENGTH;
	*block_version = ADF_VF2PF_RING_TO_SVC_VERSION;
	*buffer = data;

	return 0;
}

int adf_pf_vf_ring_to_svc_init(struct adf_accel_dev *accel_dev)
{
	u8 data[ADF_VF2PF_RING_TO_SVC_LENGTH] = {0};
	u8 len = ADF_VF2PF_RING_TO_SVC_LENGTH;
	u8 version = ADF_VF2PF_RING_TO_SVC_VERSION;
	u16 ring_to_svc_map  = 0;
	u16 byte = 0;

	if (!accel_dev->is_vf) {
		/* on the pf */
		if (!adf_iov_is_block_provider_registered(
		    ADF_VF2PF_BLOCK_MSG_GET_RING_TO_SVC_REQ))
			adf_iov_block_provider_register(
			ADF_VF2PF_BLOCK_MSG_GET_RING_TO_SVC_REQ,
			adf_pf_ring_to_svc_msg_provider);
	} else  if (accel_dev->vf.pf_version >=
		    ADF_PFVF_COMPATIBILITY_RING_TO_SVC_MAP) {
		/* on the vf */
		if (adf_iov_block_get(
			accel_dev, ADF_VF2PF_BLOCK_MSG_GET_RING_TO_SVC_REQ,
			&version, data, &len)) {
			dev_err(&GET_DEV(accel_dev),
				"QAT: Failed adf_iov_block_get\n");
			return -EFAULT;
		}
		for (byte = 0; byte < ADF_VF2PF_RING_TO_SVC_LENGTH; byte++) {
			ring_to_svc_map |=
				data[byte] << (byte * ADF_PFVF_DATA_SHIFT);
		}
		GET_HW_DATA(accel_dev)->ring_to_svc_map = ring_to_svc_map;
	}

	return 0;
}
