/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2019 Intel Corporation.
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Contact Information:
 *
 *  qat-linux@intel.com
 *
 *  BSD LICENSE
 *  Copyright(c) 2019 Intel Corporation.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "adf_common_drv.h"
#include "adf_sla.h"
#include "adf_du.h"
#include "adf_du_user.h"
#include "icp_qat_fw_init_admin.h"

static int adf_du_get_active_dev(struct adf_accel_dev **accel_dev,
				 struct adf_pci_address *pci_addr)
{
	*accel_dev = adf_devmgr_get_dev_by_bdf(pci_addr);
	if (!*accel_dev) {
		pr_err("QAT: Device bdf:%.2x:%.2x:%x not found\n",
		       pci_addr->bus, pci_addr->dev, pci_addr->func);
		return -EINVAL;
	}
	if (!adf_dev_started(*accel_dev)) {
		dev_err(&GET_DEV(*accel_dev), "Device not yet started.\n");
		goto exit;
	}

	return 0;

exit:
	adf_dev_put(*accel_dev);

	return -EFAULT;
}

static u16 adf_du_calc_util(struct adf_accel_dev *accel_dev,
			    u32 curr_util,
			    enum adf_svc_type svc_type)
{
	struct adf_dev_util_table *du_table = &(GET_DU_TABLE(accel_dev));
	u32 slices = 0;
	u64 ticks_per_svc = 0, slau_per_svc = 0;

	/* Converting ticks to slau
	 * 1. get slices_per_svc
	 *    slices_per_svc = ((no. of slices per service per AE pair) *
	 *                     (total AE pairs)) - (admin slices per service)
	 * 2. reference_ticks to ticks_per_svc
	 *    ticks_per_svc = reference_ticks * slices_per_svc
	 * 3. calculate utilization percentage per svc
	 *    percent_util_per_svc = total_ticks_per_svc / ticks_per_svc
	 * 4. convert utilization to slau
	 *    slau_per_svc = percent_util_per_svc * max_slau_per_svc
	 * NOTES:
	 *    admin slices per service = asym(2 slices) and sym(1 slice)
	 *    slices_per_svc           = slices(excluding admin AE slices)
	 *    reference_ticks          = du_table->total_util(value of AE
	 *                               ticks measured between issuing
	 *                               DU Start and DU Stop)
	 *    total_ticks_per_svc      = curr_util(for PF, sum of all VF
	 *                               values returned by DU per service.
	 *                               For VF, value returned by DU per
	 *                               service for the VF)
	 *    max_slau_per_svc         = max device capacity in slau(
	 *                               accel_dev->sla_sku.slau_supported[svc])
	 */
	slices = adf_get_slices_for_svc(accel_dev, svc_type);
	ticks_per_svc = (u64)du_table->total_util * slices;
	if (ticks_per_svc) {
		slau_per_svc =
		(u64)accel_dev->sla_sku.slau_supported[svc_type] *
		curr_util;
		do_div(slau_per_svc, ticks_per_svc);
	}

	/* return device utilization in sla units */
	return (u16)slau_per_svc;
}

static u16 adf_du_get_vf_slau(struct adf_accel_dev *accel_dev,
			      struct adf_user_du *du)
{
	struct adf_slas *cur_sla = NULL, *tmp = NULL;

	list_for_each_entry_safe(cur_sla, tmp, &accel_dev->sla_list, list) {
		if (adf_is_bdf_equal(&cur_sla->sla.pci_addr, &du->vf_addr) &&
		    cur_sla->sla.svc_type == du->svc_type)
			return cur_sla->sla.rate_in_slau;
	}

	return 0;
}

static void adf_du_get_dev_util
	    (struct adf_accel_dev *accel_dev,
	     struct adf_user_du *du,
	     u32 vf_nr)
{
	u32 current_util = 0;
	u16 max_slau_caps = accel_dev->sla_sku.slau_supported[du->svc_type];
	u8 num_banks = GET_MAX_BANKS(accel_dev), i = 0;
	u32 *dptr = NULL,
	    *du_entries = (u32 *)(GET_DU_TABLE(accel_dev).virt_addr);

	du->slau_util_percent = 0;
	du->slau_supported = 0;
	dptr = du_entries + (du->svc_type * num_banks);

	if (vf_nr == ALL_VF_DU) {
		du->slau_supported = max_slau_caps;
		for (i = 0; i < num_banks; i++)
			current_util += dptr[i];
	} else if (vf_nr < num_banks) {
		current_util = dptr[vf_nr];
		du->slau_supported = adf_du_get_vf_slau(accel_dev, du);
	}

	du->slau_utilized = adf_du_calc_util(accel_dev, current_util,
					     du->svc_type);

	if (max_slau_caps)
		du->slau_util_percent =
			(du->slau_utilized * 100) / max_slau_caps;
}

static int adf_du_query_for_pf(struct adf_accel_dev *accel_dev,
			       struct adf_user_du *query)
{
	if (!(GET_HW_DATA(accel_dev)->is_du_supported)) {
		dev_err(&GET_DEV(accel_dev), "DU Query is not supported.\n");
		return -EINVAL;
	}

	if (!(accel_dev->sla_sku.svc_supported & BIT(query->svc_type))) {
		dev_err(&GET_DEV(accel_dev),
			"Service is not enabled %d\n",
		query->svc_type);
		return -EINVAL;
	}

	adf_du_get_dev_util(accel_dev, query, ALL_VF_DU);

	return 0;
}

static int adf_du_query_for_vf(struct adf_accel_dev *accel_dev,
			       struct adf_user_du *query)
{
	int vf_nr = 0;
	int ret = 0;

	if (!(GET_HW_DATA(accel_dev)->is_du_supported)) {
		dev_err(&GET_DEV(accel_dev), "DU Query is not supported by VF driver.\n");
		return -EINVAL;
	}

	if (!(accel_dev->sla_sku.svc_supported & BIT(query->svc_type))) {
		dev_err(&GET_DEV(accel_dev),
			"Service is not enabled %d\n",
			query->svc_type);
		return -EINVAL;
	}

	ret = adf_get_vf_nr(&query->vf_addr, &vf_nr);
	if (ret) {
		dev_err(&GET_DEV(accel_dev), "Get VF number failed.\n");
		return -EINVAL;
	}

	if (adf_is_vf_nr_valid(accel_dev, vf_nr) < 0) {
		dev_err(&GET_DEV(accel_dev), "BDF not found for DU Query.\n");
		return -EINVAL;
	}

	adf_du_get_dev_util(accel_dev, query, vf_nr);

	return ret;
}

int adf_du_start(struct adf_pci_address *pci_addr)
{
	struct adf_accel_dev *accel_dev = NULL;
	int ret = 0;

	ret = adf_du_get_active_dev(&accel_dev, pci_addr);
	if (ret)
		return ret;

	if ((accel_dev)->is_vf ||
	    !(GET_HW_DATA((accel_dev))->is_du_supported)) {
		dev_err(&GET_DEV(accel_dev),
			"DU Start is not supported by VF driver.\n");
		adf_dev_put(accel_dev);
		return -EINVAL;
	}

	ret = adf_send_du_start(accel_dev);
	adf_dev_put(accel_dev);

	return ret;
}

int adf_du_stop(struct adf_pci_address *pci_addr)
{
	struct adf_accel_dev *accel_dev = NULL;
	int ret = 0;

	ret = adf_du_get_active_dev(&accel_dev, pci_addr);
	if (ret)
		return ret;

	if ((accel_dev)->is_vf ||
	    !(GET_HW_DATA((accel_dev))->is_du_supported)) {
		dev_err(&GET_DEV(accel_dev),
			"DU Stop is not supported by VF driver.\n");
		adf_dev_put(accel_dev);
		return -EINVAL;
	}

	ret = adf_send_du_stop(accel_dev);
	adf_dev_put(accel_dev);

	return ret;
}

int adf_du_query(struct adf_user_du *du)
{
	struct adf_accel_dev *accel_dev = NULL;
	int ret = 0;

	if (du->svc_type >= ADF_MAX_SERVICES) {
		pr_err("QAT: Invalid svc type\n");
		return -EINVAL;
	}

	ret = adf_du_get_active_dev(&accel_dev, &du->pf_addr);
	if (ret)
		return ret;

	ret = adf_du_query_for_pf(accel_dev, du);
	if (ret)
		dev_err(&GET_DEV(accel_dev), "Failed to query du: %d\n", ret);

	adf_dev_put(accel_dev);

	return ret;
}

int adf_du_query_vf(struct adf_user_du *du)
{
	struct adf_accel_dev *accel_dev = NULL;
	int ret = 0;

	if (du->svc_type >= ADF_MAX_SERVICES) {
		pr_err("QAT: Invalid svc type\n");
		return -EINVAL;
	}

	if (du->pf_addr.bus != du->vf_addr.bus) {
		pr_err("QAT: VF doesn't belong to PF.\n");
		return -EINVAL;
	}

	ret = adf_du_get_active_dev(&accel_dev, &du->pf_addr);
	if (ret)
		return ret;

	ret = adf_du_query_for_vf(accel_dev, du);
	if (ret) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to query du VF: %d\n", ret);
	}

	adf_dev_put(accel_dev);

	return ret;
}

int adf_du_init(struct adf_accel_dev *accel_dev)
{
	struct adf_dev_util_table du_table;
	u32 size = ADF_MAX_SERVICES * GET_MAX_BANKS(accel_dev);

	du_table.virt_addr = dma_alloc_coherent(&GET_DEV(accel_dev),
						size * sizeof(u32),
						&du_table.dma_addr,
						GFP_KERNEL);
	if (!du_table.virt_addr) {
		dev_err(&GET_DEV(accel_dev), "DU table memory allocation failed");
		return -ENOMEM;
	}
	accel_dev->du_table = du_table;
	GET_HW_DATA(accel_dev)->is_du_supported = true;

	return 0;
}

int adf_du_exit(struct adf_accel_dev *accel_dev)
{
	u32 size = ADF_MAX_SERVICES * GET_MAX_BANKS(accel_dev);
	bool *is_du_supported = &GET_HW_DATA(accel_dev)->is_du_supported;

	if (!(*is_du_supported))
		return 0;
	dma_free_coherent(&GET_DEV(accel_dev),
			  size,
			  GET_DU_TABLE(accel_dev).virt_addr,
			  accel_dev->du_table.dma_addr);
	*is_du_supported = false;

	return 0;
}
