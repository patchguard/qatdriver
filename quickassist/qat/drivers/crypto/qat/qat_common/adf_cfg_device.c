/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2017 Intel Corporation.
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
 *  Copyright(c) 2017 Intel Corporation.
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
#include <linux/pci.h>
#include "adf_cfg_device.h"

#define ADF_CFG_CAP_DC ADF_ACCEL_CAPABILITIES_COMPRESSION
#define ADF_CFG_CAP_ASYM ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC
#define ADF_CFG_CAP_SYM (ADF_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC |	\
		     ADF_ACCEL_CAPABILITIES_CIPHER |		\
		     ADF_ACCEL_CAPABILITIES_AUTHENTICATION)
#define ADF_CFG_CAP_CY (ADF_CFG_CAP_ASYM | ADF_CFG_CAP_SYM)

#define ADF_CFG_CY_RINGS \
	(CRYPTO | CRYPTO << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	CRYPTO << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	CRYPTO << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_SYM_RINGS \
	(SYM | SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	SYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	SYM << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_ASYM_RINGS \
	(ASYM | ASYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	ASYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	ASYM << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_CY_DC_RINGS \
	(CRYPTO | CRYPTO << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	NA << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_ASYM_DC_RINGS \
	(ASYM | ASYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_SYM_DC_RINGS \
	(SYM | SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

#define ADF_CFG_DC_RINGS \
	(COMP | COMP << ADF_CFG_SERV_RING_PAIR_1_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT | \
	COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT)

struct adf_cfg_enabled_services {
	const char svcs_enabled[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
	u16 rng_to_svc_msk;
	u32 enabled_cap;
};

static struct adf_cfg_enabled_services adf_cfg_svcs[] = {
	{"cy", ADF_CFG_CY_RINGS, ADF_CFG_CAP_CY},
	{"dc", ADF_CFG_DC_RINGS, ADF_CFG_CAP_DC},
	{"sym", ADF_CFG_SYM_RINGS, ADF_CFG_CAP_SYM},
	{"asym", ADF_CFG_ASYM_RINGS, ADF_CFG_CAP_ASYM},
	{"cy;dc", ADF_CFG_CY_DC_RINGS, ADF_CFG_CAP_CY | ADF_CFG_CAP_DC},
	{"dc;cy", ADF_CFG_CY_DC_RINGS, ADF_CFG_CAP_CY | ADF_CFG_CAP_DC},
	{"asym;dc", ADF_CFG_ASYM_DC_RINGS, ADF_CFG_CAP_ASYM | ADF_CFG_CAP_DC},
	{"dc;asym", ADF_CFG_ASYM_DC_RINGS, ADF_CFG_CAP_ASYM | ADF_CFG_CAP_DC},
	{"sym;dc", ADF_CFG_SYM_DC_RINGS, ADF_CFG_CAP_SYM | ADF_CFG_CAP_DC},
	{"dc;sym", ADF_CFG_SYM_DC_RINGS, ADF_CFG_CAP_SYM | ADF_CFG_CAP_DC},
};

int adf_cfg_get_ring_pairs(struct adf_cfg_device *device,
			   struct adf_cfg_instance *inst,
			   const char *process_name,
			   struct adf_accel_dev *accel_dev)
{
	int i = 0;
	int ret = -EFAULT;
	struct adf_cfg_instance *free_inst = NULL;
	struct adf_cfg_bundle *first_free_bundle = NULL;
	enum adf_cfg_bundle_type free_bundle_type;
	int first_user_bundle = 0;

	dev_dbg(&GET_DEV(accel_dev),
		"get ring pair for section %s, bundle_num is %d.\n",
				process_name, device->bundle_num);

	/* Section of user process with poll mode */
	if (strcmp(ADF_KERNEL_SEC, process_name) &&
	    strcmp(ADF_KERNEL_SAL_SEC, process_name) &&
	    (inst->polling_mode == ADF_CFG_RESP_POLL)) {
		first_user_bundle = device->max_kernel_bundle_nr + 1;
		for (i = first_user_bundle; i < device->bundle_num; i++) {
			free_inst =
				adf_cfg_get_free_instance(device,
							  device->bundles[i],
							  inst,
							  process_name);

			if (!free_inst)
				continue;

			ret = adf_cfg_get_ring_pairs_from_bundle(
					device->bundles[i], inst,
					process_name, free_inst);
			return ret;
		}
	} else {
		/* Section of in-tree, or kernel API or user process
		 * with epoll mode
		 */
		if (!strcmp(ADF_KERNEL_SEC, process_name) ||
		    !strcmp(ADF_KERNEL_SAL_SEC, process_name))
			free_bundle_type = KERNEL;
		else
			free_bundle_type = USER;

		for (i = 0; i < device->bundle_num; i++) {
			/* Since both in-tree and kernel API's bundle type
			 * are kernel, use cpumask_subset to check if the
			 * ring's affinity mask is a subset of a bundle's
			 * one.
			 */
			if ((free_bundle_type == device->bundles[i]->type) &&
			    cpumask_subset(
					&inst->affinity_mask,
					&device->bundles[i]->affinity_mask)) {
				free_inst =
					adf_cfg_get_free_instance(
							device,
							device->bundles[i],
							inst,
							process_name);

				if (!free_inst)
					continue;

				ret = adf_cfg_get_ring_pairs_from_bundle(
							device->bundles[i],
							inst,
							process_name,
							free_inst);

				return ret;

			} else if (!first_free_bundle &&
				   adf_cfg_is_free(device->bundles[i])) {
				first_free_bundle = device->bundles[i];
			}
		}

		if (first_free_bundle) {
			free_inst = adf_cfg_get_free_instance(device,
							      first_free_bundle,
							      inst,
							      process_name);

			if (!free_inst)
				return ret;

			ret = adf_cfg_get_ring_pairs_from_bundle(
					first_free_bundle, inst,
					process_name, free_inst);

			if (free_bundle_type == KERNEL) {
				device->max_kernel_bundle_nr =
					first_free_bundle->number;
			}
			return ret;
		}
	}
	pr_err("Don't have enough rings for instance %s in process %s\n",
	       inst->name, process_name);

	return ret;
}

int adf_cfg_get_services_enabled(struct adf_accel_dev *accel_dev,
				 u16 *ring_to_svc_map)
{
	char key[ADF_CFG_MAX_KEY_LEN_IN_BYTES];
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
	u32 i = 0;

	*ring_to_svc_map = 0;

	/* Get the services enabled by user */
	snprintf(key, sizeof(key), ADF_SERVICES_ENABLED);
	if (adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC, key, val))
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(adf_cfg_svcs); i++) {
		if (!strncmp(val, adf_cfg_svcs[i].svcs_enabled,
			     ADF_CFG_MAX_VAL_LEN_IN_BYTES)) {
			*ring_to_svc_map = adf_cfg_svcs[i].rng_to_svc_msk;
				return 0;
		}
	}

	dev_err(&GET_DEV(accel_dev), "Invalid services enabled: %s\n", val);

	return -EFAULT;
}
EXPORT_SYMBOL_GPL(adf_cfg_get_services_enabled);

void adf_cfg_set_asym_rings_mask(struct adf_accel_dev *accel_dev)
{
	int service;
	u16 ena_srv_mask;
	u16 service_type;
	u16 asym_mask = 0;
	struct adf_cfg_device *cfg_dev = accel_dev->cfg->dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;

	if (!cfg_dev) {
		hw_data->asym_rings_mask = ADF_CFG_DEF_ASYM_MASK;
		return;
	}

	ena_srv_mask = hw_data->ring_to_svc_map;

	/* parse each service */
	for (service = 0;
	     service < ADF_CFG_MAX_SERVICES;
	     service++) {
		service_type =
			GET_SRV_TYPE(ena_srv_mask, service);
		switch (service_type) {
		case CRYPTO:
		case ASYM:
			SET_ASYM_MASK(asym_mask, service);
			if (service_type == CRYPTO)
				service++;
			break;
		}
	}

	hw_data->asym_rings_mask = asym_mask;
}
EXPORT_SYMBOL_GPL(adf_cfg_set_asym_rings_mask);

void adf_cfg_gen_dispatch_arbiter(struct adf_accel_dev *accel_dev,
				  const u32 *thrd_to_arb_map,
				  u32 *thrd_to_arb_map_gen,
				  u32 total_engines)
{
	int engine, thread, service, bits;
	u32 thread_ability, ability_map, service_mask, service_type;
	u16 ena_srv_mask = GET_HW_DATA(accel_dev)->ring_to_svc_map;

	if (ena_srv_mask == ADF_DEFAULT_RING_TO_SRV_MAP) {
		/* if not set, return the default dispatch arbiter */
		for (engine = 0;
		     engine < total_engines;
		     engine++) {
			thrd_to_arb_map_gen[engine] = thrd_to_arb_map[engine];
		}
		return;
	}

	for (engine = 0; engine < total_engines; engine++) {
		bits = 0;
		/* ability_map is used to indicate the threads ability */
		ability_map = thrd_to_arb_map[engine];
		thrd_to_arb_map_gen[engine] = 0;
		/* parse each thread on the engine */
		for (thread = 0;
		     thread < ADF_NUM_THREADS_PER_AE;
		     thread++) {
			/* get the ability of this thread */
			thread_ability = ability_map & ADF_THRD_ABILITY_MASK;
			ability_map >>= ADF_THRD_ABILITY_BIT_LEN;
			/* parse each service */
			for (service = 0;
			     service < ADF_CFG_MAX_SERVICES;
			     service++) {
				service_type =
					GET_SRV_TYPE(ena_srv_mask, service);
				switch (service_type) {
				case CRYPTO:
					service_mask = ADF_CFG_ASYM_SRV_MASK;
					if (thread_ability & service_mask)
						thrd_to_arb_map_gen[engine] |=
								(1 << bits);
					bits++;
					service++;
					service_mask = ADF_CFG_SYM_SRV_MASK;
					break;
				case COMP:
					service_mask = ADF_CFG_DC_SRV_MASK;
					break;
				case SYM:
					service_mask = ADF_CFG_SYM_SRV_MASK;
					break;
				case ASYM:
					service_mask = ADF_CFG_ASYM_SRV_MASK;
					break;
				default:
					service_mask = ADF_CFG_UNKNOWN_SRV_MASK;
				}
				if (thread_ability & service_mask)
					thrd_to_arb_map_gen[engine] |=
								(1 << bits);
				bits++;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(adf_cfg_gen_dispatch_arbiter);

static int adf_cfg_get_caps_enabled(struct adf_accel_dev *accel_dev,
				    u32 *enabled_caps)
{
	char key[ADF_CFG_MAX_KEY_LEN_IN_BYTES];
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
	u8 i = 0;

	*enabled_caps = 0;
	/* Get the services enabled by user */
	snprintf(key, sizeof(key), ADF_SERVICES_ENABLED);
	if (adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC, key, val))
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(adf_cfg_svcs); i++) {
		if (!strncmp(val, adf_cfg_svcs[i].svcs_enabled,
			     ADF_CFG_MAX_VAL_LEN_IN_BYTES)) {
			*enabled_caps = adf_cfg_svcs[i].enabled_cap;
			return 0;
		}
	}

	dev_err(&GET_DEV(accel_dev),
		"Invalid services enabled: %s\n", val);

	return -EFAULT;
}

static int adf_cfg_check_enabled_services(struct adf_accel_dev *accel_dev,
					  u32 enabled_caps)
{
	u32 hw_caps = GET_HW_DATA(accel_dev)->accel_capabilities_mask;

	if ((enabled_caps & hw_caps) == enabled_caps)
		return 0;

	dev_err(&GET_DEV(accel_dev), "Unsupported device configuration");

	return -EFAULT;
}

static int adf_cfg_update_pf_accel_cap_mask(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	unsigned long pke_disabled = 0;
	unsigned long storage_enabled = 0;
	unsigned long  rl_enabled = 0;
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
	u32 enabled_caps = 0;

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_PKE_DISABLED, val)) {
		if (kstrtoul(val, 0, &pke_disabled))
			return -EFAULT;
	}

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_STORAGE_FIRMWARE_ENABLED, val)) {
		if (kstrtoul(val, 0, &storage_enabled))
			return -EFAULT;
	}

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_RL_FIRMWARE_ENABLED, val)) {
		if (kstrtoul(val, 0, &rl_enabled))
			return -EFAULT;
	}

	if (hw_data->get_accel_cap) {
		hw_data->accel_capabilities_mask =
			hw_data->get_accel_cap(accel_dev);
	}

	if (pke_disabled || storage_enabled) {
		hw_data->accel_capabilities_mask &=
			~ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
	}

	if (rl_enabled) {
		hw_data->accel_capabilities_mask &=
			~(ADF_ACCEL_CAPABILITIES_COMPRESSION |
			  ADF_ACCEL_CAPABILITIES_LZS_COMPRESSION);
	}

	if (adf_cfg_get_caps_enabled(accel_dev, &enabled_caps))
		return -EFAULT;

	if (adf_cfg_check_enabled_services(accel_dev, enabled_caps))
		return -EFAULT;

	if (!(enabled_caps & ADF_CFG_CAP_ASYM))
		hw_data->accel_capabilities_mask &= ~ADF_CFG_CAP_ASYM;
	if (!(enabled_caps & ADF_CFG_CAP_SYM))
		hw_data->accel_capabilities_mask &= ~ADF_CFG_CAP_SYM;
	if (!(enabled_caps & ADF_CFG_CAP_DC))
		hw_data->accel_capabilities_mask &=
			~(ADF_CFG_CAP_DC |
			  ADF_ACCEL_CAPABILITIES_LZS_COMPRESSION);

	return 0;
}

static int adf_cfg_update_vf_accel_cap_mask(struct adf_accel_dev *accel_dev)
{
	u32 enabled_caps = 0;

	if (adf_cfg_get_caps_enabled(accel_dev, &enabled_caps))
		return -EFAULT;

	if (adf_cfg_check_enabled_services(accel_dev, enabled_caps))
		return -EFAULT;

	return 0;
}

int adf_cfg_device_init(struct adf_cfg_device *device,
			struct adf_accel_dev *accel_dev)
{
	int i = 0;
	/* max_inst indicates the max instance number one bank can hold */
	int max_inst = accel_dev->hw_device->tx_rx_gap;
	int ret = -ENOMEM;
	struct adf_hw_device_data *hw_data = GET_HW_DATA(accel_dev);

	device->bundle_num = 0;
	device->bundles = (struct adf_cfg_bundle **)
		kzalloc(sizeof(struct adf_cfg_bundle *)
			* accel_dev->hw_device->num_banks,
			GFP_KERNEL);
	if (!device->bundles)
		goto failed;

	device->bundle_num = accel_dev->hw_device->num_banks;

	device->instances = (struct adf_cfg_instance **)
		kzalloc(sizeof(struct adf_cfg_instance *)
			* device->bundle_num * max_inst,
			GFP_KERNEL);
	if (!device->instances)
		goto failed;

	device->instance_index = 0;

	device->max_kernel_bundle_nr = -1;

	dev_dbg(&GET_DEV(accel_dev), "init device with bundle information\n");

	ret = -EFAULT;

	/* Update the acceleration capability mask based on User capability */
	if (!accel_dev->is_vf) {
		if (adf_cfg_update_pf_accel_cap_mask(accel_dev))
			goto failed;
	} else {
		if (adf_cfg_update_vf_accel_cap_mask(accel_dev))
			goto failed;
	}

	/* Based on the svc configured, get ring_to_svc_map */
	if (hw_data->get_ring_to_svc_map) {
		if (hw_data->get_ring_to_svc_map(accel_dev,
						 &hw_data->ring_to_svc_map))
			goto failed;
	}

	ret = -ENOMEM;
	/*
	 * 1) get the config information to generate the ring to service
	 *    mapping table
	 * 2) init each bundle of this device
	 */
	for (i = 0; i < device->bundle_num; i++) {
		device->bundles[i] =
			kzalloc(sizeof(struct adf_cfg_bundle), GFP_KERNEL);
		if (!device->bundles[i])
			goto failed;

		device->bundles[i]->max_section = max_inst;
		adf_cfg_bundle_init(device->bundles[i], device, i, accel_dev);
	}

	return 0;

failed:
	for (i = 0; i < device->bundle_num; i++) {
		if (device->bundles[i])
			adf_cfg_bundle_clear(device->bundles[i], accel_dev);
	}

	for (i = 0; i < (device->bundle_num * max_inst); i++) {
		if (device->instances && device->instances[i])
			kfree(device->instances[i]);
	}

	kfree(device->instances);
	device->instances = NULL;

	dev_err(&GET_DEV(accel_dev), "Failed to do device init\n");
	return ret;
}

void adf_cfg_device_clear(struct adf_cfg_device *device,
			  struct adf_accel_dev *accel_dev)
{
	int i = 0;

	dev_dbg(&GET_DEV(accel_dev), "clear device with bundle information\n");
	for (i = 0; i < device->bundle_num; i++) {
		if (device->bundles && device->bundles[i]) {
			adf_cfg_bundle_clear(device->bundles[i], accel_dev);
			kfree(device->bundles[i]);
			device->bundles[i] = NULL;
		}
	}

	kfree(device->bundles);
	device->bundles = NULL;

	for (i = 0; i < device->instance_index; i++) {
		if (device->instances && device->instances[i]) {
			kfree(device->instances[i]);
			device->instances[i] = NULL;
		}
	}

	kfree(device->instances);
	device->instances = NULL;
}

int adf_config_device(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *cfg = NULL;
	struct adf_cfg_device *cfg_device = NULL;
	struct adf_cfg_section *sec;
	struct list_head *list;
	int ret = -ENOMEM;

	if (!accel_dev)
		return ret;

	cfg = accel_dev->cfg;
	cfg->dev = NULL;
	cfg_device = (struct adf_cfg_device *)
			kzalloc(sizeof(*cfg_device), GFP_KERNEL);
	if (!cfg_device)
		goto failed;

	ret = -EFAULT;

	if (adf_cfg_device_init(cfg_device, accel_dev))
		goto failed;

	cfg->dev = cfg_device;

	/* GENERAL and KERNEL section must be processed before others */
	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_GENERAL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_KERNEL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_KERNEL_SAL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		/* avoid reprocessing one section */
		if (!sec->processed && !sec->is_derived) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
		}
	}

	/* newly added accel section */
	ret = adf_cfg_process_section(accel_dev,
				      ADF_ACCEL_SEC,
				      accel_dev->accel_id);
	if (ret)
		goto failed;

	/*
	 * put item-remove task after item-process
	 * because during process we may fetch values from those items
	 */
	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!sec->is_derived) {
			dev_dbg(&GET_DEV(accel_dev), "Clean up section %s\n",
				sec->name);
			ret = adf_cfg_cleanup_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
		}
	}

	ret = 0;
failed:
	if (ret) {
		if (cfg_device) {
			adf_cfg_device_clear(cfg_device, accel_dev);
			kfree(cfg_device);
			cfg->dev = NULL;
		}
		adf_cfg_del_all(accel_dev);
		dev_err(&GET_DEV(accel_dev), "Failed to config device\n");
	}

	return ret;
}
EXPORT_SYMBOL_GPL(adf_config_device);
