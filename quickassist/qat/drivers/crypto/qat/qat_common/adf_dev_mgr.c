/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <linux/mutex.h>
#include <linux/list.h>
#include "adf_cfg.h"
#include "adf_common_drv.h"

#define ADF_AE_PAIR 2
#define PKE_SLICES_PER_AE_PAIR 5
#define PKE_RL_ADMIN_SLICE 2
#define SYM_RL_ADMIN_SLICE 1

static LIST_HEAD(accel_table);
static LIST_HEAD(vfs_table);
static DEFINE_MUTEX(table_lock);
static uint32_t num_devices;
static u8 id_map[ADF_MAX_DEVICES];

struct vf_id_map {
	u32 bdf;
	u32 id;
	u32 fake_id;
	bool attached;
	struct list_head list;
};

static int adf_get_vf_id(struct adf_accel_dev *vf)
{
	return ((7 * (PCI_SLOT(accel_to_pci_dev(vf)->devfn) - 1)) +
		PCI_FUNC(accel_to_pci_dev(vf)->devfn) +
		(PCI_SLOT(accel_to_pci_dev(vf)->devfn) - 1));
}

static int adf_get_vf_num(struct adf_accel_dev *vf)
{
	return (accel_to_pci_dev(vf)->bus->number << 8) | adf_get_vf_id(vf);
}

static struct vf_id_map *adf_find_vf(u32 bdf)
{
	struct list_head *itr;

	list_for_each(itr, &vfs_table) {
		struct vf_id_map *ptr =
			list_entry(itr, struct vf_id_map, list);

		if (ptr->bdf == bdf)
			return ptr;
	}
	return NULL;
}

/**
 * adf_get_vf_real_id() - Translate fake to real device id
 *
 * The "real" id is assigned to a device when it is initially
 * bound to the driver.
 * The "fake" id is usually the same as the real id, but
 * can change when devices are unbound from the qat driver,
 * perhaps to assign the device to a guest.
 */
static int adf_get_vf_real_id(u32 fake)
{
	struct list_head *itr;

	list_for_each(itr, &vfs_table) {
		struct vf_id_map *ptr =
			list_entry(itr, struct vf_id_map, list);
		if (ptr->fake_id == fake)
			return ptr->id;
	}
	return -1;
}

/**
 * adf_clean_vf_map() - Cleans VF id mapings
 *
 * Function cleans internal ids for virtual functions.
 * @vf: flag indicating whether mappings is cleaned
 *	for vfs only or for vfs and pfs
 */
void adf_clean_vf_map(bool vf)
{
	struct vf_id_map *map;
	struct list_head *ptr, *tmp;

	mutex_lock(&table_lock);
	list_for_each_safe(ptr, tmp, &vfs_table) {
		map = list_entry(ptr, struct vf_id_map, list);
		if (map->bdf != -1) {
			id_map[map->id] = 0;
			num_devices--;
		}

		if (vf && map->bdf == -1)
			continue;

		list_del(ptr);
		kfree(map);
	}
	mutex_unlock(&table_lock);
}
EXPORT_SYMBOL_GPL(adf_clean_vf_map);

/**
 * adf_devmgr_update_class_index() - Update internal index
 * @hw_data:  Pointer to internal device data.
 *
 * Function updates internal dev index for VFs
 */
void adf_devmgr_update_class_index(struct adf_hw_device_data *hw_data)
{
	struct adf_hw_device_class *class = hw_data->dev_class;
	struct list_head *itr;
	int i = 0;

	list_for_each(itr, &accel_table) {
		struct adf_accel_dev *ptr =
				list_entry(itr, struct adf_accel_dev, list);

		if (ptr->hw_device->dev_class == class)
			ptr->hw_device->instance_id = i++;

		if (i == class->instances)
			break;
	}
}
EXPORT_SYMBOL_GPL(adf_devmgr_update_class_index);

static unsigned int adf_find_free_id(void)
{
	unsigned int i;

	for (i = 0; i < ADF_MAX_DEVICES; i++) {
		if (!id_map[i]) {
			id_map[i] = 1;
			return i;
		}
	}
	return ADF_MAX_DEVICES + 1;
}

/**
 * adf_devmgr_add_dev() - Add accel_dev to the acceleration framework
 * @accel_dev:  Pointer to acceleration device.
 * @pf:		Corresponding PF if the accel_dev is a VF
 *
 * Function adds acceleration device to the acceleration framework.
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_devmgr_add_dev(struct adf_accel_dev *accel_dev,
		       struct adf_accel_dev *pf)
{
	struct list_head *itr;
	int ret = 0;

	if (num_devices == ADF_MAX_DEVICES) {
		dev_err(&GET_DEV(accel_dev), "Only support up to %d devices\n",
			ADF_MAX_DEVICES);
		return -EFAULT;
	}

	mutex_lock(&table_lock);
	atomic_set(&accel_dev->ref_count, 0);

	/* PF on host or VF on guest */
	if (!accel_dev->is_vf || (accel_dev->is_vf && !pf)) {
		struct vf_id_map *map;

		list_for_each(itr, &accel_table) {
			struct adf_accel_dev *ptr =
				list_entry(itr, struct adf_accel_dev, list);

			if (ptr == accel_dev) {
				ret = -EEXIST;
				goto unlock;
			}
		}

		list_add_tail(&accel_dev->list, &accel_table);
		accel_dev->accel_id = adf_find_free_id();
		if (accel_dev->accel_id > ADF_MAX_DEVICES) {
			ret = -EFAULT;
			goto unlock;
		}
		num_devices++;
		map = kzalloc(sizeof(*map), GFP_KERNEL);
		if (!map) {
			ret = -ENOMEM;
			goto unlock;
		}
		map->bdf = ~0;
		map->id = accel_dev->accel_id;
		map->fake_id = map->id;
		map->attached = true;
		list_add_tail(&map->list, &vfs_table);
	} else if (accel_dev->is_vf && pf) {
		/* VF on host */
		struct vf_id_map *map;

		map = adf_find_vf(adf_get_vf_num(accel_dev));
		if (map) {
			struct vf_id_map *next;

			accel_dev->accel_id = map->id;
			list_add_tail(&accel_dev->list, &accel_table);
			map->fake_id++;
			map->attached = true;
			next = list_next_entry(map, list);
			while (next && &next->list != &vfs_table) {
				next->fake_id++;
				next = list_next_entry(next, list);
			}

			ret = 0;
			goto unlock;
		}

		map = kzalloc(sizeof(*map), GFP_KERNEL);
		if (!map) {
			ret = -ENOMEM;
			goto unlock;
		}
		accel_dev->accel_id = adf_find_free_id();
		if (accel_dev->accel_id > ADF_MAX_DEVICES) {
			kfree(map);
			ret = -EFAULT;
			goto unlock;
		}
		num_devices++;
		list_add_tail(&accel_dev->list, &accel_table);
		map->bdf = adf_get_vf_num(accel_dev);
		map->id = accel_dev->accel_id;
		map->fake_id = map->id;
		map->attached = true;
		list_add_tail(&map->list, &vfs_table);
	}
unlock:
	mutex_unlock(&table_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_devmgr_add_dev);

struct list_head *adf_devmgr_get_head(void)
{
	return &accel_table;
}

/**
 * adf_devmgr_rm_dev() - Remove accel_dev from the acceleration framework.
 * @accel_dev:  Pointer to acceleration device.
 * @pf:		Corresponding PF if the accel_dev is a VF
 *
 * Function removes acceleration device from the acceleration framework.
 * To be used by QAT device specific drivers.
 *
 * Return: void
 */
void adf_devmgr_rm_dev(struct adf_accel_dev *accel_dev,
		       struct adf_accel_dev *pf)
{
	mutex_lock(&table_lock);
	if (!accel_dev->is_vf || (accel_dev->is_vf && !pf)) {
		id_map[accel_dev->accel_id] = 0;
		num_devices--;
	} else if (accel_dev->is_vf && pf) {
		struct vf_id_map *map, *next;

		map = adf_find_vf(adf_get_vf_num(accel_dev));
		if (!map) {
			dev_err(&GET_DEV(accel_dev), "Failed to find VF map\n");
			goto unlock;
		}
		map->fake_id--;
		map->attached = false;
		next = list_next_entry(map, list);
		while (next && &next->list != &vfs_table) {
			next->fake_id--;
			next = list_next_entry(next, list);
		}
	}
unlock:
	list_del(&accel_dev->list);
	mutex_unlock(&table_lock);
}
EXPORT_SYMBOL_GPL(adf_devmgr_rm_dev);

struct adf_accel_dev *adf_devmgr_get_first(void)
{
	struct adf_accel_dev *dev = NULL;

	if (!list_empty(&accel_table))
		dev = list_first_entry(&accel_table, struct adf_accel_dev,
				       list);
	return dev;
}

/**
 * adf_devmgr_pci_to_accel_dev() - Get accel_dev associated with the pci_dev.
 * @accel_dev:  Pointer to pci device.
 *
 * Function returns acceleration device associated with the given pci device.
 * To be used by QAT device specific drivers.
 *
 * Return: pointer to accel_dev or NULL if not found.
 */
struct adf_accel_dev *adf_devmgr_pci_to_accel_dev(struct pci_dev *pci_dev)
{
	struct list_head *itr;

	mutex_lock(&table_lock);
	list_for_each(itr, &accel_table) {
		struct adf_accel_dev *ptr =
				list_entry(itr, struct adf_accel_dev, list);

		if (ptr->accel_pci_dev.pci_dev == pci_dev) {
			mutex_unlock(&table_lock);
			return ptr;
		}
	}
	mutex_unlock(&table_lock);
	return NULL;
}
EXPORT_SYMBOL_GPL(adf_devmgr_pci_to_accel_dev);

struct adf_accel_dev *adf_devmgr_get_dev_by_id(uint32_t id)
{
	struct list_head *itr;
	int real_id;

	mutex_lock(&table_lock);
	real_id = adf_get_vf_real_id(id);
	if (real_id < 0)
		goto unlock;

	id = real_id;

	list_for_each(itr, &accel_table) {
		struct adf_accel_dev *ptr =
				list_entry(itr, struct adf_accel_dev, list);
		if (ptr->accel_id == id) {
			mutex_unlock(&table_lock);
			return ptr;
		}
	}
unlock:
	mutex_unlock(&table_lock);
	return NULL;
}

int adf_devmgr_verify_id(uint32_t *id)
{
	struct adf_accel_dev *accel_dev;

	if (*id == ADF_CFG_ALL_DEVICES)
		return 0;

	accel_dev = adf_devmgr_get_dev_by_id(*id);
	if (!accel_dev)
		return -ENODEV;

	/* Correct the id if real and fake differ */
	*id = accel_dev->accel_id;
	return 0;
}

static int adf_get_num_dettached_vfs(void)
{
	struct list_head *itr;
	int vfs = 0;

	mutex_lock(&table_lock);
	list_for_each(itr, &vfs_table) {
		struct vf_id_map *ptr =
			list_entry(itr, struct vf_id_map, list);
		if (ptr->bdf != ~0 && !ptr->attached)
			vfs++;
	}
	mutex_unlock(&table_lock);
	return vfs;
}

void adf_devmgr_get_num_dev(uint32_t *num)
{
	*num = num_devices - adf_get_num_dettached_vfs();
}

/**
 * adf_dev_in_use() - Check whether accel_dev is currently in use
 * @accel_dev: Pointer to acceleration device.
 *
 * To be used by QAT device specific drivers.
 *
 * Return: 1 when device is in use, 0 otherwise.
 */
int adf_dev_in_use(struct adf_accel_dev *accel_dev)
{
	return atomic_read(&accel_dev->ref_count) != 0;
}
EXPORT_SYMBOL_GPL(adf_dev_in_use);

/**
 * adf_dev_get() - Increment accel_dev reference count
 * @accel_dev: Pointer to acceleration device.
 *
 * Increment the accel_dev refcount and if this is the first time
 * incrementing it during this period the accel_dev is in use,
 * increment the module refcount too.
 * To be used by QAT device specific drivers.
 *
 * Return: 0 when successful, EFAULT when fail to bump module refcount
 */
int adf_dev_get(struct adf_accel_dev *accel_dev)
{
	if (atomic_add_return(1, &accel_dev->ref_count) == 1)
		if (!try_module_get(accel_dev->owner))
			return -EFAULT;
	return 0;
}
EXPORT_SYMBOL_GPL(adf_dev_get);

/**
 * adf_dev_put() - Decrement accel_dev reference count
 * @accel_dev: Pointer to acceleration device.
 *
 * Decrement the accel_dev refcount and if this is the last time
 * decrementing it during this period the accel_dev is in use,
 * decrement the module refcount too.
 * To be used by QAT device specific drivers.
 *
 * Return: void
 */
void adf_dev_put(struct adf_accel_dev *accel_dev)
{
	if (atomic_sub_return(1, &accel_dev->ref_count) == 0)
		module_put(accel_dev->owner);
}
EXPORT_SYMBOL_GPL(adf_dev_put);

/**
 * adf_devmgr_in_reset() - Check whether device is in reset
 * @accel_dev: Pointer to acceleration device.
 *
 * To be used by QAT device specific drivers.
 *
 * Return: 1 when the device is being reset, 0 otherwise.
 */
int adf_devmgr_in_reset(struct adf_accel_dev *accel_dev)
{
	return test_bit(ADF_STATUS_RESTARTING, &accel_dev->status);
}
EXPORT_SYMBOL_GPL(adf_devmgr_in_reset);

/**
 * adf_dev_started() - Check whether device has started
 * @accel_dev: Pointer to acceleration device.
 *
 * To be used by QAT device specific drivers.
 *
 * Return: 1 when the device has started, 0 otherwise
 */
int adf_dev_started(struct adf_accel_dev *accel_dev)
{
	return test_bit(ADF_STATUS_STARTED, &accel_dev->status);
}
EXPORT_SYMBOL_GPL(adf_dev_started);

/*
 * adf_devmgr_get_dev_by_bdf() - Look up accel_dev by BDF
 * @pci_addr: pointer to adf_pci_address structure
 *
 * To be used by DU and SLA ioctls.
 *
 * Return: accel_dev if found, NULL otherwise.
 *
 * Note: Caller has to call adf_dev_put once finished using the accel_dev!
 */
struct adf_accel_dev *adf_devmgr_get_dev_by_bdf(
			struct adf_pci_address *pci_addr)
{
	struct adf_accel_dev *accel_dev = NULL;
	struct pci_dev *pci_dev = NULL;
	unsigned int devfn = PCI_DEVFN(pci_addr->dev, pci_addr->func);

	mutex_lock(&table_lock);
	list_for_each_entry(accel_dev, &accel_table, list) {
		pci_dev = accel_to_pci_dev(accel_dev);
		if (pci_dev->bus->number == pci_addr->bus &&
		    pci_dev->devfn == devfn) {
			adf_dev_get(accel_dev);
			mutex_unlock(&table_lock);
			return accel_dev;
		}
	}
	mutex_unlock(&table_lock);

	return NULL;
}

/*
 * adf_devmgr_get_dev_by_bus() - Look up accel_dev by pci bus
 * @bus: Bus number
 *
 * To be used by DU and SLA ioctls.
 *
 * Return: accel_dev if found, NULL otherwise.
 *
 * Note: Caller has to call adf_dev_put once finished using the accel_dev!
 */
struct adf_accel_dev *adf_devmgr_get_dev_by_pci_bus(u8 bus)
{
	struct adf_accel_dev *accel_dev = NULL;

	mutex_lock(&table_lock);
	list_for_each_entry(accel_dev, &accel_table, list)
		if (accel_to_pci_dev(accel_dev)->bus->number == bus) {
			adf_dev_get(accel_dev);
			mutex_unlock(&table_lock);
			return accel_dev;
		}
	mutex_unlock(&table_lock);

	return NULL;
}

/*
 * adf_get_vf_nr - Look up accel_dev and get vf number
 * @pci_addr: pointer to adf_pci_address structure
 * @vf_nr: Pointer to get the VF number
 *
 * To be used by DU and SLA ioctls.
 */
int adf_get_vf_nr(struct adf_pci_address *vf_pci_addr, int *vf_nr)
{
	struct adf_accel_dev *accel_dev = NULL;
	int pf_dev = 0, pf_func = 0;

	if (vf_pci_addr->func > ADF_MAX_FUNC_PER_DEV)
		return -EINVAL;

	accel_dev = adf_devmgr_get_dev_by_pci_bus(vf_pci_addr->bus);
	if (!accel_dev)
		return -EINVAL;
	pf_dev = PCI_SLOT(accel_to_pci_dev(accel_dev)->devfn);
	pf_func = PCI_FUNC(accel_to_pci_dev(accel_dev)->devfn);
	adf_dev_put(accel_dev);

	*vf_nr = (((vf_pci_addr->dev - pf_dev) << ADF_PCI_DEV_OFFSET) +
		  (vf_pci_addr->func - pf_func) - ADF_VF_OFFSET);

	return 0;
}

/*
 * adf_is_vf_nr_valid - Look up accel_dev and check vf number is valid
 * @accel_dev: pointer to acceleration device
 * @vf_nr: vf number to be checked
 *
 * To be used by DU and SLA ioctls.
 */
int adf_is_vf_nr_valid(struct adf_accel_dev *accel_dev, int vf_nr)
{
	return (vf_nr >= 0 && vf_nr < GET_MAX_BANKS(accel_dev) ? 0 : -EINVAL);
}

/*
 * adf_get_slices_for_svc - Look up accel_dev and get the slices
 * for specific svc
 * @accel_dev: pointer to acceleration device
 * @svc: svc type to be checked
 *
 */
u32 adf_get_slices_for_svc(struct adf_accel_dev *accel_dev,
			   enum adf_svc_type svc)
{
	struct adf_hw_device_data *hw_data = GET_HW_DATA(accel_dev);
	u32 num_aes = hw_data->get_num_aes(hw_data);

	if (svc == ADF_SVC_ASYM)
		return ((num_aes / ADF_AE_PAIR) * PKE_SLICES_PER_AE_PAIR) -
			PKE_RL_ADMIN_SLICE;
	else if (svc == ADF_SVC_SYM)
		return num_aes - SYM_RL_ADMIN_SLICE;

	return 0;
}

/*
 * adf_is_bdf_equal - Compare and check both the BDF addresses are same or not
 * @bdf1: BDF address to be compared
 * @bdf2: BDF address to be compared
 *
 */
bool adf_is_bdf_equal(struct adf_pci_address *bdf1,
		      struct adf_pci_address *bdf2)
{
	return bdf1->bus == bdf2->bus &&
		bdf1->dev == bdf2->dev &&
		bdf1->func == bdf2->func;
}
