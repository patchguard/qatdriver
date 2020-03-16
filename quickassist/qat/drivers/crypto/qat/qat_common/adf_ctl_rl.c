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
#include "adf_ctl_rl.h"

/*
 * adf_ctl_ioctl_sla_create - IOCTL to create the SLA
 *
 * Function receives the user input as argument and creates the SLA
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_sla_create(unsigned long arg)
{
	int ret = 0;
	struct adf_user_sla sla;
	void *sla_id_ptr = NULL;

	if (copy_from_user(&sla, (void __user *)arg, sizeof(sla))) {
		pr_err("QAT: Failed to copy sla create info from user.\n");
		return -EFAULT;
	}

	ret = adf_sla_create(&sla);
	if (ret)
		return ret;

	sla_id_ptr = &((struct adf_user_sla *)arg)->sla_id;
	return put_user(sla.sla_id, (u16 __user *)sla_id_ptr);
}

/*
 * adf_ctl_ioctl_sla_update - IOCTL to update the specific SLA
 *
 * Function receives the user input as argument and updates the SLA
 * based on sla id
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_sla_update(unsigned long arg)
{
	struct adf_user_sla sla;

	if (copy_from_user(&sla, (void __user *)arg, sizeof(sla))) {
		pr_err("QAT: Failed to copy sla update info from user.\n");
		return -EFAULT;
	}

	return adf_sla_update(&sla);
}

/*
 * adf_ctl_ioctl_sla_delete - IOCTL to delete the specific SLA
 *
 * Function receives the user input as argument and deletes the SLA
 * based on sla id
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_sla_delete(unsigned long arg)
{
	struct adf_user_sla sla;

	if (copy_from_user(&sla, (void __user *)arg, sizeof(sla))) {
		pr_err("QAT: Failed to copy sla delete info from user.\n");
		return -EFAULT;
	}

	return adf_sla_delete(&sla);
}

/*
 * adf_ctl_ioctl_sla_get_caps - IOCTL to get the capabilities of SLA
 *
 * Function receives the user input as argument and get the capability
 * information which is supported on the specific device
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_sla_get_caps(unsigned long arg)
{
	struct adf_user_sla_caps sla_caps;
	int ret = -EFAULT;

	if (copy_from_user(&sla_caps, (void __user *)arg, sizeof(sla_caps))) {
		pr_err("QAT: Failed to copy sla caps info from user.\n");
		return ret;
	}

	ret = adf_sla_get_caps(&sla_caps);
	if (ret)
		return ret;

	ret = copy_to_user((void __user *)arg, &sla_caps, sizeof(sla_caps));
	if (ret) {
		pr_err("Failed to copy qat sla capabilities to user.\n");
		return ret;
	}

	return ret;
}

/*
 * adf_ctl_ioctl_sla_get_list - IOCTL to list the SLA created
 *
 * Function receives the user input as argument and lists the SLAs
 * which are created successfully
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_sla_get_list(unsigned long arg)
{
	struct adf_user_slas slas;
	int ret = -EFAULT;

	if (copy_from_user(&slas, (void __user *)arg, sizeof(slas))) {
		pr_err("QAT: Failed to copy sla get list info from user.\n");
		return ret;
	}

	ret = adf_sla_get_list(&slas);
	if (ret)
		return ret;

	/* copy the information from adf_user_info to user space */
	ret = copy_to_user((void __user *)arg, &slas, sizeof(slas));
	if (ret) {
		pr_err("QAT: Failed to copy slas\n");
		return ret;
	}

	return ret;
}

/*
 * adf_ctl_ioctl_du_start - IOCTL to start the device utilization
 *
 * Function receives the user input as argument and starts the
 * device utilization
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_du_start(unsigned long arg)
{
	struct adf_pci_address pci_addr = {0, 0, 0};

	if (copy_from_user(&pci_addr, (void __user *)arg, sizeof(pci_addr))) {
		pr_err("QAT: Failed to copy pci_addr from user.\n");
		return -EFAULT;
	}

	return adf_du_start(&pci_addr);
}

/*
 * adf_ctl_ioctl_du_stop - IOCTL to stop the device utilization
 *
 * Function receives the user input as argument and stops the
 * device utilization
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_du_stop(unsigned long arg)
{
	struct adf_pci_address pci_addr = {0, 0, 0};

	if (copy_from_user(&pci_addr, (void __user *)arg, sizeof(pci_addr))) {
		pr_err("QAT: Failed to copy pci_addr from user.\n");
		return -EFAULT;
	}

	return adf_du_stop(&pci_addr);
}

/*
 * adf_ctl_ioctl_du_query - IOCTL to query the PF device
 *
 * Function to query the PF device and get the device utilization
 * details
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_du_query(unsigned long arg)
{
	struct adf_user_du du;
	int ret = 0;

	if (copy_from_user(&du, (void __user *)arg, sizeof(du))) {
		pr_err("QAT: Failed to copy DU query info from user.\n");
		return -EFAULT;
	}
	ret = adf_du_query(&du);
	if (ret)
		return ret;

	ret = copy_to_user((void __user *)arg, &du, sizeof(du));
	if (ret) {
		pr_err("QAT: Failed to copy DU query info to user space\n");
		ret = -EFAULT;
	}

	return ret;
}

/*
 * adf_ctl_ioctl_du_query_vf - IOCTL to query the VF device
 *
 * Function to query the VF device and get the device utilization
 * details
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_ctl_ioctl_du_query_vf(unsigned long arg)
{
	struct adf_user_du du;
	int ret = 0;

	if (copy_from_user(&du, (void __user *)arg, sizeof(du))) {
		pr_err("QAT: Failed to copy DU query info from user.\n");
		return -EFAULT;
	}
	ret = adf_du_query_vf(&du);
	if (ret)
		return ret;

	ret = copy_to_user((void __user *)arg, &du, sizeof(du));
	if (ret) {
		pr_err("QAT:Failed to copy DU query info to user space\n");
		ret = -EFAULT;
	}
	return ret;
}

