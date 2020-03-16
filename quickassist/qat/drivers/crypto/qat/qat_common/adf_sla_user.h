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
#ifndef ADF_SLA_USER_H_
#define ADF_SLA_USER_H_

#include "adf_cfg_common.h"

#define ADF_MAX_SLA 64

/*
 *
 * @ingroup sla
 *
 * struct adf_user_service - For a given service, specifies the max
 * rate the device can sustain and the actual available rate, that is,
 * not yet allocated.
 *
 * @svc_type:                service type
 * @max_svc_rate_in_slau:    maximum rate defined in sla units
 * @avail_svc_rate_in_slau:  available rate defined in sla units
 */
struct adf_user_service {
	enum adf_svc_type svc_type;
	u16 max_svc_rate_in_slau;
	u16 avail_svc_rate_in_slau;
} __packed;

/*
 *
 * @ingroup sla
 *
 * struct adf_user_sla_caps - For a given device, specifies the maximum
 * number of SLAs, the number of SLAs still available and the number of SLAs
 * already allocated. Also, for each service, it provides details about
 * the rate still available.
 *
 * @pf_addr:	    BDF address of physical function for this device
 * @max_slas:       maximum number of SLAs supported on this device
 * @avail_slas:     number of SLAs still available
 * @used_slas:      number of SLAs already allocated
 *
 * @services:       for each service type, provides details about the rate still
 *                  available
 */
struct adf_user_sla_caps {
	struct adf_pci_address pf_addr;
	u16 max_slas;
	u16 avail_slas;
	u16 used_slas;
	struct adf_user_service services[ADF_MAX_SERVICES];
} __packed;

/*
 *
 * @ingroup sla
 *
 * struct adf_user_sla - parameters required to request an SLA
 *
 * @pci_addr:	    For IOCTL_SLA_CREATE this will be the BDF address of the
 *		    virtual function. For IOCTL_SLA_UPDATE/IOCTL_SLA_DELETE this
 *		    will be the BDF address of the physical function to which
 *		    the VF belongs to
 * @sla_id:	    For IOCTL_SLA_CREATE this is an output parameter. Kernel
 *		    will populate this with the sla_id which is device specific.
 *		    User has to keep track of both pf_addr and sla_id to later
 *		    update/delete the sla.
 *		    For IOCTL_SLA_CREATE/IOCTL_SLA_UPDATE this is an input
 *		    parameter that paired with pci_addr set to the PF BDF, will
 *		    uniquely identify the SLA system wide
 * @svc_type:       service type to request SLA for
 * @rate_in_slau:   rate requested in sla units. Must be lower or equal
 *		    to adf_user_sla_caps.services[svc_type].
 *		    avail_svc_rate_in_slau
 */
struct adf_user_sla {
	struct adf_pci_address pci_addr;
	u16 sla_id;
	enum adf_svc_type svc_type;
	u16 rate_in_slau;
} __packed;

/*
 *
 * @ingroup sla
 *
 * struct adf_user_slas - to be used with IOCTL_SLA_GET_LIST to retrieve the
 * list of allocated SLAs.
 *
 * @pf_addr:	BDF address of physical function for this device
 * @slas:       array of allocated SLAs.
 * @used_slas:  actual number of SLA allocated. Entries in slas from 0 to
 *              used_slas are valid.
 */
struct adf_user_slas {
	struct adf_pci_address pf_addr;
	struct adf_user_sla slas[ADF_MAX_SLA];
	u16 used_slas;
};

#endif
