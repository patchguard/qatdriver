/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.7.L.4.8.0-00005
 *
 ****************************************************************************/
/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "cpa.h"
#include "adf_kernel_types.h"
#include "adf_cfg_common.h"
#include "icp_sal_du.h"
#include "icp_adf_du.h"

CpaStatus icp_sal_userDuStart(struct adf_pci_address *pPf)
{
    return icp_adf_userDuStart(pPf);
}

CpaStatus icp_sal_userDuStop(struct adf_pci_address *pPf)
{
    return icp_adf_userDuStop(pPf);
}

CpaStatus icp_sal_userDuQuery(struct adf_pci_address *pPf,
                              enum adf_svc_type svcType,
                              Cpa16U *pCapacityInSlaUnit,
                              Cpa16U *pUtilInSlaUnit,
                              Cpa16U *pUtilInPct)
{
    return icp_adf_userDuQuery(
        pPf, svcType, pCapacityInSlaUnit, pUtilInSlaUnit, pUtilInPct);
}

CpaStatus icp_sal_userDuQueryVf(struct adf_pci_address *pPf,
                                struct adf_pci_address *pVf,
                                enum adf_svc_type svcType,
                                Cpa16U *pSlaInSlaUnit,
                                Cpa16U *pUtilInSlaUnit,
                                Cpa16U *pUtilInPct)
{
    return icp_adf_userDuQueryVf(
        pPf, pVf, svcType, pSlaInSlaUnit, pUtilInSlaUnit, pUtilInPct);
}
