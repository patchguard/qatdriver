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
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/

#include "rl_utils.h"
#include "sla_manager.h"
#include "icp_sal_sla.h"

/* SLA mgr commands */
static const char *pSlaCommands[] =
    {"create", "update", "delete", "delete_all", "caps", "list", "unknown"};

/*
 ******************************************************************
 * @ingroup sla
 *        Dump the SLA Capabilities
 *
 * @description
 *        This function is used to dump the SLA Capabilities.
 *
 * @param[in]  pCaps  pointer to capabilities structure
 *
 * @retval None
 *
 ******************************************************************
 */
static void slaMgrDumpCaps(struct adf_user_sla_caps *pCaps)
{
    int i;

    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "[%.2x:%.2x.%x]\n",
            pCaps->pf_addr.bus,
            pCaps->pf_addr.dev,
            pCaps->pf_addr.func);

    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "\tSLA supported: %d\n",
            pCaps->max_slas);
    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "\tSLA available: %d\n",
            pCaps->avail_slas);
    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "\tSLA taken:     %d\n",
            pCaps->used_slas);

    osalLog(OSAL_LOG_LVL_USER, OSAL_LOG_DEV_STDOUT, "\n\t[SERVICES]:\n");
    for (i = 0; i < ADF_MAX_SERVICES; i++)
    {
        osalLog(OSAL_LOG_LVL_USER,
                OSAL_LOG_DEV_STDOUT,
                "\t%s: supported=%d available=%d\n",
                rlSvcToStr(pCaps->services[i].svc_type),
                pCaps->services[i].max_svc_rate_in_slau,
                pCaps->services[i].avail_svc_rate_in_slau);
    }
}

/*
 ******************************************************************
 * @ingroup sla
 *        Dump the list of SLAs
 *
 * @description
 *        This function is used to dump the list of SLAs.
 *
 * @param[in]  pSlaList  pointer to list of SLAs
 *
 * @retval None
 *
 ******************************************************************
 */
static void slaMgrDumpList(struct adf_user_slas *pSlaList)
{
    int i;

    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "[%.2x:%.2x.%x]: %d SLAs set\n",
            pSlaList->pf_addr.bus,
            pSlaList->pf_addr.dev,
            pSlaList->pf_addr.func,
            pSlaList->used_slas);
    for (i = 0; i < ADF_MAX_SLA; ++i)
    {
        /* skip if pci address bus not set */
        if (0 == pSlaList->slas[i].pci_addr.bus)
            continue;

        osalLog(OSAL_LOG_LVL_USER,
                OSAL_LOG_DEV_STDOUT,
                "BDF=%.2x:%.2x.%x ID=%d svc=%s rate_in_sla_units=%d\n",
                pSlaList->slas[i].pci_addr.bus,
                pSlaList->slas[i].pci_addr.dev,
                pSlaList->slas[i].pci_addr.func,
                pSlaList->slas[i].sla_id,
                rlSvcToStr(pSlaList->slas[i].svc_type),
                pSlaList->slas[i].rate_in_slau);
    }
}

sla_mgr_cmd_t slaMgrStrToCmd(Cpa8U *pString)
{
    int i;
    size_t stringLen = strnlen((const char *)pString, RL_MAX_CMD_STR_CHAR);

    for (i = SLA_MGR_CMD_CREATE; i < SLA_MGR_CMD_UNKNOWN; i++)
    {
        if (stringLen != strnlen(pSlaCommands[i], RL_MAX_CMD_STR_CHAR))
            continue;
        if (!strncmp((const char *)pString, pSlaCommands[i], stringLen))
            return i;
    }
    return SLA_MGR_CMD_UNKNOWN;
}

CpaStatus slaMgrStrToRate(Cpa16U *pRate, Cpa8U *pString)
{
    unsigned long val = 0;
    Cpa8U *ptr = NULL;

    if (!pString)
    {
        osalLog(
            OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR, "Invalid rate value\n");
        return CPA_STATUS_FAIL;
    }

    errno = 0;
    val = strtoul((const char *)pString, (char **)&ptr, RL_BASE_DEC);

    if ('\0' == *pString || '\0' != *ptr || UINT32_MAX < val ||
        ((0 == val || ULONG_MAX == val) &&
         (ERANGE == errno || EINVAL == errno)))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Invalid rate value = %s\n",
                pString);
        return CPA_STATUS_FAIL;
    }

    *pRate = val;
    return CPA_STATUS_SUCCESS;
}

void slaMgrStrToSlaId(Cpa16U *pSlaId, Cpa8U *pString)
{
    *pSlaId = strtoul((const char *)pString, NULL, 10);
}

CpaStatus slaMgrGetCaps(struct sla_mgr_args *pUsrArgs)
{
    struct adf_user_sla_caps caps;
    struct adf_pci_address pciAddr;

    rlCopyPciAddr(&pciAddr, &pUsrArgs->pciAddr);

    if (CPA_STATUS_SUCCESS != icp_sal_userSlaGetCaps(&pciAddr, &caps))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed to get SLA caps for %.2x:%.2x.%x\n",
                pciAddr.bus,
                pciAddr.dev,
                pciAddr.func);
        return CPA_STATUS_FAIL;
    }

    slaMgrDumpCaps(&caps);

    return CPA_STATUS_SUCCESS;
}

CpaStatus slaMgrGetList(struct sla_mgr_args *pUsrArgs)
{
    struct adf_pci_address pciAddr;
    struct adf_user_slas pSlaList;

    rlCopyPciAddr(&pciAddr, &pUsrArgs->pciAddr);

    /* Get list of SLAs */
    if (CPA_STATUS_SUCCESS != icp_sal_userSlaGetList(&pciAddr, &pSlaList))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed to get list of SLAs for %.2x:%.2x.%x\n",
                pciAddr.bus,
                pciAddr.dev,
                pciAddr.func);
        return CPA_STATUS_FAIL;
    }

    slaMgrDumpList(&pSlaList);

    return CPA_STATUS_SUCCESS;
}

CpaStatus slaMgrCreateSla(struct sla_mgr_args *pUsrArgs)
{
    struct adf_user_sla sla;
    Cpa16U slaId;

    rlCopyPciAddr(&sla.pci_addr, &pUsrArgs->pciAddr);
    sla.svc_type = pUsrArgs->svcType;
    sla.rate_in_slau = pUsrArgs->rateInSlaUnits;
    if (CPA_STATUS_SUCCESS != icp_sal_userSlaCreate(&sla, &slaId))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed to create SLA svc=%s rate_in_sla_units=%d for",
                rlSvcToStr(sla.svc_type),
                sla.rate_in_slau);
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                " %.2x:%.2x.%x\n",
                sla.pci_addr.bus,
                sla.pci_addr.dev,
                sla.pci_addr.func);
        return CPA_STATUS_FAIL;
    }

    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "SLA id=%d for %.2x:%.2x.%x svc=%s rate_in_sla_units=%d created!\n",
            slaId,
            sla.pci_addr.bus,
            sla.pci_addr.dev,
            sla.pci_addr.func,
            rlSvcToStr(sla.svc_type),
            sla.rate_in_slau);

    return CPA_STATUS_SUCCESS;
}

CpaStatus slaMgrUpdateSla(struct sla_mgr_args *pUsrArgs)
{
    struct adf_user_sla sla;
    struct adf_pci_address pciAddr;
    Cpa16U newRate;
    Cpa16U slaId;

    rlCopyPciAddr(&pciAddr, &pUsrArgs->pciAddr);
    sla.svc_type = pUsrArgs->svcType;
    newRate = pUsrArgs->rateInSlaUnits;
    slaId = pUsrArgs->slaId;
    sla.rate_in_slau = pUsrArgs->rateInSlaUnits;
    if (CPA_STATUS_SUCCESS != icp_sal_userSlaUpdate(&pciAddr, slaId, newRate))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed to update SLA id=%d rate_in_sla_units=%d for",
                slaId,
                sla.rate_in_slau);
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                " %.2x:%.2x:%.x\n",
                pciAddr.bus,
                pciAddr.dev,
                pciAddr.func);
        return CPA_STATUS_FAIL;
    }

    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "SLA id=%d for %.2x:%.2x.%x rate_in_sla_units=%d updated!\n",
            slaId,
            pciAddr.bus,
            pciAddr.dev,
            pciAddr.func,
            newRate);

    return CPA_STATUS_SUCCESS;
}

CpaStatus slaMgrDeleteSla(struct sla_mgr_args *pUsrArgs)
{
    struct adf_pci_address pciAddr;
    Cpa16U slaId;

    rlCopyPciAddr(&pciAddr, &pUsrArgs->pciAddr);
    slaId = pUsrArgs->slaId;

    if (CPA_STATUS_SUCCESS != icp_sal_userSlaDelete(&pciAddr, slaId))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed SLA delete. svc=%s for %.2x:%.2x.%x\n",
                rlSvcToStr(ADF_SVC_SYM),
                pciAddr.bus,
                pciAddr.dev,
                pciAddr.func);
        return CPA_STATUS_FAIL;
    }
    osalLog(OSAL_LOG_LVL_USER,
            OSAL_LOG_DEV_STDOUT,
            "SLA id=%d for %.2x:%.2x.%x deleted!\n",
            slaId,
            pciAddr.bus,
            pciAddr.dev,
            pciAddr.func);

    return CPA_STATUS_SUCCESS;
}

CpaStatus slaMgrDeleteSlaList(struct sla_mgr_args *pUsrArgs)
{
    int i;
    struct adf_pci_address pciAddr;
    struct adf_user_slas slas;

    rlCopyPciAddr(&pciAddr, &pUsrArgs->pciAddr);

    /* Get list of SLAs */
    if (CPA_STATUS_SUCCESS != icp_sal_userSlaGetList(&pciAddr, &slas))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                "Failed to get list of SLAs for device");
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDERR,
                " %.2x:%.2x.%x\n",
                pciAddr.bus,
                pciAddr.dev,
                pciAddr.func);
        return CPA_STATUS_FAIL;
    }
    for (i = 0; i < ADF_MAX_SLA; ++i)
    {
        /* skip if pci address bus not set */
        if (0 == slas.slas[i].pci_addr.bus)
            continue;

        if (CPA_STATUS_SUCCESS !=
            icp_sal_userSlaDelete(&pciAddr, slas.slas[i].sla_id))
        {
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDERR,
                    "Failed to delete SLA svc=%s for",
                    rlSvcToStr(ADF_SVC_SYM));
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDERR,
                    " %.2x:%.2x.%x\n",
                    pciAddr.bus,
                    pciAddr.dev,
                    pciAddr.func);
            continue;
        }
    }
    return CPA_STATUS_SUCCESS;
}
