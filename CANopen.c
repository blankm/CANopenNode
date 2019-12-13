/*
 * Main CANopen stack file. It combines Object dictionary (CO_OD) and all other
 * CANopen source files. Configuration information are read from CO_OD.h file.
 *
 * @file        CANopen.c
 * @ingroup     CO_CANopen
 * @author      Janez Paternoster
 * @copyright   2010 - 2015 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */


#include "CANopen.h"


/* If defined, global variables will be used, otherwise CANopen objects will
   be generated with COcalloc(). */
/* #define CO_USE_GLOBALS */

/* If defined, the user provides an own implemetation for calculating the
 * CRC16 CCITT checksum. */
/* #define CO_USE_OWN_CRC16 */


#include <stdlib.h> /*  for malloc, free if used, check driver include for mapping */
static uint32_t CO_memoryUsed = 0; /* informative */



/* Global variables ***********************************************************/

static CO_Context_t *CO_Context; /* Pointer to context struct */

static CO_t COO;
CO_t *CO = NULL;

static CO_CANrx_t *CO_CANmodule_rxArray0;
static CO_CANtx_t *CO_CANmodule_txArray0;
static CO_OD_extension_t *CO_SDO_ODExtensions;
static CO_HBconsNode_t *CO_HBcons_monitoredNodes;

#ifndef CO_TRACE_BUFFER_SIZE_FIXED
#define CO_TRACE_BUFFER_SIZE_FIXED 100
#endif

static uint32_t *CO_traceTimeBuffers;
static int32_t *CO_traceValueBuffers;
static uint32_t *CO_traceBufferSize;

static uint16_t *inhibitTimeEMCY;           /*1015, Data Type: UNSIGNED16 */
static uint16_t *producerHeartbeatTime;     /*1017, Data Type: UNSIGNED16 */
static uint32_t *NMTStartup;                /*1F80, Data Type: UNSIGNED32 */
static uint8_t  *errorRegister;             /*1001, Data Type: UNSIGNED8 */
static uint8_t  *errorBehavior;             /*1029, Data Type: UNSIGNED8, Array[6] */
static uint32_t *synchronousWindowLength;   /*1007, Data Type: UNSIGNED32 */

static inline uint16_t CO_RXCAN_NO_MSGS(CO_Context_t * context)
{
    //(1+CO_NO_SYNC+CO_NO_EMERGENCY+CO_NO_TIME+CO_NO_RPDO+CO_NO_SDO_SERVER+CO_NO_SDO_CLIENT+CO_NO_HB_CONS)
    uint16_t rxcnt = 1;
    rxcnt += context->features.CO_NO_SYNC;
    rxcnt += context->features.CO_NO_EMERGENCY;
    rxcnt += context->features.CO_NO_TIME;
    rxcnt += context->features.CO_NO_RPDO;
    rxcnt += context->features.CO_NO_SDO_SERVER;
    rxcnt += context->features.CO_NO_SDO_CLIENT;
    rxcnt += context->features.CO_NO_HB_CONS;
    return rxcnt;
}

static inline uint16_t CO_TXCAN_NO_MSGS(CO_Context_t * context)
{
    //(CO_NO_NMT_MASTER+CO_NO_SYNC+CO_NO_EMERGENCY+CO_NO_TIME+CO_NO_TPDO+CO_NO_SDO_SERVER+CO_NO_SDO_CLIENT+CO_NO_HB_PROD+CO_NO_LSS_SERVER+CO_NO_LSS_CLIENT)
    uint16_t txcnt = 0;
    rxcnt += context->features.CO_NO_NMT_MASTER;
    rxcnt += context->features.CO_NO_SYNC;
    rxcnt += context->features.CO_NO_EMERGENCY;
    rxcnt += context->features.CO_NO_TIME;
    rxcnt += context->features.CO_NO_TPDO;
    rxcnt += context->features.CO_NO_SDO_SERVER;
    rxcnt += context->features.CO_NO_SDO_CLIENT;
    rxcnt += context->features.CO_NO_HB_PROD;
    rxcnt += context->features.CO_NO_LSS_SERVER;
    rxcnt += context->features.CO_NO_LSS_CLIENT;
    return txcnt;
}


CO_ReturnError_t CO_verifyFeatures(CO_Context_t * context)
{
    /* Check if all features make sense */
    uint8_t errs = 0;
    if(context->features.CO_NO_SYNC != 1)
        errs++;
    if(context->features.CO_NO_NMT_MASTER > 1)
        errs++;
    if(context->features.CO_NO_EMERGENCY != 1)
        errs++;
    if(context->features.CO_NO_SDO_SERVER == 0)
        errs++;
    if(context->features.CO_NO_SDO_CLIENT > 128)
        errs++;
    if(context->features.CO_NO_RPDO < 1 || context->features.CO_NO_RPDO > 0x200)
        errs++;
    if(context->features.CO_NO_TPDO < 1 || context->features.CO_NO_TPDO > 0x200)
        errs++;
    if(context->features.CO_NO_LSS_SERVER > 1)
        errs++;
    if(context->features.CO_NO_LSS_CLIENT > 1)
        errs++;
    if(context->features.CO_NO_TIME > 1)
        errs++;
    // TODO: I guess we don't allow LSS Server and Client at the same time, I guess different contexts are needed for different roles
    if((context->features.CO_NO_LSS_CLIENT > 0) && (context->features.CO_NO_LSS_SERVER > 0))
        errs++;
    
    /* Now comes the more complicated part, we have to check inside OD */
    /* consumerHeartBeatTime Array size */
    // TODO: Do this!!
    /* errorStatusBits string length */
    // TODO: Do this!!

    if(errs > 0)
        return CO_ERROR_PARAMETERS;
    else
        return CO_ERROR_NO;
}


/* ATTENTION: Globals can only be used with a default context, otherwise the defines are unknown */
#ifdef CO_USE_GLOBALS

/* Indexes for CANopenNode message objects ************************************/
    #ifdef ODL_consumerHeartbeatTime_arrayLength
        #define CO_NO_HB_CONS   ODL_consumerHeartbeatTime_arrayLength
    #else
        #define CO_NO_HB_CONS   0
    #endif
    #define CO_NO_HB_PROD      1                                      /*  Producer Heartbeat Cont */ 

    #define CO_RXCAN_NMT       0                                      /*  index for NMT message */  //<<<<<<<<<<<<<<<<<< KEEP
    #define CO_RXCAN_SYNC      1                                      /*  index for SYNC message */ //<<<<<<<<<<<<<<<<<< KEEP
    #define CO_RXCAN_EMERG    (CO_RXCAN_SYNC+CO_NO_SYNC)              /*  index for Emergency message */
    #define CO_RXCAN_TIME     (CO_RXCAN_EMERG+CO_NO_EMERGENCY)        /*  index for TIME message */
    #define CO_RXCAN_RPDO     (CO_RXCAN_TIME+CO_NO_TIME)              /*  start index for RPDO messages */
    #define CO_RXCAN_SDO_SRV  (CO_RXCAN_RPDO+CO_NO_RPDO)              /*  start index for SDO server message (request) */
    #define CO_RXCAN_SDO_CLI  (CO_RXCAN_SDO_SRV+CO_NO_SDO_SERVER)     /*  start index for SDO client message (response) */
    #define CO_RXCAN_CONS_HB  (CO_RXCAN_SDO_CLI+CO_NO_SDO_CLIENT)     /*  start index for Heartbeat Consumer messages */
    #define CO_RXCAN_LSS      (CO_RXCAN_CONS_HB+CO_NO_HB_CONS)        /*  index for LSS rx message */
    /* total number of received CAN messages */
    

    #define CO_TXCAN_NMT       0                                      /*  index for NMT master message */ //<<<<<<<<<<<<<<<<<< KEEP
    #define CO_TXCAN_SYNC      CO_TXCAN_NMT+CO_NO_NMT_MASTER          /*  index for SYNC message */
    #define CO_TXCAN_EMERG    (CO_TXCAN_SYNC+CO_NO_SYNC)              /*  index for Emergency message */
    #define CO_TXCAN_TIME     (CO_TXCAN_EMERG+CO_NO_EMERGENCY)        /*  index for TIME message */
    #define CO_TXCAN_TPDO     (CO_TXCAN_TIME+CO_NO_TIME)              /*  start index for TPDO messages */
    #define CO_TXCAN_SDO_SRV  (CO_TXCAN_TPDO+CO_NO_TPDO)              /*  start index for SDO server message (response) */
    #define CO_TXCAN_SDO_CLI  (CO_TXCAN_SDO_SRV+CO_NO_SDO_SERVER)     /*  start index for SDO client message (request) */
    #define CO_TXCAN_HB       (CO_TXCAN_SDO_CLI+CO_NO_SDO_CLIENT)     /*  index for Heartbeat message */
    #define CO_TXCAN_LSS      (CO_TXCAN_HB+CO_NO_HB_PROD)             /*  index for LSS tx message */
    /* total number of transmitted CAN messages */
    

#endif



/* Helper function for NMT master *********************************************/
CO_CANtx_t *NMTM_txBuff = 0;

CO_ReturnError_t CO_sendNMTcommand(CO_t *CO_this, uint8_t command, uint8_t nodeID)
{
    if (NMTM_txBuff == 0)
    {
        /* error, CO_CANtxBufferInit() was not called for this buffer. */
        return CO_ERROR_TX_UNCONFIGURED; /* -11 */
    }
    NMTM_txBuff->data[0] = command;
    NMTM_txBuff->data[1] = nodeID;

    CO_ReturnError_t error = CO_ERROR_NO;

    /* Protect access to NMT operatingState and resetCommand */
    CO_LOCK_NMT();

    /* Apply NMT command also to this node, if set so. */
    if (nodeID == 0 || nodeID == CO_this->NMT->nodeId)
    {
        switch (command)
        {
        case CO_NMT_ENTER_OPERATIONAL:
            if ((*CO_this->NMT->emPr->errorRegister) == 0)
            {
                CO_this->NMT->operatingState = CO_NMT_OPERATIONAL;
            }
            break;
        case CO_NMT_ENTER_STOPPED:
            CO_this->NMT->operatingState = CO_NMT_STOPPED;
            break;
        case CO_NMT_ENTER_PRE_OPERATIONAL:
            CO_this->NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
            break;
        case CO_NMT_RESET_NODE:
            CO_this->NMT->resetCommand = CO_RESET_APP;
            break;
        case CO_NMT_RESET_COMMUNICATION:
            CO_this->NMT->resetCommand = CO_RESET_COMM;
            break;
        default:
            error = CO_ERROR_ILLEGAL_ARGUMENT;
            break;
        }
    }

    CO_UNLOCK_NMT();

    if (error == CO_ERROR_NO)
        return CO_CANsend(CO_this->CANmodule[0], NMTM_txBuff); /* 0 = success */
    else
    {
        return error;
    }
}

/******************************************************************************/
CO_ReturnError_t CO_new(CO_Context_t *context)
{
    uint16_t i;

    uint16_t errCnt;

    if(CO_verifyFeatures(context) != CO_ERROR_NO)
        return CO_ERROR_PARAMETERS;

    // TODO: Address these!
    /* Verify parameters from CO_OD */
    if(   sizeof(OD_TPDOCommunicationParameter_t) != sizeof(CO_TPDOCommPar_t)
       || sizeof(OD_TPDOMappingParameter_t) != sizeof(CO_TPDOMapPar_t)
       || sizeof(OD_RPDOCommunicationParameter_t) != sizeof(CO_RPDOCommPar_t)
       || sizeof(OD_RPDOMappingParameter_t) != sizeof(CO_RPDOMapPar_t))
    {
        return CO_ERROR_PARAMETERS;
    }

    
    if(sizeof(OD_SDOClientParameter_t) != sizeof(CO_SDOclientPar_t)){
        return CO_ERROR_PARAMETERS;
    }
    

    /* Initialize CANopen object */

    if(CO == NULL) /* Use malloc only once */
    {    
        uint16_t norxmsgs = CO_RXCAN_NO_MSGS(context);
        uint16_t notxmsgs = CO_TXCAN_NO_MSGS(context);
        CO = &COO;
        CO->CANmodule[0]                    = (CO_CANmodule_t *)    COcalloc(1, sizeof(CO_CANmodule_t));
        CO_CANmodule_rxArray0               = (CO_CANrx_t *)        COcalloc(norxmsgs, sizeof(CO_CANrx_t));
        CO_CANmodule_txArray0               = (CO_CANtx_t *)        COcalloc(notxmsgs, sizeof(CO_CANtx_t));
        /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
         * otherwise it can't be accessed like an array atferwards! */
        CO->SDO                             = (CO_SDO_t *)          COcalloc(context->features.CO_NO_SDO_SERVER, sizeof(CO_SDO_t));        
        CO_SDO_ODExtensions                 = (CO_OD_extension_t*)  COcalloc(context->numberOfODElements, sizeof(CO_OD_extension_t));
        CO->em                              = (CO_EM_t *)           COcalloc(1, sizeof(CO_EM_t));
        CO->emPr                            = (CO_EMpr_t *)         COcalloc(1, sizeof(CO_EMpr_t));
        CO->NMT                             = (CO_NMT_t *)          COcalloc(1, sizeof(CO_NMT_t));
        CO->SYNC                            = (CO_SYNC_t *)         COcalloc(1, sizeof(CO_SYNC_t));
        CO->TIME                            = (CO_TIME_t *)         COcalloc(1, sizeof(CO_TIME_t)); 
        /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
         * otherwise it can't be accessed like an array atferwards! */
        CO->RPDO                            = (CO_RPDO_t *)         COcalloc(context->features.CO_NO_RPDO, sizeof(CO_RPDO_t));
        /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
         * otherwise it can't be accessed like an array atferwards! */ 
        CO->TPDO[i]                         = (CO_TPDO_t *)         COcalloc(context->features.CO_NO_TPDO, sizeof(CO_TPDO_t));
        CO->HBcons                          = (CO_HBconsumer_t *)   COcalloc(1, sizeof(CO_HBconsumer_t));
        CO_HBcons_monitoredNodes            = (CO_HBconsNode_t *)   COcalloc(context->features.CO_NO_HB_CONS, sizeof(CO_HBconsNode_t));

        if(context->features.CO_NO_LSS_SERVER > 0)
        {
            CO->LSSslave                    = (CO_LSSslave_t *)     COcalloc(1, sizeof(CO_LSSslave_t));
        }
        else
        {
            CO->LSSslave                    = NULL;
        }
        if(context->features.CO_NO_LSS_CLIENT > 0)
        {
            CO->LSSmaster                   = (CO_LSSmaster_t *)    COcalloc(1, sizeof(CO_LSSmaster_t));
        }
        else
        {
            CO->LSSmaster                   = NULL;
        }
        if(context->features.CO_NO_SDO_CLIENT > 0)
        {
            /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
             * otherwise it can't be accessed like an array afterwards! */ 
            CO->SDOclient                   = (CO_SDOclient_t *)    COcalloc(context->features.CO_NO_SDO_CLIENT, sizeof(CO_SDOclient_t));
        }
        else
        {
            CO->SDOclient                   = NULL;
        }
        if(context->features.CO_NO_TRACE > 0)
        {
            /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
             * otherwise it can't be accessed like an array atferwards! */ 
            CO->trace                       = (CO_trace_t *)        COcalloc(context->features.CO_NO_TRACE, sizeof(CO_trace_t));
            CO_traceTimeBuffers             = (uint32_t *)          COcalloc(context->features.CO_NO_TRACE, sizeof(uint32_t) * CO_TRACE_BUFFER_SIZE_FIXED);
            CO_traceValueBuffers            = (int32_t *)           COcalloc(context->features.CO_NO_TRACE, sizeof(int32_t) * CO_TRACE_BUFFER_SIZE_FIXED);
            for (i = 0; i < (uint16_t)context->features.CO_NO_TRACE; i++)
            {
                if (CO_traceTimeBuffers[i] != NULL && CO_traceValueBuffers[i] != NULL)
                {
                    CO_traceBufferSize[i] = CO_TRACE_BUFFER_SIZE_FIXED; //FIXME: OD_traceConfig[i].size 
                }
                else
                {
                    CO_traceBufferSize[i] = 0;
                }
            }
        } 
    }

    CO_memoryUsed = sizeof(CO_CANmodule_t)
                  + sizeof(CO_CANrx_t) * CO_RXCAN_NO_MSGS
                  + sizeof(CO_CANtx_t) * CO_TXCAN_NO_MSGS
                  + sizeof(CO_SDO_t) * CO_NO_SDO_SERVER
                  + sizeof(CO_OD_extension_t) * CO_OD_NoOfElements
                  + sizeof(CO_EM_t)
                  + sizeof(CO_EMpr_t)
                  + sizeof(CO_NMT_t)
                  + sizeof(CO_SYNC_t)
                  + sizeof(CO_TIME_t)
                  + sizeof(CO_RPDO_t) * CO_NO_RPDO
                  + sizeof(CO_TPDO_t) * CO_NO_TPDO
                  + sizeof(CO_HBconsumer_t)
                  + sizeof(CO_HBconsNode_t) * CO_NO_HB_CONS;
    if(context->features.CO_NO_LSS_SERVER == 1)
    {
        CO_memoryUsed += sizeof(CO_LSSslave_t);
    }
  
    if(context->features.CO_NO_LSS_CLIENT == 1)
    {
        CO_memoryUsed += sizeof(CO_LSSmaster_t);
    }
    CO_memoryUsed += sizeof(CO_SDOclient_t) * (uint32_t)context->features.CO_NO_SDO_CLIENT;
    
    CO_memoryUsed += sizeof(CO_trace_t) * (uint32_t)context->features.CO_NO_TRACE;
    CO_memoryUsed += (uint32_t)context->features.CO_NO_TRACE * 2 * sizeof(int32_t) * CO_TRACE_BUFFER_SIZE_FIXED;
    

    errCnt = 0;
    if(CO->CANmodule[0]                 == NULL) errCnt++;
    if(CO_CANmodule_rxArray0            == NULL) errCnt++;
    if(CO_CANmodule_txArray0            == NULL) errCnt++;
    if(CO->SDO                          == NULL) errCnt++;    
    if(CO_SDO_ODExtensions              == NULL) errCnt++;
    if(CO->em                           == NULL) errCnt++;
    if(CO->emPr                         == NULL) errCnt++;
    if(CO->NMT                          == NULL) errCnt++;
    if(CO->SYNC                         == NULL) errCnt++;
    if(CO->TIME                         == NULL) errCnt++;
    if(CO->RPDO                         == NULL) errCnt++;
    if(CO->TPDO                         == NULL) errCnt++;
    if(CO->HBcons                       == NULL) errCnt++;
    if(CO_HBcons_monitoredNodes         == NULL) errCnt++;
    if(context->features.CO_NO_LSS_SERVER == 1 && \
       CO->LSSslave                     == NULL) errCnt++;
    if(context->features.CO_NO_LSS_CLIENT == 1 && \
       CO->LSSmaster                    == NULL) errCnt++;
    if(context->features.CO_NO_SDO_CLIENT > 0 && \
       CO->SDOclient                    == NULL) errCnt++;
    if(context->features.CO_NO_TRACE > 0 && \
       CO->trace                        == NULL) errCnt++;
    
    if(errCnt != 0) return CO_ERROR_OUT_OF_MEMORY;

    return CO_ERROR_NO;
}


/******************************************************************************/
CO_ReturnError_t CO_CANinit(
        void                   *CANdriverState,
        uint16_t                bitRate,
        CO_Context_t           *context)
{
    CO_ReturnError_t err;

    CO->CANmodule[0]->CANnormal = false;
    CO_CANsetConfigurationMode(CANdriverState);

    uint16_t norxmsgs = CO_RXCAN_NO_MSGS(context);
    uint16_t notxmsgs = CO_TXCAN_NO_MSGS(context);

    err = CO_CANmodule_init(
            CO->CANmodule[0],
            CANdriverState,
            CO_CANmodule_rxArray0,
            norxmsgs,
            CO_CANmodule_txArray0,
            notxmsgs,
            bitRate);

    return err;
}

/******************************************************************************/
CO_ReturnError_t CO_LSSinit(
        uint8_t                 nodeId,
        uint16_t                bitRate,
        CO_Context_t           *context)
{
    CO_LSS_address_t lssAddress;
    CO_ReturnError_t err;

    // Calculate the LSS msg box (rx&tx)

    uint16_t rxidxcan_LSS = (uint16_t)context->features.CO_NO_HB_CONS + \
                            (uint16_t)context->features.CO_NO_SDO_CLIENT + \
                            (uint16_t)context->features.CO_NO_SDO_SERVER + \
                            context->features.CO_NO_RPDO + \
                            (uint16_t)context->features.CO_NO_TIME + \
                            (uint16_t)context->features.CO_NO_EMERGENCY + \
                            (uint16_t)context->features.CO_NO_SYNC + 1;

    uint16_t txidxcan_LSS = (uint16_t)context->features.CO_NO_HB_PROD + \
                            (uint16_t)context->features.CO_NO_SDO_CLIENT + \
                            (uint16_t)context->features.CO_NO_SDO_SERVER + \
                            context->features.CO_NO_TPDO + \
                            (uint16_t)context->features.CO_NO_TIME + \
                            (uint16_t)context->features.CO_NO_EMERGENCY + \
                            (uint16_t)context->features.CO_NO_SYNC + \
                            (uint16_t)context->features.CO_NO_NMT_MASTER;

    // Get the device's information out of OD
    lssAddress.identity.vendorID = 0;
    lssAddress.identity.productCode = 0;
    lssAddress.identity.revisionNumber = 0;
    lssAddress.identity.serialNumber = 0;

    /* Poll over OD to find OD_identity */
    for (uint16_t i = 0; i < context->numberOfODElements; i++)
    {
        if (context->usedOD[i].index == 0x1018 && context->usedOD[i].maxSubIndex > 0 && context->usedOD[i].attribute == 0 && context->usedOD[i].length == 0 && context->usedOD[i].pData != NULL)
        {
            /* Looks like this is the idendity record */
            CO_OD_entryRecord_t * recP = (CO_OD_entryRecord_t *)context->usedOD[i].pData;
            
            if(recP[0].pData != NULL) 
            {
                uint8_t maxSub = *((uint8_t *) recP[0].pData);

                if(maxSub == 4) // This is the expcted size 
                {
                    if(recP[1].pData != NULL && recP[1].length == 4)
                        lssAddress.identity.vendorID = *((uint32_t *)recP[1].pData); // subindex 1 holds vendor ID as uint32_t
                    if(recP[2].pData != NULL && recP[2].length == 4)
                        lssAddress.identity.productCode = *((uint32_t *)recP[2].pData); // subindex 2 holds productCode as uint32_t
                    if(recP[3].pData != NULL && recP[3].length == 4)
                        lssAddress.identity.revisionNumber = *((uint32_t *)recP[3].pData); // subindex 3 holds revisionNumber as uint32_t
                    if(recP[4].pData != NULL && recP[4].length == 4)
                        lssAddress.identity.serialNumber = *((uint32_t *)recP[4].pData); // subindex 4 holds serialNumber as uint32_t
                }
                /* Exit early, we're done here (even if data is not valid)*/
                break;
            }
        }
    }

    if(lssAddress.identity.vendorID == 0 || lssAddress.identity.productCode == 0 || lssAddress.identity.revisionNumber == 0 || lssAddress.identity.serialNumber == 0)
    {
        /* Something went wrong when reading data from OD */
        return CO_ERROR_DATA_CORRUPT;
    }


    lssAddress.identity.productCode = OD_identity.productCode;
    lssAddress.identity.revisionNumber = OD_identity.revisionNumber;
    lssAddress.identity.serialNumber = OD_identity.serialNumber;
    lssAddress.identity.vendorID = OD_identity.vendorID;
    err = CO_LSSslave_init(
            CO->LSSslave,
            lssAddress,
            bitRate,
            nodeId,
            CO->CANmodule[0],
            rxidxcan_LSS,
            CO_CAN_ID_LSS_SRV,
            CO->CANmodule[0],
            txidxcan_LSS,
            CO_CAN_ID_LSS_CLI);

    return err;
}

/******************************************************************************/
CO_ReturnError_t CO_CANopenInit(
        uint8_t                 nodeId,
        CO_Context_t           *context)
{
    uint16_t i;
    CO_ReturnError_t err;

    uint16_t rxidxcan;
    uint16_t txidxcan;

    /* Verify CANopen Node-ID */
    if(nodeId<1 || nodeId>127) {
        return CO_ERROR_PARAMETERS;
    }

    uint8_t maxSub = 0;
    CO_OD_entryRecord_t *recP = NULL;

    rxidxcan = context->features.CO_NO_RPDO + \
                (uint16_t)context->features.CO_NO_TIME + \
                (uint16_t)context->features.CO_NO_EMERGENCY + \
                (uint16_t)context->features.CO_NO_SYNC + 1;

    txidxcan = context->features.CO_NO_TPDO + \
                (uint16_t)context->features.CO_NO_TIME + \
                (uint16_t)context->features.CO_NO_EMERGENCY + \
                (uint16_t)context->features.CO_NO_SYNC + \
                (uint16_t)context->features.CO_NO_NMT_MASTER;

    /*1200[1], Data Type: OD_SDOServerParameter_t, Array[1] */
    /* Poll over OD to find OD_SDOServerParameter */
    for (i = 0; i < context->numberOfODElements; i++)
    {
        if (context->usedOD[i].index == 0x1200 && context->usedOD[i].maxSubIndex > 0 && context->usedOD[i].attribute == 0 && context->usedOD[i].length == 0 && context->usedOD[i].pData != NULL)
        {
            /* Looks like this is the wanted record */
            recP = (CO_OD_entryRecord_t *)context->usedOD[i].pData;

            if (recP[0].pData != NULL)
            {
                maxSub = *((uint8_t *)recP[0].pData);
            }

            /* Exit early, we're done here (even if data is not valid)*/
            break;
        }
    }

    for (i = 0; i < (uint16_t)context->features.CO_NO_SDO_SERVER; i++)
    {
        uint32_t COB_IDClientToServer;
        uint32_t COB_IDServerToClient;
        if (i == 0)
        {
            /*Default SDO server must be located at first index*/
            COB_IDClientToServer = CO_CAN_ID_RSDO + nodeId;
            COB_IDServerToClient = CO_CAN_ID_TSDO + nodeId;
        }
        else
        {
            /* 1200[1], Data Type: OD_SDOServerParameter_t, Array[1] */
            COB_IDClientToServer = 0; //OD_SDOServerParameter[i].COB_IDClientToServer;
            COB_IDServerToClient = 0; //OD_SDOServerParameter[i].COB_IDServerToClient;

            // Okay, now we must select according to i, which points to the wanted server paramters
            // But only if this is in the valid range
            uint16_t subC2S = (i * 2) + 1;
            uint16_t subS2C = (i * 2) + 2;

            if (subS2C <= maxSub)
            {
                if(recP[subS2C].length == 4 && recP[subS2C].pData != NULL)
                {
                    COB_IDServerToClient = *((uint32_t *)recP[subS2C].pData; // Subindex subS2C holds COB_IDServerToClient as uint32_t
                }
                if(recP[subC2S].length == 4 && recP[subC2S].pData != NULL)
                {
                    COB_IDClientToServer = *((uint32_t *)recP[subC2S].pData; // Subindex subC2S holds COB_IDClientToServer as uint32_t
                }
            }
        }

        err = CO_ERROR_DATA_CORRUPT; /* Mark already, so in case no COBIDs are assigned, we know that OD is corrupted */

        if(COB_IDClientToServer != 0 && COB_IDServerToClient != 0)
        {
            err = CO_SDO_init(
                CO->SDO[i],
                COB_IDClientToServer,
                COB_IDServerToClient,
                OD_H1200_SDO_SERVER_PARAM + i,
                i == 0 ? 0 : CO->SDO[0],
                context->usedOD,
                context->CO_OD_NoOfElements,
                CO_SDO_ODExtensions,
                nodeId,
                CO->CANmodule[0],
                rxidxcan + i,
                CO->CANmodule[0],
                txidxcan + i);
        }
        
        /* Exit early in case things go wrong */
        if(err != CO_ERROR_NO)
        {
            return err;
        }
    }

    /* This check is normally not really needed, but I use it as an optimization *
     * since a lot of stack based variables are encapsulated and freed immediately *
     * so on less powerful devices, stack usage is reduced (blankm 20191213) */
    if (context->features.CO_NO_EMERGENCY > 0) 
    {
        /*2100, Data Type: OCTET_STRING, Array[10] */
        /* Poll over OD to find OD_errorStatusBits */
        /*1001, Data Type: UNSIGNED8 */
        /* Poll over OD to find OD_errorRegister */
        /*1003, Data Type: UNSIGNED32, Array[8] */
        /* Poll over OD to find OD_preDefinedErrorField */

        /*1015, Data Type: UNSIGNED16 */
        /* We also need (for global usage) OD_inhibitTimeEMCY */

        uint16_t errorStatusBitsLen = 0;
        OCTET_STRING *errorStatusBitsP = NULL;
        uint8_t *errorRegisterP = NULL;
        uint16_t preDefinedErrorFieldLen = 0;
        uint32_t *preDefinedErrorFieldP = NULL;

        inhibitTimeEMCY = NULL;

        for (i = 0; i < context->numberOfODElements; i++)
        {
            if (context->usedOD[i].index == 0x2100 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length != 0 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                errorStatusBitsP = (OCTET_STRING *)context->usedOD[i].pData;
                errorStatusBitsLen = context->usedOD[i].length;
            }

            if (context->usedOD[i].index == 0x1001 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 1 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                errorRegisterP = (uint8_t *)context->usedOD[i].pData;
                errorRegister = errorRegisterP;
            }

            if (context->usedOD[i].index == 0x1003 && context->usedOD[i].maxSubIndex != 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted array */
                preDefinedErrorFieldP = (uint32_t *)context->usedOD[i].pData;
                preDefinedErrorFieldLen = context->usedOD[i].length;
            }

            if (context->usedOD[i].index == 0x1015 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 2 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                inhibitTimeEMCY = (uint16_t *)context->usedOD[i].pData;
            }

            /* Check if all found */
            if (errorRegisterP != NULL && errorStatusBitsP != NULL && preDefinedErrorFieldP != NULL || inhibitTimeEMCY != NULL)
            {
                /* Exit early, we're done here */
                break;
            }
        }

        rxidxcan = (uint16_t)context->features.CO_NO_SYNC + 1;
        txidxcan = (uint16_t)context->features.CO_NO_SYNC +
                   (uint16_t)context->features.CO_NO_NMT_MASTER;

        if (errorStatusBitsP == NULL || errorRegisterP == NULL || preDefinedErrorFieldP == NULL || inhibitTimeEMCY == NULL)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        err = CO_EM_init(
            CO->em,
            CO->emPr,
            CO->SDO[0],
            errorStatusBitsP,
            errorStatusBitsLen,
            errorRegisterP,
            preDefinedErrorFieldP,
            preDefinedErrorFieldLen,
            CO->CANmodule[0],
            rxidxcan,
            CO->CANmodule[0],
            txidxcan,
            (uint16_t)CO_CAN_ID_EMERGENCY + nodeId);

        if (err)
        {
            return err;
        }
    }

    txidxcan = (uint16_t)context->features.CO_NO_SDO_CLIENT + \
                (uint16_t)context->features.CO_NO_SDO_SERVER + \
                context->features.CO_NO_TPDO + \
                (uint16_t)context->features.CO_NO_TIME + \
                (uint16_t)context->features.CO_NO_EMERGENCY + \
                (uint16_t)context->features.CO_NO_SYNC + \
                (uint16_t)context->features.CO_NO_NMT_MASTER;

    err = CO_NMT_init(
            CO->NMT,
            CO->emPr,
            nodeId,
            500,
            CO->CANmodule[0],
            CO_RXCAN_NMT,
            CO_CAN_ID_NMT_SERVICE,
            CO->CANmodule[0],
            txidxcan,
            CO_CAN_ID_HEARTBEAT + nodeId);

    if (err)
    {
        return err;
    }

    if (context->features.CO_NO_NMT_MASTER > 0)
    {
        NMTM_txBuff = CO_CANtxBufferInit(/* return pointer to 8-byte CAN data buffer, which should be populated */
                CO->CANmodule[0], /* pointer to CAN module used for sending this message */
                CO_TXCAN_NMT,     /* index of specific buffer inside CAN module */
                0x0000,           /* CAN identifier */
                0,                /* rtr */
                2,                /* number of data bytes */
                0);               /* synchronous message flag bit */
    }

    if (context->features.CO_NO_LSS_CLIENT > 0)
    {
        rxidxcan = (uint16_t)context->features.CO_NO_HB_CONS +
                    (uint16_t)context->features.CO_NO_SDO_CLIENT +
                    (uint16_t)context->features.CO_NO_SDO_SERVER +
                    context->features.CO_NO_RPDO +
                    (uint16_t)context->features.CO_NO_TIME +
                    (uint16_t)context->features.CO_NO_EMERGENCY +
                    (uint16_t)context->features.CO_NO_SYNC + 1;

        txidxcan = (uint16_t)context->features.CO_NO_HB_PROD +
                    (uint16_t)context->features.CO_NO_SDO_CLIENT +
                    (uint16_t)context->features.CO_NO_SDO_SERVER +
                    context->features.CO_NO_TPDO +
                    (uint16_t)context->features.CO_NO_TIME +
                    (uint16_t)context->features.CO_NO_EMERGENCY +
                    (uint16_t)context->features.CO_NO_SYNC +
                    (uint16_t)context->features.CO_NO_NMT_MASTER;

        err = CO_LSSmaster_init(
            CO->LSSmaster,
            CO_LSSmaster_DEFAULT_TIMEOUT,
            CO->CANmodule[0],
            rxidxcan,
            CO_CAN_ID_LSS_CLI,
            CO->CANmodule[0],
            txidxcan,
            CO_CAN_ID_LSS_SRV);

        if (err)
        {
            return err;
        }
    }

    /* This check is normally not really needed, but I use it as an optimization *
     * since a lot of stack based variables are encapsulated and freed immediately *
     * so on less powerful devices, stack usage is reduced (blankm 20191213) */
    if (context->features.CO_NO_SYNC > 0)
    {
        /*1005, Data Type: UNSIGNED32 */
        /* Poll over OD to find OD_COB_ID_SYNCMessage */
        /*1006, Data Type: UNSIGNED32 */
        /* Poll over OD to find OD_communicationCyclePeriod */
        /*1019, Data Type: UNSIGNED8 */
        /* Poll over OD to find OD_synchronousCounterOverflowValue */

        /* Addtionally, for global usage: */        
        /* uint16_t *producerHeartbeatTime;     1017, Data Type: UNSIGNED16 */
        /* uint32_t *NMTStartup;                1F80, Data Type: UNSIGNED32 */
        /* uint8_t  *errorBehavior;             1029, Data Type: UNSIGNED8, Array[6] */
        /* uint32_t *synchronousWindowLength;   1007, Data Type: UNSIGNED32 */

        uint32_t *COB_ID_SYNCMessageP = NULL;
        uint32_t *communicationCyclePeriodP = NULL;
        uint8_t *synchronousCounterOverflowValueP = NULL;

        for (i = 0; i < context->numberOfODElements; i++)
        {
            if (context->usedOD[i].index == 0x1005 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                COB_ID_SYNCMessageP = (uint32_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1007 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                synchronousWindowLength = (uint32_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1029 && context->usedOD[i].maxSubIndex > 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 1 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted array */
                errorBehavior = (uint8_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1F80 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                NMTStartup = (uint32_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1017 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 2 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                producerHeartbeatTime = (uint16_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1006 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                communicationCyclePeriodP = (uint32_t *)context->usedOD[i].pData;
            }

            if (context->usedOD[i].index == 0x1019 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 1 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                synchronousCounterOverflowValueP = (uint8_t *)context->usedOD[i].pData;
            }

            if (COB_ID_SYNCMessageP != NULL && communicationCyclePeriodP != NULL && synchronousCounterOverflowValueP != NULL && producerHeartbeatTime != NULL && NMTStartup != NULL && synchronousWindowLength != NULL && errorBehavior != NULL)
            {
                /* Exit early, we're done here */
                break;
            }
        }

        if (COB_ID_SYNCMessageP == NULL || communicationCyclePeriodP == NULL || synchronousCounterOverflowValueP == NULL || producerHeartbeatTime == NULL || NMTStartup == NULL || synchronousWindowLength == NULL || errorBehavior == NULL)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        err = CO_SYNC_init(
            CO->SYNC,
            CO->em,
            CO->SDO[0],
            &CO->NMT->operatingState,
            *COB_ID_SYNCMessageP,
            *communicationCyclePeriodP,
            *synchronousCounterOverflowValueP,
            CO->CANmodule[0],
            CO_RXCAN_SYNC,
            CO->CANmodule[0],
            (uint16_t)context->features.CO_TXCAN_SYNC);

        if (err)
        {
            return err;
        }
    }

    /* This check is normally not really needed, but I use it as an optimization *
     * since a lot of stack based variables are encapsulated and freed immediately *
     * so on less powerful devices, stack usage is reduced (blankm 20191213) */
    if (context->features.CO_NO_TIME > 0)
    {
        /*1012, Data Type: UNSIGNED32 */
        /* Poll over OD to find OD_COB_ID_TIME */
        uint32_t *COB_ID_TIMEP = NULL;

        for (i = 0; i < context->numberOfODElements; i++)
        {
            if (context->usedOD[i].index == 0x1012 && context->usedOD[i].maxSubIndex == 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted entry */
                COB_ID_TIMEP = (uint32_t *)context->usedOD[i].pData;

                /* Exit early, we're done here */
                break;
            }
        }

        if (COB_ID_TIMEP == NULL)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        rxidxcan = (uint16_t)context->features.CO_NO_EMERGENCY +
                   (uint16_t)context->features.CO_NO_SYNC + 1;

        txidxcan = (uint16_t)context->features.CO_NO_EMERGENCY +
                   (uint16_t)context->features.CO_NO_SYNC +
                   (uint16_t)context->features.CO_NO_NMT_MASTER;

        err = CO_TIME_init(
            CO->TIME,
            CO->em,
            CO->SDO[0],
            &CO->NMT->operatingState,
            *COB_ID_TIMEP,
            0,
            CO->CANmodule[0],
            rxidxcan,
            CO->CANmodule[0],
            txidxcan);

        if (err)
        {
            return err;
        }
    }

    rxidxcan = (uint16_t)context->features.CO_NO_TIME +
                (uint16_t)context->features.CO_NO_EMERGENCY +
                (uint16_t)context->features.CO_NO_SYNC + 1;

    for (i = 0; i < context->features.CO_NO_RPDO; i++)
    {
        CO_CANmodule_t *CANdevRx = CO->CANmodule[0];
        uint16_t CANdevRxIdx = rxidxcan + i;

        /*1400[4], Data Type: OD_RPDOCommunicationParameter_t, Array[4] */
        /* Poll over OD to find OD_RPDOCommunicationParameter */

        /*1600[4], Data Type: OD_RPDOMappingParameter_t, Array[4] */
        /* Poll over OD to find OD_RPDOMappingParameter */

        recP = NULL;         // reuse the existing recordPointer
        maxSub = 0;          // reuse the existing variable on how many sub indexes are available
        uint8_t maxSub2 = 0; // We need moe since we have to records to track
        CO_OD_entryRecord_t *recP2 = NULL;

        uint16_t searchindex_1400 = 0x1400 + i;
        uint16_t searchindex_1600 = 0x1600 + i;

        for (uint16_t j = 0; j < context->numberOfODElements; j++)
        {
            if (context->usedOD[j].index == searchindex_1400 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
            {
                /* Looks like this is the wanted record */
                recP = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                if (recP[0].pData != NULL && recP[0].length == 1)
                {
                    maxSub = *((uint8_t *)recP[0].pData);
                }
            }

            if (context->usedOD[j].index == searchindex_1600 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
            {
                /* Looks like this is the wanted record */
                recP2 = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                if (recP2[0].pData != NULL && recP2[0].length == 1)
                {
                    maxSub2 = *((uint8_t *)recP2[0].pData);
                }
            }

            if (recP2 != NULL && recP != NULL)
            {
                /* Exit early, we're done here (even if data is maybe not valid)*/
                break;
            }
        }

        /* Check if data is valid */
        // TODO blankm 20191213: Maybe we could do the sizecheck against sizeof and the typedefs then MUST be in default_OD.
        if (recP2 == NULL || recP == NULL || maxSub < 2 || maxSub2 == 0)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        /* recP now points to CO_OD_entryRecord_t, which points to the contents of CO_RPDOCommPar_t */
        /* recP2 now points to CO_OD_entryRecord_t, which points to the contents of CO_RPDOMapPar_t */

        /* Check Data consistency */
        /* But we only check for the mandatory entries. We can't know if the others are present, *
         * only if we would check max Subindex. But they are currently no used by the stack and *
         * also we can assume that if the first ones where okay the others will be most likely, too */
        if(recP[1].pData == NULL || recP[1].length != 4 || recP[2].pData == NULL || recP[2].length != 1)
        {
            return CO_ERROR_DATA_CORRUPT;
        }
        if(recP2[1].pData == NULL || recP2[1].length != 4)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        /* ATTENTION: Data has to be passed by reference, not copy!! */
        /* We do not really have pointers to the struct, only pointers to the elements of the struct */
        /* So we hope that pointing to the first element points correctly to the struct */
        // FIXME: Try to find a better way to get the struct pointer.
        err = CO_RPDO_init(
                CO->RPDO[i],
                CO->em,
                CO->SDO[0],
                CO->SYNC,
               &CO->NMT->operatingState,
                nodeId,
                ((i<4) ? (CO_CAN_ID_RPDO_1+i*0x100) : 0),
                0,
                (CO_RPDOCommPar_t*) recP[0]->pData,
                (CO_RPDOMapPar_t*) recP2[0]->pData,
                OD_H1400_RXPDO_1_PARAM+i,
                OD_H1600_RXPDO_1_MAPPING+i,
                CANdevRx,
                CANdevRxIdx);

        if (err)
        {
            return err;
        }
    }

    txidxcan = (uint16_t)context->features.CO_NO_TIME +
                (uint16_t)context->features.CO_NO_EMERGENCY +
                (uint16_t)context->features.CO_NO_SYNC +
                (uint16_t)context->features.CO_NO_NMT_MASTER;

    for (i = 0; i < context->features.CO_NO_TPDO; i++)
    {
        /*1800[4], Data Type: OD_TPDOCommunicationParameter_t, Array[4] */
        /* Poll over OD to find OD_TPDOCommunicationParameter */

        /*1A00[4], Data Type: OD_TPDOMappingParameter_t, Array[4] */
        /* Poll over OD to find OD_TPDOMappingParameter */

        recP = NULL;         // reuse the existing recordPointer
        maxSub = 0;          // reuse the existing variable on how many sub indexes are available
        uint8_t maxSub2 = 0; // We need more since we have to records to track
        CO_OD_entryRecord_t *recP2 = NULL;

        uint16_t searchindex_1800 = 0x1800 + i;
        uint16_t searchindex_1A00 = 0x1A00 + i;

        for (uint16_t j = 0; j < context->numberOfODElements; j++)
        {
            if (context->usedOD[j].index == searchindex_1800 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
            {
                /* Looks like this is the wanted record */
                recP = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                if (recP[0].pData != NULL && recP[0].length == 1)
                {
                    maxSub = *((uint8_t *)recP[0].pData);
                }
            }

            if (context->usedOD[j].index == searchindex_1A00 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
            {
                /* Looks like this is the wanted record */
                recP2 = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                if (recP2[0].pData != NULL && recP2[0].length == 1)
                {
                    maxSub2 = *((uint8_t *)recP2[0].pData);
                }
            }

            if (recP2 != NULL && recP != NULL)
            {
                /* Exit early, we're done here (even if data is maybe not valid) */
                break;
            }
        }

        /* Check if data is valid */
        // TODO blankm 20191213: Maybe we could do the sizecheck against sizeof and the typedefs then MUST be in default_OD.
        if (recP2 == NULL || recP == NULL || maxSub < 2 || maxSub2 == 0)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        /* recP now points to CO_OD_entryRecord_t, which points to the contents of CO_TPDOCommPar_t */
        /* recP2 now points to CO_OD_entryRecord_t, which points to the contents of CO_TPDOMapPar_t */

        /* Check Data consistency */
        /* But we only check for the mandatory entries. We can't know if the others are present, *
         * only if we would check max Subindex. But they are currently no used by the stack and *
         * also we can assume that if the first ones where okay the others will be most likely, too */
        if(recP[1].pData == NULL || recP[1].length != 4 || recP[2].pData == NULL || recP[2].length != 1)
        {
            return CO_ERROR_DATA_CORRUPT;
        }
        if(recP2[1].pData == NULL || recP2[1].length != 4)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        /* ATTENTION: Data has to be passed by reference, not copy!! */
        /* We do not really have pointers to the struct, only pointers to the elements of the struct */
        /* So we hope that pointing to the first element points correctly to the struct */
        // FIXME: Try to find a better way to get the struct pointer.

        err = CO_TPDO_init(
            CO->TPDO[i],
            CO->em,
            CO->SDO[0],
            &CO->NMT->operatingState,
            nodeId,
            ((i < 4) ? (CO_CAN_ID_TPDO_1 + i * 0x100) : 0),
            0,
            (CO_TPDOCommPar_t *)recP[0]->pData,
            (CO_TPDOMapPar_t *)recP2[0]->pData,
            OD_H1800_TXPDO_1_PARAM + i,
            OD_H1A00_TXPDO_1_MAPPING + i,
            CO->CANmodule[0],
            txidxcan + i);

        if (err)
        {
            return err;
        }
    }

    if (context->features.CO_NO_HB_CONS > 0)
    {
        rxidxcan = (uint16_t)context->features.CO_NO_SDO_CLIENT +
                   (uint16_t)context->features.CO_NO_SDO_SERVER +
                   context->features.CO_NO_RPDO +
                   (uint16_t)context->features.CO_NO_TIME +
                   (uint16_t)context->features.CO_NO_EMERGENCY +
                   (uint16_t)context->features.CO_NO_SYNC + 1;

        /*1016, Data Type: UNSIGNED32, Array[4] */
        /* Poll over OD to find OD_consumerHeartbeatTime */

        uint16_t consumerHeartbeatTimeLen = 0;
        uint32_t *consumerHeartbeatTimeP = NULL;

        for (i = 0; i < context->numberOfODElements; i++)
        {
            if (context->usedOD[i].index == 0x1016 && context->usedOD[i].maxSubIndex != 0 && context->usedOD[i].attribute != 0 && context->usedOD[i].length == 4 && context->usedOD[i].pData != NULL)
            {
                /* Looks like this is the wanted array */
                consumerHeartbeatTimeP = (uint32_t *)context->usedOD[i].pData;
                consumerHeartbeatTimeLen = context->usedOD[i].length;

                /* Exit early, we're done here */
                break;
            }
        }

        /* Check for valid data */
        if (consumerHeartbeatTimeLen != (uint16_t)context->features.CO_NO_HB_CONS)
        {
            /* Seems like there is a config mismatch */
            return CO_ERROR_DATA_CORRUPT;
        }

        err = CO_HBconsumer_init(
            CO->HBcons,
            CO->em,
            CO->SDO[0],
            consumerHeartbeatTimeP,
            CO_HBcons_monitoredNodes,
            context->features.CO_NO_HB_CONS,
            CO->CANmodule[0],
            rxidxcan);

        if (err)
        {
            return err;
        }
    }

    if(context->features.CO_NO_SDO_CLIENT > 0)
    {
        rxidxcan = (uint16_t)context->features.CO_NO_SDO_SERVER +
                    context->features.CO_NO_RPDO +
                    (uint16_t)context->features.CO_NO_TIME +
                    (uint16_t)context->features.CO_NO_EMERGENCY +
                    (uint16_t)context->features.CO_NO_SYNC + 1;

        txidxcan = (uint16_t)context->features.CO_NO_SDO_SERVER +
                    context->features.CO_NO_TPDO +
                    (uint16_t)context->features.CO_NO_TIME +
                    (uint16_t)context->features.CO_NO_EMERGENCY +
                    (uint16_t)context->features.CO_NO_SYNC +
                    (uint16_t)context->features.CO_NO_NMT_MASTER;

        for (i = 0; i < (uint16_t)context->features.CO_NO_SDO_CLIENT; i++)
        {
            /*1280[4], Data Type: OD_SDOClientParameter_t, Array[4] */
            /* Poll over OD to find OD_SDOClientParameter */

            recP = NULL;         // reuse the existing recordPointer
            maxSub = 0;          // reuse the existing variable on how many sub indexes are available

            uint16_t searchindex_1280 = 0x1280 + i;

            for (uint16_t j = 0; j < context->numberOfODElements; j++)
            {
                if (context->usedOD[j].index == searchindex_1280 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
                {
                    /* Looks like this is the wanted record */
                    recP = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                    if (recP[0].pData != NULL && recP[0].length == 1)
                    {
                        maxSub = *((uint8_t *)recP[0].pData);
                    }

                    /* Exit early, we're done here (even if data is maybe not valid) */
                    break;
                }
            }

            /* Check if data is valid */
            if (recP == NULL || maxSub == 0)
            {
                return CO_ERROR_DATA_CORRUPT;
            }

            /* recP now points to CO_OD_entryRecord_t, which points to the contents of OD_SDOClientParameter_t */

            /* Check Data consistency */
            if (maxSub != 3 || recP[1].pData == NULL || recP[1].length != 4 || recP[2].pData == NULL || recP[2].length != 4 || recP[3].pData == NULL || recP[3].length != 1)
            {
                return CO_ERROR_DATA_CORRUPT;
            }            

            /* ATTENTION: Data has to be passed by reference, not copy!! */
            /* We do not really have pointers to the struct, only pointers to the elements of the struct */
            /* So we hope that pointing to the first element points correctly to the struct */
            // FIXME: Try to find a better way to get the struct pointer.

            err = CO_SDOclient_init(
                CO->SDOclient[i],
                CO->SDO[0],
                (CO_SDOclientPar_t *)recP[0].pData,
                CO->CANmodule[0],
                rxidxcan + i,
                CO->CANmodule[0],
                txidxcan + i);

            if (err)
            {
                return err;
            }
        }
    }

    if(context->features.CO_NO_TRACE > 0)
    {
        // FIXME blankm 20191213: I don't know what OD entry 2400 traceEnable is for, it is not used throughout the stack?
              
        for (i = 0; i < (uint16_t)context->features.CO_NO_TRACE; i++)
        {
            /*2301[2], Data Type: OD_traceConfig_t, Array[2] */
            /* Poll over OD to find OD_traceConfig */

            /*2401[2], Data Type: OD_trace_t, Array[2] */
            /* Poll over OD to find OD_trace */

            recP = NULL;         // reuse the existing recordPointer
            maxSub = 0;          // reuse the existing variable on how many sub indexes are available
            uint8_t maxSub2 = 0; // We need more since we have to records to track
            CO_OD_entryRecord_t *recP2 = NULL;

            // We must start @ 2301/2401! This is not an error. 
            // 0x2400 contains traceEnable and subsquently start is at 0x2401 and for equality 0x2301 instead of 0x2300.
            uint16_t searchindex_2300 = 0x2301 + i;
            uint16_t searchindex_2400 = 0x2401 + i;

            for (uint16_t j = 0; j < context->numberOfODElements; j++)
            {
                if (context->usedOD[j].index == searchindex_2300 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
                {
                    /* Looks like this is the wanted record */
                    recP = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                    if (recP[0].pData != NULL && recP[0].length == 1)
                    {
                        maxSub = *((uint8_t *)recP[0].pData);
                    }
                }

                if (context->usedOD[j].index == searchindex_2400 && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
                {
                    /* Looks like this is the wanted record */
                    recP2 = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                    if (recP2[0].pData != NULL && recP2[0].length == 1)
                    {
                        maxSub2 = *((uint8_t *)recP2[0].pData);
                    }
                }

                if (recP2 != NULL && recP != NULL)
                {
                    /* Exit early, we're done here (even if data is maybe not valid) */
                    break;
                }
            }

            /* Check if data is valid */
            // TODO blankm 20191213: Maybe we could do the sizecheck against sizeof and the typedefs then MUST be in default_OD.
            if (recP2 == NULL || recP == NULL || maxSub != 8 || maxSub2 != 6)
            {
                return CO_ERROR_DATA_CORRUPT;
            }

            /* recP now points to CO_OD_entryRecord_t, which points to the contents of OD_traceConfig_t */
            /* recP2 now points to CO_OD_entryRecord_t, which points to the contents of OD_trace_t */

            /* Check Data consistency */
            if (recP[1].pData == NULL || recP[1].length != 4 || recP[2].pData == NULL || recP[2].length != 1 || recP[5].pData == NULL || recP[5].length != 4 || recP[6].pData == NULL || recP[6].length != 1 || recP[7].pData == NULL || recP[7].length != 1  || recP[8].pData == NULL || recP[8].length != 4)
            {
                return CO_ERROR_DATA_CORRUPT;
            }
            if (recP2[1].pData == NULL || recP2[1].length != 4 || recP2[2].pData == NULL || recP2[2].length != 4 || recP2[3].pData == NULL || recP2[3].length != 4 || recP2[4].pData == NULL || recP2[4].length != 4 || recP2[6].pData == NULL || recP2[6].length != 4)
            {
                return CO_ERROR_DATA_CORRUPT;
            }

            CO_trace_init(
                CO->trace[i],
                CO->SDO[0],
                *((uint8_t *)recP[2].pData),
                CO_traceTimeBuffers[i],
                CO_traceValueBuffers[i],
                CO_traceBufferSize[i],
                ((uint32_t *)recP[5].pData),
                ((uint8_t *)recP[6].pData),
                ((uint8_t *)recP[7].pData),
                ((uint32_t *)recP[8].pData),
                ((int32_t *)rec[2].pData),
                ((int32_t *)rec[3].pData),
                ((int32_t *)rec[4].pData),
                ((uint32_t *)rec[6].pData),
                OD_INDEX_TRACE_CONFIG + i,
                OD_INDEX_TRACE + i);
        }
    }

    return CO_ERROR_NO;
}


/******************************************************************************/
CO_ReturnError_t CO_init(
        void                   *CANdriverState,
        uint8_t                 nodeId,
        uint16_t                bitRate,
        CO_Context_t           *context)
{
    CO_ReturnError_t err;

    err = CO_new(context);
    if (err) {
        return err;
    }

    err = CO_CANinit(CANdriverState, bitRate, context);
    if (err) {
        CO_delete(CANdriverState);
        return err;
    }

    err = CO_CANopenInit(nodeId, context);
    if (err) {
        CO_delete(CANdriverState);
        return err;
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_delete(
        void                   *CANdriverState, 
        CO_Context_t           *context)
{
    uint16_t i;

    CO_CANsetConfigurationMode(CANdriverState);
    CO_CANmodule_disable(CO->CANmodule[0]);

    for (i = 0; i < (uint16_t)context->features.CO_NO_TRACE; i++)
    {
        COfree(CO->trace[i]);
        COfree(CO_traceTimeBuffers[i]);
        COfree(CO_traceValueBuffers[i]);
    }

    for (i = 0; i < (uint16_t)context_features.CO_NO_SDO_CLIENT; i++)
    {
        COfree(CO->SDOclient[i]);
    }

    if (context->features.CO_NO_LSS_SERVER > 0)
    {
        COfree(CO->LSSslave);
    }
    if (context->features.CO_NO_LSS_CLIENT > 0)
    {
        COfree(CO->LSSmaster);
    }
    COfree(CO_HBcons_monitoredNodes);
    COfree(CO->HBcons);
    for (i = 0; i < context->features.CO_NO_RPDO; i++)
    {
        COfree(CO->RPDO[i]);
    }
    for (i = 0; i < context->features.CO_NO_TPDO; i++)
    {
        COfree(CO->TPDO[i]);
    }
    COfree(CO->SYNC);
    COfree(CO->NMT);
    COfree(CO->emPr);
    COfree(CO->em);
    COfree(CO_SDO_ODExtensions);
    for (i = 0; i < (uint16_t)context->features.CO_NO_SDO_SERVER; i++)
    {
        COfree(CO->SDO[i]);
    }
    COfree(CO_CANmodule_txArray0);
    COfree(CO_CANmodule_rxArray0);
    COfree(CO->CANmodule[0]);
    COfree(CO->TIME);
    CO = NULL;
}

/******************************************************************************/
CO_NMT_reset_cmd_t CO_process(
        CO_t                   *CO_this,
        uint16_t                timeDifference_ms,
        uint16_t               *timerNext_ms)
{
    uint8_t i;
    bool_t NMTisPreOrOperational = false;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
#ifdef CO_USE_LEDS
    static uint16_t ms50 = 0;
#endif /* CO_USE_LEDS */

    if(CO_this->NMT->operatingState == CO_NMT_PRE_OPERATIONAL || CO_this->NMT->operatingState == CO_NMT_OPERATIONAL)
        NMTisPreOrOperational = true;

#ifdef CO_USE_LEDS
    ms50 += timeDifference_ms;
    if(ms50 >= 50){
        ms50 -= 50;
        CO_NMT_blinkingProcess50ms(CO_this->NMT);
    }
#endif /* CO_USE_LEDS */
    if(timerNext_ms != NULL){
        if(*timerNext_ms > 50){
            *timerNext_ms = 50;
        }
    }


    for(i=0; i<CO_NO_SDO_SERVER; i++){
        CO_SDO_process(
                CO_this->SDO[i],
                NMTisPreOrOperational,
                timeDifference_ms,
                1000,
                timerNext_ms);
    }

    CO_EM_process(
            CO_this->emPr,
            NMTisPreOrOperational,
            timeDifference_ms * 10,
           *inhibitTimeEMCY,
            timerNext_ms);


    reset = CO_NMT_process(
            CO_this->NMT,
            timeDifference_ms,
           *producerHeartbeatTime,
           *NMTStartup,
           *errorRegister,
           *errorBehavior,
            timerNext_ms);


    CO_HBconsumer_process(
            CO_this->HBcons,
            NMTisPreOrOperational,
            timeDifference_ms);
    
    
    CO_TIME_process(
            CO_this->TIME, 
            timeDifference_ms);

    return reset;
}


/******************************************************************************/
bool_t CO_process_SYNC_RPDO(
        CO_t                   *CO_this,
        uint32_t                timeDifference_us,
        CO_Context_t           *context)
{
    uint16_t i;
    bool_t syncWas = false;

    switch(CO_SYNC_process(CO_this->SYNC, timeDifference_us, *synchronousWindowLength)){
        case 1:     //immediately after the SYNC message
            syncWas = true;
            break;
        case 2:     //outside SYNC window
            CO_CANclearPendingSyncPDOs(CO_this->CANmodule[0]);
            break;
        default:
            break;
    }

    for (i = 0; i < context->features.CO_NO_RPDO; i++)
    {
        CO_RPDO_process(CO_this->RPDO[i], syncWas);
    }

    return syncWas;
}


/******************************************************************************/
void CO_process_TPDO(
        CO_t                   *CO_this,
        bool_t                  syncWas,
        uint32_t                timeDifference_us,
        CO_Context_t           *context)
{
    uint16_t i;

    /* Verify PDO Change Of State and process PDOs */
    for (i = 0; i < context->features.CO_NO_TPDO; i++)
    {
        if (!CO_this->TPDO[i]->sendRequest)
            CO_this->TPDO[i]->sendRequest = CO_TPDOisCOS(CO_this->TPDO[i]);
        CO_TPDO_process(CO_this->TPDO[i], CO_this->SYNC, syncWas, timeDifference_us);
    }
}
