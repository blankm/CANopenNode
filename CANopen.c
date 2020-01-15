/*
 * Main CANopen stack file. It combines Object dictionary (CO_OD) and all other
 * CANopen source files. Configuration information are read from CO_OD.h file.
 *
 * @file        CANopen.c
 * @ingroup     CO_CANopen
 * @author      Janez Paternoster
 * @copyright   2010 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

static CO_t COO;
CO_t *CO = NULL;

CO_CANrx_t *CO_CANmodule_rxArray0;
CO_CANtx_t *CO_CANmodule_txArray0;
CO_OD_extension_t *CO_SDO_ODExtensions;
CO_HBconsNode_t *CO_HBcons_monitoredNodes;

#ifndef CO_TRACE_BUFFER_SIZE_FIXED
#define CO_TRACE_BUFFER_SIZE_FIXED 100
#endif

// static uint32_t *CO_traceTimeBuffers;
// static int32_t *CO_traceValueBuffers;
// static uint32_t *CO_traceBufferSize;

/* Verify features from CO_OD *************************************************/
    /* generate error, if features are not correctly configured for this project */
    #if        CO_NO_NMT_MASTER                           >  1     \
            || CO_NO_SYNC                                 >  1     \
            || CO_NO_EMERGENCY                            != 1     \
            || CO_NO_SDO_SERVER                           == 0     \
            || CO_NO_TIME                                 >  1     \
            || CO_NO_SDO_CLIENT                           > 128    \
            || (CO_NO_RPDO < 1 || CO_NO_RPDO > 0x200)              \
            || (CO_NO_TPDO < 1 || CO_NO_TPDO > 0x200)              \
            || ODL_consumerHeartbeatTime_arrayLength      == 0     \
            || ODL_errorStatusBits_stringLength           < 10     \
            || CO_NO_LSS_SERVER                           >  1     \
            || CO_NO_LSS_CLIENT                           >  1     \
            || (CO_NO_LSS_SERVER > 0 && CO_NO_LSS_CLIENT > 0)
        #error Features from CO_OD.h file are not corectly configured for this project!
    #endif

/* The following idexes are never calculated, but are fixed */
#define CO_RXCAN_NMT       0                /*  index for NMT message */
#define CO_RXCAN_SYNC      1                /*  index for SYNC message */
#define CO_TXCAN_NMT       0                /*  index for NMT master message */

static inline uint16_t CO_RXCAN_NO_MSGS(CO_Context_t * context)
{
    //(1+CO_NO_SYNC+CO_NO_EMERGENCY+CO_NO_TIME+CO_NO_RPDO+CO_NO_SDO_SERVER+CO_NO_SDO_CLIENT+CO_NO_HB_CONS)
    uint16_t rxcnt = 1;
    rxcnt += context->features->CO_NO_SYNC;
    rxcnt += context->features->CO_NO_EMERGENCY;
    rxcnt += context->features->CO_NO_TIME;
    rxcnt += context->features->CO_NO_RPDO;
    rxcnt += context->features->CO_NO_SDO_SERVER;
    rxcnt += context->features->CO_NO_SDO_CLIENT;
    rxcnt += context->features->CO_NO_HB_CONS;
    return rxcnt;
}

static inline uint16_t CO_TXCAN_NO_MSGS(CO_Context_t * context)
{
    //(CO_NO_NMT_MASTER+CO_NO_SYNC+CO_NO_EMERGENCY+CO_NO_TIME+CO_NO_TPDO+CO_NO_SDO_SERVER+CO_NO_SDO_CLIENT+CO_NO_HB_PROD+CO_NO_LSS_SERVER+CO_NO_LSS_CLIENT)
    uint16_t txcnt = 0;
    txcnt += context->features->CO_NO_NMT_MASTER;
    txcnt += context->features->CO_NO_SYNC;
    txcnt += context->features->CO_NO_EMERGENCY;
    txcnt += context->features->CO_NO_TIME;
    txcnt += context->features->CO_NO_TPDO;
    txcnt += context->features->CO_NO_SDO_SERVER;
    txcnt += context->features->CO_NO_SDO_CLIENT;
    txcnt += context->features->CO_NO_HB_PROD;
    txcnt += context->features->CO_NO_LSS_SERVER;
    txcnt += context->features->CO_NO_LSS_CLIENT;
    return txcnt;
}


CO_ReturnError_t CO_verifyFeatures(CO_Context_t * context)
{
    /* Check if all features make sense */
    uint8_t errs = 0;
    if(context->features->CO_NO_SYNC != 1)
        errs++;
    if(context->features->CO_NO_NMT_MASTER > 1)
        errs++;
    if(context->features->CO_NO_EMERGENCY != 1)
        errs++;
    if(context->features->CO_NO_SDO_SERVER == 0)
        errs++;
    if(context->features->CO_NO_SDO_CLIENT > 128)
        errs++;
    if(context->features->CO_NO_RPDO < 1 || context->features->CO_NO_RPDO > 0x200)
        errs++;
    if(context->features->CO_NO_TPDO < 1 || context->features->CO_NO_TPDO > 0x200)
        errs++;
    if(context->features->CO_NO_LSS_SERVER > 1)
        errs++;
    if(context->features->CO_NO_LSS_CLIENT > 1)
        errs++;
    if(context->features->CO_NO_TIME > 1)
        errs++;
    // TODO: I guess we don't allow LSS Server and Client at the same time, I guess different contexts are needed for different roles
    if((context->features->CO_NO_LSS_CLIENT > 0) && (context->features->CO_NO_LSS_SERVER > 0))
        errs++;

    /* Check if array size ODL_consumerHeartbeatTime_arrayLength == 0 */
    /* This is equivalent to */
    if(context->features->CO_NO_HB_CONS == 0)
    {
        errs++;
    }
    
    /* Now comes the more complicated part, we have to check inside OD */
    
    /* errorStatusBits string length, OCTET_STRING, len = 10, idx 0x2100 */
    /* Poll over OD to find errorStatusBits */
    uint16_t j;
    for (j = 0; j < context->numberOfODElements; j++)
    {
        if (context->usedOD[j].index == 0x2100 && context->usedOD[j].maxSubIndex == 0 && context->usedOD[j].attribute != 0 && context->usedOD[j].pData != NULL)
        {
            /* Looks like this is the wanted array (it is at least a variable) */
            /* Check if the found index is a variable with len >= 10 */

            if (context->usedOD[j].length < 10)
            {
                // The errorStatusBits is too short
                errs++;
            }
            else
            {
                // Nothing to do
            }
            /* Exit early, we're done here (even if data is not valid)*/
            break;
        }
    }
    /* Check if errorStatusBits was found, othwerise the OD is corrupt */
    if( j >= context->numberOfODElements)
    {
        // errorStatusBits couldn't be found
        errs++;
    }

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
    


#ifdef CO_USE_GLOBALS
    static CO_CANmodule_t       COO_CANmodule;
    static CO_CANrx_t           COO_CANmodule_rxArray0[CO_RXCAN_NO_MSGS];
    static CO_CANtx_t           COO_CANmodule_txArray0[CO_TXCAN_NO_MSGS];
    static CO_SDO_t             COO_SDO[CO_NO_SDO_SERVER];
    static CO_OD_extension_t    COO_SDO_ODExtensions[CO_OD_NoOfElements];
    static CO_EM_t              COO_EM;
    static CO_EMpr_t            COO_EMpr;
    static CO_NMT_t             COO_NMT;
#if CO_NO_SYNC == 1
    static CO_SYNC_t            COO_SYNC;
#endif
#if CO_NO_TIME == 1
    static CO_TIME_t            COO_TIME;
#endif
    static CO_RPDO_t            COO_RPDO[CO_NO_RPDO];
    static CO_TPDO_t            COO_TPDO[CO_NO_TPDO];
    static CO_HBconsumer_t      COO_HBcons;
    static CO_HBconsNode_t      COO_HBcons_monitoredNodes[CO_NO_HB_CONS];
#if CO_NO_LSS_SERVER == 1
    static CO_LSSslave_t        CO0_LSSslave;
#endif

/**
 * Allocate and initialize memory for CANopen object
 *
 * Function must be called in the communication reset section.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT,
 * CO_ERROR_OUT_OF_MEMORY
 */
CO_ReturnError_t CO_new(void);


/**
 * Initialize CAN driver
 *
 * Function must be called in the communication reset section.
 *
 * @param CANdriverState Pointer to the CAN module, passed to CO_CANmodule_init().
 * @param bitRate CAN bit rate.
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT,
 * CO_ERROR_ILLEGAL_BAUDRATE, CO_ERROR_OUT_OF_MEMORY
 */
CO_ReturnError_t CO_CANinit(
        void                   *CANdriverState,
        uint16_t                bitRate);


/**
 * Initialize CANopen LSS slave
 *
 * Function must be called in the communication reset section.
 *
 * @param nodeId Node ID of the CANopen device (1 ... 127) or CO_LSS_NODE_ID_ASSIGNMENT
 * @param bitRate CAN bit rate.
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT
 */
CO_ReturnError_t CO_LSSinit(
        uint8_t                 nodeId,
        uint16_t                bitRate);


/**
 * Initialize CANopen stack.
 *
 * Function must be called in the communication reset section.
 *
 * @param nodeId Node ID of the CANopen device (1 ... 127).
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT
 */
CO_ReturnError_t CO_CANopenInit(
        uint8_t                 nodeId);

#else /* CO_NO_LSS_SERVER == 0 */
/**
 * Initialize CANopen stack.
 *
 * Function must be called in the communication reset section.
 *
 * @param CANdriverState Pointer to the user-defined CAN base structure, passed to CO_CANmodule_init().
 * @param nodeId Node ID of the CANopen device (1 ... 127).
 * @param bitRate CAN bit rate.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT,
 * CO_ERROR_OUT_OF_MEMORY, CO_ERROR_ILLEGAL_BAUDRATE
 */
CO_ReturnError_t CO_init(
        void                   *CANdriverState,
        uint8_t                 nodeId,
        uint16_t                bitRate);

#endif /* CO_NO_LSS_SERVER == 0 */


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

    /* Protect access to NMT operatingState and resetCommand */
    CO_LOCK_NMT();

        CO_ReturnError_t error = CO_ERROR_NO;

        /* Apply NMT command also to this node, if set so. */
        if(nodeID == 0 || nodeID == CO_this->NMT->nodeId){
            switch(command){
                case CO_NMT_ENTER_OPERATIONAL:
                    if((*CO_this->NMT->emPr->errorRegister) == 0) {
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

        if(error == CO_ERROR_NO)
        {
            error = CO_CANsend(CO_this->CANmodule[0], NMTM_txBuff); /* 0 = success */
        }

        CO_UNLOCK_NMT();
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

    // TODO: If these checks are really needed for generated code, then please add!
    /* Verify parameters from CO_OD */
    // if(   sizeof(OD_TPDOCommunicationParameter_t) != sizeof(CO_TPDOCommPar_t)
    //    || sizeof(OD_TPDOMappingParameter_t) != sizeof(CO_TPDOMapPar_t)
    //    || sizeof(OD_RPDOCommunicationParameter_t) != sizeof(CO_RPDOCommPar_t)
    //    || sizeof(OD_RPDOMappingParameter_t) != sizeof(CO_RPDOMapPar_t))
    // {
    //     return CO_ERROR_PARAMETERS;
    // }
    
    // if(sizeof(OD_SDOClientParameter_t) != sizeof(CO_SDOclientPar_t)){
    //     return CO_ERROR_PARAMETERS;
    // }
    

    /* Initialize CANopen object */

    CO_memset((uint8_t*)CO, 0, sizeof(CO_t));
    CO->CANmodule[0]                    = &COO_CANmodule;
    CO_CANmodule_rxArray0               = &COO_CANmodule_rxArray0[0];
    CO_CANmodule_txArray0               = &COO_CANmodule_txArray0[0];
    for(i=0; i<CO_NO_SDO_SERVER; i++)
        CO->SDO[i]                      = &COO_SDO[i];
    CO_SDO_ODExtensions                 = &COO_SDO_ODExtensions[0];
    CO->em                              = &COO_EM;
    CO->emPr                            = &COO_EMpr;
    CO->NMT                             = &COO_NMT;
  #if CO_NO_SYNC == 1
    CO->SYNC                            = &COO_SYNC;
  #endif
  #if CO_NO_TIME == 1
    CO->TIME                            = &COO_TIME;
  #endif
    for(i=0; i<CO_NO_RPDO; i++)
        CO->RPDO[i]                     = &COO_RPDO[i];
    for(i=0; i<CO_NO_TPDO; i++)
        CO->TPDO[i]                     = &COO_TPDO[i];
    CO->HBcons                          = &COO_HBcons;
    CO_HBcons_monitoredNodes            = &COO_HBcons_monitoredNodes[0];
  #if CO_NO_LSS_SERVER == 1
    CO->LSSslave                        = &CO0_LSSslave;
  #endif
  #if CO_NO_LSS_CLIENT == 1
    CO->LSSmaster                       = &CO0_LSSmaster;
  #endif
  #if CO_NO_SDO_CLIENT != 0
    for(i=0; i<CO_NO_SDO_CLIENT; i++) {
      CO->SDOclient[i]                  = &COO_SDOclient[i];
    }
  #endif
  #if CO_NO_TRACE > 0
    for(i=0; i<CO_NO_TRACE; i++) {
        CO->trace[i]                    = &COO_trace[i];
        CO_traceTimeBuffers[i]          = &COO_traceTimeBuffers[i][0];
        CO_traceValueBuffers[i]         = &COO_traceValueBuffers[i][0];
        CO_traceBufferSize[i]           = CO_TRACE_BUFFER_SIZE_FIXED;
    }
  #endif
#else
    if(CO == NULL){    /* Use malloc only once */
        CO = &COO;
        CO->CANmodule[0]                    = (CO_CANmodule_t *)    COcalloc(1, sizeof(CO_CANmodule_t));
        CO_CANmodule_rxArray0               = (CO_CANrx_t *)        COcalloc(norxmsgs, sizeof(CO_CANrx_t));
        CO_CANmodule_txArray0               = (CO_CANtx_t *)        COcalloc(notxmsgs, sizeof(CO_CANtx_t));
        /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
         * otherwise it can't be accessed like an array afterwards! */
        CO->SDO                             = (CO_SDO_t *)          COcalloc(context->features->CO_NO_SDO_SERVER, sizeof(CO_SDO_t));        
        CO_SDO_ODExtensions                 = (CO_OD_extension_t*)  COcalloc(context->numberOfODElements, sizeof(CO_OD_extension_t));
        CO->em                              = (CO_EM_t *)           COcalloc(1, sizeof(CO_EM_t));
        CO->emPr                            = (CO_EMpr_t *)         COcalloc(1, sizeof(CO_EMpr_t));
        CO->NMT                             = (CO_NMT_t *)          COcalloc(1, sizeof(CO_NMT_t));
	  #if CO_NO_SYNC == 1
        CO->SYNC                            = (CO_SYNC_t *)         COcalloc(1, sizeof(CO_SYNC_t));      #endif
      #if CO_NO_TIME == 1
        CO->TIME                            = (CO_TIME_t *)         COcalloc(1, sizeof(CO_TIME_t));
      #endif
        for(i=0; i<CO_NO_RPDO; i++){
            CO->RPDO[i]                     = (CO_RPDO_t *)         COcalloc(1, sizeof(CO_RPDO_t));
        }
        else
        {
            CO->LSSslave                    = NULL;
        }
        if(context->features->CO_NO_LSS_CLIENT > 0)
        {
            CO->LSSmaster                   = (CO_LSSmaster_t *)    COcalloc(1, sizeof(CO_LSSmaster_t));
        }
        else
        {
            CO->LSSmaster                   = NULL;
        }
        if(context->features->CO_NO_SDO_CLIENT > 0)
        {
            /* Attention, MUST BE ALLOCATED AT ONCE HERE! *
             * otherwise it can't be accessed like an array afterwards! */ 
            CO->SDOclient                   = (CO_SDOclient_t *)    COcalloc(context->features->CO_NO_SDO_CLIENT, sizeof(CO_SDOclient_t));
        }
        else
        {
            CO->SDOclient                   = NULL;
        }

        uint32_t neededBuffersize = 0;

        if(context->features->CO_NO_TRACE > 0)
        {

            /* For each config'd trace in OD, we need a bufferarray of int32_t(values), uint32_t(time) and one place to store the size */
            /* Since we don't know the number of traces in advance, we can't assign a array of pointers */
            /* Thus there is is just one pointer of each type, that points to an allocated array of pointers which then point to each traces information */

            
            /* So first we need the number of traces to allocate the number of pointers. */
            /* Then we iterate over each traceConfig to get the traces size to allocate memory for each trace and assign the trace's pointers and size information */

            CO->trace                       = (CO_trace_t *)        COcalloc(context->features->CO_NO_TRACE, sizeof(CO_trace_t));

            /* We now have the trace contexts in memory. So there is no need to save external pointers somewhere */
            /* We can store the size directly in the trace's contexts */
            /* We keep counting the needed amount of tracebuffers (value and time) to allocate the memory at the end. */
            /* During trace init the memory is taken then (pointers are adjusted as needed from size information in trace contexts) */

            /* Go on only if assignment worked */
            if (CO->trace != NULL)
            {
                /* Get trace buffer sizes out of OD */
                /* OD_traceConfig[i].size */
                /*2301[2], Data Type: OD_traceConfig_t, Array[2] */

                for (i = 0; i < (uint16_t)context->features->CO_NO_TRACE; i++)
                {
                    uint16_t wantedIndex = 0x2301 + i;

                    /* Poll over OD to find OD_traceConfig[i] */
                    for (uint16_t j = 0; j < context->numberOfODElements; j++)
                    {
                        if (context->usedOD[j].index == wantedIndex && context->usedOD[j].maxSubIndex > 0 && context->usedOD[j].attribute == 0 && context->usedOD[j].length == 0 && context->usedOD[j].pData != NULL)
                        {
                            /* Looks like this is the wanted record */
                            CO_OD_entryRecord_t *recP = (CO_OD_entryRecord_t *)context->usedOD[j].pData;

                            if (recP[0].pData != NULL)
                            {
                                uint8_t maxSub = *((uint8_t *)recP[0].pData);

                                if (maxSub == 8) // This is the expected size
                                {
                                    if (recP[1].pData != NULL && recP[1].length == 4)
                                    {
                                        uint32_t thisBuffersize = *((uint32_t *)recP[1].pData); // subindex 1 holds needed size of the trace
                                        neededBuffersize += thisBuffersize;
                                        CO->trace[i].bufferSize = thisBuffersize;
                                    }
                                }
                                /* Exit early, we're done here (even if data is not valid)*/
                                break;
                            }
                        }
                    }
                }

                // Now we've got the number of buffers we need for all traces
                uint32_t *traceTimeBuffers = (uint32_t *)COcalloc(neededBuffersize, sizeof(uint32_t));
                int32_t *traceValueBuffers = (int32_t *)COcalloc(neededBuffersize, sizeof(int32_t));

                // Check if assignment was okay */
                if (traceTimeBuffers == NULL || traceValueBuffers == NULL)
                {
                    // Free stuff and mark it as faulty
                    COfree(traceValueBuffers);
                    COfree(traceTimeBuffers);
                    COfree(CO->trace);
                }
                else
                {
                    /* Now assign the pointers */
                    /* For that we count upwards */
                    for (i = 0; i < (uint16_t)context->features->CO_NO_TRACE; i++)
                    {
                        CO->trace[i].timeBuffer = traceTimeBuffers;
                        CO->trace[i].valueBuffer = traceValueBuffers;
                        traceTimeBuffers += CO->trace[i].bufferSize;
                        traceValueBuffers += CO->trace[i].bufferSize;
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
  #if CO_NO_SYNC == 1
                  + sizeof(CO_SYNC_t)
  #endif
  #if CO_NO_TIME == 1 
                  + sizeof(CO_TIME_t)
  #endif
                  + sizeof(CO_RPDO_t) * CO_NO_RPDO
                  + sizeof(CO_TPDO_t) * CO_NO_TPDO
                  + sizeof(CO_HBconsumer_t)
                  + sizeof(CO_HBconsNode_t) * CO_NO_HB_CONS;
    if(context->features->CO_NO_LSS_SERVER == 1)
    {
        CO_memoryUsed += sizeof(CO_LSSslave_t);
    }

    errCnt = 0;
    if(CO->CANmodule[0]                 == NULL) errCnt++;
    if(CO_CANmodule_rxArray0            == NULL) errCnt++;
    if(CO_CANmodule_txArray0            == NULL) errCnt++;
    if(CO->SDO                          == NULL) errCnt++;    
    if(CO_SDO_ODExtensions              == NULL) errCnt++;
    if(CO->em                           == NULL) errCnt++;
    if(CO->emPr                         == NULL) errCnt++;
    if(CO->NMT                          == NULL) errCnt++;
  #if CO_NO_SYNC == 1
    if(CO->SYNC                         == NULL) errCnt++;
  #endif
  #if CO_NO_TIME == 1 
    if(CO->TIME                     	== NULL) errCnt++;
  #endif
    for(i=0; i<CO_NO_RPDO; i++){
        if(CO->RPDO[i]                  == NULL) errCnt++;
    }
    for(i=0; i<CO_NO_TPDO; i++){
        if(CO->TPDO[i]                  == NULL) errCnt++;
    }
    if(CO->HBcons                       == NULL) errCnt++;
    if(CO_HBcons_monitoredNodes         == NULL) errCnt++;
    if(context->features->CO_NO_LSS_SERVER == 1 && \
       CO->LSSslave                     == NULL) errCnt++;
    if(context->features->CO_NO_LSS_CLIENT == 1 && \
       CO->LSSmaster                    == NULL) errCnt++;
    if(context->features->CO_NO_SDO_CLIENT > 0 && \
       CO->SDOclient                    == NULL) errCnt++;
    if(context->features->CO_NO_TRACE > 0 && \
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

    uint16_t rxidxcan_LSS = (uint16_t)context->features->CO_NO_HB_CONS + \
                            (uint16_t)context->features->CO_NO_SDO_CLIENT + \
                            (uint16_t)context->features->CO_NO_SDO_SERVER + \
                            context->features->CO_NO_RPDO + \
                            (uint16_t)context->features->CO_NO_TIME + \
                            (uint16_t)context->features->CO_NO_EMERGENCY + \
                            (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;

    uint16_t txidxcan_LSS = (uint16_t)context->features->CO_NO_HB_PROD + \
                            (uint16_t)context->features->CO_NO_SDO_CLIENT + \
                            (uint16_t)context->features->CO_NO_SDO_SERVER + \
                            context->features->CO_NO_TPDO + \
                            (uint16_t)context->features->CO_NO_TIME + \
                            (uint16_t)context->features->CO_NO_EMERGENCY + \
                            (uint16_t)context->features->CO_NO_SYNC + \
                            (uint16_t)context->features->CO_NO_NMT_MASTER;

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

    rxidxcan = context->features->CO_NO_RPDO + \
                (uint16_t)context->features->CO_NO_TIME + \
                (uint16_t)context->features->CO_NO_EMERGENCY + \
                (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;

    txidxcan = context->features->CO_NO_TPDO + \
                (uint16_t)context->features->CO_NO_TIME + \
                (uint16_t)context->features->CO_NO_EMERGENCY + \
                (uint16_t)context->features->CO_NO_SYNC + \
                (uint16_t)context->features->CO_NO_NMT_MASTER;

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

    for (i = 0; i < (uint16_t)context->features->CO_NO_SDO_SERVER; i++)
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
                    COB_IDServerToClient = *((uint32_t *)recP[subS2C].pData); // Subindex subS2C holds COB_IDServerToClient as uint32_t
                }
                if(recP[subC2S].length == 4 && recP[subC2S].pData != NULL)
                {
                    COB_IDClientToServer = *((uint32_t *)recP[subC2S].pData); // Subindex subC2S holds COB_IDClientToServer as uint32_t
                }
            }
        }

        err = CO_ERROR_DATA_CORRUPT; /* Mark already, so in case no COBIDs are assigned, we know that OD is corrupted */

        if(COB_IDClientToServer != 0 && COB_IDServerToClient != 0)
        {
            err = CO_SDO_init(
                &CO->SDO[i],
                COB_IDClientToServer,
                COB_IDServerToClient,
                OD_H1200_SDO_SERVER_PARAM + i,
                i == 0 ? NULL : &CO->SDO[0],
                context->usedOD,
                context->numberOfODElements,
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
    if (context->features->CO_NO_EMERGENCY > 0) 
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
            if (errorRegisterP != NULL && errorStatusBitsP != NULL && preDefinedErrorFieldP != NULL && inhibitTimeEMCY != NULL)
            {
                /* Exit early, we're done here */
                break;
            }
        }

        rxidxcan = (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;
        txidxcan = (uint16_t)context->features->CO_NO_SYNC +
                   (uint16_t)context->features->CO_NO_NMT_MASTER;

        if (errorStatusBitsP == NULL || errorRegisterP == NULL || preDefinedErrorFieldP == NULL || inhibitTimeEMCY == NULL)
        {
            return CO_ERROR_DATA_CORRUPT;
        }

        err = CO_EM_init(
            CO->em,
            CO->emPr,
           &CO->SDO[0],
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

    txidxcan = (uint16_t)context->features->CO_NO_SDO_CLIENT + \
                (uint16_t)context->features->CO_NO_SDO_SERVER + \
                context->features->CO_NO_TPDO + \
                (uint16_t)context->features->CO_NO_TIME + \
                (uint16_t)context->features->CO_NO_EMERGENCY + \
                (uint16_t)context->features->CO_NO_SYNC + \
                (uint16_t)context->features->CO_NO_NMT_MASTER;

    rxidxcan = CO_RXCAN_NMT; 

    err = CO_NMT_init(
            CO->NMT,
            CO->emPr,
            nodeId,
            500,
            CO->CANmodule[0],
            rxidxcan,
            CO_CAN_ID_NMT_SERVICE,
            CO->CANmodule[0],
            txidxcan,
            CO_CAN_ID_HEARTBEAT + nodeId);

    if (err)
    {
        return err;
    }

    if (context->features->CO_NO_NMT_MASTER > 0)
    {
        NMTM_txBuff = CO_CANtxBufferInit(/* return pointer to 8-byte CAN data buffer, which should be populated */
                CO->CANmodule[0], /* pointer to CAN module used for sending this message */
                CO_TXCAN_NMT,     /* index of specific buffer inside CAN module */
                0x0000,           /* CAN identifier */
                0,                /* rtr */
                2,                /* number of data bytes */
                0);               /* synchronous message flag bit */
    }

    if (context->features->CO_NO_LSS_CLIENT > 0)
    {
        rxidxcan = (uint16_t)context->features->CO_NO_HB_CONS +
                    (uint16_t)context->features->CO_NO_SDO_CLIENT +
                    (uint16_t)context->features->CO_NO_SDO_SERVER +
                    context->features->CO_NO_RPDO +
                    (uint16_t)context->features->CO_NO_TIME +
                    (uint16_t)context->features->CO_NO_EMERGENCY +
                    (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;

        txidxcan = (uint16_t)context->features->CO_NO_HB_PROD +
                    (uint16_t)context->features->CO_NO_SDO_CLIENT +
                    (uint16_t)context->features->CO_NO_SDO_SERVER +
                    context->features->CO_NO_TPDO +
                    (uint16_t)context->features->CO_NO_TIME +
                    (uint16_t)context->features->CO_NO_EMERGENCY +
                    (uint16_t)context->features->CO_NO_SYNC +
                    (uint16_t)context->features->CO_NO_NMT_MASTER;

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
    if (context->features->CO_NO_SYNC > 0)
    {
        /*1005, Data Type: UNSIGNED32 */
        /* Poll over OD to find OD_COB_ID_SYNCMessage */
        /*1006, Data Type: UNSIGNED32 */
        /* Poll over OD to find OD_communicationCyclePeriod */
        /*1019, Data Type: UNSIGNED8 */
        /* Poll over OD to find OD_synchronousCounterOverflowValue */

#if CO_NO_SYNC == 1
    err = CO_SYNC_init(
            CO->SYNC,
            CO->em,
           &CO->SDO[0],
           &CO->NMT->operatingState,
           *COB_ID_SYNCMessageP,
           *communicationCyclePeriodP,
           *synchronousCounterOverflowValueP,
            CO->CANmodule[0],
            rxidxcan,
            CO->CANmodule[0],
            txidxcan);

        if (err)
        {
            return err;
        }
    }
#endif

#if CO_NO_TIME == 1
    err = CO_TIME_init(
            CO->TIME,
            CO->em,
            &CO->SDO[0],
            &CO->NMT->operatingState,
            *COB_ID_TIMEP,
            0,
            CO->CANmodule[0],
            rxidxcan,
            CO->CANmodule[0],
            txidxcan);

    if(err){return err;}
#endif

    for(i=0; i<CO_NO_RPDO; i++){
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

        // FIXME: Comm parameter editability should be part of config!

        err = CO_RPDO_init(
               &CO->RPDO[i],
                CO->em,
               &CO->SDO[0],
                CO->SYNC,
               &CO->NMT->operatingState,
                nodeId,
                ((i<4) ? (CO_CAN_ID_RPDO_1+i*0x100) : 0),
                0,
                (CO_RPDOCommPar_t*) recP[0].pData,
                (CO_RPDOMapPar_t*) recP2[0].pData,
                OD_H1400_RXPDO_1_PARAM+i,
                OD_H1600_RXPDO_1_MAPPING+i,
                CANdevRx,
                CANdevRxIdx);

        if (err)
        {
            return err;
        }
    }

    for (i = 0; i < context->features->CO_NO_TPDO; i++)
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

        // FIXME: Comm parameter editability should be part of config!

        err = CO_TPDO_init(
                CO->TPDO[i],
                CO->em,
                CO->SDO[0],
                CO->SYNC,
               &CO->NMT->operatingState,
                nodeId,
                ((i<4) ? (CO_CAN_ID_TPDO_1+i*0x100) : 0),
                0,
                (CO_TPDOCommPar_t*) &OD_TPDOCommunicationParameter[i],
                (CO_TPDOMapPar_t*) &OD_TPDOMappingParameter[i],
                OD_H1800_TXPDO_1_PARAM+i,
                OD_H1A00_TXPDO_1_MAPPING+i,
                CO->CANmodule[0],
                CO_TXCAN_TPDO+i);

        if (err)
        {
            return err;
        }
    }

    if (context->features->CO_NO_HB_CONS > 0)
    {
        rxidxcan = (uint16_t)context->features->CO_NO_SDO_CLIENT +
                   (uint16_t)context->features->CO_NO_SDO_SERVER +
                   context->features->CO_NO_RPDO +
                   (uint16_t)context->features->CO_NO_TIME +
                   (uint16_t)context->features->CO_NO_EMERGENCY +
                   (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;

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
        if (consumerHeartbeatTimeLen != (uint16_t)context->features->CO_NO_HB_CONS)
        {
            /* Seems like there is a config mismatch */
            return CO_ERROR_DATA_CORRUPT;
        }

        err = CO_HBconsumer_init(
            CO->HBcons,
            CO->em,
           &CO->SDO[0],
            consumerHeartbeatTimeP,
            CO_HBcons_monitoredNodes,
            context->features->CO_NO_HB_CONS,
            CO->CANmodule[0],
            rxidxcan);

        if (err)
        {
            return err;
        }
    }

    if(context->features->CO_NO_SDO_CLIENT > 0)
    {
        rxidxcan = (uint16_t)context->features->CO_NO_SDO_SERVER +
                    context->features->CO_NO_RPDO +
                    (uint16_t)context->features->CO_NO_TIME +
                    (uint16_t)context->features->CO_NO_EMERGENCY +
                    (uint16_t)context->features->CO_NO_SYNC + CO_RXCAN_SYNC;

        txidxcan = (uint16_t)context->features->CO_NO_SDO_SERVER +
                    context->features->CO_NO_TPDO +
                    (uint16_t)context->features->CO_NO_TIME +
                    (uint16_t)context->features->CO_NO_EMERGENCY +
                    (uint16_t)context->features->CO_NO_SYNC +
                    (uint16_t)context->features->CO_NO_NMT_MASTER;

        for (i = 0; i < (uint16_t)context->features->CO_NO_SDO_CLIENT; i++)
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
               &CO->SDOclient[i],
               &CO->SDO[0],
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

    if(context->features->CO_NO_TRACE > 0)
    {
        // FIXME blankm 20191213: I don't know what OD entry 2400 traceEnable is for, it is not used throughout the stack?
              
        for (i = 0; i < (uint16_t)context->features->CO_NO_TRACE; i++)
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
               &CO->trace[i], //ok
               &CO->SDO[0], //ok
                *((uint8_t *)recP[2].pData),
                CO->trace[i].timeBuffer, //ok
                CO->trace[i].valueBuffer, //ok
                CO->trace[i].bufferSize, //ok
                ((uint32_t *)recP[5].pData), /* traceConfig[i].map       == idx 2301ff, subindex 5, uint32 */
                ((uint8_t *)recP[6].pData),  /* traceConfig[i].format    == idx 2301ff, subindex 6, uint8  */
                ((uint8_t *)recP[7].pData),  /* traceConfig[i].trigger   == idx 2301ff, subindex 7, uint8  */
                ((int32_t *)recP[8].pData), /* traceConfig[i].threshold == idx 2301ff, subindex 8, int32  */
                ((int32_t *)recP2[2].pData),   /* trace[i].value           == idx 2401ff, subindex 2, int32  */
                ((int32_t *)recP2[3].pData),   /* trace[i].min             == idx 2401ff, subindex 3, int32  */
                ((int32_t *)recP2[4].pData),   /* trace[i].max             == idx 2401ff, subindex 4, int32  */
                ((uint32_t *)recP2[6].pData),  /* trace[i].triggerTime     == idx 2401ff, subindex 6, uint32 */
                OD_INDEX_TRACE_CONFIG + i, /* This is a define in CO_trace.h which doesn't need altering, it is 0x2301 */
                OD_INDEX_TRACE + i);       /* This is a define in CO_trace.h which doesn't need altering, it is 0x2401 */
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
        CO_delete(CANdriverState, context);
        return err;
    }

    err = CO_CANinit(CANdriverState, bitRate, context);
    if (err) {
        CO_delete(CANdriverState, context);
        return err;
    }

    err = CO_CANopenInit(nodeId, context);
    if (err) {
        CO_delete(CANdriverState, context);
        return err;
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_delete(
        void                   *CANdriverState, 
        CO_Context_t           *context)
{
    CO_CANsetConfigurationMode(CANdriverState);
    CO_CANmodule_disable(CO->CANmodule[0]);

    // Cleaning the trace stuff is easy since all is allocated at once and thus can be freed at once
    if(context->features->CO_NO_TRACE > 0)
    {
        COfree(CO->trace[0].timeBuffer);
        COfree(CO->trace[0].valueBuffer);
        COfree(CO->trace);
    }

    // Cleaning SDOClients is easy since all is allocated at once and thus can be freed at once
    COfree(CO->SDOclient);    

    if (context->features->CO_NO_LSS_SERVER > 0)
    {
        COfree(CO->LSSslave);
    }
    if (context->features->CO_NO_LSS_CLIENT > 0)
    {
        COfree(CO->LSSmaster);
    }
    // Cleaning CO_HBcons_monitoredNodes is easy since all is allocated at once and thus can be freed at once
    COfree(CO_HBcons_monitoredNodes);
    COfree(CO->HBcons);
    // Cleaning RPDO is easy since all is allocated at once and thus can be freed at once
    COfree(CO->RPDO);
    // Cleaning TPDO is easy since all is allocated at once and thus can be freed at once
    COfree(CO->TPDO);
    
  #if CO_NO_SYNC == 1
    COfree(CO->SYNC);
  #endif
  #if CO_NO_TIME == 1
    COfree(CO->TIME);
  #endif
    COfree(CO->NMT);
    COfree(CO->emPr);
    COfree(CO->em);
    // Cleaning SDO and extensions is easy since all is allocated at once and thus can be freed at once
    COfree(CO_SDO_ODExtensions);
    COfree(CO->SDO);
    
    // Cleaning the arrays is easy since all is allocated at once and thus can be freed at once
    COfree(CO_CANmodule_txArray0);
    COfree(CO_CANmodule_rxArray0);
    COfree(CO->CANmodule[0]);
    CO = NULL;
}

/******************************************************************************/
CO_NMT_reset_cmd_t CO_process(
        CO_t                   *CO_this,
        CO_Context_t           *context,
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


    for(i=0; i<context->features->CO_NO_SDO_SERVER; i++){
        CO_SDO_process(
               &CO_this->SDO[i],
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
            errorBehavior,
            timerNext_ms);


    CO_HBconsumer_process(
            CO_this->HBcons,
            NMTisPreOrOperational,
            timeDifference_ms);

#if CO_NO_TIME == 1
    CO_TIME_process(
            CO_this->TIME, 
            timeDifference_ms);
#endif

    return reset;
}


/******************************************************************************/
#if CO_NO_SYNC == 1
bool_t CO_process_SYNC(
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

    return syncWas;
}
#endif

/******************************************************************************/
void CO_process_RPDO(
        CO_t                   *co,
        bool_t                  syncWas)
{
    uint16_t i;

    for(i=0; i<CO_NO_RPDO; i++){
        CO_RPDO_process(&CO_this->RPDO[i], syncWas);
    }
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
    for (i = 0; i < context->features->CO_NO_TPDO; i++)
    {
        if (!CO_this->TPDO[i].sendRequest)
            CO_this->TPDO[i].sendRequest = CO_TPDOisCOS(&CO_this->TPDO[i]);
        CO_TPDO_process(&CO_this->TPDO[i], CO_this->SYNC, syncWas, timeDifference_us);
    }
}
