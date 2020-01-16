# Listing of all dependencies out of CO_OD.h into stack and ideas on how to solve it

CRITICAL PARTS:
+ RX/TX buffer indexing
+ GLOBALS (can't be used toghether with dynamic sizing of elements)

## Basic DataTypes (CANopen DATA DYPES)
Written by libEDS in hardcoded fashion, can be easily changed and/or placed somewhere else.

## Device Info
Written by libEDS, Should be added also in a machine-readable form as struct

## Features
Written by libEDS, should be changed into to a context-information used by the stack.

### Feature verification
Should be put into a function running against the context-information, and should be called 
before init(but out of init functions and not by user).

### Feature define usage
+ CO_NO_SYNC: 
  + Feature verification
  + Index calculation and message object requirement calculation
+ CO_NO_EMERGENCY:
  + Feature verification
  + Index calculation and message object requirement calculation
+ CO_NO_SDO_SERVER:
  + Feature verification
  + Index calculation and message object requirement calculation
  + Globals: Size of SDO array (for server side)
  + initialization of global/normal SDO (server) array variables and allocation and verification of allocation
  + memory usage calculation (if trace is used)
  + Initialization of SDO servers
  + memory freeing
  + SDO server processing
  + CO object typedef (not needed there imho)
+ CO_NO_SDO_CLIENT:
  + Inclusion checks (SDOmaster.h)
  + CO object typedef content control
  + Feature verification
  + Index calculation and message object requirement calculation
  + Globals: Size of SDOclient Array 
  + init of array and allocation and verification of allocation
  + sizecheck of variables during init
  + memory usage calculation (if trace is used)
  + Initialization of SDO clients (and the visibility of this function during compilation)
  + memory freeing
+ CO_NO_RPDO:
  + CO object typedef (not needed there imho)
  + Feature verification
  + Index calculation and message object requirement calculation
  + Globals: Size of RPDO array
  + initialization of global/normal RPDO array variables and allocation and verification of allocation
  + memory usage calculation (if trace is used)
  + RPDO init
  + freeing
  + RPDO processing
+ CO_NO_TPDO:
  + CO object typedef (not needed there imho)
  + Feature verification
  + Index calculation and message object requirement calculation
  + Globals: Size of TPDO array
  + initialization of global/normal TPDO array variables and allocation and verification of allocation
  + memory usage calculation (if trace is used)
  + TPDO init
  + freeing
  + TPDO processing
+ CO_NO_NMT_MASTER:
  + Feature verification
  + Index calculation and message object requirement calculation
  + NMT Send command function conditional compilation
  + Conditional CO_CANtxBufferInit
+ CO_NO_TRACE: (This is not that critical, since this is no runtime specific changing parameter)
  + CO object typedef (also conditional inclusion in typedef)
  + conditional header inclusion
+ CO_NO_LSS_SERVER:
  + Conditional inclusion of headers/functions
  + Conditional typdef of CO object
  + feature verification
  + message object requirement calculation
  + globals: inclusion of slave variables
  + initialization of global/normal variable and allocation and verification of allocation
  + memory usage calculation (if trace is used)
  + freeing
+ CO_NO_LSS_CLIENT:
  + Feature verification
  + message object requirement calculation
  + globals: inclusion of master variables
  + initialization of global/normal variable and allocation and verification of allocation
  + memory usage calculation (if trace is used)
  + freeing
  + conditional inclusion of functions/headers
  + Conditional typdef of CO object



## Object Dictionary Lenght
Written by libEDS, should be changed into to a context-information used by the stack

+ CO_OD_NoOfElements:
  + pointer to external CO_OD (not needed there imho)
  + size definition of globals (SDO_ODExtensions) or allocation of SDO_ODExtensions
  + memory usage calulcation (SDO_ODExtensions)
  + SDO_Init

## Tyepdef Records (TYPE DEFINITIONS FOR RECORDS)

For each record a typedef is generated. The typedef is only used within CO_OD. 
But some of them are used for size comparison. 
This could be written hardcoded into another information struct supplied with the
CO_OD.

## Variable structures (RAM, ROM, EEPROM)

CO_OD_RAM, CO_OD_ROM, sCO_OD_EEPROM are currently only used internally.
If one would need an auto-update functionality for eeprom then some pointer would come handy.

## OD Record aliases (ALIASES FOR OBJECT DICTIONARY VARIABLES) (OK, can realized w\ existing OD infos)

Generated for each recorded. Some are used out in the stack:
+ OD_errorRegister
+ OD_preDefinedErrorField
  + ODL_preDefinedErrorField_arrayLength
+ OD_COB_ID_SYNCMessage
+ OD_communicationCyclePeriod
+ OD_synchronousWindowLength
+ OD_inhibitTimeEMCY
+ OD_consumerHeartbeatTime
  + ODL_consumerHeartbeatTime_arrayLength
+ OD_producerHeartbeatTime
+ OD_identity
+ OD_synchronousCounterOverflowValue
+ OD_errorBehavior
  + ... (all sub items)
+ OD_SDOServerParameter
+ OD_RPDOCommunicationParameter
+ OD_TPDOCommunicationParameter
+ OD_RPDOMappingParameter
+ OD_TPDOMappingParameter
+ OD_NMTStartup
+ OD_errorStatusBits
  + ...
+ OD_CANNodeID
+ OD_CANBitRate
+ OD_powerOnCounter ?
+ APPLICATION SPECIFIC OTHERS AND MAYBE FUTURE STACK EXTENSIONS

All these should be supplied not for direct access by should be using a pointer instead.
This should be organized as an array which works like a linked list or something

This is coming easy, all information is (sometimes coded differently) in the OD and the links. Not need for extra structures here.
