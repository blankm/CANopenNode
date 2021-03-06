/*
 * CAN module object for the Atmel SAM3X microcontroller.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @author      Olof Larsson
 * @copyright   2014 Janez Paternoster
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


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H


#include <stddef.h>         /* for 'NULL' */
#include <stdint.h>         /* for 'int8_t' to 'uint64_t' */

#include "asf.h"
#include <sam3x_ek.h>
#include <can.h>


/** Endianness */
#define CO_LITTLE_ENDIAN


/* CAN module base address */
#define ADDR_CAN1               CAN0
#define ADDR_CAN2               CAN1
/*
Remember to set:
 #define CONF_BOARD_CAN0
 #define CONF_BOARD_CAN1
in conf_board.h
*/


/* Critical sections */
#define CO_LOCK_CAN_SEND()      //taskENTER_CRITICAL()
#define CO_UNLOCK_CAN_SEND()    //taskEXIT_CRITICAL()

#define CO_LOCK_EMCY()          //taskENTER_CRITICAL()
#define CO_UNLOCK_EMCY()        //taskEXIT_CRITICAL()

#define CO_LOCK_OD()            //taskENTER_CRITICAL()
#define CO_UNLOCK_OD()          //taskEXIT_CRITICAL()

    #define CO_LOCK_NMT()           //taskENTER_CRITICAL()
    #define CO_UNLOCK_NMT()         //taskEXIT_CRITICAL()

/**
 * @name Memory/heap management functions mapping
 * Here the user may select heap management functions according to needs.
 * @{
 */
#define COmalloc(size) malloc(size) 
/** Calloc */
#define COcalloc(items, size) calloc(items, size) 
/** Free */
#define COfree(loc) free(loc)
/** @} */

/* Data types */
/* int8_t to uint64_t are defined in stdint.h */
typedef unsigned char           bool_t;
typedef float                   float32_t;
typedef long double             float64_t;
typedef char                    char_t;
typedef unsigned char           oChar_t;
typedef unsigned char           domain_t;


/* CAN receive message structure as aligned in CAN module. */
typedef struct{
    /** CAN identifier. It must be read through CO_CANrxMsg_readIdent() function. */
    uint32_t            ident;
    uint8_t             DLC ;
    uint8_t             data[8];
    can_mb_conf_t       mbConf;      /* Reference to controller's mailboxes */
}CO_CANrxMsg_t;


/* Received message object */
typedef struct{
    uint16_t            ident;
    uint16_t            mask;
    void               *object;
    void              (*pFunct)(void *object, const CO_CANrxMsg_t *message);
}CO_CANrx_t;


/* Transmit message object. */
typedef struct{
    uint32_t            ident;
    uint8_t             DLC;
    uint8_t             data[8];
    volatile bool_t     bufferFull;
    volatile bool_t     syncFlag;
    bool_t              rtr;
}CO_CANtx_t;


/* CAN module object. */
typedef struct{
    Can                *CANdriverState;
    CO_CANrx_t         *rxArray;
    uint16_t            rxSize;
    CO_CANtx_t         *txArray;
    uint16_t            txSize;
    volatile bool_t     CANnormal;
    volatile bool_t     useCANrxFilters;
    volatile bool_t     bufferInhibitFlag;
    volatile bool_t     firstCANtxMessage;
    volatile uint16_t   CANtxCount;
    uint32_t            errOld;
    void               *em;
    can_mb_conf_t       rxMbConf[CANMB_NUMBER-1]; /* Reference to controller's mailboxes */
    can_mb_conf_t       txMbConf;
}CO_CANmodule_t;


/* CAN interrupt receives and transmits CAN messages. */
void CO_CANinterrupt(CO_CANmodule_t *CANmodule);


#endif
