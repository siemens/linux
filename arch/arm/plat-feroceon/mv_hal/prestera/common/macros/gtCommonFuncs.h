/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef __gtCommonFuncsh
#define __gtCommonFuncsh

#include <gtOs/gtGenTypes.h>
#include <common/os/gtTypes.h>

/* Return the mask including "numOfBits" bits.          */
#define GT_BIT_MASK(numOfBits) (~(0xFFFFFFFF << (numOfBits)))

/* Return the mask including "numOfBits" bits.          */
#define BIT8_MASK(numOfBits) (~(0xFF << (numOfBits)))

/* Calculate the field mask for a given offset & length */
/* e.g.: BIT_MASK(8,2) = 0xFFFFFCFF                     */
#define FIELD_8_MASK_NOT(offset,len)                      \
        (~(BIT8_MASK((len)) << (offset)))

/* Returns the info located at the specified offset & length in data.   */
#define U8_GET_FIELD(data,offset,length)           \
        (((data) >> (offset)) & ((1 << (length)) - 1))


/* Sets the field located at the specified offset & length in data.     */
#define U8_SET_FIELD(data,offset,length,val)           \
   (data) = (((data) & FIELD_8_MASK_NOT((offset),(length))) | ((val) <<(offset)))

/* Return BIT MASK started from MSB and ended at LSB */
#define BIT_STRING(msb, lsb) ( (msb-lsb == 31) ? 0xFFFFFFFF : \
     (BIT_MASK(msb-lsb+1) << lsb) )


/* Calculate the field mask for a given offset & length */
/* e.g.: BIT_MASK(8,2) = 0xFFFFFCFF                     */
#define FIELD_MASK_NOT(offset,len)                      \
        (~(BIT_MASK((len)) << (offset)))

/* Calculate the field mask for a given offset & length */
/* e.g.: BIT_MASK(8,2) = 0x00000300                     */
#define FIELD_MASK(offset,len)                      \
        ( (BIT_MASK((len)) << (offset)) )

/* Returns the info located at the specified offset & length in data.   */
#define U32_GET_FIELD(data,offset,length)           \
        (((data) >> (offset)) & ((1 << (length)) - 1))


/* Sets the field located at the specified offset & length in data.     */
#define U32_SET_FIELD(data,offset,length,val)           \
   (data) = (((data) & FIELD_MASK_NOT((offset),(length))) | ((val) <<(offset)))

#define CALC_MASK(fieldLen, fieldOffset, mask)     \
            if(((fieldLen) + (fieldOffset)) >= 32)     \
                (mask) = (0 - (1<< (fieldOffset)));    \
            else                                   \
          (mask) = (((1<<((fieldLen) + (fieldOffset)))) - (1 << (fieldOffset)))


#define GT_CALC_IP_PREFIX(ipAddr,prefixLen)         \
        (((ipAddr) >> (32 - (prefixLen))) << (32 - (prefixLen)))


#define BYTESWAP(data)                      \
        (((data) << 24))  |                 \
        (((data) & 0xff00)      << 8)   |   \
        (((data) & 0xff0000)    >> 8)   |   \
        (((data) >> 24))


/* A macro for performing left shifting on a 64 bit data            */
/* highData -   A 32 bit word including the MSB part of the data    */
/* lowData  -   A 32 bit word including the LSB part of the data    */
/* shift    -   Number of bits to shift.                            */
/* overFlow -   The (left) part of highData which overflowed.       */
#define U64_SHIFT_LEFT(overFlow,highData,lowData,shift)         \
        (overFlow) = ((overFlow) << (shift)) | ((highData) >> (32 - (shift))); \
        (highData) = ((highData) << (shift));                      \
        (highData)|= ((lowData) >> (32 - (shift)));              \
        (lowData)  = ((lowData) << (shift))

#define U64_SHIFT_RIGHT(highData,lowData,underFlow,shift)         \
        (underFlow) = ((underFlow) >> (shift)) | ((lowData) << (32 - (shift)));\
        (lowData)   = ((lowData) >> (shift));                      \
        (lowData)  |= ((highData) << (32 - (shift)));              \
        (highData)  = ((highData) >> (shift))

#define BOOL2BIT(x) (((x) == GT_TRUE) ? 1 : 0)

#define BIT2BOOL(x) (((x) == 1) ? GT_TRUE : GT_FALSE)
/* Return the minimum of x & y. */
#ifndef MIN
#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#endif  /* MIN */
#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif  /* MAX */


#define U8_SWAP_BITS(x)         \
        ((((x)&0x80)>>7)|(((x)&0x40)>>5)|(((x)&0x20)>>3)|(((x)&0x10)>>1) \
         |(((x)&0x1)<<7) | (((x)&0x2)<<5) | (((x)&0x4)<<3) |(((x)&0x8)<<1) )


/* integer which its 'x' bit is set */
#define BYTE_BIT_MASK(x)   \
        (1 << (x))

/* Get the byte of the x bit from an array of bytes */
#define BYTES_ARRAY_GET_BYTE(a,x) \
        ( ( (GT_U8 *) a)[(x)/8] )

/* checks wheter bit 'x' in 'a' is set and than returns TRUE,  */
/* otherwise return FALSE.                                     */
#define BYTES_ARRAY_CHKBIT(a,x) \
    ( ( BYTES_ARRAY_GET_BYTE(a,x) & BYTE_BIT_MASK((x)%8) ) >> ((x)%8) )

/* Clear (reset) bit 'x' in an array of bytes */
#define BYTES_ARRAY_CLRBIT(a,x) \
    (BYTES_ARRAY_GET_BYTE(a,x) &=  ~(BYTE_BIT_MASK((x)%8) ) )

/* Clear (reset) bit 'x' in an array of bytes */
#define BYTES_ARRAY_SETBIT(a,x) \
    (BYTES_ARRAY_GET_BYTE(a,x) |=  BYTE_BIT_MASK((x)%8) )

/*   INVERT bit 'x' in in an array of bytes. */
#define BYTES_ARRAY_INVBIT(a,x) \
    ( BYTES_ARRAY_GET_BYTE(a,x) ^= BYTE_BIT_MASK((x)%8) )

/* read bit # offset from 32 bits data. Return 0 or 1 */
#define U32_GET_BIT(data,offset) (((data) & (1 << (offset))) >> (offset))

/* write 1 to bit # "offset" in 32 bits data */
#define U32_SET_BIT(data,offset) ((data) |= (1 << (offset)))

/* write 0 to bit # "offset" in 32 bits data */
#define U32_UNSET_BIT(data,offset) ((data) &= (~(1 << (offset))))

#define BYTES_ARR_TO_U32(arr)    (GT_U32)((arr)[0] | ((arr)[1] << 8) | \
                            ((arr)[2] << 16) | ((arr)[3] << 24))
#define U32_TO_BYTES_ARR(val, arr)  ((arr)[3] = (GT_U8)((val) >> 24));              \
                     ((arr)[2] = (GT_U8)(((val) & 0x00ff0000)>> 16));\
                     ((arr)[1] = (GT_U8)(((val) & 0x0000ff00) >> 8));\
                     ((arr)[0] = (GT_U8)(((val) & 0x000000ff)))

extern GT_VOID gtBreakOnFail
(
    GT_VOID
);

extern GT_STATUS gtStatus;

/* Global SysConf control library code for sysLog registeration */

#define CHECK_STATUS(origFunc) \
{ \
    GT_STATUS gtStatus = origFunc; \
    if (GT_OK != gtStatus) \
    { \
        return gtStatus; \
    } \
} 



#define CHECK_STATUS_EXIT(origFunc, exitFunc) \
{ \
    gtStatus = origFunc;    \
    if (GT_OK != gtStatus)  \
    {                       \
        gtBreakOnFail();    \
        exitFunc;           \
        return gtStatus;    \
    }                       \
} 

#define CHECK_IF_LIB_INIT(vlnIsInit) \
if (GT_FALSE == vlnIsInit) \
{ \
    return GT_NOT_INITIALIZED; \
} 

#define CHECK_INPUT_PARAM(p1) \
if (NULL == p1) \
{ \
    return GT_BAD_PARAM; \
}

#endif /* __gtCommonFuncsh */

