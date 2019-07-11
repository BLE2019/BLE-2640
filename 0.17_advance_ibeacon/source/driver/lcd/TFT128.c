/*
 * Copyright (c) 2015, Ghostyu Co. Ltd.,
 *          All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <TFT128.h>
#include <ti/mw/lcd/LCDDogm1286_util.h>

/* macro to calculate minimum value */
#define MIN(a,b) (((a)<(b))?(a):(b))

/*convert the lcd buffer to ascii sizes*/
typedef short (*buf_asc)[32] ;

/* Externs */
extern const LCD_Config LCD_config;
extern const SPI_Config SPI_config[];

/* PIN driver state object */
static PIN_State pinState;

/* PIN driver handle */
static PIN_Handle hPin;

/* Used to check status and initialization */
static int LCD_count = -1;

/* Static LCD functions */
static bool LCD_initHw();

static void LCD_initSet(LCD_Handle handle, const LCD_Command *pcCmd);

static void LCD_sendCommand(LCD_Handle handle, const char *pcCmd,
    unsigned char ucLen);

static void LCD_sendData(LCD_Handle handle, const char *pcData,
    unsigned short usLen);

//static void LCD_gotoXY(LCD_Handle handle, unsigned char ucX, unsigned char ucY);

static void LCD_setRegion(LCD_Handle handle,unsigned char ucX,unsigned char ucY,
    unsigned char ucXend,unsigned char ucYend);

static bool LCD_sendArray(LCD_Handle handle, const char *pcData,
    unsigned short usLen);

static void LCD_bufferLine(LCD_Handle handle,
    unsigned char ucXFrom, unsigned char ucYFrom, unsigned char ucXTo,
    unsigned char ucYTo, unsigned char ucDraw);

static void LCD_doBufferClearPage(LCD_Handle handle,LCD_Page iPage, bool blocking);

static void LCD_doBufferPrintString(LCD_Handle handle,
    const char *pcStr, unsigned char ucX, unsigned char ucY, bool blocking);

static void LCD_doBufferSetHLine(LCD_Handle handle, 
    unsigned char ucXFrom, unsigned char ucXTo, unsigned char ucY,
    bool blocking);

static void LCD_doBufferClearHLine(LCD_Handle handle,
    unsigned char ucXFrom, unsigned char ucXTo, unsigned char ucY,
    bool blocking);

static void LCD_doBufferSetVLine(LCD_Handle handle, 
    unsigned char ucX, unsigned char ucYFrom, unsigned char ucYTo,
    bool blocking);

static void LCD_doBufferClearVLine(LCD_Handle handle,
    unsigned char ucX, unsigned char ucYFrom, unsigned char ucYTo,
    bool blocking);

static void LCD_doBufferSetPx(LCD_Handle handle, bool color,unsigned char ucX, unsigned char ucY);

static void LCD_fullRegion(LCD_Handle handle, Color color, unsigned char ucXFrom,
                           unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo);

static void LCD_Update(LCD_Handle handle,unsigned char ucXFrom,
                           unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo);

/* Font data for 8*16 font */
const char LCD_alphabet[] = {

// font = Calibri
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//" ",0
//0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,//"!",1
//0x00, 0x00, 0x00, 0x48, 0x48, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//""",2
//0x00, 0x00, 0x00, 0x00, 0x24, 0x24, 0x7E, 0x24, 0x24, 0xFE, 0x48, 0x48, 0x00, 0x00, 0x00, 0x00,//"#",3
//0x00, 0x00, 0x08, 0x08, 0x7C, 0x40, 0x40, 0x38, 0x0C, 0x06, 0x06, 0x7C, 0x10, 0x00, 0x00, 0x00,//"$",4
//0x00, 0x00, 0x00, 0x00, 0x70, 0x91, 0x92, 0x74, 0x0D, 0x0A, 0x12, 0x21, 0x00, 0x00, 0x00, 0x00,//"%",5
//0x00, 0x00, 0x00, 0x1C, 0x26, 0x22, 0x24, 0x18, 0x68, 0x47, 0x43, 0x3C, 0x00, 0x00, 0x00, 0x00,//"&",6
//0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"'",7
//0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x40, 0x40, 0x40, 0x40, 0x40, 0x60, 0x20, 0x20, 0x00, 0x00,//"(",8
//0x00, 0x00, 0x00, 0x40, 0x20, 0x20, 0x20, 0x30, 0x30, 0x30, 0x20, 0x20, 0x20, 0x40, 0x00, 0x00,//")",9
//0x00, 0x00, 0x00, 0x10, 0x74, 0x18, 0x74, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"*",10
//0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x7E, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,//"+",11
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x40, 0x80, 0x00, 0x00,//",",12
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"-",13
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,//".",14
//0x00, 0x00, 0x00, 0x08, 0x08, 0x18, 0x10, 0x10, 0x20, 0x20, 0x60, 0x40, 0xC0, 0x80, 0x00, 0x00,//"/",15
//0x00, 0x00, 0x00, 0x00, 0x3C, 0x46, 0x42, 0x42, 0x42, 0x42, 0x46, 0x3C, 0x00, 0x00, 0x00, 0x00,//"0",16
//0x00, 0x00, 0x00, 0x00, 0x38, 0x48, 0x08, 0x08, 0x08, 0x08, 0x08, 0x7E, 0x00, 0x00, 0x00, 0x00,//"1",17
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x04, 0x04, 0x0C, 0x08, 0x10, 0x60, 0x7E, 0x00, 0x00, 0x00, 0x00,//"2",18
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x04, 0x04, 0x38, 0x0C, 0x06, 0x06, 0x7C, 0x00, 0x00, 0x00, 0x00,//"3",19
//0x00, 0x00, 0x00, 0x00, 0x0C, 0x14, 0x24, 0x24, 0x44, 0xFE, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,//"4",20
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x40, 0x40, 0x7C, 0x06, 0x06, 0x04, 0x7C, 0x00, 0x00, 0x00, 0x00,//"5",21
//0x00, 0x00, 0x00, 0x00, 0x3E, 0x60, 0x40, 0x7C, 0x42, 0x42, 0x46, 0x3C, 0x00, 0x00, 0x00, 0x00,//"6",22
//0x00, 0x00, 0x00, 0x00, 0x7E, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00,//"7",23
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x46, 0x64, 0x38, 0x6C, 0x42, 0x42, 0x7C, 0x00, 0x00, 0x00, 0x00,//"8",24
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x46, 0x42, 0x46, 0x3A, 0x06, 0x04, 0x78, 0x00, 0x00, 0x00, 0x00,//"9",25
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x20, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,//":",26
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x20, 0x00, 0x00, 0x00, 0x20, 0x40, 0x40, 0x00, 0x00,//";",27
//0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0C, 0x70, 0x40, 0x38, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00,//"<",28
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"=",29
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x0C, 0x06, 0x18, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,//">",30
//0x00, 0x00, 0x00, 0x38, 0x4C, 0x04, 0x04, 0x18, 0x10, 0x10, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,//"?",31
//0x00, 0x00, 0x00, 0x00, 0x0F, 0x10, 0x27, 0x28, 0x48, 0x49, 0x4E, 0x20, 0x38, 0x07, 0x00, 0x00,//"@",32
//0x00, 0x00, 0x00, 0x00, 0x18, 0x14, 0x34, 0x26, 0x22, 0x7E, 0x41, 0xC1, 0x00, 0x00, 0x00, 0x00,//"A",33
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x62, 0x62, 0x7C, 0x62, 0x62, 0x62, 0x7C, 0x00, 0x00, 0x00, 0x00,//"B",34
//0x00, 0x00, 0x00, 0x00, 0x3E, 0x60, 0x40, 0x40, 0x40, 0x40, 0x61, 0x3E, 0x00, 0x00, 0x00, 0x00,//"C",35
//0x00, 0x00, 0x00, 0x00, 0x7E, 0x63, 0x61, 0x61, 0x61, 0x61, 0x63, 0x7C, 0x00, 0x00, 0x00, 0x00,//"D",36
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x7E, 0x00, 0x00, 0x00, 0x00,//"E",37
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00,//"F",38
//0x00, 0x00, 0x00, 0x00, 0x3F, 0x60, 0x40, 0x47, 0x41, 0x41, 0x61, 0x1F, 0x00, 0x00, 0x00, 0x00,//"G",39
//0x00, 0x00, 0x00, 0x00, 0x61, 0x61, 0x61, 0x7F, 0x61, 0x61, 0x61, 0x61, 0x00, 0x00, 0x00, 0x00,//"H",40
//0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00,//"I",41
//0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0xE0, 0x00, 0x00, 0x00, 0x00,//"J",42
//0x00, 0x00, 0x00, 0x00, 0x66, 0x6C, 0x68, 0x70, 0x78, 0x6C, 0x64, 0x62, 0x00, 0x00, 0x00, 0x00,//"K",43
//0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7C, 0x00, 0x00, 0x00, 0x00,//"L",44
//0x00, 0x00, 0x00, 0x00, 0x70, 0x50, 0x58, 0x48, 0x49, 0x45, 0x47, 0x42, 0x00, 0x00, 0x00, 0x00,//"M",45
//0x00, 0x00, 0x00, 0x00, 0x61, 0x51, 0x59, 0x49, 0x45, 0x47, 0x43, 0x41, 0x00, 0x00, 0x00, 0x00,//"N",46
//0x00, 0x00, 0x00, 0x00, 0x3F, 0x61, 0x40, 0x40, 0x40, 0x40, 0x61, 0x3E, 0x00, 0x00, 0x00, 0x00,//"O",47
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x62, 0x62, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00,//"P",48
//0x00, 0x00, 0x00, 0x00, 0x3F, 0x61, 0x40, 0x40, 0x40, 0x40, 0x61, 0x3F, 0x00, 0x00, 0x00, 0x00,//"Q",49
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x62, 0x62, 0x7C, 0x6C, 0x66, 0x62, 0x62, 0x00, 0x00, 0x00, 0x00,//"R",50
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x40, 0x40, 0x30, 0x0C, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00, 0x00,//"S",51
//0x00, 0x00, 0x00, 0x00, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,//"T",52
//0x00, 0x00, 0x00, 0x00, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x21, 0x3E, 0x00, 0x00, 0x00, 0x00,//"U",53
//0x00, 0x00, 0x00, 0x00, 0xC1, 0x43, 0x62, 0x22, 0x24, 0x34, 0x1C, 0x18, 0x00, 0x00, 0x00, 0x00,//"V",54
//0x00, 0x00, 0x00, 0x00, 0x43, 0x47, 0x65, 0x25, 0x24, 0x28, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,//"W",55
//0x00, 0x00, 0x00, 0x00, 0x42, 0x64, 0x3C, 0x18, 0x18, 0x2C, 0x66, 0x42, 0x00, 0x00, 0x00, 0x00,//"X",56
//0x00, 0x00, 0x00, 0x00, 0x46, 0x44, 0x2C, 0x38, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,//"Y",57
//0x00, 0x00, 0x00, 0x00, 0x7C, 0x04, 0x08, 0x10, 0x30, 0x20, 0x40, 0xFE, 0x00, 0x00, 0x00, 0x00,//"Z",58
//0x00, 0x00, 0x00, 0x70, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x70, 0x00, 0x00,//"[",59
//0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x20, 0x20, 0x30, 0x10, 0x18, 0x08, 0x08, 0x04, 0x00, 0x00,//"\",60
//0x00, 0x00, 0x00, 0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x60, 0x00, 0x00,//"]",61
//0x00, 0x00, 0x00, 0x00, 0x18, 0x28, 0x24, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"^",62
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00,//"_",63
//0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"`",64
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x04, 0x1C, 0x64, 0x44, 0x7C, 0x00, 0x00, 0x00, 0x00,//"a",65
//0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x5C, 0x62, 0x42, 0x42, 0x62, 0x5C, 0x00, 0x00, 0x00, 0x00,//"b",66
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x40, 0x40, 0x40, 0x40, 0x3C, 0x00, 0x00, 0x00, 0x00,//"c",67
//0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x3A, 0x46, 0x42, 0x42, 0x46, 0x3A, 0x00, 0x00, 0x00, 0x00,//"d",68
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x46, 0x7E, 0x40, 0x40, 0x3C, 0x00, 0x00, 0x00, 0x00,//"e",69
//0x00, 0x00, 0x00, 0x38, 0x20, 0x60, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00,//"f",70
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x44, 0x44, 0x78, 0x40, 0x7C, 0xC6, 0x7C, 0x00, 0x00,//"g",71
//0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x5C, 0x66, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00, 0x00,//"h",72
//0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,//"i",73
//0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xC0, 0x00, 0x00,//"j",74
//0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x4C, 0x58, 0x70, 0x58, 0x48, 0x44, 0x00, 0x00, 0x00, 0x00,//"k",75
//0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,//"l",76
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x66, 0x46, 0x46, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00,//"m",77
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x66, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00, 0x00,//"n",78
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00, 0x00, 0x00,//"o",79
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x62, 0x42, 0x42, 0x62, 0x5C, 0x40, 0x40, 0x00, 0x00,//"p",80
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x46, 0x42, 0x42, 0x46, 0x3A, 0x02, 0x02, 0x00, 0x00,//"q",81
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x60, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,//"r",82
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x40, 0x60, 0x18, 0x08, 0x78, 0x00, 0x00, 0x00, 0x00,//"s",83
//0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0xF8, 0x60, 0x60, 0x60, 0x60, 0x38, 0x00, 0x00, 0x00, 0x00,//"t",84
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x42, 0x42, 0x42, 0x46, 0x3A, 0x00, 0x00, 0x00, 0x00,//"u",85
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x44, 0x4C, 0x28, 0x28, 0x30, 0x00, 0x00, 0x00, 0x00,//"v",86
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x4C, 0x4A, 0x2A, 0x32, 0x31, 0x00, 0x00, 0x00, 0x00,//"w",87
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x68, 0x30, 0x38, 0x68, 0x44, 0x00, 0x00, 0x00, 0x00,//"x",88
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x44, 0x6C, 0x28, 0x28, 0x10, 0x10, 0x20, 0x00, 0x00,//"y",89
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x18, 0x10, 0x20, 0x40, 0x78, 0x00, 0x00, 0x00, 0x00,//"z",90
//0x00, 0x00, 0x00, 0x30, 0x20, 0x20, 0x20, 0x20, 0x40, 0x20, 0x20, 0x20, 0x20, 0x30, 0x00, 0x00,//"{",91
//0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00,//"|",92
//0x00, 0x00, 0x00, 0x60, 0x20, 0x20, 0x20, 0x20, 0x10, 0x20, 0x20, 0x20, 0x20, 0x60, 0x00, 0x00,//"}",93
//0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//"~",94
//font = 宋体
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //" "
0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x10,0x00,0x00, //"!"
0x00,0x00,0x6C,0x6C,0x24,0x24,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00, //"""
0x00,0x24,0x24,0x24,0x24,0xFE,0x48,0x48,0x48,0x48,0xFC,0x90,0x90,0x90,0x90,0x00, //"#"
0x00,0x10,0x3C,0x54,0x92,0x90,0x50,0x38,0x14,0x12,0x12,0x92,0x54,0x78,0x10,0x00, //"$"
0x00,0x00,0x22,0x5C,0x94,0xA8,0x48,0x10,0x10,0x24,0x2A,0x52,0x54,0x88,0x00,0x00, //"%"
0x00,0x00,0x30,0x48,0x48,0x50,0x20,0x6E,0x54,0x94,0x8C,0x88,0x8A,0x74,0x00,0x00, //"&"
0x00,0x00,0x30,0x30,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //"'"
0x00,0x04,0x08,0x10,0x10,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x08,0x04,0x00, //"("
0x00,0x80,0x40,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x10,0x20,0x20,0x40,0x80,0x00, //")"
0x00,0x00,0x00,0x00,0x10,0x54,0x38,0x10,0x38,0x54,0x10,0x00,0x00,0x00,0x00,0x00, //"*"
0x00,0x00,0x00,0x10,0x10,0x10,0x10,0xFE,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00, //"+"
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x20,0x00, //","
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //"-"
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00, //"."
0x00,0x00,0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x40,0x40,0x80,0x80,0x00,0x00, //"/"
0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00, //"0"
0x00,0x00,0x10,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00, //"1"
0x00,0x00,0x38,0x44,0x82,0x82,0x04,0x08,0x10,0x20,0x40,0x82,0x84,0xFC,0x00,0x00, //"2"
0x00,0x00,0x38,0x44,0x82,0x02,0x04,0x38,0x04,0x02,0x02,0x82,0x44,0x38,0x00,0x00, //"3"
0x00,0x00,0x04,0x0C,0x14,0x14,0x24,0x24,0x44,0x44,0xFE,0x04,0x04,0x0E,0x00,0x00, //"4"
0x00,0x00,0xFC,0x80,0x80,0x80,0xB8,0xC4,0x82,0x02,0x02,0x82,0x84,0x78,0x00,0x00, //"5"
0x00,0x00,0x3C,0x42,0x82,0x80,0xB8,0xC4,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00, //"6"
0x00,0x00,0x7E,0x42,0x82,0x04,0x04,0x08,0x08,0x08,0x10,0x10,0x10,0x10,0x00,0x00, //"7"
0x00,0x00,0x38,0x44,0x82,0x82,0x44,0x38,0x44,0x82,0x82,0x82,0x44,0x38,0x00,0x00, //"8"
0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x46,0x3A,0x02,0x82,0x44,0x38,0x00,0x00, //"9"
0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00,0x00,0x00, //":"
0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x20,0x00,0x00, //";"
0x00,0x00,0x00,0x00,0x06,0x18,0x60,0x80,0x60,0x18,0x06,0x00,0x00,0x00,0x00,0x00, //"<"
0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00, //"="
0x00,0x00,0x00,0x00,0xC0,0x30,0x0C,0x02,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00, //">"
0x00,0x38,0x44,0x82,0x82,0x02,0x04,0x08,0x10,0x10,0x10,0x00,0x10,0x10,0x00,0x00, //"?"
0x00,0x00,0x38,0x44,0x82,0x9A,0xAA,0xAA,0xAA,0xAA,0xAA,0x96,0x80,0x42,0x3C,0x00, //"@"
0x00,0x00,0x10,0x10,0x10,0x28,0x28,0x28,0x44,0x44,0x7C,0x44,0x44,0xEE,0x00,0x00, //"A"
0x00,0x00,0xFC,0x42,0x42,0x42,0x42,0x7C,0x42,0x42,0x42,0x42,0x42,0xFC,0x00,0x00, //"B"
0x00,0x00,0x3C,0x44,0x82,0x80,0x80,0x80,0x80,0x80,0x82,0x82,0x44,0x38,0x00,0x00, //"C"
0x00,0x00,0xF8,0x44,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x44,0xF8,0x00,0x00, //"D"
0x00,0x00,0xFC,0x44,0x42,0x40,0x44,0x7C,0x44,0x40,0x40,0x42,0x44,0xFC,0x00,0x00, //"E"
0x00,0x00,0xFC,0x44,0x42,0x40,0x44,0x7C,0x44,0x40,0x40,0x40,0x40,0xF0,0x00,0x00, //"F"
0x00,0x00,0x34,0x4C,0x82,0x80,0x80,0x80,0x8E,0x84,0x84,0x84,0x4C,0x34,0x00,0x00, //"G"
0x00,0x00,0xEE,0x44,0x44,0x44,0x44,0x7C,0x44,0x44,0x44,0x44,0x44,0xEE,0x00,0x00, //"H"
0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00, //"I"
0x00,0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x88,0x88,0x70,0x00,0x00, //"J"
0x00,0x00,0xEE,0x44,0x48,0x48,0x50,0x60,0x50,0x48,0x48,0x44,0x44,0xEE,0x00,0x00, //"K"
0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x42,0x44,0xFC,0x00,0x00, //"L"
0x00,0x00,0xC6,0x44,0x6C,0x6C,0x6C,0x54,0x54,0x54,0x44,0x44,0x44,0xEE,0x00,0x00, //"M"
0x00,0x00,0xCE,0x44,0x64,0x64,0x64,0x54,0x54,0x4C,0x4C,0x4C,0x44,0xE4,0x00,0x00, //"N"
0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00, //"O"
0x00,0x00,0xF8,0x44,0x42,0x42,0x42,0x44,0x78,0x40,0x40,0x40,0x40,0xE0,0x00,0x00, //"P"
0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0xBA,0x44,0x3C,0x02,0x00, //"Q"
0x00,0x00,0xF0,0x48,0x44,0x44,0x44,0x48,0x70,0x48,0x44,0x44,0x44,0xE6,0x00,0x00, //"R"
0x00,0x00,0x3C,0x44,0x82,0x80,0x40,0x30,0x0C,0x02,0x02,0x82,0x44,0x78,0x00,0x00, //"S"
0x00,0x00,0x7C,0x54,0x92,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00, //"T"
0x00,0x00,0xEE,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00, //"U"
0x00,0x00,0xEE,0x44,0x44,0x44,0x44,0x28,0x28,0x28,0x28,0x10,0x10,0x10,0x00,0x00, //"V"
0x00,0x00,0xEE,0x44,0x54,0x54,0x54,0x54,0x54,0x54,0x28,0x28,0x28,0x28,0x00,0x00, //"W"
0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x28,0x28,0x44,0x44,0xEE,0x00,0x00, //"X"
0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x28,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00, //"Y"
0x00,0x00,0x7E,0x44,0x84,0x08,0x08,0x10,0x20,0x20,0x40,0x82,0x84,0xFC,0x00,0x00, //"Z"
0x00,0x1C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1C,0x00, //"["
0x00,0x00,0xEE,0x44,0x54,0x54,0xFE,0x54,0x54,0x54,0x28,0x28,0x28,0x28,0x00,0x00, //"\"
0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x70,0x00, //"]"
0x00,0x30,0x48,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //"^"
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00, //"_"
0x00,0x40,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //"`"
0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x84,0x04,0x7C,0x84,0x84,0x8C,0x76,0x00,0x00, //"a"
0x00,0x00,0xC0,0x40,0x40,0x40,0x58,0x64,0x42,0x42,0x42,0x42,0x64,0x58,0x00,0x00, //"b"
0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x80,0x80,0x80,0x80,0x44,0x38,0x00,0x00, //"c"
0x00,0x00,0x0C,0x04,0x04,0x04,0x34,0x4C,0x84,0x84,0x84,0x84,0x4C,0x36,0x00,0x00, //"d"
0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x84,0x84,0xFC,0x80,0x80,0x84,0x78,0x00,0x00, //"e"
0x00,0x00,0x18,0x24,0x20,0x20,0xF8,0x20,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00, //"f"
0x00,0x00,0x00,0x00,0x00,0x00,0x3A,0x44,0x44,0x78,0x80,0x7C,0x82,0x82,0x7C,0x00, //"g"
0x00,0x00,0xC0,0x40,0x40,0x40,0x58,0x64,0x44,0x44,0x44,0x44,0x44,0xEE,0x00,0x00, //"h"
0x00,0x00,0x10,0x10,0x00,0x00,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00, //"i"
0x00,0x00,0x10,0x10,0x00,0x00,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x90,0x60,0x00, //"j"
0x00,0x00,0xC0,0x40,0x40,0x40,0x5C,0x48,0x50,0x60,0x50,0x48,0x44,0xEE,0x00,0x00, //"k"
0x00,0x00,0x30,0x10,0x10,0x10,0x10,0x11,0x10,0x10,0x10,0x10,0x10,0x39,0x00,0x00, //"l"
0x00,0x00,0x00,0x00,0x00,0x00,0xAC,0xD2,0x92,0x92,0x92,0x92,0x92,0xD6,0x00,0x00, //"m"
0x00,0x00,0x00,0x00,0x00,0x00,0x58,0xE4,0x44,0x44,0x44,0x44,0x44,0xEE,0x00,0x00, //"n"
0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00, //"o"
0x00,0x00,0x00,0x00,0x00,0x00,0xD8,0x64,0x42,0x42,0x42,0x64,0x58,0x40,0xE0,0x00, //"p"
0x00,0x00,0x00,0x00,0x00,0x00,0x36,0x4C,0x84,0x84,0x84,0x4C,0x34,0x04,0x0E,0x00, //"q"
0x00,0x00,0x00,0x00,0x00,0x00,0x6C,0x30,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00, //"r"
0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x88,0x84,0x60,0x18,0x84,0x44,0x78,0x00,0x00, //"s"
0x00,0x00,0x00,0x20,0x20,0x20,0xF8,0x20,0x20,0x20,0x20,0x20,0x24,0x18,0x00,0x00, //"t"
0x00,0x00,0x00,0x00,0x00,0x00,0xC6,0x42,0x42,0x42,0x42,0x42,0x46,0x3A,0x00,0x00, //"u"
0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x28,0x10,0x10,0x00,0x00, //"v"
0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x54,0x54,0x28,0x28,0x28,0x00,0x00, //"w"
0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x28,0x10,0x10,0x28,0x44,0xEE,0x00,0x00, //"x"
0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0xA0,0xC0,0x00, //"y"
0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x44,0x88,0x10,0x20,0x42,0x84,0xFC,0x00,0x00, //"z"
0x00,0x0C,0x10,0x10,0x10,0x10,0x10,0x60,0x10,0x10,0x10,0x10,0x10,0x10,0x0C,0x00, //"{"
0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00, //"|"
0x00,0xC0,0x20,0x20,0x20,0x20,0x20,0x18,0x20,0x20,0x20,0x20,0x20,0x20,0xC0,0x00, //"}"
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72,0x8C,0x00,0x00,0x00,0x00,0x00,0x00,0x00 //"~"

};
void LCD_delayMs(UInt ms)
{
  UInt i,j;
  if(ms == 0)
  {
    return;
  }
  for(i = 0 ; i < ms ; i++)
  {
    for(j = 0 ; j < 16000;j++);
  }
}

//******************************************************************************
// fn : LCD_close
//
// brief : Function assumes that the handle is not NULL
//
// param : none
//
// return : none
void LCD_close(LCD_Handle handle)
{
    unsigned char i = 0;

    Assert_isTrue((handle != NULL) && (LCD_count != -1), NULL);

    /* Get the pointers to the LCD object and buffer */
    LCD_Object *object = handle->object;
    LCD_Buffer *buffers = object->lcdBuffers;

    /* Destroy the semaphores */
    for (i = 0; i < object->nBuffers; i++)
    {
        Semaphore_destruct(&(buffers[i].bufMutex));
    }

    /* Close SPI */
    SPI_close(object->spiHandle);

    /* Close pin driver and de-allocate pins */
    PIN_close(hPin);

    /* Mark object as closed*/
    object->isOpen = FALSE;
}

/*
 *  ======== LCD_init ========
 */
void LCD_init()
{
    /* Allow multiple calls for LCD_init */
    if (LCD_count >= 0)
    {
        return;
    }
    LCD_count++;

    LCD_Handle handle = (LCD_Handle)&(LCD_config);
    LCD_Object *object;

    /* Get the pointer to the object */
    object = handle->object;

    /* Mark the object as available */
    object->isOpen = FALSE;
}

/*
 *  ======== LCD_Params_init ========
 */
void LCD_Params_init(LCD_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    params->lcdWriteTimeout = BIOS_WAIT_FOREVER;
    params->spiBitRate = 12000000;               //12M bits/S
    params->spiFrameFormat = SPI_POL0_PHA0;
}

/*
 *  ======== LCD_open ========
 */
LCD_Handle LCD_open(LCD_Buffer *buffers, uint8_t nBuffers,
    LCD_Params *lcdParams)
{
    unsigned int key;
    LCD_Params defaultParams;
    LCD_Object *lcdObject;
    LCD_HWAttrs const *lcdHwAttrs;
    SPI_Handle spiHandle;
    LCD_Handle handle = (LCD_Handle) &(LCD_config);

    /* Get the pointer to the object and hwAttrs. */
    lcdObject = handle->object;
    lcdHwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if the LCD is open. */
    key = Hwi_disable();

    /* Determine if the device index was already opened. */
    if (lcdObject->isOpen == TRUE)
    {
        Hwi_restore(key);
        Log_warning0("LCD already in use.");
        return (NULL);
    }

    /* Mark the handle as being used. */
    lcdObject->isOpen = TRUE;
    Hwi_restore(key);
    
    /* If params are NULL use defaults. */
    if (lcdParams == NULL)
    {
        LCD_Params_init(&defaultParams);
        lcdParams = &defaultParams;
    }

    /* If buffers are NULL, or nBuffers <= 0, return */
    if((buffers == NULL) ||(nBuffers <= 0))
    {
        Log_warning0("No buffer is availible for the LCD driver");
        return (NULL);
    }

    /* Initialize SPI parameters. Master mode, blocking mode
     * and dataSize=8 is default and should not be changed.
     * The other parameters can be set from LCD parameters.
     */
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = lcdParams->spiBitRate;
    spiParams.frameFormat = lcdParams->spiFrameFormat;

    /* Try open the SPI */
    spiHandle = SPI_open(lcdHwAttrs->spiIndex, &spiParams);
    if (!spiHandle)
    {
        return (NULL);
    }

    /* Initialize the LCD object */
    lcdObject->spiHandle = spiHandle;
    lcdObject->lcdWriteTimeout = lcdParams->lcdWriteTimeout;
    lcdObject->lcdBuffers = buffers;
    lcdObject->nBuffers = nBuffers;

    /* Create a counting semaphore for each buffer */
    unsigned char i = 0;
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    for (i = 0; i < nBuffers; i++)
    {
        Semaphore_construct(&(buffers[i].bufMutex), 1, &semParams);
    }

    /* Create a binary semaphore for the LCD */
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&lcdObject->lcdMutex, 1, &semParams);

    /* Configure the hardware module */
    if(!LCD_initHw(handle))
    {
       //Hw initialization failed
       return (NULL);
    }

    /* Send LCD init commands */
    LCD_initSet(handle, (const LCD_Command *) (lcdHwAttrs->LCD_initCmd));

    Log_print0(Diags_USER1, "LCD: LCD is opened");
    return (handle);
}

static void LCD_initSet(LCD_Handle handle,const LCD_Command* pcCmd)
{
  Assert_isTrue((handle != NULL), NULL);
  unsigned int cmdLen = 0;
  
  //需要延时，否则初始化不通过。
  //Task_sleep(2);
  
  LCD_sendCommand(handle,&pcCmd->sleepExit.cmdId,1);
  
  /* Delay ~120 ms for LCD to be wake up. */
  LCD_delayMs(pcCmd->sleepExit.delayMs);

  cmdLen += sizeof(LCD_Sleep_CMD);
  
  LCD_Sleep_CMD *pSleepCmd = (LCD_Sleep_CMD *)pcCmd;
  LCD_Param_CMD *pParamCmd = (LCD_Param_CMD *)(pSleepCmd + 1);
  while(cmdLen < sizeof(LCD_Command))
  {
    LCD_sendCommand(handle,&pParamCmd->cmdId,1);
    if(pParamCmd->count > 0)
    {
      LCD_sendData(handle,pParamCmd->param,pParamCmd->count);
    }
    pParamCmd++;
    cmdLen += sizeof(LCD_Param_CMD);
  }
}
/*
 *  ======== LCD_writeLine ========
 *  This function writes one line of the specified buffer
 *  and sends it to the display.
 */
void LCD_writeLine(LCD_Handle handle, char *str,
    unsigned int uiValue, unsigned char ucFormat, unsigned char ucLine)
{
    LCD_Page pageNo = (LCD_Page) (ucLine % LCD_PAGE_COUNT);

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;


    /* Pend on the LCD Mutex */
    Log_print0(Diags_USER1, "LCD_writeLine: pending on LCD mutex.");
    if (!Semaphore_pend(Semaphore_handle(&object->lcdMutex),
            object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */
        Log_warning0("Waiting for access to LCD timed out, exiting LCD_writeLine.");
        return;
    }

    /* Get pointers to the buffer and its semaphore. */
    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    if (ucFormat)
    {

        unsigned char maxLen = 50; // max string length
        unsigned char buf[50];
        unsigned char strLength;

        /* Check that there is a null termination in the string */
        const char *end = (const char *) memchr(str, '\0', maxLen);
        if (end == NULL)
            strLength = maxLen;
        else
            strLength = end - str;

        memset(buf, ' ', maxLen);
        memcpy(buf, str, strLength);

        /* Get number of characters in string */
        unsigned char ucNumOfDigits = LCD_getIntLength(uiValue, ucFormat);

        /* String length + 2 spaces + number of digits cannot exceed one line */
        if ((strLength + 2 + ucNumOfDigits) > (LCD_COLS / LCD_CHAR_WIDTH))
        {
            Log_warning1("LCD_writeLine does not support a string size larger than %d characters.",
                    (LCD_COLS/LCD_CHAR_WIDTH));
            Semaphore_post(Semaphore_handle(&object->lcdMutex));
            return;
        }

        /* Copy to local buffer and send */
        _itoa(uiValue, &buf[strLength + 2], ucFormat);

        /* Pend on buffer semaphore */
 
        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            Semaphore_post(Semaphore_handle(&object->lcdMutex));
            return;
        }

        /* Clear the page */
        LCD_doBufferClearPage(handle,  pageNo, false);

        /* write buffer*/
        LCD_doBufferPrintString(handle, (char*) buf, 0, pageNo,
                false);

    }
    else
    {

        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            Semaphore_post(Semaphore_handle(&object->lcdMutex));
            return;
        }

        /* Clear the page */
        LCD_doBufferClearPage(handle, pageNo, false);

        /* write buffer*/
        LCD_doBufferPrintString(handle, str, 0, pageNo, false);

    }

    /* Finished with buffer - post on semaphores*/
    Semaphore_post(Semaphore_handle(pSem));
    Log_print0(Diags_USER1, "LCD_writeLine: posting LCD mutex.");
    Semaphore_post(Semaphore_handle(&object->lcdMutex));
}
/*
 *  ======== LCD_bufferClear ========
 *  This function empties the specified LCD buffer
 *  by filling it with zeros.
 */
void LCD_bufferClear(LCD_Handle handle)
{
  Assert_isTrue((handle != NULL), NULL);
  
  LCD_fullRegion(handle,handle->object->pColorInfo->blackColor,0,0,LCD_COLS - 1,LCD_ROWS - 1);

}
/*
 * ======== LCD_bufferClearPage ========
 * This function clears the specified page of a buffer
 */
void LCD_bufferClearPage(LCD_Handle handle, LCD_Page iPage)
{
    /* Call clear page function with use of semaphore */
    LCD_doBufferClearPage(handle, iPage, true);
}

/*
 * ======== LCD_bufferClearPart ========
 * This function clears the pixels in a given piece of a page.
 */
void LCD_bufferClearPart(LCD_Handle handle,
    unsigned char ucXFrom, unsigned char ucXTo, LCD_Page iPageFrom,
    LCD_Page iPageTo)
{
  unsigned char ucYFrom,ucYTo;
  
  /* Get pointer to the LCD object */
  LCD_Object *object = handle->object;
  
  /* Get pointers to the buffer and its semaphore. */
  Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);
  
  /* Pend on the semaphore */
  if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
  {
    /* Semaphore timed out, log and return. */ 
    return;
  }
  ucYFrom = iPageFrom * LCD_PAGE_ROWS;
  
  if(iPageTo  >=  LCD_PAGE_COUNT)
  {
    iPageTo = LCD_PAGE7; 
  }
  ucYTo = (iPageTo + 1) * LCD_PAGE_ROWS - 1;
  /* Clear buffer part */   
  LCD_fullRegion(handle,handle->object->pColorInfo->blackColor,ucXFrom,ucYFrom,ucXTo,ucYTo);
  
  /* Finished with buffer - post on semaphore*/
  Semaphore_post(Semaphore_handle(pSem));
}

/*
 * ======== LCD_bufferInvert ========
 * This function inverts the pixels (bits) in a given region of the
 * a buffer.
 */
void LCD_bufferInvert(LCD_Handle handle, unsigned char ucXFrom,
                          unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo)
{
    unsigned char ucI, ucJ, ucPow;
    unsigned char ucFirstPage, ucLastPage, ucFirstPageMask, ucLastPageMask;

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    /* Get pointers to the buffer and its semaphore. */
    unsigned short *pcBuf = object->lcdBuffers[0].pcBuffer;
    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    /* Find the first and last page to invert on */
    ucFirstPage = ucYFrom / LCD_PAGE_ROWS;
    ucLastPage = ucYTo / LCD_PAGE_ROWS;

    /* Find the bitmask to invert with on first page */
    ucFirstPageMask = 0xFF;
    ucPow = 1;

    /* Generate invert bitmask for the first page */
    for (ucI = 0; ucI < LCD_PAGE_ROWS; ucI++)
    {
        if (ucYFrom - ucFirstPage * LCD_PAGE_ROWS > ucI)
        {
            ucFirstPageMask -= ucPow;
            ucPow *= 2;
        }
    }

    /* Find the bitmask to invert with on the last page */
    ucLastPageMask = 0x00;
    ucPow = 1;
    for (ucI = 0; ucI < LCD_PAGE_ROWS; ucI++)
    {
        if (ucYTo - ucLastPage * LCD_PAGE_ROWS >= ucI)
        {
            ucLastPageMask += ucPow;
            ucPow *= 2;
        }
    }

    /* Prevent error if ucFirstPage==ucLastPage */
    if (ucFirstPage == ucLastPage)
    {
        ucLastPageMask ^= 0xFF;
    }

    /* Pend on the semaphore */
    if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */


        return;
    }

    /* Invert the given part of the first page */
    for (ucI = ucXFrom; ucI <= ucXTo; ucI++)
    {
        *(pcBuf + (ucFirstPage * LCD_COLS + ucI)) ^= ucFirstPageMask;
    }

    /* Invert the pages between first and last in the given section */
    for (ucI = ucFirstPage + 1; ucI <= ucLastPage - 1; ucI++)
    {
        for (ucJ = ucXFrom; ucJ <= ucXTo; ucJ++)
        {
            *(pcBuf + (ucI * LCD_COLS + ucJ)) ^= 0xFF;
        }
    }

    /* Invert the given part of the last page */
    for (ucI = ucXFrom; ucI <= ucXTo; ucI++)
    {
        *(pcBuf + (ucLastPage * LCD_COLS + ucI)) ^= ucLastPageMask;
    }

    /* Finished with buffer - post on semaphore*/
    Semaphore_post(Semaphore_handle(pSem));
}

/*
 *  ======== LCD_bufferInvertPage ========
 *  This function inverts a range of columns in the display buffer on a
 *  specified page.
 */
void LCD_bufferInvertPage(LCD_Handle handle, unsigned char ucXFrom,
                             unsigned char ucXTo, LCD_Page iPage)
{
    unsigned char ucI;
    unsigned short usFirstPos = iPage * LCD_COLS + ucXFrom;
    unsigned char ucRange = ucXTo - ucXFrom;

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    /* Get pointers to the buffer and its semaphore. */
    unsigned short *pcBuf = object->lcdBuffers[0].pcBuffer;
    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    /* Pend on the semaphore */
    if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */

        return;
    }

    /* Invert buffer range */
    for (ucI = 0; ucI <= ucRange; ucI++)
    {
        *(pcBuf + (usFirstPos + ucI)) ^= 0xFF;
    }

    /* Finished with buffer - post on semaphore*/
    Semaphore_post(Semaphore_handle(pSem));
}

/*
 *  ======== LCD_bufferPrintString ========
 *  This function writes a string to the specified buffer
 */
void LCD_bufferPrintString(LCD_Handle handle, const char *pcStr,
                              unsigned char ucX, LCD_Page iPage)
{
  unsigned char ucY = 0;
  ucY = iPage * LCD_PAGE_ROWS;
  /* Call print string function with use of semaphore */
  LCD_doBufferPrintString(handle, pcStr, ucX, ucY, true);
}

/*
 *  ======== LCD_bufferSetLine ========
 *  This function draws a line into the specified buffer.
 */
void LCD_bufferSetLine(LCD_Handle handle, unsigned char ucXFrom,
                       unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo)
{
    /* Draw line */
    LCD_bufferLine(handle, ucXFrom, ucYFrom, ucXTo, ucYTo, 1);
}

/*
 *  ======== LCD_bufferClearLine ========
 *  This function clears a line intoo the specified buffer.
 */
void LCD_bufferClearLine(LCD_Handle handle, unsigned char ucXFrom,
                          unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo)
{
    /* Clear line */
    LCD_bufferLine(handle, ucXFrom, ucYFrom, ucXTo, ucYTo, 0);
}

/*
 *  ======== LCD_bufferSetHLine ========
 *  This function draws a horizontal line into the specified buffer.
 */
void LCD_bufferSetHLine(LCD_Handle handle, unsigned char ucXFrom,
                        unsigned char ucXTo, unsigned char ucY)
{
    /* Call LCD_doBufferSetHLine with use of semaphore */
    LCD_doBufferSetHLine(handle, ucXFrom, ucXTo, ucY, true);

}

/*
 *  ======== LCD_bufferClearHLine ========
 *  This function clears a horizontal line from the specified buffer.
 */
void LCD_bufferClearHLine(LCD_Handle handle, unsigned char ucXFrom,
                          unsigned char ucXTo, unsigned char ucY)
{
    /* Call LCD_doBufferClearHLine with use of semaphore */
    LCD_doBufferClearHLine(handle, ucXFrom, ucXTo, ucY, true);

}

/*
 *  ======== LCD_bufferSetVLine ========
 *  This function draws a vertical line into the specified buffer.
 */
void LCD_bufferSetVLine(LCD_Handle handle, unsigned char ucX,
                        unsigned char ucYFrom, unsigned char ucYTo)
{

    /* Call LCD_doBufferSetVLine with use of semaphore */
    LCD_doBufferSetVLine(handle,ucX, ucYFrom, ucYTo, true);

}

/*
 *  ======== LCD_bufferClearVLine ========
 *  This function clears a vertical line from the specified buffer.
 */
void LCD_bufferClearVLine(LCD_Handle handle,  unsigned char ucX,
                          unsigned char ucYFrom, unsigned char ucYTo)
{

    /* Call LCD_doBufferClearVLine with use of semaphore */
    LCD_doBufferClearVLine(handle,  ucX, ucYFrom, ucYTo, true);

}

/*
 *  ======== LCD_bufferHArrow ========
 *  This function draws a horizontal arrow to the specified buffer.
 */
void LCD_bufferHArrow(LCD_Handle handle,  unsigned char ucXFrom,
                      unsigned char ucXTo, unsigned char ucY)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    /* Get pointer to the buffer semaphore. */
    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    /* Pend on the semaphore*/
    if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */
        return;
    }

    if (ucXTo > ucXFrom)
    {
        /* Draw left-to-right arrow */
        LCD_doBufferSetHLine(handle,  ucXFrom, ucXTo, ucY, false);
        LCD_doBufferSetVLine(handle,  ucXTo - 1, ucY - 1, ucY + 1,
                false);
        LCD_doBufferSetVLine(handle,  ucXTo - 2, ucY - 2, ucY + 2,
                false);
    }
    else if (ucXTo < ucXFrom)
    {
        /* Draw right-to-left arrow */
        LCD_doBufferSetHLine(handle,  ucXTo, ucXFrom, ucY, false);
        LCD_doBufferSetVLine(handle,  ucXTo + 1, ucY - 1, ucY + 1,
                false);
        LCD_doBufferSetVLine(handle,  ucXTo + 2, ucY - 2, ucY + 2,
                false);
    }

    /* Finished with buffer - post on semaphore*/
    Semaphore_post(Semaphore_handle(pSem));
}

/*
 *  ======== LCD_bufferVArrow ========
 *  This function draws a vertical arrow to the specified buffer.
 */
void LCD_bufferVArrow(LCD_Handle handle,  unsigned char ucX,
                      unsigned char ucYFrom, unsigned char ucYTo)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    /* Get pointers to the buffer semaphore. */
    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    /* Pend on the semaphore*/
    if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */
        return;
    }

    /* Draw the line */
    LCD_doBufferSetVLine(handle, ucX, ucYFrom, ucYTo, false);

    /* Draw arrowhead */
    LCD_doBufferSetHLine(handle,  ucX - 1, ucX + 1, ucYTo - 1, false);
    LCD_doBufferSetHLine(handle,  ucX - 2, ucX + 2, ucYTo - 2, false);

    /* Finished with buffer - post on semaphore*/
    Semaphore_post(Semaphore_handle(pSem));
}

/*
 *  ======== LCD_bufferSetPx ========
 *  This function sets a pixel in the specified buffer.
 */
void LCD_bufferSetPx(LCD_Handle handle, unsigned char ucX, unsigned char ucY)
{
    /* Call LCD_doBufferSetPx with use of semaphore */
    LCD_doBufferSetPx(handle,true, ucX, ucY);
}

/*
 *  ======== LCD_bufferCopy ========
 *  This function copies the content of one buffer to another
 */
void LCD_bufferCopy(LCD_Handle handle, unsigned int fromBufIndex, unsigned int toBufIndex)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    /* Do a check on buffer index */
    if ((fromBufIndex >= object->nBuffers) || (toBufIndex >= object->nBuffers))
    {
        Log_warning1("The LCD driver has only %d buffers availible", object->nBuffers);
        return;
    }

    /* Get pointers to the buffer and its semaphore. */
    UShort *pcFromBuffer = object->lcdBuffers[fromBufIndex].pcBuffer;
    UShort *pcToBuffer = object->lcdBuffers[toBufIndex].pcBuffer;
    Semaphore_Struct *pSemFrom = &(object->lcdBuffers[fromBufIndex].bufMutex);
    Semaphore_Struct *pSemTo = &(object->lcdBuffers[toBufIndex].bufMutex);

    /* Get buffer sizes */
    unsigned int fromBufSize = object->lcdBuffers[fromBufIndex].bufSize;
    unsigned int toBufSize = object->lcdBuffers[toBufIndex].bufSize;

    UShort *pcTmpToBuf = pcToBuffer;
    short *pcTmpFromBuf = (short *) pcFromBuffer;
    register unsigned short i;

    /* If buffers are the same, do nothing */
    if (pcFromBuffer == pcToBuffer)
    {
        Log_print0(Diags_USER1, "Buffers are the same, nothing to be done.");
        return;
    }
    /* Return if to-buffer is smaller than from-buffer */
    if (toBufSize < fromBufSize)
    {
        Log_print0(Diags_USER1, "The receive buffer cannot be smaller than the transmit buffer. Copy aborted.");
        return;
    }

    /* Pend on the semaphores*/
    if (!Semaphore_pend(Semaphore_handle(pSemFrom), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */
        Log_warning1("Copying from LCD buffer (%p) timed out, exiting LCD_bufferCopy.",
                fromBufIndex);
        return;
    }
    /* Pend on the semaphore*/
    if (!Semaphore_pend(Semaphore_handle(pSemTo), object->lcdWriteTimeout))
    {
        /* Semaphore timed out, log and return. */
        Log_warning1("Copying to LCD buffer (%p) timed out, exiting LCD_bufferCopy.",
                toBufIndex);
        return;
    }

    /* Copy */
    for (i = 0; i < fromBufSize; i++)
    {
        pcTmpToBuf[i] = pcTmpFromBuf[i];
    }

    /* Finished with buffers - post on semaphores*/
    Semaphore_post(Semaphore_handle(pSemFrom));
    Semaphore_post(Semaphore_handle(pSemTo));
}

//******************************************************************************
// fn : LCD_hwInit
//
// brief : This functions initializes the LCD hardware module.It returns true 
//         if initialization was successful, false otherwise
// 
// param : handle -> the lcd handle
//         pcData -> point to the address of data area
//         usLen  -> the data len
//
// return : none
static bool LCD_initHw(LCD_Handle handle)
{
    /* Locals */
    PIN_Config lcdPinTable[3];
    uint32_t i = 0;
    LCD_HWAttrs const *hwAttrs;

    /* get the pointer to the hwAttrs */
    hwAttrs = handle->hwAttrs;

    /* Populate LCD pin table and initilize pins*/
    lcdPinTable[i++] = hwAttrs->lcdModePin      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL;
    lcdPinTable[i++] = hwAttrs->lcdCsnPin       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL;
    lcdPinTable[i++] = PIN_TERMINATE;

    /* Open and assign pins through pin driver*/
    hPin = PIN_open(&pinState, lcdPinTable);
    if(!hPin)
    {
        /* Pin allocation failed, pins may already be allocated */
        return (false);
    }
    return (true);
}

//******************************************************************************
// fn : LCD_sendCommand
//
// brief : This function sends @e ucLen bytes of commands to the LCD controller.
// 
// param : handle -> the lcd handle
//         pcData -> point to the address of data area
//         usLen  -> the data len
//
// return : none
static void LCD_sendCommand(LCD_Handle handle, const char *pcCmd, unsigned char ucLen)
{
    /* Get the pointer to the LCD object*/
    LCD_HWAttrs const *hwAttrs = handle->hwAttrs;

    Assert_isTrue((handle != NULL), NULL);

    /* Set LCD mode signal low (command) */
    PIN_setOutputValue(hPin, hwAttrs->lcdModePin, 0);
    
    /* Set LCD CSn low (spi active) */
    PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 0);

    /* Do SPI transfer */
    if (LCD_sendArray(handle, pcCmd, ucLen)) {
        /* Clear CSn */
        PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 1);
    }

}
//******************************************************************************
// fn : LCD_sendData
//
// brief : the send display data to lcd
// 
// param : handle -> the lcd handle
//         pcData -> point to the address of data area
//         usLen  -> the data len
//
// return : none
static void LCD_sendData(LCD_Handle handle, const char *pcData, unsigned short usLen)
{
    /* Get the pointer to the LCD object*/
    LCD_HWAttrs const *hwAttrs = handle->hwAttrs;

    Assert_isTrue((handle != NULL), NULL);

    /* Set LCD mode signal (data) */
    PIN_setOutputValue(hPin, hwAttrs->lcdModePin, 1);
    /* Set LCD CSn low (spi active) */
    PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 0);

    /* Do SPI transfer */
    if (LCD_sendArray(handle, pcData, usLen)) {
        /* Clear CSn */
        PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 1);
    }
}

//******************************************************************************
// fn : LCD_gotoXY
//
// brief : Function that sets the internal data cursor of the LCD to the
//         location specified by @e ucX and @e ucY. When data is sent to the 
//         display, data will start printing at internal cursor location.
//
// param : handle -> the lcd handle
//         ucX    -> x position
//         ucY    -> y postion
//
// return :none
//static void LCD_gotoXY(LCD_Handle handle, unsigned char ucX, unsigned char ucY)
//{
//  LCD_setRegion(handle,ucX,ucY,ucX,ucY);
//}
//******************************************************************************
// fn : LCD_setRegion
//
// brief : Function that sets the internal display area. 
//
// param : handle -> the lcd handle
//         ucX    -> x position
//         ucY    -> y postion
//         ucXend -> the end x position
//         ucYend -> the end y position
//
// return :none
static void LCD_setRegion(LCD_Handle handle,unsigned char ucX,unsigned char ucY,
    unsigned char ucXend,unsigned char ucYend)
{
  Assert_isTrue((handle != NULL), NULL);
  unsigned char tmp[] = { 0x00, 0x00, 0x00,0x00 }; 
  tmp[0] = 0x2a;
  LCD_sendCommand(handle, (char *) tmp, 1);    //row curor
  tmp[0] = 0X00;
  tmp[1] = ucX + 2;
  tmp[2] = 0x00;
  tmp[3] = ucXend + 2;
  LCD_sendData(handle,(char const*)tmp,4);
  
  tmp[0] = 0X2b;                              //column curor
  LCD_sendCommand(handle, (char *) tmp, 1); 
  tmp[0] = 0X00;
  tmp[1] = ucY + 3;
  tmp[2] = 0x00;
  tmp[3] = ucYend + 3;
	LCD_sendData(handle,(char const*)tmp,4);
  
  tmp[0] = 0x2c;                              
  LCD_sendCommand(handle, (char *) tmp, 1); 
  
}
//******************************************************************************
// fn : LCD_fullRegin
//
// brief : full the speified area with color
//
// param : handle ->  the lcd handle
//         color  ->  the color
//         ucXFrom ->  x start points
//         ucYFrom ->  y start points
//         ucXTo   ->  x end points
//         ucYTo   ->  y end points
//
// return 
static void LCD_fullRegion(LCD_Handle handle, Color color, unsigned char ucXFrom,
                           unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo)
{
  Assert_isTrue((handle != NULL), NULL);
  
   /* Get the pointer to the LCD object*/
  LCD_HWAttrs const *hwAttrs = handle->hwAttrs;
  LCD_Object *object = handle->object;
  
  unsigned short i;
  unsigned short numPoint = 0;
  unsigned short  times = 0;
  unsigned short  remainPoint = 0;
  
  numPoint = (ucXTo  - ucXFrom + 1) * (ucYTo - ucYFrom + 1);
  
  LCD_setRegion(handle,ucXFrom,ucYFrom,ucXTo,ucYTo);
  
  for(i = 0; i < 512; i++)
  {
    *(object->lcdBuffers[0].pcBuffer + i) = color;
  }
  
  /* Set LCD mode signal (data) */
  PIN_setOutputValue(hPin, hwAttrs->lcdModePin, 1);
  /* Set LCD CSn low (spi active) */
  PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 0); 
  
  times = numPoint / 512;
  remainPoint = numPoint % 512;
    
  for( i = 0 ;i < times; i++)
  {
    LCD_sendArray(handle, (char const*)object->lcdBuffers[0].pcBuffer, 1024);
  }
  if(remainPoint > 0)
  {
    LCD_sendArray(handle, (char const*)object->lcdBuffers[0].pcBuffer, remainPoint * 2);
  }
  
  PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 1);
}
/*
 *  ======== LCD_sendArray ========
 *  This function sends @e usLen bytes from starting from address
 *  @e pcData over SPI to the LCD controller. This function only pushes
 *  data to the SPI module. It does not manipulate the LCD display's
 *  CSn signal, nor the LCD mode signal (A0).
 */
static bool LCD_sendArray(LCD_Handle handle, const char *pcData, unsigned short usLen)
{
  
  /* Get the pointer to the LCD object*/
  LCD_Object *object = handle->object;
  
  /* Do SPI transfer */
  SPI_Transaction spiTransaction;
  spiTransaction.arg = NULL;
  spiTransaction.count = usLen;
  spiTransaction.txBuf = (Ptr)pcData;
  spiTransaction.rxBuf = NULL;
  
  bool ret = SPI_transfer(object->spiHandle, &spiTransaction);
  if(ret == false)
  {
    return false;
  }
  /* return... */
  return true;
}

/*
 *  ======== LCD_bufferLine ========
 *  Local function. Draws or clears (based on @e ucDraw) a line from
 *  @e (ucXFrom,ucYFrom) to @e (ucXTo,ucYTo). Uses Bresenham's line algorithm.
 */
static void LCD_bufferLine(LCD_Handle handle, unsigned char ucXFrom,
                           unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo, unsigned char ucDraw)
{
  signed char cX, cY, cDeltaY, cDeltaX, cD;
  signed char cXDir, cYDir;
  
  if (ucXFrom == ucXTo)
  {
    /* Vertical line */
    if (ucDraw)
    {
      LCD_doBufferSetVLine(handle,  ucXFrom, ucYFrom, ucYTo,
                           false);
    }
    else
    {
      LCD_doBufferClearVLine(handle,  ucXFrom, ucYFrom, ucYTo,
                             false);
    }
  }
  else if (ucYFrom == ucYTo)
  {
    /* Horizontal line */
    if (ucDraw)
    {
      LCD_doBufferSetHLine(handle,  ucXFrom, ucXTo, ucYFrom,
                           false);
    }
    else
    {
      LCD_doBufferClearHLine(handle,  ucXFrom, ucXTo, ucYFrom,
                             false);
    }
  }
  else
  {
    
    /* Diagonal Line => Bresenham's algorithm
    * Determine X and Y direction
    */
    cXDir = (ucXFrom > ucXTo) ? -1 : 1;
    cYDir = (ucYFrom > ucYTo) ? -1 : 1;
    
    /* Set start position and calculate X and Y delta */
    cX = ucXFrom;
    cY = ucYFrom;
    cDeltaY = ucYTo - ucYFrom;
    cDeltaX = ucXTo - ucXFrom;
    
    /* Take absolute value of X and Y delta */
    if (cDeltaY < 0)
    {
      cDeltaY *= -1;
    }
    if (cDeltaX < 0)
    {
      cDeltaX *= -1;
    }
    
    /* Determine principal direction and draw line */
    if (cDeltaX >= cDeltaY)
    {
      cD = (cDeltaY << 1) - cDeltaX;
      while (cX != ucXTo)
      {
        LCD_doBufferSetPx(handle,ucDraw,  cX, cY );
        
        if (cD < 0)
        {
          cD += (cDeltaY << 1);
        }
        else
        {
          cD += ((cDeltaY - cDeltaX) << 1);
          cY += cYDir;
        }
        cX += cXDir;
      }
    }
    else
    {
      cD = (cDeltaX << 1) - cDeltaY;
      while (cY != ucYTo)
      {
        
        LCD_doBufferSetPx(handle, ucDraw, cX, cY);
        if (cD < 0)
        {
          cD += (cDeltaX << 1);
        }
        else
        {
          cD += ((cDeltaX - cDeltaY) << 1);
          cX += cXDir;
        }
        cY += cYDir;
      }
    }
  }
}
//******************************************************************************
// fn : LCD_doBufferClearPage
//
// brief : This function clears the page specified by @e iPage in the given buffer
//         If blocking is set to true, the task execution will be blocked until all
//         buffer modifications have finished.
//
// param : handle -> the lcd hanle
//         iPage -> the line
//         blocking -> 
//
// return : none
static void LCD_doBufferClearPage(LCD_Handle handle,
                                 LCD_Page iPage, bool blocking)
{
    unsigned char ucYFrom;

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;
    
    Semaphore_Struct *pSem = &(object->lcdMutex);

    /* Pend on the semaphore if blocking option is set*/
    if (blocking)
    {
        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            return;
        }
    }

    /* Clear page in buffer */
    ucYFrom = iPage * LCD_COLS;
    
    LCD_fullRegion(handle,object->pColorInfo->blackColor,0,ucYFrom,127,ucYFrom + LCD_COLS);
    
    /* Finished with buffer - post on semaphore*/
    if (blocking)
    {
        Semaphore_post(Semaphore_handle(pSem));
    }
}

//******************************************************************************
// fn : LCD_doBufferPrintString
//
// brief : This function writes a string to the specified buffer,If blocking is 
//         set to true, the task execution will be blocked until all buffer 
//         modifications have finished.
//
// param : handle -> the lcd hanle
//         pcStr -> the address of display string
//         ucX -> the x position
//         ucY -> the y position
//         blocking -> whether blocking or not 
//
// return : none
static void LCD_doBufferPrintString(LCD_Handle handle, const char *pcStr,
                                    unsigned char ucX, unsigned char ucY, bool blocking)
{
  Assert_isTrue((handle != NULL), NULL);
  
  unsigned char i,j,ucI,y;
  unsigned char k;
  unsigned char xFirstPosition = 0;
  unsigned char row;
  unsigned char ucStrSize = LCD_getStringLength(pcStr);
  /* Get pointer to the LCD object */
  LCD_Object *object = handle->object;
  
  buf_asc asc = (buf_asc)(object->lcdBuffers[0].pcBuffer);
  
  Semaphore_Struct *pSem = &(object->lcdMutex);
  
  /* Pend on the semaphore if blocking option is set*/
  if (blocking)
  {
    if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
    {
      
      return;
    }
  }
  row = xFirstPosition = ucX;
  y = ucY;
  
  /* Running through each letter in input string */
  for (ucI = 0; ucI < ucStrSize; ucI++)
  {
    k = pcStr[ucI];                               //Get the alpha
    if (k == 13) 
    {
      ucX = xFirstPosition;
      y += LCD_PAGE_ROWS;                         //one page = 16 rows
    }
    else 
    {
      if (k > 32)                                //可显示字符是从ascii=32 开始
      {
        k -= 32; 
      }
      else 
      {
        k = 0;                                   //不可显示字符都以空格显示
      }
      
      for(i = 0; i < 16 ; i++)
      {
        for( j = 0 ; j < 8; j++) 
        {
          if(LCD_alphabet[k * 16 + i] & (0x80 >> j))	
          {
            asc[i][(ucI % 4)*8 +j] = object->pColorInfo->frontColor;
          }
          else 
          {
            asc[i][(ucI % 4)*8 +j] = object->pColorInfo->blackColor;
          }
        }
        
      }
      ucX += 8;
    }
    
    //512显存点，只能存放4个16*8尺寸大小的字符
    if((ucI % 4) == 3)
    {
      LCD_Update(handle,row,y, ucX - 1 ,y + 15);
      row = ucX;
    }
  }
  
  k = ucI % 4;
  if(k )
  {
    //将余下的显存空间，全部设置成背景色 
    for(char x = k  ; x < 4 ; x++)
    {
      for(i = 0; i < 16 ; i++)
      {
        for( j = 0 ; j < 8; j++) 
        {
          asc[i][x*8 +j] = object->pColorInfo->blackColor;
        }
      }
    }
    LCD_Update(handle,row,y,row + 31,y + 15);
  }
  
  if (blocking)
  {
    Semaphore_post(Semaphore_handle(pSem));
  }
}
//******************************************************************************
// fn : LCD_Update
// 
// brief : copy the memery to lcd static memery.the display memery size is 1024 
//          bytes. so,the region size must be 512 pixels
//
// params : handle    -> the lcd handle
//          ucXFrom   -> the start x point
//          ucYFrom   -> the start y point
//          ucXTo     -> the end x point
//          ucYTo     -> the end y point
// return : none
static void LCD_Update(LCD_Handle handle,unsigned char ucXFrom,
                           unsigned char ucYFrom, unsigned char ucXTo, unsigned char ucYTo)
{
  Assert_isTrue((handle != NULL), NULL);
  
   /* Get the pointer to the LCD object*/
  LCD_HWAttrs const *hwAttrs = handle->hwAttrs;
  LCD_Object *object = handle->object;
  
  unsigned short numPoint = 0;
  
  numPoint = (ucXTo  - ucXFrom + 1) * (ucYTo - ucYFrom + 1);
  
  LCD_setRegion(handle,ucXFrom,ucYFrom,ucXTo,ucYTo);
  
  /* Set LCD mode signal (data) */
  PIN_setOutputValue(hPin, hwAttrs->lcdModePin, 1);
  /* Set LCD CSn low (spi active) */
  PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 0); 
   
  LCD_sendArray(handle, (char const*)object->lcdBuffers[0].pcBuffer, numPoint *2);
 
  PIN_setOutputValue(hPin, hwAttrs->lcdCsnPin, 1);
  
}
/*
 *  ======== LCD_doBufferSetHLine ========
 *  This function draws a horizontal line into the specified buffer.
 *  If blocking is set to true, the task execution will be blocked until all
 *  buffer modifications have finished.
 */
static void LCD_doBufferSetHLine(LCD_Handle handle, unsigned char ucXFrom,
                                 unsigned char ucXTo, unsigned char ucY, bool blocking)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    Semaphore_Struct *pSem = &(object->lcdMutex);

    /* Switch draw direction if ucXTo < ucXFrom */
    if (ucXTo < ucXFrom)
    {
        unsigned char ucTemp = ucXFrom;
        ucXFrom = ucXTo;
        ucXTo = ucTemp;
    }

    /* Pend on the semaphore if blocking option is set*/
    if (blocking)
    {
      if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
      {
        /* Semaphore timed out, log and return. */
        return;
      }
    }
    /* Draw line */
    LCD_fullRegion(handle,object->pColorInfo->frontColor,ucXFrom,ucXTo,ucY,ucY);

    /* Finished with buffer - post on semaphore*/
    if (blocking)
    {
        Semaphore_post(Semaphore_handle(pSem));
    }
}

/*
 *  ======== LCD_doBufferClearHLine ========
 *  This function clears a horizontal line from the specified buffer.
 *  If blocking is set to true, the task execution will be blocked until all
 *  buffer modifications have finished.
 *
 */
static void LCD_doBufferClearHLine(LCD_Handle handle,
                                   unsigned char ucXFrom, unsigned char ucXTo, unsigned char ucY, bool blocking)
{
    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    Semaphore_Struct *pSem = &(object->lcdBuffers[0].bufMutex);

    /* Switch draw direction if ucXTo < ucXFrom */
    if (ucXTo < ucXFrom)
    {
        unsigned char ucTemp = ucXFrom;
        ucXFrom = ucXTo;
        ucXTo = ucTemp;
    }

    /* Pend on the semaphore if blocking option is set*/
    if (blocking)
    {
        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            return;
        }
    }
    /* Clear line */
    LCD_fullRegion(handle,object->pColorInfo->blackColor,ucXFrom,ucXTo,ucY,ucY);
    
    /* Finished with buffer - post on semaphore*/
    if (blocking)
    {
        Semaphore_post(Semaphore_handle(pSem));
    }
}
//******************************************************************************
// fn : LCD_doBufferSetVLine
//
// brief : This function draws a vertical line into the specified buffer.If 
//         blocking is set to true, the task execution will be blocked until all
//         buffer modifications have finished.
// 
// param :  handle   -> the lcd handle
//          ucX      -> the  x point
//          ucYFrom  -> the start y point
//          ucYTo    -> the end  y point
//          blocking -> block or not
//
// return : none
static void LCD_doBufferSetVLine(LCD_Handle handle,  unsigned char ucX,
                                 unsigned char ucYFrom, unsigned char ucYTo, bool blocking)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    Semaphore_Struct *pSem = &(object->lcdMutex);

    /* Pend on the semaphore if blocking option is set*/
    if (blocking)
    {
        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            return;
        }
    }
    /* Draw line */
    LCD_fullRegion(handle,object->pColorInfo->frontColor,ucX,ucX,ucYFrom,ucYTo);
    

    if (blocking)
    {
        Semaphore_post(Semaphore_handle(pSem));
    }
}
//******************************************************************************
// fn : LCD_doBufferClearVLine
//
// brief : This function sets a pixel in the specified buffer.
//
// param : handle -> the lcd handle
//         color  -> true = fill with frontColor
//                   false = fill with blackColor
//         ucX    -> the x position(0-127)
//         ucY    -> the y position(0-127)
//
// return : none
static void LCD_doBufferClearVLine(LCD_Handle handle,  unsigned char ucX,
                                   unsigned char ucYFrom, unsigned char ucYTo, bool blocking)
{

    /* Get pointer to the LCD object */
    LCD_Object *object = handle->object;

    Semaphore_Struct *pSem = &(object->lcdMutex);

    /* Pend on the semaphore if blocking option is set*/
    if (blocking)
    {
        if (!Semaphore_pend(Semaphore_handle(pSem), object->lcdWriteTimeout))
        {
            /* Semaphore timed out, log and return. */
            return;
        }
    }

    /* Clear line from buffer */
    LCD_fullRegion(handle,object->pColorInfo->blackColor,ucX,ucX,ucYFrom,ucYTo);

    /* Finished with buffer - post on semaphore*/
    if (blocking)
    {
        Semaphore_post(Semaphore_handle(pSem));
    }
}

//******************************************************************************
// fn : LCD_doBufferSetPx
//
// brief : This function sets a pixel in the specified buffer.
//
// param : handle -> the lcd handle
//         color  -> true = fill with frontColor
//                   false = fill with blackColor
//         ucX    -> the x position(0-127)
//         ucY    -> the y position(0-127)
//
// return : none
static void LCD_doBufferSetPx(LCD_Handle handle,bool color, unsigned char ucX,
                              unsigned char ucY)
{
  unsigned short rgbData;

  /* Get pointer to the LCD object */
  LCD_Object *object = handle->object;
  
  if(color)
  {
    rgbData = object->pColorInfo->frontColor;
  }
  else
  {
    rgbData = object->pColorInfo->blackColor;
  }

  /* Draw pixel */
  LCD_setRegion(handle,ucX,ucY,ucX+1,ucY+1);
  
  LCD_sendData(handle,(char const*)rgbData,2);
   
}
