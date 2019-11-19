#include "TFT_touch.h"
#include "hw_spi.h"
#include "LCDFONT.h"
#include "hw_gpio.h"
#include "GT20L16S1Y.h"
#include "simple_observer.h"

struct tp_pix_  tp_pixad, tp_pixlcd; //当前触控坐标的AD值，前触控坐标的像素值
//定义LCD的尺寸
#define LCD_W 300
#define LCD_H 300
//定义LCD背景字体颜色
uint16_t BACK_COLOR = BLUE; //背景色
uint16_t POINT_COLOR = WHITE; //字体色
/*
TFT_SPI_RESET（IOID_0）   RES
Board_LCD_MODE（IOID_1）   DC
Board_LCD_CSN（IOID_11）  CS1
*/
PIN_Handle CS_GPIOHandle;
PIN_State CS_GPIOState;
PIN_Config CS_GPIOTable[] = {

    Board_SPI_FLASH_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS2  //1106
    //Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS1
    PIN_TERMINATE
};

#define GET_BIT(src, n) ((src) & (0x01 << (n)))
#define SET_BIT(dst, n) ((dst) |= (0x01 << (n)))



void CS_GPIOInit(void)
{
    CS_GPIOHandle = PIN_open(&CS_GPIOState, CS_GPIOTable);
}
void CS_GPIOSet(PIN_Id pin, uint8_t flag)
{
    PIN_setOutputValue(CS_GPIOHandle, pin, flag);
}
void CS_GPIOclose(void)
{
    PIN_close(CS_GPIOHandle);
}

PIN_Handle CS1_GPIOHandle;
PIN_State CS1_GPIOState;
PIN_Config CS1_GPIOTable[] = {

    Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS2
    //Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS1
    PIN_TERMINATE
};
void CS1_GPIOInit(void)
{
    CS1_GPIOHandle = PIN_open(&CS1_GPIOState, CS1_GPIOTable);
}
void CS1_GPIOSet(PIN_Id pin, uint8_t flag)
{
    PIN_setOutputValue(CS1_GPIOHandle, pin, flag);
}
void CS1_GPIOclose(void)
{
    PIN_close(CS1_GPIOHandle);
}

void TFT_delay(uint32_t times)
{
    while(times > 0) {
        times--;
    }
}
//置位RESET
static inline void TFT_SetRes()
{
    HwGPIOSet(Board_3V3_EN, 1);
}
//清除RESET
static inline void TFT_ClearRes()
{
    HwGPIOSet(Board_3V3_EN, 0);
}
//写8位数据
void LCD_WR_DATA8(uint8_t TFT_txbuff)
{
    uint8_t TFT_rxbuf_data[1], TFT_txbuf_data[1];
    TFT_txbuf_data[0] = TFT_txbuff;
    HwGPIOSet(Board_LCD_MODE, 1); //写数据
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data, 1);
}
//写寄存器
void LCD_WR_REG(uint8_t TFT_txbuff1)
{
    uint8_t TFT_rxbuf_reg[1], TFT_txbuf_reg[1];
    TFT_txbuf_reg[0] = TFT_txbuff1;
    HwGPIOSet(Board_LCD_MODE, 0); //写寄存器
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_reg, TFT_rxbuf_reg, 1);
}
//写16位数据
void LCD_WR_DATA16(uint16_t TFT_txbuff2)
{
    uint8_t TFT_rxbuf_data[2], TFT_txbuf_data[2];
    TFT_txbuf_data[0] = TFT_txbuff2 >> 8;
    TFT_txbuf_data[1] = TFT_txbuff2;
    HwGPIOSet(Board_LCD_MODE, 1); //写16bit数据
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data, 2);
}
//TFT初始化
void TFT_Init_GX(void)
{
    //HwGPIOSet(Board_3V3_EN, 0);
    //TFT_delay(20);
    HwGPIOSet(Board_3V3_EN, 1);
    TFT_delay(20);

    //HwGPIOInit();//端口初始化
    //TFT_ClearRes();
    //TFT_delay(200);
    //TFT_SetRes();
    //TFT_delay(1000);
    //HwGPIOSet(Board_GLED,0);//背光关闭
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0xb9);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0x11);
    TFT_delay(120);
    LCD_WR_REG(0xb9);
    LCD_WR_DATA8(0x00);
    //--------------------------------Display and color format setting----------------------------//
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0x00);  //from 00 to A0, 1112
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);
    LCD_WR_REG(0x21);
    LCD_WR_REG(0x2A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xEF);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xCB);
    //--------------------------------ST7789H2S Frame rate setting----------------------------------//
    LCD_WR_REG(0xb2);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);
    LCD_WR_REG(0xb7);
    LCD_WR_DATA8(0x35);
    //---------------------------------ST7789H2S Power setting--------------------------------------//
    LCD_WR_REG(0xbb);
    LCD_WR_DATA8(0x15);
    LCD_WR_REG(0xc0);
    LCD_WR_DATA8(0x2c);
    LCD_WR_REG(0xc2);
    LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xc3);
    LCD_WR_DATA8(0x12);
    LCD_WR_REG(0xc4);
    LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xc6);
    LCD_WR_DATA8(0xe1);
    LCD_WR_REG(0xd0);
    LCD_WR_DATA8(0xa4);
    LCD_WR_DATA8(0xa1);
    //gama
    LCD_WR_REG(0xe0);
    LCD_WR_DATA8(0xd0);
    LCD_WR_DATA8(0xc6);
    LCD_WR_DATA8(0x0b);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x2f);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x2d);
    LCD_WR_REG(0xe1);
    LCD_WR_DATA8(0xd0);
    LCD_WR_DATA8(0xc6);
    LCD_WR_DATA8(0x0b);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x2e);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x2d);
    LCD_WR_REG(0x29);
    TFT_Clear(GRED);                          //白色背景
}
#if 0
void TFT_Init_GX_zy(void)
{
    //HwGPIOInit();//端口初始化
    TFT_ClearRes();
    TFT_delay(1000);
    TFT_SetRes();
    TFT_delay(1000);
    //HwGPIOSet(Board_GLED,0);//背光关闭
    ;
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0xb9);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0x11);
    TFT_delay(120);
    LCD_WR_REG(0xb9);
    LCD_WR_DATA8(0x00);

    //--------------------------------Display and color format setting----------------------------//
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0x21);

    LCD_WR_REG(0x2A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xEF);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xCB);

    //--------------------------------ST7789H2S Frame rate setting----------------------------------//
    LCD_WR_REG(0xb2);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);
    LCD_WR_REG(0xb7);
    LCD_WR_DATA8(0x35);

    //---------------------------------ST7789H2S Power setting--------------------------------------//
    LCD_WR_REG(0xbb);
    LCD_WR_DATA8(0x15);
    LCD_WR_REG(0xc0);
    LCD_WR_DATA8(0x2c);

    LCD_WR_REG(0xc2);
    LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xc3);
    LCD_WR_DATA8(0x12);
    LCD_WR_REG(0xc4);
    LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xc6);
    LCD_WR_DATA8(0xe1);
    LCD_WR_REG(0xd0);
    LCD_WR_DATA8(0xa4);
    LCD_WR_DATA8(0xa1);

    //gama
    LCD_WR_REG(0xe0);
    LCD_WR_DATA8(0xd0);
    LCD_WR_DATA8(0xc6);
    LCD_WR_DATA8(0x0b);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x2f);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x2d);

    LCD_WR_REG(0xe1);
    LCD_WR_DATA8(0xd0);
    LCD_WR_DATA8(0xc6);
    LCD_WR_DATA8(0x0b);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x2e);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x2d);

    LCD_WR_REG(0x29);



#if 0
    //TM+9341
    WriteComm(0xCF);
    WriteData(0x00);
    WriteData(0x81);
    WriteData(0X30);

    WriteComm(0xED);
    WriteData(0x64);
    WriteData(0x03);
    WriteData(0X12);
    WriteData(0X81);

    WriteComm(0xE8);
    WriteData(0x85);
    WriteData(0x10);
    WriteData(0x7A);

    WriteComm(0xCB);
    WriteData(0x39);
    WriteData(0x2C);
    WriteData(0x00);
    WriteData(0x34);
    WriteData(0x02);

    WriteComm(0xF7);
    WriteData(0x20);
    WriteComm(0xEA);
    WriteData(0x00);
    WriteData(0x00);

    WriteComm(0xC0);        //Power control
    WriteData(0x21);      //VRH[5:0]

    WriteComm(0xC1);        //Power control
    WriteData(0x11);      //SAP[2:0];BT[3:0]

    WriteComm(0xC5);        //VCM control
    WriteData(0x51);
    WriteData(0x38);

    WriteComm(0xC7);        //VCM control2
    WriteData(0X9F);

    WriteComm(0x36);        // Memory Access Control
    WriteData(0x00);

    WriteComm(0x3A);
    WriteData(0x55);

    WriteComm(0xB1);
    WriteData(0x00);
    WriteData(0x18);

    WriteComm(0xB6);        // Display Function Control
    WriteData(0x0A);
    WriteData(0xA2);

    WriteComm(0xF2);        // 3Gamma Function Disable
    WriteData(0x00);

    WriteComm(0x26);        //Gamma curve selected
    WriteData(0x01);

    WriteComm(0xE0);        //Set Gamma
    WriteData(0x0F);
    WriteData(0x23);
    WriteData(0x1F);
    WriteData(0x0B);
    WriteData(0x0E);
    WriteData(0x08);
    WriteData(0x4B);
    WriteData(0XA8);
    WriteData(0x3B);
    WriteData(0x0A);
    WriteData(0x14);
    WriteData(0x06);
    WriteData(0x10);
    WriteData(0x09);
    WriteData(0x00);

    WriteComm(0XE1);        //Set Gamma
    WriteData(0x00);
    WriteData(0x1C);
    WriteData(0x20);
    WriteData(0x04);
    WriteData(0x10);
    WriteData(0x08);
    WriteData(0x34);
    WriteData(0x47);
    WriteData(0x44);
    WriteData(0x05);
    WriteData(0x0B);
    WriteData(0x09);
    WriteData(0x2F);
    WriteData(0x36);
    WriteData(0x0F);

    WriteComm(0x21);

    WriteComm(0x11);        //Exit Sleep
    Delay(120);
    WriteComm(0x29);        //Display on
    WriteComm(0x2C);        //Display on
#endif

#if 0
    //---------------------------------------------------------------------------------------------------//
    //Delay 120ms

    WriteComm(0x11);

    Delay(120);                //Delay 120ms

    WriteComm(0x36);
    WriteData(0x00);

    WriteComm(0x3A);
    WriteData(0x05);

    WriteComm(0xB2);
    WriteData(0x0C);
    WriteData(0x0C);
    WriteData(0x00);
    WriteData(0x33);
    WriteData(0x33);

    WriteComm(0xB7);
    WriteData(0x35);

    WriteComm(0xBB);
    WriteData(0x1A);

    WriteComm(0xC0);
    WriteData(0x2C);

    WriteComm(0xC2);
    WriteData(0x01);

    WriteComm(0xC3);
    WriteData(0x0B);

    WriteComm(0xC4);
    WriteData(0x20);

    WriteComm(0xC6);
    WriteData(0x0F);

    WriteComm(0xD0);
    WriteData(0xA4);
    WriteData(0xA1);

    WriteComm(0x21);

    WriteComm(0xE0);
    WriteData(0x00);
    WriteData(0x19);
    WriteData(0x1E);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x15);
    WriteData(0x3D);
    WriteData(0x44);
    WriteData(0x51);
    WriteData(0x12);
    WriteData(0x03);
    WriteData(0x00);
    WriteData(0x3F);
    WriteData(0x3F);

    WriteComm(0xE1);
    WriteData(0x00);
    WriteData(0x18);
    WriteData(0x1E);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x25);
    WriteData(0x3F);
    WriteData(0x43);
    WriteData(0x52);
    WriteData(0x33);
    WriteData(0x03);
    WriteData(0x00);
    WriteData(0x3F);
    WriteData(0x3F);

    WriteComm(0x29);

#endif
#if 1
#endif

#if 0 //old version of C51 BY GUANXIAN
    WriteComm(0xb9);
    WriteData(0x00);
    WriteComm(0x11);
    Delay(120);
    WriteComm(0xb9);
    WriteData(0x00);
    //Delay 120ms
    //--------------------------------Display and color format setting----------------------------//

    WriteComm(0x36);
    WriteData(0x00);

    WriteComm(0x3a);
    WriteData(0x05);


    WriteComm(0x21);

    WriteComm (0x2a);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0xef);
    WriteComm (0x2b);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0xcb);

    //--------------------------------ST7789H2S Frame rate setting----------------------------------//


    WriteComm(0xb2);
    WriteData(0x0c);
    WriteData(0x0c);
    WriteData(0x00);
    WriteData(0x33);
    WriteData(0x33);

    WriteComm(0xb7);
    WriteData(0x35); //35 35

    //---------------------------------ST7789H2S Power setting--------------------------------------//

    WriteComm(0xbb);
    WriteData(0x15); // 2c  16  调整VCOM flikcer
    WriteComm(0xc0);
    WriteData(0x2c);

    WriteComm(0xc2);
    WriteData(0x01);

    WriteComm(0xc3);
    WriteData(0x12); //调整对比度 调小对比度大03

    WriteComm(0xc4);
    WriteData(0x20); //20

    WriteComm(0xc6);
    WriteData(0xe1);// 0xef column inversion

    WriteComm(0xd0);
    WriteData(0xa4);
    WriteData(0xa1);


    //gamma
    WriteComm(0xe0);
    WriteData(0xD0);
    WriteData(0x06);
    WriteData(0x0B);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x06);
    WriteData(0x2F);
    WriteData(0x44);
    WriteData(0x45);
    WriteData(0x18);
    WriteData(0x14);
    WriteData(0x14);
    WriteData(0x27);
    WriteData(0x2D);


    WriteComm(0xe1);
    WriteData(0xD0);
    WriteData(0x06);
    WriteData(0x0B);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x05);
    WriteData(0x2E);
    WriteData(0x43);
    WriteData(0x45);
    WriteData(0x18);
    WriteData(0x14);
    WriteData(0x14);
    WriteData(0x27);
    WriteData(0x2D);

    WriteComm(0x29);
#endif
}
#endif
void TFT_Init(void)
{
    //HwGPIOInit();//端口初始化
    TFT_ClearRes();
    TFT_delay(100);
    TFT_SetRes();
    TFT_delay(100);
    //HwGPIOSet(Board_GLED,0);//背光关闭
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x2D);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x2D);

    LCD_WR_REG(0xB3);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x2D);

    LCD_WR_REG(0xB4); //Column inversion
    LCD_WR_DATA8(0x07);

    //ST7735R Power Sequence
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0xA2);
    LCD_WR_DATA8(0x02);
    LCD_WR_DATA8(0x84);
    LCD_WR_REG(0xC1);
    LCD_WR_DATA8(0xC5);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x8A);
    LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x8A);
    LCD_WR_DATA8(0xEE);

    LCD_WR_REG(0xC5); //VCOM
    LCD_WR_DATA8(0x0E);

    LCD_WR_REG(0x36); //MX, MY, RGB mode
    LCD_WR_DATA8(0xc0);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x55);

    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x18);

    LCD_WR_REG(0xB6);    // Display Function Control
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0xA2);



    LCD_WR_REG(0xF2);    // 3Gamma Function Disable
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x26);    //Gamma curve selected
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xE0);    //Set Gamma
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x20);
    LCD_WR_DATA8(0x1E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x0B);
    LCD_WR_DATA8(0x50);
    LCD_WR_DATA8(0XBA);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x21);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0XE1);    //Set Gamma
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x19);
    LCD_WR_DATA8(0x19);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x02);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x0F);

    LCD_WR_REG(0x11);    //Exit Sleep
    TFT_delay(120);
    LCD_WR_REG(0x29);    //Display on
    TFT_Clear(WHITE);//白色背景
}
//设置坐标
void Address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    //   LCD_WR_REG(0x2a);//列地址设置
    //   LCD_WR_DATA8(0x00);
    //   LCD_WR_DATA8(x1+2);
    //   LCD_WR_DATA8(0x00);
    //   LCD_WR_DATA8(x2+2);
    //
    //   LCD_WR_REG(0x2b);//行地址设置
    //   LCD_WR_DATA8(0x00);
    //   LCD_WR_DATA8(y1+1);
    //   LCD_WR_DATA8(0x00);
    //   LCD_WR_DATA8(y2+1);
    //
    //   LCD_WR_REG(0x2C);  //储存器写
    LCD_WR_REG(0x2a);//列地址设置
    LCD_WR_DATA16(x1 + 2);
    LCD_WR_DATA16(x2 + 2);
    LCD_WR_REG(0x2b);//行地址设置
    LCD_WR_DATA16(y1 + 1);
    LCD_WR_DATA16(y2 + 1);
    LCD_WR_REG(0x2c);//储存器写


}
//清屏函数
//Color:要清屏的填充色
void TFT_Clear(uint16_t Color)
{
    uint16_t i, j;
    Address_set(0, 0, LCD_W - 1, LCD_H - 1);
    for(i = 0; i < LCD_W; i++) {
        for (j = 0; j < LCD_H; j++) {
            LCD_WR_DATA16(Color);
        }
    }
}
//指定位置填充颜色
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    Address_set(xsta, ysta, xend, yend);   //设置光标位置
    for(i = ysta; i <= yend; i++) {
        for(j = xsta; j <= xend; j++) {
            LCD_WR_DATA16(color);//设置光标位置
        }
    }
}
//写字符
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t mode)
{
    uint8_t temp;
    uint8_t pos, t;
    uint16_t colortemp = POINT_COLOR;
    if(x > LCD_W - 16 || y > LCD_H - 16) {
        return;
    }
    //设置窗口
    num = num - ' '; //得到偏移后的值
    Address_set(x, y, x + 8 - 1, y + 16 - 1); //设置光标位置
    //非叠加方式
    if(!mode) {
        for(pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];  //调用1608字体
            for(t = 0; t < 8; t++) {
                if( temp & 0x01) {
                    POINT_COLOR = colortemp;
                } else {
                    POINT_COLOR = BACK_COLOR;
                }

                LCD_WR_DATA16(POINT_COLOR);
                temp >>= 1;
            }
        }
    } else {
        for(pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];      //调用1608字体
            for(t = 0; t < 8; t++) {
                if(temp & 0x01) {
                    LCD_DrawPoint(x + t, y + pos);    //画一个点
                }
                temp >>= 1;
            }
        }
    }
    POINT_COLOR = colortemp;
}
//画点
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(uint16_t x, uint16_t y)
{
    Address_set(x, y, x, y); //设置光标位置
    LCD_WR_DATA16(POINT_COLOR);
}
//显示字符串
//x,y:起点坐标
//*p:字符串起始地址
//用16字体
void LCD_ShowString(uint16_t x, uint16_t y, uint8_t *p)
{
    while(*p != '\0') {
        if (x > LCD_W - 16) {
            x = 0;
            y += 16;
        }

        if (y > LCD_H - 16) {
            y = x = 0;
            TFT_Clear(RED);
        }

        LCD_ShowChar(x, y, *p, 0);
        x += 8;
        p++;
    }
    memset(g_lcdBuffer, 0, MAX_LCD_BUFF_LEN);
}

void LCD_ShowLineDot(uint8_t data)
{
    uint8_t i;
    uint16_t colortemp = POINT_COLOR;
    for (i = 0; i < 8; i++) {
        if( data & 0x01) {
            POINT_COLOR = colortemp;
        } else {
            POINT_COLOR = BACK_COLOR;
        }

        LCD_WR_DATA16(POINT_COLOR);
        data >>= 1;
    }
}

static void RotateHzk16Font(uint8_t *dst, uint8_t *src)
{
    int i, j;
    //第一列后8个字节
    for (i = 0; i < 8; ++i) {
        for (j = 0; j < 8; ++j) {
            if (GET_BIT(src[16 + j * 2], 7 - i)) {
                SET_BIT(dst[i * 2], j);
            }
        }
    }
    //第一列前8个字节
    for (i = 0; i < 8; ++i) {
        for (j = 0; j < 8; ++j) {
            if (GET_BIT(src[j * 2], 7 - i)) {
                SET_BIT(dst[1 + i * 2], j);
            }
        }
    }
    //第二列后8个字节
    for (i = 0; i < 8; ++i) {
        for (j = 0; j < 8; ++j) {
            if (GET_BIT(src[17 + j * 2], 7 - i)) {
                SET_BIT(dst[16 + i * 2], j);
            }
        }
    }
    //第一列前8个字节
    for (i = 0; i < 8; ++i) {
        for (j = 0; j < 8; ++j) {
            if (GET_BIT(src[1 + j * 2], 7 - i)) {
                SET_BIT(dst[17 + i * 2], j);
            }
        }
    }
}


void LCD_ShowGB2312(uint16_t x, uint16_t y, uint8_t th, uint8_t tl)
{
    uint8_t i, j;
    uint8_t *temp;
    uint8_t  r_buffer[32];
    uint8_t ziku[32];
    // uint8_t newbuff[32];

    uint8_t tmp[2];
    tmp[0] = th;
    tmp[1] = tl;

    get_hanziaddr(tmp, r_buffer);

    Address_set(x, y, x + 15, y + 15);    //设置区域

    for (i = 0; i < 16; i++) {
        ziku[2 * i] = r_buffer[15 - i];
        ziku[2 * i + 1] = r_buffer[31 - i];
    }

    temp = ziku;

    //RotateHzk16Font(newbuff, ziku);
    //temp = newbuff;

    for ( j = 0; j < 32; j++) {
        for (i = 0; i < 8; i++) {
            if ((*temp & (1 << i)) != 0) {
                LCD_WR_DATA16(POINT_COLOR);
            } else {
                LCD_WR_DATA16(BACK_COLOR);
            }
        }
        temp++;
    }
}
//在指定位置生成一个汉字(16*16大小)
void showhanzi16(uint16_t x, uint16_t y, uint8_t index)
{
    uint8_t i, j;
    uint8_t *temp;
    uint8_t ziku[32];
    Address_set(x, y, x + 7, y + 15); //设置区域
    for(i = 0; i < 16; i++) {
        ziku[2 * i] = GB20L16S1Y[15 - i];
        ziku[2 * i + 1] = GB20L16S1Y[31 - i];
    }

    temp = ziku;
    for(j = 0; j < 16; j++) {
        for(i = 0; i < 8; i++) {
            if((*temp & (1 << i)) != 0) {
                LCD_WR_DATA16(POINT_COLOR);
            } else {
                LCD_WR_DATA16(BACK_COLOR);
            }
        }
        temp++;
    }
}

void LCD_ShowMixString(uint16_t x, uint16_t y, uint8_t *text)
{
    uint8_t i = 0;
    while((text[i] > 0x00)) {
        if(((text[i] >= 0xb0) && (text[i] <= 0xf7)) && (text[i + 1] >= 0xa1)) {
            LCD_ShowGB2312(x, y, text[i], text[i + 1]);
            i += 2;
            x += 16;
        } else if ((text[i] >= 0x20) && (text[i] <= 0x7e)) {
            LCD_ShowChar(x, y, text[i], 0);
            i += 1;
            x += 8;
        } else {
            i++;
        }
    }
    //    showhanzi16(50, 50, 0);
}
#if 1
void  fontchange_new(uint8_t *input, uint8_t *newbuff)
{
    uint8_t fontbuf;
    uint8_t buff, i, step, tmp;
    for(step = 0; step < 8; step++) { //16X16 左上角部分 总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            tmp = *(input + i);
            buff = tmp >> step;     //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        *newbuff = fontbuf;         //将当前运算的一字节新数据存当前运算地址.
        newbuff++;
    }

    for(step = 0; step < 8; step++) {  //16X16 右上角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            tmp = *(input + i + 8);
            buff = tmp >> step;
            //       buff=input[i+8]>>step;       //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        //     newbuff[step+8]=fontbuf;         //将当前运算的一字节新数据存当前运算地址.
        *newbuff = fontbuf;         //将当前运算的一字节新数据存当前运算地址.
        newbuff++;

    }
    for(step = 0; step < 8; step++) { //16X16 左下角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            tmp = *(input + i + 16);
            buff = tmp >> step;  //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        *newbuff = fontbuf;         //将当前运算的一字节新数据存当前运算地址.
        newbuff++;
    }

    for(step = 0; step < 8; step++) {  //16X16 右下角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            tmp = *(input + 24 + i);
            buff = tmp >> step;  //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        *newbuff = fontbuf;         //将当前运算的一字节新数据存当前运算地址.
        newbuff++;
    }
}

void  fontchange(uint8_t *input, uint8_t *newbuff)
{
    uint8_t fontbuf;
    uint8_t buff, i, step;
    for(step = 0; step < 8; step++) { //16X16 左上角部分 总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            buff = input[i] >> step;     //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        newbuff[step] = fontbuf;         //将当前运算的一字节新数据存当前运算地址.
    }

    for(step = 0; step < 8; step++) {  //16X16 右上角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            buff = input[i + 8] >> step; //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        newbuff[step + 8] = fontbuf;     //将当前运算的一字节新数据存当前运算地址.
    }
    for(step = 0; step < 8; step++) { //16X16 左下角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            buff = input[i + 16] >> step; //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        newbuff[step + 16] = fontbuf;    //将当前运算的一字节新数据存当前运算地址.
    }
    for(step = 0; step < 8; step++) {  //16X16 右下角部分总循环8步
        fontbuf = 0;                      //初始化 字节运算缓存器等于0
        buff = 0;                         //字节缓冲器初始化.
        for(i = 0; i < 8; i++) {         //第一层循环，取竖置横排8字节第1个点。
            fontbuf <<= 1;                  //数据自右向左移动,共移动7位,合并一字节
            buff = input[i + 24] >> step; //将当前转换的列地址fontbuff[i]右移step步.
            buff &= 0x01;                   //保留最低位置，
            fontbuf |= buff;                //字节缓存器等于新的运算字节和最低位相与
        }
        newbuff[step + 24] = fontbuf;    //将当前运算的一字节新数据存当前运算地址.
    }
}
#endif
/********************************************************************************
下面为触摸屏函数
Board_SPI_FLASH_CSN 为cs2
********************************************************************************/
//读spi//Board_RLED为触摸屏片选引脚
void touch_WR_DATA8(uint8_t touch_txbuff)
{
    uint8_t touch_rxbuf_data[1], touch_txbuf_data[1];
    touch_rxbuf_data[0] = touch_txbuff;
    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf_data, touch_rxbuf_data, 1);

}
uint16_t touch_Read_12bit()             //SPI读数据
{
    uint16_t Num = 0;
    uint8_t touch_rxbuf[2];
    uint8_t touch_txbuf[2] = {0x00, 0x00};

    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf, touch_rxbuf, 2);
    //Num=((touch_rxbuf[0]&0x78)>>3)*256+((touch_rxbuf[1]&0xf1)>>3)+(touch_rxbuf[0]<<5);
    Num = touch_rxbuf[0] * 256 + touch_rxbuf[1];
    return Num;
}
//从XPT2046读ADC值    0x90=y   0xd0=x
uint16_t ADS_Read_AD(uint8_t CMD)      //需要24个时钟周期
{
    uint16_t l;
    uint8_t touch_rxbuf[3];
    uint8_t touch_txbuf[3] = {CMD};

    TFT_delay(65535);//延时一个spi时钟
    CS_GPIOclose();
    //CS_GPIOSet(Board_SPI_FLASH_CSN, 0);
    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf, touch_rxbuf, 3);
    CS_GPIOInit();
    CS_GPIOSet(Board_SPI_FLASH_CSN, 1);
    //l=touch_rxbuf[1]*256+touch_rxbuf[2];
    l = ((touch_rxbuf[0] & 0x78) >> 3) * 256 + ((touch_rxbuf[1] & 0xf1) >> 3) + (touch_rxbuf[0] << 5);
    return l;
}
//读取一个坐标值
//连续读取 READ_TIMES次数据
#define READ_TIMES 15 //读取次数
#define LOST_VAL 5    //丢弃值
uint16_t ADS_Read_XY(uint8_t xy)
{
    uint16_t i, j;
    uint16_t buf[READ_TIMES];
    uint16_t sum = 0;
    uint16_t temp;
    for(i = 0; i < READ_TIMES; i++) {
        //TFT_delay(65535);
        buf[i] = ADS_Read_AD(xy);
    }
    for(i = 0; i < READ_TIMES - 1; i++) { //排序
        for(j = i + 1; j < READ_TIMES; j++) {
            if(buf[i] > buf[j]) { //升序排列
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    sum = 0;
    for(i = LOST_VAL; i < READ_TIMES - LOST_VAL; i++) {
        sum += buf[i];
    }
    temp = sum / (READ_TIMES - 2 * LOST_VAL);
    return temp;
}
//带滤波的坐标读取
//最小值不能小于100
uint8_t Read_ADS(uint16_t *x, uint16_t *y)
{
    uint16_t xtemp, ytemp;
    xtemp = ADS_Read_XY(CMD_RDX);
    ytemp = ADS_Read_XY(CMD_RDY);
    if(xtemp < 100 || ytemp < 100) {
        return 0;    //读数失败
    }
    *x = xtemp;
    *y = ytemp;
    return 1;//读数成功
}

//连续两次读取有效的AD值，且两次的偏差不能超过ERR_RANGE值，满足条件则读值正确
#define ERR_RANGE 100 //误差范围
uint8_t Read_ADS2(uint16_t *x, uint16_t *y)
{
    uint16_t x1, y1;
    uint16_t x2, y2;
    uint8_t flag;
    flag = Read_ADS(&x1, &y1);
    if(flag == 0) {
        return(0);
    }
    flag = Read_ADS(&x2, &y2);
    if(flag == 0) {
        return(0);
    }
    if(((x2 <= x1 && x1 < x2 + ERR_RANGE) || (x1 <= x2 && x2 < x1 + ERR_RANGE)) //前两次采样在+-ERR_RANGE
       && ((y2 <= y1 && y1 < y2 + ERR_RANGE) || (y1 <= y2 && y2 < y1 + ERR_RANGE))) {
        *x = (x1 + x2) >> 1;
        *y = (y1 + y2) >> 1;
        return 1;
    } else {
        return 0;
    }
}
//精确读取一次坐标，校准的时候用
uint8_t Read_TP_Once(void)
{
    uint8_t re = 0;
    uint16_t x1, y1;
    while(re == 0) {
        while(!Read_ADS2(&tp_pixad.x, &tp_pixad.y));
        TFT_delay(1000);
        while(!Read_ADS2(&x1, &y1));
        if(tp_pixad.x == x1 && tp_pixad.y == y1) {
            re = 1;
        }
    }
    return re;
}
