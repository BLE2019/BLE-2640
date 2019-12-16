
#include "TFT_touch.h"
#include "hw_spi.h"
#include "LCDFONT.h"
#include "hw_gpio.h"
#include "GT20L16S1Y.h"
struct tp_pix_ tp_pixad, tp_pixlcd; //��ǰ���������ADֵ��ǰ�������������ֵ

//����LCD�ĳߴ�
#define LCD_W 240
#define LCD_H 240

//����LCD����������ɫ
uint16_t BACK_COLOR = GREEN; //����ɫ
uint16_t POINT_COLOR = RED; //����ɫ

/*
TFT_SPI_RESET��IOID_0�� RES
Board_LCD_MODE��IOID_1��     DC
Board_LCD_CSN��IOID_11��    CS1
*/
PIN_Handle CS_GPIOHandle;
PIN_State CS_GPIOState;
PIN_Config CS_GPIOTable[] = {
    Board_SPI_FLASH_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, //CS2  //1106

    //Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS1
    PIN_TERMINATE
};


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
    Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, //CS2

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
    while (times > 0) {
        times--;
    }
}


//��λRESET
static inline void TFT_SetRes()
{
    HwGPIOSet(Board_3V3_EN, 1);
}


//���RESET
static inline void TFT_ClearRes()
{
    HwGPIOSet(Board_3V3_EN, 0);
}


//д8λ����
void LCD_WR_DATA8(uint8_t TFT_txbuff)
{
    uint8_t TFT_rxbuf_data[1], TFT_txbuf_data[1];
    TFT_txbuf_data[0] = TFT_txbuff;
    HwGPIOSet(Board_LCD_MODE, 1);                   //д����
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data, 1);
}


//д�Ĵ���
void LCD_WR_REG(uint8_t TFT_txbuff1)
{
    uint8_t TFT_rxbuf_reg[1], TFT_txbuf_reg[1];
    TFT_txbuf_reg[0] = TFT_txbuff1;
    HwGPIOSet(Board_LCD_MODE, 0);                   //д�Ĵ���
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_reg, TFT_rxbuf_reg, 1);
}


//д16λ����
void LCD_WR_DATA16(uint16_t TFT_txbuff2)
{
    uint8_t TFT_rxbuf_data[2], TFT_txbuf_data[2];
    TFT_txbuf_data[0] = TFT_txbuff2 >> 8;
    TFT_txbuf_data[1] = TFT_txbuff2;
    HwGPIOSet(Board_LCD_MODE, 1);                   //д16bit����
    HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data, 2);
}


//TFT��ʼ��
void TFT_Init_GX(void)
{
    //HwGPIOInit();//�˿ڳ�ʼ��
    TFT_ClearRes();
    TFT_delay(200);
    TFT_SetRes();
    TFT_delay(1000);
    //HwGPIOSet(Board_GLED,0);//����ر�
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
    TFT_Clear(BACK_COLOR);                          //��ɫ����
}


//��������
void Address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WR_REG(0x2a);                               //�е�ַ����
    LCD_WR_DATA16(x1 + 2);
    LCD_WR_DATA16(x2 + 2);
    LCD_WR_REG(0x2b);                               //�е�ַ����
    LCD_WR_DATA16(y1 + 1);
    LCD_WR_DATA16(y2 + 1);
    LCD_WR_REG(0x2c);                               //������д
}


//��������
//Color:Ҫ���������ɫ
void TFT_Clear(uint16_t Color)
{
    uint16_t i, j;
    Address_set(0, 0, LCD_W - 1, LCD_H - 1);

    for (i = 0; i < LCD_W; i++) {
        for (j = 0; j < LCD_H; j++) {
            LCD_WR_DATA16(Color);
        }
    }
}


//ָ��λ�������ɫ
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    Address_set(xsta, ysta, xend, yend);            //���ù��λ��

    for (i = ysta; i <= yend; i++) {
        for (j = xsta; j <= xend; j++) {
            LCD_WR_DATA16(color);                   //���ù��λ��
        }
    }
}


//д�ַ�
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t mode)
{
    uint8_t temp;
    uint8_t pos, t;
    uint16_t colortemp = POINT_COLOR;

    if (x > LCD_W - 16 || y > LCD_H - 16) {
        return;
    }

    //���ô���
    num = num - ' ';                                //�õ�ƫ�ƺ��ֵ
    Address_set(x, y, x + 8 - 1, y + 16 - 1);       //���ù��λ��

    //�ǵ��ӷ�ʽ
    if (!mode) {
        for (pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];                        //����1608����

            for (t = 0; t < 8; t++) {
                if (temp & 0x01) {
                    POINT_COLOR = colortemp;
                } else {
                    POINT_COLOR = BACK_COLOR;
                }

                LCD_WR_DATA16(POINT_COLOR);
                temp >>= 1;
            }
        }
    } else {
        for (pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];                        //����1608����

            for (t = 0; t < 8; t++) {
                if (temp & 0x01) {
                    LCD_DrawPoint(x + t, y + pos);    //��һ����
                }

                temp >>= 1;
            }
        }
    }

    POINT_COLOR = colortemp;
}


//����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(uint16_t x, uint16_t y)
{
    Address_set(x, y, x, y);                        //���ù��λ��
    LCD_WR_DATA16(POINT_COLOR);
}


//��ʾ�ַ���
//x,y:�������
//*p:�ַ�����ʼ��ַ
//��16����
void LCD_ShowString(uint16_t x, uint16_t y, uint8_t *p)
{
    while (*p != '\0') {
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
}


void LCD_ShowLineDot(uint8_t data)
{
    uint8_t i;
    uint16_t colortemp = POINT_COLOR;

    for (i = 0; i < 8; i++) {
        if (data & 0x01) {
            colortemp = POINT_COLOR;
        } else {
            colortemp = BACK_COLOR;
        }

        LCD_WR_DATA16(colortemp);
        data >>= 1;
    }
}


void LCD_ShowGB2312(uint16_t x, uint16_t y, uint8_t th, uint8_t tl)
{
    uint8_t i, j;
    uint8_t *temp;
    uint8_t r_buffer[32];
    uint8_t x_buffer[32];
    uint8_t tmp[2];
    tmp[0] = th;
    tmp[1] = tl;
    get_hanziaddr(tmp, x_buffer);

    for (i = 0; i < 16; i++) {
        r_buffer[2 * i] = GB20L16S1Y[15 - i];
        r_buffer[2 * i + 1] = GB20L16S1Y[31 - i];
    }

    Address_set(x, y, x + 15, y + 15);              //��������
    temp = r_buffer;

    for (j = 0; j < 16; j++) {
        LCD_ShowLineDot(*temp);
        temp++;
        LCD_ShowLineDot(*temp);
        temp++;
    }
}


//��ָ��λ������һ������(16*16��С)
void showhanzi16(uint16_t x, uint16_t y, uint8_t index)
{
    uint8_t i, j;
    uint8_t *temp;
    uint8_t ziku[32];
    Address_set(x, y, x + 15, y + 15);              //��������

    for (i = 0; i < 16; i++) {
        ziku[2 * i] = GB20L16S1Y[15 - i];
        ziku[2 * i + 1] = GB20L16S1Y[31 - i];
    }

    temp = ziku;

    for (j = 0; j < 32; j++) {
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


void LCD_ShowMixString(uint16_t x, uint16_t y, uint8_t *text)
{
    uint8_t i = 0;

    while ((text[i] > 0x00)) {
        if (((text[i] >= 0xb0) && (text[i] <= 0xf7)) && (text[i + 1] >= 0xa1)) {
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
}


/********************************************************************************
����Ϊ����������
Board_SPI_FLASH_CSN Ϊcs2
********************************************************************************/
//��spi//Board_RLEDΪ������Ƭѡ����
void touch_WR_DATA8(uint8_t touch_txbuff)
{
    uint8_t touch_rxbuf_data[1], touch_txbuf_data[1];
    touch_rxbuf_data[0] = touch_txbuff;
    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf_data, touch_rxbuf_data, 1);
}


uint16_t touch_Read_12bit() //SPI������
{
    uint16_t Num = 0;
    uint8_t touch_rxbuf[2];
    uint8_t touch_txbuf[2] = {
        0x00, 0x00
    };
    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf, touch_rxbuf, 2);
    //Num=((touch_rxbuf[0]&0x78)>>3)*256+((touch_rxbuf[1]&0xf1)>>3)+(touch_rxbuf[0]<<5);
    Num = touch_rxbuf[0] * 256 + touch_rxbuf[1];
    return Num;
}


//��XPT2046��ADCֵ    0x90=y   0xd0=x
uint16_t ADS_Read_AD(uint8_t CMD) //��Ҫ24��ʱ������
{
    uint16_t l;
    uint8_t touch_rxbuf[3];
    uint8_t touch_txbuf[3] = {
        CMD
    };
    TFT_delay(65535);                               //��ʱһ��spiʱ��
    CS_GPIOclose();
    //CS_GPIOSet(Board_SPI_FLASH_CSN, 0);
    HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf, touch_rxbuf, 3);
    CS_GPIOInit();
    CS_GPIOSet(Board_SPI_FLASH_CSN, 1);
    //l=touch_rxbuf[1]*256+touch_rxbuf[2];
    l = ((touch_rxbuf[0] & 0x78) >> 3) * 256 + ((touch_rxbuf[1] & 0xf1) >> 3) + (touch_rxbuf[0] << 5);
    return l;
}


//��ȡһ������ֵ
//������ȡ READ_TIMES������
#define READ_TIMES 15 //��ȡ����
#define LOST_VAL 5    //����ֵ


uint16_t ADS_Read_XY(uint8_t xy)
{
    uint16_t i, j;
    uint16_t buf[READ_TIMES];
    uint16_t sum = 0;
    uint16_t temp;

    for (i = 0; i < READ_TIMES; i++) {
        //TFT_delay(65535);
        buf[i] = ADS_Read_AD(xy);
    }

    for (i = 0; i < READ_TIMES - 1; i++) { //����
        for (j = i + 1; j < READ_TIMES; j++) {
            if (buf[i] > buf[j]) { //��������
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    sum = 0;

    for (i = LOST_VAL; i < READ_TIMES - LOST_VAL; i++) {
        sum += buf[i];
    }

    temp = sum / (READ_TIMES - 2 * LOST_VAL);
    return temp;
}


//���˲��������ȡ
//��Сֵ����С��100
uint8_t Read_ADS(uint16_t *x, uint16_t *y)
{
    uint16_t xtemp, ytemp;
    xtemp = ADS_Read_XY(CMD_RDX);
    ytemp = ADS_Read_XY(CMD_RDY);

    if (xtemp < 100 || ytemp < 100) {
        return 0;    //����ʧ��
    }

    *x = xtemp;
    *y = ytemp;
    return 1; //�����ɹ�
}


//�������ζ�ȡ��Ч��ADֵ�������ε�ƫ��ܳ���ERR_RANGEֵ�������������ֵ��ȷ
#define ERR_RANGE 100 //��Χ


uint8_t Read_ADS2(uint16_t *x, uint16_t *y)
{
    uint16_t x1, y1;
    uint16_t x2, y2;
    uint8_t flag;
    flag = Read_ADS(&x1, &y1);

    if (flag == 0) {
        return (0);
    }

    flag = Read_ADS(&x2, &y2);

    if (flag == 0) {
        return (0);
    }

    if (((x2 <= x1 && x1 < x2 + ERR_RANGE) || (x1 <= x2 && x2 < x1 + ERR_RANGE)) //ǰ���β�����+-ERR_RANGE
            && ((y2 <= y1 && y1 < y2 + ERR_RANGE) || (y1 <= y2 && y2 < y1 + ERR_RANGE))) {
        *x = (x1 + x2) >> 1;
        *y = (y1 + y2) >> 1;
        return 1;
    } else {
        return 0;
    }
}


//��ȷ��ȡһ�����꣬У׼��ʱ����
uint8_t Read_TP_Once(void)
{
    uint8_t re = 0;
    uint16_t x1, y1;

    while (re == 0) {
        while (!Read_ADS2(&tp_pixad.x, &tp_pixad.y));

        TFT_delay(1000);

        while (!Read_ADS2(&x1, &y1));

        if (tp_pixad.x == x1 && tp_pixad.y == y1) {
            re = 1;
        }
    }

    return re;
}

