#include <ti/drivers/pin/PINCC26XX.h>
#include "string.h"
#include "board.h"
#include <ti/mw/display/Display.h>

#include "hw_uart.h"
#include "hw_i2c.h"
#include "hw_gpio.h"

#include "hw_charge.h"
//#include "pincc26xx.h"


extern Display_Handle dispHandle;

PIN_Config chrgPinsCfg[] =
{
    Board_CHRG_STATE          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,

    PIN_TERMINATE
};

PIN_State  chrgPins;
PIN_Handle hChrgPins;

static uint8_t chrg_cnt = 0;
static void Board_chrgCallback(PIN_Handle hPin, PIN_Id pinId)
{
#ifdef EVAL_BOARD_1106
        HwGPIOSet(Board_RLED,chrg_cnt);
    HwGPIOSet(Board_GLED,chrg_cnt);
#else
        HwGPIOSet(Board_MOTOR,chrg_cnt);
#endif

    chrg_cnt = 1 - chrg_cnt;

}
void Board_initChrg()
{  
  // Initialize KEY pins. Enable int after callback registered
  hChrgPins = PIN_open(&chrgPins, chrgPinsCfg);
  PIN_registerIntCb(hChrgPins, Board_chrgCallback);

  PIN_setConfig(hChrgPins, PIN_BM_IRQ, Board_CHRG_STATE        | PIN_IRQ_NEGEDGE);
 

#ifdef POWER_SAVING
  //Enable wakeup
  PIN_setConfig(hChrgPins, PINCC26XX_BM_WAKEUP, Board_CHRG_STATE | PINCC26XX_WAKEUP_NEGEDGE);
# endif //POWER_SAVING
}

