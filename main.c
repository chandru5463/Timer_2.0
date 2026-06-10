/*===========================================================================
 * main.c â€” LED Timer Control + TM1637 3-Digit 7-Segment Display
 * Target : MS51 @ 16 MHz (HIRC)
 *
 * Keys 1-9  â†’  5 s â€“ 45 s relay ON time  (key Ã— 5 seconds)
 *
 * Display state machine
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  BOOT      :  " On"  shown for 2 s  (TM1637 splash)
 *  IDLE      :  "000"  shown until a valid key is pressed
 *  COUNTDOWN :  remaining seconds (ceil, 3 digits, e.g. "045" â†’ "001")
 *  DONE      :  "OFF"  shown until the next valid key press
 *
 * Pin map
 * â”€â”€â”€â”€â”€â”€â”€
 *  RELAY/LED  P0.5
 *  ROW1-4     P0.4, P0.3, P0.1, P0.0   (push-pull output)
 *  COL1-3     P1.0, P1.1, P1.2          (quasi-bidir, ext. pull-down)
 *  TM_CLK     P1.3                       (quasi-bidir, ext. 10 kÎ© pull-up)
 *  TM_DIO     P1.4                       (quasi-bidir, ext. 10 kÎ© pull-up)
 *===========================================================================*/

#include "numicro_8051.h"
#include "timer.h"

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  PIN DEFINITIONS
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */

#define NO_RELAY    P15                /* Output: Normally Open relay         */
#define POWER_ON    P30                /* Output: Power On LED                */
#define SET_TIMER   P06                /*Blink twice to set timer             */

/* Direct Buttons Mappings */
#define BTN1_PIN    P01                /* 5 min                               */
#define BTN2_PIN    P03                /* 10 min                              */
#define BTN3_PIN    P04                /* 15 min                              */
#define BTN4_PIN    P00                /* 20 min                              */
#define BTN5_PIN    P10                /* 25 min                              */
#define BTN6_PIN    P11                /* 30 min                              */
#define BTN7_PIN    P12                /* 40 min                              */
#define BTN8_PIN    P16                /* 60 min                              */
#define BTN9_PIN    P17                /* Emergency Stop / OFF                */

/* Button Configuration
 * Set BUTTON_ACTIVE_STATE to:
 *   0 : Active-Low (button connects pin to GND when pressed, e.g. using internal pull-up)
 *   1 : Active-High (button connects pin to VCC when pressed, e.g. using external pull-down)
 */
#define BUTTON_ACTIVE_STATE  0

#define READ_BTN(pin)        (BUTTON_ACTIVE_STATE ? ((pin) == 1) : ((pin) == 0))

/* TM1637 two-wire bus (external 10 kÎ© pull-ups to 3.3 V / 5 V on both)     */
#define TM_CLK      P13                /* Serial clock                        */
#define TM_DIO      P14                /* Serial data I/O                     */

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  TIMER 1 â€” 1 ms countdown  (ISR at vector 0x1B, interrupt 3)
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */

volatile uint16_t g_sec_countdown = 0;   /* decremented each 1 s by ISR      */
uint16_t g_prev_button_state = 0x0000;  /* last active-state tracking of BTN1-9 */
static uint16_t ms_ticks = 0;

void Timer1_ISR(void) __interrupt(3)
{
    uint8_t sfrs_tmp = SFRS;
    SFRS = 0;

    TH1 = TH1TMP;                      /* reload for next 1 ms period        */
    TL1 = TL1TMP;
    clr_TCON_TF1;

    ms_ticks++;
    if (ms_ticks >= 1000U)
    {
        ms_ticks = 0;
        if (g_sec_countdown > 0)
            g_sec_countdown--;
    }

    if (sfrs_tmp) { ENABLE_SFR_PAGE1; }
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  SOFTWARE DELAY â€” keypad debounce and splash screen timing only
 *  (~267 inner iterations â‰ˆ 1 ms at 16 MHz; does NOT affect LED accuracy)
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */

static void Delay_ms_soft(uint16_t ms)
{
    volatile uint16_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 267; j++);
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  TM1637 DRIVER
 *
 *  Protocol summary (from Titan Micro TM1637 Datasheet V2.4)
 *  â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  â€¢ START  : DIO HIGH â†’ LOW while CLK is HIGH
 *  â€¢ STOP   : DIO LOW  â†’ HIGH while CLK is HIGH
 *  â€¢ Data   : changed on CLK LOW, sampled on CLK rising edge, LSB first
 *  â€¢ ACK    : chip pulls DIO LOW on 8th CLK falling edge; released on 9th CLK
 *  â€¢ Max CLK: 500 kHz  (datasheet Â§4, Fmax)
 *
 *  Write flow (auto-increment mode)
 *  â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *   START â†’ 0x40 â†’ STOP          (Data Command: write, auto-increment)
 *   START â†’ 0xC0 â†’ d0 â†’ d1 â†’ d2 â†’ STOP  (Address + 3 bytes of segment data)
 *   START â†’ 0x8x â†’ STOP          (Display Control: ON + brightness)
 *
 *  GPIO mode: quasi-bidirectional on both CLK and DIO.
 *   â€¢ Quasi-bidir acts as open-drain with weak pull (external 10 kÎ© handles it).
 *   â€¢ Writing 1 â†’ releases pin (external pull-up pulls HIGH).
 *   â€¢ Writing 0 â†’ drives pin LOW.
 *   â€¢ Readable when set to 1 (needed for ACK on DIO).
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */

/* â”€â”€ Segment encoding (common anode, SEG-A in bit 0) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *
 *    bit 0 = a  (top)
 *    bit 1 = b  (upper-right)
 *    bit 2 = c  (lower-right)
 *    bit 3 = d  (bottom)
 *    bit 4 = e  (lower-left)
 *    bit 5 = f  (upper-left)
 *    bit 6 = g  (middle)
 *    bit 7 = dp (decimal point â€” not connected on most 3-digit modules)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

static const uint8_t SEG_TABLE[10] =
{
    0x3F,  /* 0 :  a b c d e f       */
    0x06,  /* 1 :    b c             */
    0x5B,  /* 2 :  a b   d e   g     */
    0x4F,  /* 3 :  a b c d     g     */
    0x66,  /* 4 :    b c     f g     */
    0x6D,  /* 5 :  a   c d   f g     */
    0x7D,  /* 6 :  a   c d e f g     */
    0x07,  /* 7 :  a b c             */
    0x7F,  /* 8 :  a b c d e f g     */
    0x6F   /* 9 :  a b c d   f g     */
};

/*  Special patterns used for "On" and "OFF" splash states                   */
#define SEG_BLANK   0x00U     /*  (all off)                                  */
#define SEG_O       0x3FU     /*  O  â€” segments a,b,c,d,e,f  (same as '0')  */
#define SEG_n       0x54U     /*  n  â€” segments c,e,g  (lower n shape)       */
#define SEG_F       0x71U     /*  F  â€” segments a,e,f,g                      */

/*  TM1637 command bytes                                                      */
#define TM_CMD_DATA_AUTO   0x40U   /* write data, auto-increment address     */
#define TM_CMD_ADDR_C0     0xC0U   /* starting address = GRID1               */
#define TM_CMD_DISP_ON     0x8BU   /* display ON, brightness = 10/16 pulse   */

/* â”€â”€ TM_Delay â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 * ~5 Âµs at 16 MHz â†’ effective CLK â‰ˆ 100 kHz (datasheet max: 500 kHz)
 * The extra margin protects against worst-case 8051 instruction timing.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€*/
static void TM_Delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 10; i++);
}

/* â”€â”€ START condition â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void TM_Start(void)
{
    TM_DIO = 1;  TM_CLK = 1;  TM_Delay();
    TM_DIO = 0;               TM_Delay();  /* DIO HIGHâ†’LOW while CLK HIGH   */
    TM_CLK = 0;               TM_Delay();
}

/* â”€â”€ STOP condition â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static void TM_Stop(void)
{
    TM_CLK = 0;  TM_DIO = 0;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_DIO = 1;               TM_Delay();  /* DIO LOWâ†’HIGH while CLK HIGH   */
}

/* â”€â”€ Write one byte LSB-first; clock through ACK on 9th pulse â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *
 *  The TM1637 pulls DIO LOW during the ACK slot (8th CLK falling edge).
 *  We release DIO = 1 on the 9th clock and do not check the ACK level â€”
 *  acceptable for display-only use; omitting the read simplifies the driver
 *  and avoids any quasi-bidir read-back glitch concerns.
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€*/
static void TM_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        TM_CLK = 0;                    TM_Delay();
        TM_DIO = (data & 0x01U) ? 1 : 0;  /* set bit while CLK low          */
                                       TM_Delay();
        TM_CLK = 1;                    TM_Delay();  /* chip latches on rising edge */
        data >>= 1;
    }

    /* 9th clock: release DIO so chip can drive ACK (we ignore its value)    */
    TM_CLK = 0;  TM_DIO = 1;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_CLK = 0;               TM_Delay();
}

/* â”€â”€ TM_ShowDigits â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  Core display routine.
 *  d0 â†’ GRID1 (leftmost), d1 â†’ GRID2 (middle), d2 â†’ GRID3 (rightmost)
 *
 *  Follows the auto-increment flow documented in the TM1637 datasheet:
 *    1. Send Data Command  (0x40 â€” write mode, auto-increment)
 *    2. Send Address 0xC0  followed by 3 segment bytes (GRID1..GRID3)
 *    3. Send Display Control command (ON + brightness)
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€*/
static void TM_ShowDigits(uint8_t d0, uint8_t d1, uint8_t d2)
{
    /* Step 1 â€” Data Command */
    TM_Start();
    TM_WriteByte(TM_CMD_DATA_AUTO);
    TM_Stop();

    /* Step 2 â€” Starting address + segment data */
    TM_Start();
    TM_WriteByte(TM_CMD_ADDR_C0);
    TM_WriteByte(d0);
    TM_WriteByte(d1);
    TM_WriteByte(d2);
    TM_Stop();

    /* Step 3 â€” Display Control: ON, brightness = 10/16 */
    TM_Start();
    TM_WriteByte(TM_CMD_DISP_ON);
    TM_Stop();
}

/* â”€â”€ High-level display helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

/*  Show remaining minutes as a 3-digit decimal (000 â€“ 999) using 8-bit math */
static void TM_ShowMinutes(uint8_t mins)
{
    TM_ShowDigits(
        SEG_TABLE[mins / 100U],
        SEG_TABLE[(mins / 10U) % 10U],
        SEG_TABLE[mins % 10U]
    );
}

/*  " On" â€” power-on splash (blank + O + n)                                  */
static void TM_ShowON(void)
{
    TM_ShowDigits(SEG_O, SEG_n,SEG_BLANK);
}

/*  "OFF" â€” timer complete, waiting for next key                             */
static void TM_ShowOFF(void)
{
    TM_ShowDigits(SEG_O, SEG_F, SEG_F);
}

/*  "000" â€” idle/ready state shown after splash                              */
static void TM_ShowZeros(void)
{
    TM_ShowDigits(SEG_TABLE[0], SEG_TABLE[0], SEG_TABLE[0]);
}

static uint16_t Read_All_Buttons(void)
{
    uint16_t state = 0;
    if (READ_BTN(BTN1_PIN)) state |= 0x0001U;
    if (READ_BTN(BTN2_PIN)) state |= 0x0002U;
    if (READ_BTN(BTN3_PIN)) state |= 0x0004U;
    if (READ_BTN(BTN4_PIN)) state |= 0x0008U;
    if (READ_BTN(BTN5_PIN)) state |= 0x0010U;
    if (READ_BTN(BTN6_PIN)) state |= 0x0020U;
    if (READ_BTN(BTN7_PIN)) state |= 0x0040U;
    if (READ_BTN(BTN8_PIN)) state |= 0x0080U;
    if (READ_BTN(BTN9_PIN)) state |= 0x0100U;
    return state;
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  DIRECT BUTTON SCANNER (Edge-Triggered, Non-blocking)
 *  Returns 1â€“9 for a confirmed transition from released to pressed.
 *  Returns 0xFF if no transition is detected.
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */

static uint8_t Scan_Buttons(void)
{
    uint16_t current_state = Read_All_Buttons();
    uint8_t pressed_btn = 0xFFU;
    uint8_t i;
    uint16_t mask = 0x0001U;

    /* Detect rising edge of active state: was released (0) -> is pressed (1) */
    for (i = 0; i < 9; i++)
    {
        if (!(g_prev_button_state & mask) && (current_state & mask))
        {
            /* Debounce: wait 10 ms and re-read */
            Delay_ms_soft(10);
            
            /* Re-read using the bit-mask to check specific pin state */
            if (Read_All_Buttons() & mask)
            {
                pressed_btn = i + 1;
                /* Lock the state as pressed so we don't repeat-trigger */
                g_prev_button_state |= mask;
                break;
            }
        }
        mask <<= 1;
    }

    /* Release tracking: clear bits in g_prev_button_state if pins are released (0 in current_state) */
    g_prev_button_state &= current_state;

    return pressed_btn;
}

static void Load_New_Timer(uint8_t key)
{
    uint16_t secs_on;
    switch(key) {
        case 1: secs_on = 5U  * 60U; TM_ShowMinutes(5); break;
        case 2: secs_on = 10U * 60U; TM_ShowMinutes(10); break;
        case 3: secs_on = 15U * 60U; TM_ShowMinutes(15); break;
        case 4: secs_on = 20U * 60U; TM_ShowMinutes(20); break;
        case 5: secs_on = 25U * 60U; TM_ShowMinutes(25); break;
        case 6: secs_on = 30U * 60U; TM_ShowMinutes(30); break;
        case 7: secs_on = 40U * 60U; TM_ShowMinutes(40); break;
        case 8: secs_on = 60U * 60U; TM_ShowMinutes(60); break;
        default: secs_on = 0; break;
    }

    /* Blink SET_TIMER twice */
    SET_TIMER = 1; Delay_ms_soft(500);
    SET_TIMER = 0; Delay_ms_soft(500);
    SET_TIMER = 1; Delay_ms_soft(500);
    SET_TIMER = 0; Delay_ms_soft(500);

    /* Load countdown (IRQ-safe) */
    EA = 0;
    g_sec_countdown = secs_on;
    EA = 1;
}

void main(void)
{
    uint16_t prev_min = 0xFFFFU;
    /* â”€â”€ Clock: switch to internal 16 MHz HIRC â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    TA = 0xAA; TA = 0x55; CKEN  |=  0x20;   /* enable HIRC oscillator        */
    TA = 0xAA; TA = 0x55; CKSWT &= ~0x07;   /* select HIRC as system clock   */

    /* â”€â”€ GPIO Configuration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

    /* P0.6  SET_TIMER â€” push-pull output                                     */
    P0M1 &= ~0x40U;
    P0M2 |=  0x40U;

    /* P3.0  POWER_ON â€” push-pull output                                      */
    P3M1 &= ~0x01U;
    P3M2 |=  0x01U;

    /* P1.5  NO_RELAY â€” push-pull output                                      */
    P1M1 &= ~0x20U;
    P1M2 |=  0x20U;

    /* P0.0, P0.1, P0.3, P0.4  BTN4, BTN1, BTN2, BTN3 â€” quasi-bidirectional (mask = 0x1B) */
    P0M1 &= ~0x1BU;
    P0M2 &= ~0x1BU;

    /* P1.0, P1.1, P1.2, P1.6, P1.7  BTN5-9 â€” quasi-bidirectional (mask = 0xC7) */
    P1M1 &= ~0xC7U;
    P1M2 &= ~0xC7U;

    /* P1.3 (TM_CLK), P1.4 (TM_DIO) â€” quasi-bidirectional  (mask = 0x18)
     * Quasi-bidir acts as open-drain with weak internal pull.
     * External 10 kÎ© pull-ups supply the HIGH level per TM1637 spec.        */
    P1M1 &= ~0x18U;
    P1M2 &= ~0x18U;

    /* â”€â”€ Initial pin states â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    POWER_ON  = 1;            /* Power On LED ON                              */
    SET_TIMER = 0;            /* Set Timer LED OFF at boot                    */
    NO_RELAY  = 0;            /* NO relay OFF at boot                         */
    TM_CLK    = 1;            /* bus idle: both lines HIGH                    */
    TM_DIO    = 1;
    BTN1_PIN  = 1;            /* Enable input / pull-ups on all buttons       */
    BTN2_PIN  = 1;
    BTN3_PIN  = 1;
    BTN4_PIN  = 1;
    BTN5_PIN  = 1;
    BTN6_PIN  = 1;
    BTN7_PIN  = 1;
    BTN8_PIN  = 1;
    BTN9_PIN  = 1;

    /* Read initial button state to prevent false triggers on startup */
    g_prev_button_state = Read_All_Buttons();

    /* If BTN9 is held down at boot, enter diagnostic mode */
    if (READ_BTN(BTN9_PIN))
    {
        while (1)
        {
            uint16_t state = Read_All_Buttons();
            uint8_t d0 = (state & 0x0001 ? 0x01 : 0) | (state & 0x0002 ? 0x02 : 0) | (state & 0x0004 ? 0x04 : 0);
            uint8_t d1 = (state & 0x0008 ? 0x01 : 0) | (state & 0x0010 ? 0x02 : 0) | (state & 0x0020 ? 0x04 : 0);
            uint8_t d2 = (state & 0x0040 ? 0x01 : 0) | (state & 0x0080 ? 0x02 : 0) | (state & 0x0100 ? 0x04 : 0);
            TM_ShowDigits(d0, d1, d2);
            Delay_ms_soft(50);
        }
    }

    /* â”€â”€ Timer 1: 1 ms interrupt base â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
    /*  Reload value = 65535 âˆ’ (16 000 000 / 12 / 1000) â‰ˆ 64202             */
    Timer1_AutoReload_Interrupt_Initial(16, 1000);
    EA = 1;                   /* global interrupt enable                      */

    /* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
     *  POWER-ON SEQUENCE
     *  1.  " On" splash for 2 s
     *  2.  "000" idle â€” wait for first key press
     * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
    TM_ShowON();
    Delay_ms_soft(3000U);

    TM_ShowZeros();

    /* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
     *  MAIN LOOP (Single-Loop State Machine)
     * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
    while (1)
    {
        uint8_t key = Scan_Buttons();

        /* Emergency Stop / OFF: Key 9 pressed */
        if (key == 9U)
        {
            SET_TIMER = 0;
            NO_RELAY  = 0;
            EA = 0;
            g_sec_countdown = 0;
            EA = 1;
            TM_ShowOFF();
            prev_min = 0xFFFFU;
        }
        /* Valid Timer Select: Keys 1 to 8 */
        else if (key >= 1U && key <= 8U)
        {
            Load_New_Timer(key);
            NO_RELAY = 1;
            prev_min = 0xFFFFU;
        }

        /* Non-blocking Countdown Update */
        if (g_sec_countdown > 0)
        {
            uint16_t secs_now;
            uint16_t min_now;

            EA = 0;
            secs_now = g_sec_countdown;
            EA = 1;

            /* Calculate Minutes Remaining (Ceiling)
               Example: 300s (5 mins) / 60 = 5. Display shows 005.
               At 239s (3.98 mins), (239+59)/60 = 4. Display shows 004. */
            min_now = (secs_now + 59U) / 60U;

            if (min_now != prev_min)
            {
                prev_min = min_now;
                TM_ShowMinutes((uint8_t)min_now);
            }
        }
        /* Timer Expiration Transition */
        else if (NO_RELAY == 1)
        {
            NO_RELAY  = 0;
            SET_TIMER = 0;
            TM_ShowOFF();
        }
    }
}
