/*===========================================================================
 * Timer_project - Version_2.0 (TM1650 PCB Migration)
 * main.c — LED Timer Control + TM1650 3-Digit 7-Segment CC Display
 * Target : MS51 @ 16 MHz (HIRC)
 *
 * Keys 1-9  →  5 min to 60 min relay ON countdown timer
 * Key 10    →  Manual Start/Stop & Emergency Stop button (P0.5)
 *
 * Display state machine
 * ─────────────────────
 *  BOOT      :  " On"  shown for 3 s  (TM1650 splash)
 *  IDLE      :  "000"  shown until a valid key is pressed
 *  COUNTDOWN :  remaining minutes (ceil, 3 digits, e.g. "045" → "001")
 *  MANUAL ON :  " On"  shown while relay is ON without timer limit
 *  DONE/STOP :  "OFF"  shown when stopped or timer expires
 *
 * Pin map (New PCB)
 * ─────────────────
 *  NO_RELAY   P0.6                       (push-pull output)
 *  POWER_ON   P3.0                       (push-pull output)
 *  SET_TIMER  P0.7                       (push-pull output)
 *  MANUAL_BTN P0.5                       (quasi-bidir, internal pull-up)
 *  BTN1 ( 5m) P1.2                       (quasi-bidir, internal pull-up)
 *  BTN2 (10m) P1.1                       (quasi-bidir, internal pull-up)
 *  BTN3 (15m) P1.0                       (quasi-bidir, internal pull-up)
 *  BTN4 (20m) P0.0                       (quasi-bidir, internal pull-up)
 *  BTN5 (30m) P0.3                       (quasi-bidir, internal pull-up)
 *  BTN6 (40m) P0.4                       (quasi-bidir, internal pull-up)
 *  BTN7 (45m) P0.1                       (quasi-bidir, internal pull-up)
 *  BTN8 (50m) P1.6                       (quasi-bidir, internal pull-up)
 *  BTN9 (60m) P1.7                       (quasi-bidir, internal pull-up)
 *  TM_CLK     P1.3                       (quasi-bidir, ext. 10 kΩ pull-up)
 *  TM_DIO     P1.4                       (quasi-bidir, ext. 10 kΩ pull-up)
 *===========================================================================*/

#include "numicro_8051.h"
#include "timer.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  PIN DEFINITIONS & BRIGHTNESS CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Display Brightness Macro:
 * Level 1 (12.5%) to 7 (87.5%), Level 8 or 0 (100%). Level 6 = ~70%-75% duty cycle */
#define TM1650_BRIGHTNESS_LEVEL  6

#define NO_RELAY    P06                /* Output: Normally Open relay         */
#define POWER_ON    P30                /* Output: Power On LED                */
#define SET_TIMER   P07                /* Output: Set Timer LED indicator     */
#define MANUAL_BTN  P05                /* Input: Manual Start/Stop button     */

/* Direct Buttons Mappings */
#define BTN1_PIN    P12                /* 5 min                               */
#define BTN2_PIN    P11                /* 10 min                              */
#define BTN3_PIN    P10                /* 15 min                              */
#define BTN4_PIN    P00                /* 20 min                              */
#define BTN5_PIN    P03                /* 30 min                              */
#define BTN6_PIN    P04                /* 40 min                              */
#define BTN7_PIN    P01                /* 45 min                              */
#define BTN8_PIN    P16                /* 50 min                              */
#define BTN9_PIN    P17                /* 60 min                              */

/* Button Configuration
 * Set BUTTON_ACTIVE_STATE to:
 *   0 : Active-Low (button connects pin to GND when pressed)
 *   1 : Active-High (button connects pin to VCC when pressed)
 */
#define BUTTON_ACTIVE_STATE  0

#define READ_BTN(pin)        (BUTTON_ACTIVE_STATE ? ((pin) == 1) : ((pin) == 0))

/* TM1650 two-wire bus (external 10 kΩ pull-ups to 3.3 V / 5 V on both)      */
#define TM_CLK      P13                /* Serial clock                        */
#define TM_DIO      P14                /* Serial data I/O                     */

/* TM1650 Register Addresses */
#define TM1650_CMD_CTRL     0x48U      /* System Control Register             */
#define TM1650_DIG1_ADDR    0x68U      /* DIG1 (Leftmost)                     */
#define TM1650_DIG2_ADDR    0x6AU      /* DIG2 (Middle)                       */
#define TM1650_DIG3_ADDR    0x6CU      /* DIG3 (Rightmost)                    */

/* ═══════════════════════════════════════════════════════════════════════════
 *  TIMER 1 — 1 ms countdown  (ISR at vector 0x1B, interrupt 3)
 * ═══════════════════════════════════════════════════════════════════════════ */

volatile uint16_t g_sec_countdown = 0;   /* decremented each 1 s by ISR      */
volatile uint8_t  g_is_manual_mode = 0;  /* 1 if relay started manually      */
uint16_t g_prev_button_state = 0x0000;  /* active-state tracking for 10 btns */
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

/* ═══════════════════════════════════════════════════════════════════════════
 *  SOFTWARE DELAY — keypad debounce and splash screen timing only
 * ═══════════════════════════════════════════════════════════════════════════ */

static void Delay_ms_soft(uint16_t ms)
{
    volatile uint16_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 267; j++);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TM1650 DISPLAY DRIVER (Common Cathode)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Segment encoding (Common Cathode, SEG-A in bit 0) */
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

/* Special patterns */
#define SEG_BLANK   0x00U     /*  (all off)                                  */
#define SEG_O       0x3FU     /*  O  — segments a,b,c,d,e,f                  */
#define SEG_n       0x54U     /*  n  — segments c,e,g                        */
#define SEG_F       0x71U     /*  F  — segments a,e,f,g                      */

static void TM_Delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 10; i++);
}

static void TM_Start(void)
{
    TM_DIO = 1;  TM_CLK = 1;  TM_Delay();
    TM_DIO = 0;               TM_Delay();  /* DIO HIGH→LOW while CLK HIGH   */
    TM_CLK = 0;               TM_Delay();
}

static void TM_Stop(void)
{
    TM_CLK = 0;  TM_DIO = 0;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_DIO = 1;               TM_Delay();  /* DIO LOW→HIGH while CLK HIGH   */
}

static void TM_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        TM_CLK = 0;                    TM_Delay();
        TM_DIO = (data & 0x01U) ? 1 : 0;  /* set bit while CLK low          */
                                       TM_Delay();
        TM_CLK = 1;                    TM_Delay();  /* latch on rising edge    */
        data >>= 1;
    }

    /* 9th clock: release DIO for ACK */
    TM_CLK = 0;  TM_DIO = 1;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_CLK = 0;               TM_Delay();
}

/* TM1650 Bus Command Write: Sends address and data packet */
static void TM1650_Write(uint8_t addr, uint8_t data)
{
    TM_Start();
    TM_WriteByte(addr);
    TM_WriteByte(data);
    TM_Stop();
}

/* Core Display Routine */
static void TM_ShowDigits(uint8_t d0, uint8_t d1, uint8_t d2)
{
    /* Step 1 — Write System Control Register (0x48): Set Brightness & Display ON */
    uint8_t ctrl_val = (uint8_t)(((TM1650_BRIGHTNESS_LEVEL % 8U) << 4U) | 0x01U);
    TM1650_Write(TM1650_CMD_CTRL, ctrl_val);

    /* Step 2 — Write Digits DIG1, DIG2, DIG3 */
    TM1650_Write(TM1650_DIG1_ADDR, d0);
    TM1650_Write(TM1650_DIG2_ADDR, d1);
    TM1650_Write(TM1650_DIG3_ADDR, d2);
}

/* High-level display helpers */
static void TM_ShowMinutes(uint8_t mins)
{
    TM_ShowDigits(
        SEG_TABLE[mins / 100U],
        SEG_TABLE[(mins / 10U) % 10U],
        SEG_TABLE[mins % 10U]
    );
}

static void TM_ShowON(void)
{
    TM_ShowDigits(SEG_O, SEG_n, SEG_BLANK);
}

static void TM_ShowOFF(void)
{
    TM_ShowDigits(SEG_O, SEG_F, SEG_F);
}

static void TM_ShowZeros(void)
{
    TM_ShowDigits(SEG_TABLE[0], SEG_TABLE[0], SEG_TABLE[0]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DIRECT BUTTON SCANNER (10 Buttons: Keys 1–9 + Manual Start/Stop)
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t Read_All_Buttons(void)
{
    uint16_t state = 0;
    if (READ_BTN(BTN1_PIN))   state |= 0x0001U; /* 5 min                */
    if (READ_BTN(BTN2_PIN))   state |= 0x0002U; /* 10 min               */
    if (READ_BTN(BTN3_PIN))   state |= 0x0004U; /* 15 min               */
    if (READ_BTN(BTN4_PIN))   state |= 0x0008U; /* 20 min               */
    if (READ_BTN(BTN5_PIN))   state |= 0x0010U; /* 30 min               */
    if (READ_BTN(BTN6_PIN))   state |= 0x0020U; /* 40 min               */
    if (READ_BTN(BTN7_PIN))   state |= 0x0040U; /* 45 min               */
    if (READ_BTN(BTN8_PIN))   state |= 0x0080U; /* 50 min               */
    if (READ_BTN(BTN9_PIN))   state |= 0x0100U; /* 60 min               */
    if (READ_BTN(MANUAL_BTN)) state |= 0x0200U; /* Manual Start/Stop    */
    return state;
}

static uint8_t Scan_Buttons(void)
{
    uint16_t current_state = Read_All_Buttons();
    uint8_t pressed_btn = 0xFFU;
    uint8_t i;
    uint16_t mask = 0x0001U;

    for (i = 0; i < 10; i++)
    {
        if (!(g_prev_button_state & mask) && (current_state & mask))
        {
            Delay_ms_soft(10);
            if (Read_All_Buttons() & mask)
            {
                pressed_btn = i + 1;
                g_prev_button_state |= mask;
                break;
            }
        }
        mask <<= 1;
    }

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
        case 5: secs_on = 30U * 60U; TM_ShowMinutes(30); break;
        case 6: secs_on = 40U * 60U; TM_ShowMinutes(40); break;
        case 7: secs_on = 45U * 60U; TM_ShowMinutes(45); break;
        case 8: secs_on = 50U * 60U; TM_ShowMinutes(50); break;
        case 9: secs_on = 60U * 60U; TM_ShowMinutes(60); break;
        default: secs_on = 0; break;
    }

    /* Blink SET_TIMER LED twice */
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

    /* Clock: switch to internal 16 MHz HIRC */
    TA = 0xAA; TA = 0x55; CKEN  |=  0x20;   /* enable HIRC oscillator        */
    TA = 0xAA; TA = 0x55; CKSWT &= ~0x07;   /* select HIRC as system clock   */

    /* GPIO Configuration */

    /* P0.6 NO_RELAY — push-pull output */
    P0M1 &= ~0x40U;
    P0M2 |=  0x40U;

    /* P0.7 SET_TIMER — push-pull output */
    P0M1 &= ~0x80U;
    P0M2 |=  0x80U;

    /* P3.0 POWER_ON — push-pull output */
    P3M1 &= ~0x01U;
    P3M2 |=  0x01U;

    /* P0.0, P0.1, P0.3, P0.4, P0.5 (BTN4, BTN7, BTN5, BTN6, MANUAL_BTN) — quasi-bidirectional (mask = 0x3B) */
    P0M1 &= ~0x3BU;
    P0M2 &= ~0x3BU;

    /* P1.0, P1.1, P1.2, P1.3, P1.4, P1.6, P1.7 (BTN3, BTN2, BTN1, TM_CLK, TM_DIO, BTN8, BTN9) — quasi-bidirectional (mask = 0xDF) */
    P1M1 &= ~0xDFU;
    P1M2 &= ~0xDFU;

    /* Initial pin states */
    POWER_ON   = 1;           /* Power On LED ON                             */
    SET_TIMER  = 0;           /* Set Timer LED OFF                           */
    NO_RELAY   = 0;           /* Relay OFF                                   */
    TM_CLK     = 1;           /* Bus idle: HIGH                              */
    TM_DIO     = 1;
    MANUAL_BTN = 1;           /* Enable pull-ups on all buttons              */
    BTN1_PIN   = 1;
    BTN2_PIN   = 1;
    BTN3_PIN   = 1;
    BTN4_PIN   = 1;
    BTN5_PIN   = 1;
    BTN6_PIN   = 1;
    BTN7_PIN   = 1;
    BTN8_PIN   = 1;
    BTN9_PIN   = 1;

    g_prev_button_state = Read_All_Buttons();

    /* Diagnostic mode if BTN9 is held down at boot */
    if (READ_BTN(BTN9_PIN))
    {
        while (1)
        {
            uint16_t state = Read_All_Buttons();
            uint8_t d0 = (state & 0x0001 ? 0x01 : 0) | (state & 0x0002 ? 0x02 : 0) | (state & 0x0004 ? 0x04 : 0) | (state & 0x0008 ? 0x08 : 0);
            uint8_t d1 = (state & 0x0010 ? 0x01 : 0) | (state & 0x0020 ? 0x02 : 0) | (state & 0x0040 ? 0x04 : 0) | (state & 0x0080 ? 0x08 : 0);
            uint8_t d2 = (state & 0x0100 ? 0x01 : 0) | (state & 0x0200 ? 0x02 : 0);
            TM_ShowDigits(d0, d1, d2);
            Delay_ms_soft(50);
        }
    }

    /* Timer 1: 1 ms interrupt base */
    Timer1_AutoReload_Interrupt_Initial(16, 1000);
    EA = 1;                   /* global interrupt enable                      */

    /* POWER-ON SEQUENCE */
    TM_ShowON();
    Delay_ms_soft(3000U);
    TM_ShowZeros();

    /* MAIN LOOP */
    while (1)
    {
        uint8_t key = Scan_Buttons();

        /* Manual Start / Stop & Emergency Stop Button (Key 10) */
        if (key == 10U)
        {
            if (NO_RELAY == 1)
            {
                /* Stop relay, clear timer & manual state */
                NO_RELAY         = 0;
                SET_TIMER        = 0;
                g_is_manual_mode = 0;
                EA = 0;
                g_sec_countdown  = 0;
                EA = 1;
                TM_ShowOFF();
                prev_min = 0xFFFFU;
            }
            else
            {
                /* Start relay manually without timer */
                NO_RELAY         = 1;
                SET_TIMER        = 0;
                g_is_manual_mode = 1;
                EA = 0;
                g_sec_countdown  = 0;
                EA = 1;
                TM_ShowON();
                prev_min = 0xFFFFU;
            }
        }
        /* Valid Timer Select: Keys 1 to 9 (5 min to 60 min) */
        else if (key >= 1U && key <= 9U)
        {
            g_is_manual_mode = 0;
            Load_New_Timer(key);
            NO_RELAY = 1;
            prev_min = 0xFFFFU;
        }

        /* Non-blocking Countdown Update */
        if (g_sec_countdown > 0 && !g_is_manual_mode)
        {
            uint16_t secs_now;
            uint16_t min_now;

            EA = 0;
            secs_now = g_sec_countdown;
            EA = 1;

            min_now = (secs_now + 59U) / 60U;

            if (min_now != prev_min)
            {
                prev_min = min_now;
                TM_ShowMinutes((uint8_t)min_now);
            }
        }
        /* Timer Expiration Transition */
        else if (NO_RELAY == 1 && !g_is_manual_mode && g_sec_countdown == 0)
        {
            NO_RELAY  = 0;
            SET_TIMER = 0;
            TM_ShowOFF();
        }
    }
}
