/***************************************************************************************
 * PROJECT:     MIDI Demo
 * FileName:    main.c
 * 5-28-17:     Stripped all non-PIC795 code.
 * 5-30-17:     Records and plays back on LMMS. Servo number is built in to velocity.
 * 
 ***************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

#define FRAME_DELAY 17

#define true TRUE
#define false FALSE

/** INCLUDES *******************************************************/
#include "usb.h"
#include "HardwareProfile.h"
#include "usb_function_midi.h"

#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/

#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#define RX_BUFFER_ADDRESS_TAG
#define TX_BUFFER_ADDRESS_TAG
#define MIDI_EVENT_ADDRESS_TAG

unsigned char ReceivedDataBuffer[64] RX_BUFFER_ADDRESS_TAG;
unsigned char ToSendDataBuffer[64] TX_BUFFER_ADDRESS_TAG;
USB_AUDIO_MIDI_EVENT_PACKET midiData MIDI_EVENT_ADDRESS_TAG;


USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
WORD_VAL ReadPOT(void);

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = FALSE;

unsigned short XBEERxLength = 0;
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned short XBEETxLength = 0;
unsigned char XBEETxBuffer[MAXBUFFER];

#define MAXPOTS 1
unsigned short ADresult[MAXPOTS]; // read the result of channel 0 conversion from the idle buffer

void ConfigAd(void);

unsigned char frameFlag = FALSE;

int main(void) {
    InitializeSystem();
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();

    printf("\rTESTING MIDI I/O");

#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif

    while (1) {
#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif

        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
    }//end while
}//end main

static void InitializeSystem(void) {
    AD1PCFG = 0xFFFF;
    SYSTEMConfigPerformance(60000000);

#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem

void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0) {
        led_count = 10000U;
    }
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            //if (mGetLED_1()) {
            //    mLED_2_On();
            //} else {
            //    mLED_2_Off();
            //}
        }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                //if (mGetLED_1()) {
                //    mLED_2_Off();
                //} else {
                //    mLED_2_On();
                //}
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
    ;
}

// For debugging?

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    ;
}//end

void USBCBStdSetDscHandler(void) {
    ;
}//end

void USBCBInitEP(void) {
    //enable the HID endpoint
    USBEnableEndpoint(MIDI_EP, USB_OUT_ENABLED | USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again,
            //until a new suspend condition is detected.

            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}

// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned short HOSTRxIndex = 0;
    unsigned char ch;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            if (HOSTRxIndex < MAXBUFFER) HOSTRxBuffer[HOSTRxIndex++] = ch;
            if (ch == '\r') {
                HOSTRxBuffer[HOSTRxIndex] = '\0'; // $$$$
                HOSTRxBufferFull = TRUE;
                HOSTRxIndex = 0;
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_0); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AN2 (A2 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN0_ANA

    // USE AN2
#define PARAM5 SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |\
SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}

void UserInit(void) {
    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize all of the push buttons
    mInitAllSwitches();

    //initialize the variable holding the handle for the last
    // transmission
    USBTxHandle = NULL;
    USBRxHandle = NULL;

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    // Set up XBEE at 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);

    ConfigAd();

    // Set up Timer 2 for 100 microsecond roll-over rate
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);

    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) {
    static unsigned short frameDelay = 0;

    mT2ClearIntFlag(); // clear the interrupt flag

    frameDelay++;
    if (frameDelay >= FRAME_DELAY) {
        frameDelay = 0;
        frameFlag = TRUE;
    }

}

#define SEND_SERVO 0x90
#define NOTE_OFF 0x80

void ProcessIO(void) {
    static BYTE pitch = 0x3C;
    unsigned short potValue = 0;
    static unsigned char sentNoteOff = TRUE;
    static unsigned char LEDflag = TRUE;
    unsigned char MIDIreceivedCommand;
    static unsigned short USBtimeout = 0;
    unsigned char RXcommandServoNumber;
    unsigned char TXcommandServoNumber = 1;

    if (HOSTRxBufferFull) {
        printf("\rRECEIVED: %s", HOSTRxBuffer);
        HOSTRxBufferFull = FALSE;
    }

    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (!USBHandleBusy(USBRxHandle)) {
        //We have received a MIDI packet from the host, process it and then
        //  prepare to receive the next packet

        //INSERT MIDI PROCESSING CODE HERE

        //Get ready for next packet (this will overwrite the old data)
        USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
        if (LEDflag) {
            mLED_4_On();
            LEDflag = FALSE;
            LATBbits.LATB2 = 1;
        } else {
            mLED_4_Off();
            LEDflag = TRUE;
            LATBbits.LATB2 = 0;
        }

        MIDIreceivedCommand = ReceivedDataBuffer[1] & 0xF0;        

        if (MIDIreceivedCommand == 0x90 && ReceivedDataBuffer[3] != 0) {
            RXcommandServoNumber = (ReceivedDataBuffer[3] - 1) & 0x0F;
            USBtimeout = 30;
            printf("\r%X, %X, %X, %X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
            if (XBEETxLength == 0) {
                XBEETxBuffer[0] = RXcommandServoNumber;
                XBEETxBuffer[1] = ReceivedDataBuffer[2] * 2;
                if (XBEETxBuffer[1] == 0xFF) XBEETxBuffer[1] = 254;
                XBEETxLength = 2;
                while (!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte(XBEEuart, 0xFF);
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
            }
        }
    } else if (sw2 != 0) {
        mLED_2_Off();
        mLED_4_Off();
    }

    //if (frameFlag) {
    //    frameFlag = FALSE;

    if (!USBHandleBusy(USBTxHandle)) {
        if (sentNoteOff == FALSE) {
            midiData.Val = 0; // must set all unused values to 0 
            midiData.CableNumber = 0;
            midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
            midiData.DATA_0 = 0x90; //Note off
            midiData.DATA_1 = pitch; //pitch
            midiData.DATA_2 = 0x00;            

            USBTxHandle = USBTxOnePacket(MIDI_EP, (BYTE*) & midiData, 4);
            sentNoteOff = TRUE;
        } else if (sw2 == 0 && frameFlag) {
            if (USBtimeout) USBtimeout--;
            frameFlag = FALSE;
            if (LEDflag) {
                mLED_2_On();
                LEDflag = FALSE;
                LATBbits.LATB2 = 1;
            } else {
                mLED_2_Off();
                LEDflag = TRUE;
                LATBbits.LATB2 = 0;
            }

            potValue = (ADresult[0] / 8) + 1;
            ConfigAd();
            if (potValue > 127) potValue = 127;
            if (pitch != potValue){
                pitch = potValue; //  + 0x3C;                
                midiData.Val = 0; //must set all unused values to 0
                midiData.CableNumber = 0;
                midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
                midiData.DATA_0 = 0x90; //Note on
                midiData.DATA_1 = pitch; //pitch
                midiData.DATA_2 = 0x40 | (TXcommandServoNumber + 1);
            
                USBTxHandle = USBTxOnePacket(MIDI_EP, (BYTE*) & midiData, 4);
                sentNoteOff = FALSE;
            }
            if (!USBtimeout && XBEETxLength == 0) {
                XBEETxBuffer[0] = 0;
                XBEETxBuffer[1] = pitch * 2;
                if (XBEETxBuffer[1] == 0xFF) XBEETxBuffer[1] = 254;
                XBEETxLength = 2;
                while (!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte(XBEEuart, 0xFF); // Send SSC Servo start character
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
            }
        }
    }
    //}
}//end ProcessIO

void __ISR(XBEE_VECTOR, ipl2) IntXbeeHandler(void) {
    static unsigned short RxIndex = 0;
    static unsigned short TxIndex = 0;
    unsigned char ch = '\r';

    if (XBEEbits.OERR || XBEEbits.FERR) {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));
        if (UARTReceivedDataIsAvailable(XBEEuart)) {
            ch = UARTGetDataByte(XBEEuart);
            if (RxIndex < MAXBUFFER - 2)
                XBEERxBuffer[RxIndex++] = ch;

            if (ch == '\r') {
                XBEERxLength = RxIndex;
                RxIndex = 0;
            }
            /*/
                        if (!UARTtimeout){
                            CloseTimer1();
                            ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
                            OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 18724);  // $$$$
                        }
                        OEpin = 1;           // Keep output enable high so display is off
                        UARTtimeout=UART_TIMEOUT;
                    }
             */
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
        if (XBEETxLength) {
            if (TxIndex < MAXBUFFER) ch = XBEETxBuffer[TxIndex++];
            while (!UARTTransmitterIsReady(XBEEuart));
            UARTSendDataByte(XBEEuart, ch);
            if (TxIndex >= XBEETxLength) {
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
                XBEETxLength = 0;
                TxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    }

}

/** EOF main.c *************************************************/
#endif

