/***********************************************************************************************************
 * PROJECT:     MIDI Demo
 * FileName:    main.c
 * 5-28-17:     Stripped all non-PIC795 code.
 * 5-30-17:     Records and plays back on LMMS. Servo number is built in to velocity.
 * 6-1-17:      Fixed NOTE OFF bug. Works well at 16 ms sampling rate with servo 0.
 * 6-1-17:      Assigns channel numbers to servos. Implemented HOST keyboard menu to enable/disable servos.
 * 6-2-17:      Added low pass and window filtering.
 * 6-4-17:      Got rid of bugs that were causing servos to jerk.
 *              Implemented STANDBY, PLAY, and RECORD for both USB and MIDI modes.
 *              Records and plays simultaneously on multiple servos.
 *              servoNumber is MIDI number + 1. So servo #0 is reserved.
 *              Resolution is eight bits.
 * 6-6-17:      Records and plays 10 bit MIDI USB.
 *              Also got MIDI MODE working with 10 bit RECORD and PLAY
 * 6-7-17:      Encoder interrupts work.
 * 6-8-17:      Added scaling multiplier for encoder input
 * 6-10-17:     Changed servo resolution to 8 bits. Combined servo number with servo data.
 *              Allow seven servos. Use Pololu 6 bit command and data protocol.
 *              Board number is channel number.
 * 6-12-17:     DMX sending and receiving is working using LED Matrix Controller Board Rev 1
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

#define	DMX_STANDBY	0
#define	DMX_BREAK 	1
#define DMX_MARK 	2
#define DMX_START 	3
#define DMX_DATA	4
#define DMXLENGTH 	16
#define TICK		10000

#define MAXSCALE 24
#define MAXPOTVALUE 255
#define FRAME_DELAY 8
#define MAXFILTER 8
#define MAXWINDOW 5
#define MAXSERVO 7 // 12
#define MAXPOTS 1
#define MAXBOARD 16

#define true TRUE
#define false FALSE

#define USE_USB

union {
    unsigned char byte[2];
    unsigned short integer;
} convert;

#define lowByte byte[0]
#define highByte byte[1]  

/** INCLUDES *******************************************************/
#include "usb.h"
#include "HardwareProfile.h"
#include "usb_function_midi.h"
#include "Delay.h"

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

// unsigned char ReceivedDataBuffer[64] RX_BUFFER_ADDRESS_TAG;
BYTE ReceivedDataBuffer[64];
unsigned char ToSendDataBuffer[64] TX_BUFFER_ADDRESS_TAG;
// USB_AUDIO_MIDI_EVENT_PACKET midiData MIDI_EVENT_ADDRESS_TAG;
BYTE midiData[64]; //  MIDI_EVENT_ADDRESS_TAG;


USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessUSB(void);
void ProcessMIDI(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
WORD_VAL ReadPOT(void);

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define DMXuart UART5
#define DMXbits U5STAbits
#define DMX_VECTOR _UART_5_VECTOR

#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

unsigned short XBEERxLength = 0;
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned short XBEETxLength = 0;
unsigned char XBEETxBuffer[MAXBUFFER];

unsigned short MIDITxLength = 0;
unsigned char MIDITxBuffer[MAXBUFFER];
unsigned char MIDIRxBuffer[MAXBUFFER];
unsigned short MIDIRxIndex = 0;

unsigned char DMXRxBuffer[MAXBUFFER+1];
unsigned char DMXTxBuffer[MAXBUFFER+1];
unsigned char DMXflag=false;
static unsigned char DMXstate=DMX_STANDBY;

unsigned short ADresult[MAXPOTS]; // read the result of channel 0 conversion from the idle buffer

void ConfigAd(void);

unsigned char frameFlag = false;
unsigned short milliSecondCounter = 0;



unsigned char controlCommand = 0;
short getInteger(unsigned char *ptrString);
unsigned char getCommand(unsigned char *ptrString);
short windowFilter(short newValue);
unsigned char swap(short *ptrDataA, short *ptrDataB);
unsigned char sortData(short *arrSortData);
short lowPassFilter(short newValue);

#define SET_DISPLAY 4 // CTL-D
#define SET_SERVO 19 // CTL-S
#define SET_RECORD 18 // CTL-R
#define SET_STANDBY 1 // CTL-A
#define SET_SCALE 3 // CTL-C
#define SET_PLAY 16 // CTL-P
#define NUMBER_ERROR 32767

struct servoType {
    unsigned char ID;
    unsigned short position;
    unsigned char updated;
    unsigned char enabled;
};

struct servoType arrServo[MAXBUFFER];
unsigned short servoDataIndex = 0;
unsigned short MIDIStateMachine(void);
//unsigned char convertMIDItoData(unsigned char *arrMIDIdata, unsigned short *servoData, unsigned char *servoNumber);
//unsigned char convertDATAtoMIDI(unsigned char servoNumber, unsigned short intTenBit, unsigned char *arrMIDIdata);
// unsigned char convertDATAtoPOLU(unsigned char servoNumber, unsigned short tenBitValue, unsigned char *arrPOLUdata);
unsigned char convertDATAtoPOLU(unsigned char boardNumber, unsigned char servoNumber, unsigned short tenBitValue, unsigned char *arrPOLUdata);
unsigned char convertDATAtoMIDI(unsigned char boardNumber, unsigned char servoNumber, unsigned short intTenBit, unsigned char *arrMIDIdata);
unsigned char convertMIDItoData(unsigned char *boardNumber, unsigned char *arrMIDIdata, unsigned short *servoData, unsigned char *servoNumber);
short MIDItimeout = 0;
unsigned char MIDIbufferFull = false;
short EncoderCounter = 32;

#define MIDI_TIMEOUT 2
short scaleValue = 4; // was 16

#define STANDBY 0
#define PLAY 1
#define RECORD 2

unsigned char mode = STANDBY;
unsigned char displayMode = true;
short servoNumber = 0;
unsigned char boardNumber = 12;

int main(void) {
    short value = 0;
    short i = 0, j = 0;
    unsigned char setCommand = 0;
    short displayCounter = 10;
    unsigned short DMXdataCounter = 0;
    unsigned char ch;

    for (i = 0; i < MAXSERVO; i++) {
        arrServo[i].position = 127;
        arrServo[i].updated = false;
    }

    InitializeSystem();
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();

#ifdef USE_USB
    printf("\r\rTESTING DMX RECEIVE");
#else
    printf("\r\rTESTING MIDI IO:");
#endif
    while (1) {
        if (DMXflag){
            DMXflag = false;        
            //if (displayCounter) displayCounter--;
            //else {
                displayCounter = 10;
                printf("\rDMX #%d: ", DMXdataCounter++);
                
                for (j = 0; j < DMXLENGTH; j++){
                    ch = DMXTxBuffer[j] = DMXRxBuffer[j];
                    printf("%d, ", ch);
                }
            //}
        }
        if (controlCommand) {
            switch (controlCommand) {
                case SET_SERVO:
                    mode = STANDBY;
                    printf("\rSET SERVO: %d", servoNumber);
                    break;
                case SET_SCALE:
                    mode = STANDBY;
                    printf("\rSET POT SCALE: %d", scaleValue);
                    break;
                case SET_DISPLAY:
                    if (displayMode) {
                        displayMode = false;
                        printf("\rDISPLAY OFF");
                    } else {
                        displayMode = true;
                        printf("\rDISPLAY ON");
                    }
                    break;
                case SET_RECORD:
                    mode = RECORD;
                    printf("\rRECORD MODE");
                    break;
                case SET_STANDBY:
                    mode = STANDBY;
                    printf("\rSTANDBY MODE");
                    break;
                case SET_PLAY:
                    mode = PLAY;
                    printf("\rPLAY MODE");
                    break;
                default:
                    break;
            }
            setCommand = controlCommand;
            controlCommand = 0;
        } else if (HOSTRxBufferFull) {
            HOSTRxBufferFull = false;
            value = getInteger(HOSTRxBuffer);
            if (setCommand == SET_SERVO) {
                if (value == NUMBER_ERROR) printf("\rNo number entered");
                else if (value < 0 || value > MAXSERVO) printf("\rInvalid Servo");
                else {
                    servoNumber = (unsigned char) value;
                    printf("\rUSE SERVO #%d", servoNumber);
                }
            } else if (setCommand == SET_SCALE) {
                if (value == NUMBER_ERROR) printf("\rNo number entered");
                else if (value < 1 || value > MAXSCALE) printf("\rUse value from 1 to %d", MAXSCALE);
                else {
                    scaleValue = (short) value;
                    printf("\rPOT SCALE x%d", scaleValue);
                }
            }
        }

#ifdef USE_USB    
#if defined(USB_INTERRUPT)
        USBDeviceAttach();
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif
        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessUSB() function.
        ProcessUSB();


#else
        ProcessMIDI();
#endif        
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

#ifdef USE_USB
    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
#endif
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

    if (USBGetRemoteWakeupStatus() == true) {
        if (USBIsBusSuspended() == true) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = false; //So we don't execute this code again,
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
    return true;
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
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

    //  set AN2 (A2 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN3_ANA

    // USE AN2
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 |\
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

void __ISR(XBEE_VECTOR, ipl2) IntXbeeHandler(void) {
    static unsigned short RxIndex = 0;
    static unsigned short TxIndex = 0;
    unsigned char ch;

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
#define ENTER 13
#define BACKSPACE 8
// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (ch == '\n' || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == ENTER) {
                HOSTRxBuffer[HOSTRxIndex] = '\0'; // $$$$
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) controlCommand = ch;
            else if (HOSTRxIndex < MAXBUFFER) HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

#define MAXNUMLENGTH 8

short getInteger(unsigned char *ptrString) {
    unsigned char ch = 0, strNumber[MAXNUMLENGTH + 1];
    unsigned char negativeFlag = false;
    short i = 0, j = 0, value = 32767;

    while (ptrString[i] != NULL && j < MAXNUMLENGTH) {
        ch = ptrString[i];
        if (ch == '\0') {
            strNumber[j] = '\0';
            break;
        }
        if (ch == '-') negativeFlag = true;
        else if (isdigit(ch)) strNumber[j++] = ch;
        i++;
    }
    if (j) value = atoi(strNumber);
    if (negativeFlag) value = 0 - value;
    return (value);
}

unsigned char getCommand(unsigned char *ptrString) {
    unsigned char ch = 0;
    short i = 0;

    while (ptrString[i] != NULL && i < MAXBUFFER) {
        ch = ptrString[i];
        if (isalpha(ch)) return (ch);
    }
    return (0);
}

unsigned char swap(short *ptrDataA, short *ptrDataB) {
    short tempVal;
    if (ptrDataA == NULL || ptrDataB == NULL) return (false);
    tempVal = *ptrDataA;
    *ptrDataA = *ptrDataB;
    *ptrDataB = tempVal;
    return (true);
}

unsigned char sortData(short *arrSortData) {
    short i, j;

    if (arrSortData == NULL) return (false);
    for (i = 0; i < (MAXWINDOW - 1); i++) {
        for (j = i + 1; j < MAXWINDOW; j++) {
            if (arrSortData[i] > arrSortData[j]) {
                swap(&arrSortData[i], &arrSortData[j]);
            }
        }
    }
    return (true);
}

#define CENTERINDEX ((MAXWINDOW-1)/2)

short windowFilter(short newValue) {
    short centerValue;
    static short arrInData[MAXWINDOW];
    short arrFilterData[MAXWINDOW];
    static unsigned short i = 0, j = 0, k = 0;
    static unsigned char startFlag = true;

    if (startFlag) {
        startFlag = false;
        for (i = 0; i < MAXWINDOW; i++) arrInData[i] = 0;
        i = 0;
    }
    if (i >= MAXWINDOW) i = 0;
    arrInData[i++] = newValue;

    for (j = 0; j < MAXWINDOW; j++) arrFilterData[j] = arrInData[j];
    if (sortData(arrFilterData) == false) printf("\rERROR");

    centerValue = arrFilterData[CENTERINDEX];
    return (centerValue);
}

short lowPassFilter(short newValue) {
    static short arrInData[MAXFILTER];
    static unsigned short i = 0, j = 0;
    static unsigned char startFlag = true;
    short result;
    long sum;

    if (startFlag) {
        startFlag = false;
        for (i = 0; i < MAXFILTER; i++) arrInData[i] = 0;
        i = 0;
    }
    if (i >= MAXFILTER) i = 0;
    arrInData[i++] = newValue;

    sum = 0;
    for (j = 0; j < MAXFILTER; j++) sum = sum + (long) arrInData[j];

    result = (short) (sum / MAXFILTER);
    return (result);
}



void __ISR(MIDI_VECTOR, ipl2) IntMIDIHandler(void) {
    static unsigned short TxIndex = 0;
    unsigned char ch;

    if (MIDIbits.OERR || MIDIbits.FERR) {
        if (UARTReceivedDataIsAvailable(MIDIuart))
            ch = UARTGetDataByte(MIDIuart);
        XBEEbits.OERR = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(MIDIuart));

        if (UARTReceivedDataIsAvailable(MIDIuart)) {
            ch = UARTGetDataByte(MIDIuart);
            if (ch != 0xfe) {
                MIDItimeout = MIDI_TIMEOUT;
                if (MIDIRxIndex >= MAXBUFFER) MIDIRxIndex = 0;
                MIDIRxBuffer[MIDIRxIndex++] = ch;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
        if (MIDITxLength) {
            if (TxIndex < MIDITxLength) {
                if (TxIndex < MAXBUFFER) ch = MIDITxBuffer[TxIndex++];
                while (!UARTTransmitterIsReady(MIDIuart));
                UARTSendDataByte(MIDIuart, ch);
            } else {
                INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
                MIDITxLength = 0;
                TxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    }

}

unsigned short MIDIStateMachine(void) {
    unsigned char ch;
    static unsigned char MIDIcommand = 0x90;
    static unsigned char MIDIchannel = 0;
    static unsigned char MIDIstate = 0;
    static unsigned short servoDataIndex = 0;
    static unsigned short scanIndex = 0;
    static unsigned char highIn = 0;
    static unsigned char lowIn = 0;

    while (scanIndex != MIDIRxIndex) {
        if (scanIndex >= MAXBUFFER) scanIndex = 0;
        ch = MIDIRxBuffer[scanIndex++];

        if (ch & 0x80) MIDIstate = 0;

        switch (MIDIstate) {
            case 0:
                MIDIcommand = ch & 0xF0;
                MIDIchannel = ch & 0x0F;
                if (MIDIcommand == 0x90) MIDIstate = 1;
                break;
            case 1:
                highIn = ch;
                MIDIstate = 2;
                break;
            case 2:
                if (ch != 0) {
#define MAXCHANNEL 15
                    if (MIDIchannel <= MAXCHANNEL) {
                        convert.highByte = highIn;
                        convert.lowByte = ch << 1;
                        arrServo[servoDataIndex].ID = MIDIchannel;
                        arrServo[servoDataIndex].position = convert.integer >> 4;
                        servoDataIndex++;
                        if (servoDataIndex >= MAXBUFFER) servoDataIndex = 0;
                    }
                }
                MIDIstate = 1;
                break;
            default:
                break;
        }
    }
    return (servoDataIndex);
}

void ProcessUSB(void) {
    static short potValue = 0;
    static short ADpotValue = 0;
    static short previousADPotValue = 0;
    static short previousPotValue = 0;
    static unsigned char sentNoteOff = true;
    static unsigned char LEDflag = true;
    static unsigned short LEDcounter = 0;
    static unsigned char arrPOLUdata[8];
    static unsigned char arrMIDIdata[8];
    unsigned short servoData = 0;
    unsigned char ReceivedServoNumber;
    long lngPotValue;
    static unsigned short previousEncoderCounter = 0;

    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (!USBHandleBusy(USBRxHandle)) {
        USBRxHandle = USBRxOnePacket(MIDI_EP, ReceivedDataBuffer, 64);
        LEDcounter++;
        if (LEDcounter > 10) {
            LEDcounter = 0;
            if (LEDflag) {
                mLED_4_On();
                LEDflag = false;
                LATBbits.LATB2 = 1;
            } else {
                mLED_4_Off();
                LEDflag = true;
                LATBbits.LATB2 = 0;
            }
        }

        if (mode != STANDBY && XBEETxLength == 0) {
            if (convertMIDItoData(&boardNumber, ReceivedDataBuffer, &servoData, &ReceivedServoNumber)) {
                if (displayMode) printf("\rCH%d POT: %d => MIDI: %d", ReceivedServoNumber, potValue, servoData);

                convertDATAtoPOLU(boardNumber, ReceivedServoNumber, servoData, arrPOLUdata);

                while (!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte(XBEEuart, arrPOLUdata[0]); // Send SSC Servo start character                
                XBEETxBuffer[0] = arrPOLUdata[1];
                XBEETxBuffer[1] = arrPOLUdata[2];
                XBEETxBuffer[2] = arrPOLUdata[3];
                XBEETxBuffer[3] = arrPOLUdata[4];
                XBEETxBuffer[4] = arrPOLUdata[5];
                XBEETxLength = 5;

                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
            }
        }
    }

    if (frameFlag) {
        frameFlag = false;
        if (sentNoteOff == false && !USBHandleBusy(USBTxHandle)) {
            arrMIDIdata[0] = MIDI_CIN_NOTE_ON;
            arrMIDIdata[1] = arrMIDIdata[1] & 0b11101111; // Clear bit #4 to change 0x90 ON to 0x80 OFF command
            USBTxHandle = USBTxOnePacket(MIDI_EP, arrMIDIdata, 4);
            sentNoteOff = true;
        } else {
            short rawValue = (short) (ADresult[0]);
            short windowValue = windowFilter(rawValue);
            ADpotValue = lowPassFilter(windowValue);
            ADpotValue = rawValue;
            

            if (previousEncoderCounter != EncoderCounter) {
                previousEncoderCounter = EncoderCounter;
                lngPotValue = (long) (EncoderCounter * scaleValue);
                if (lngPotValue >= MAXPOTVALUE) lngPotValue = MAXPOTVALUE;
                else if (lngPotValue < 0) lngPotValue = 0;
                potValue = (short) lngPotValue;
            }

            ConfigAd();
            
            /*
            if (previousADPotValue != ADpotValue){
                previousADPotValue = ADpotValue;
                printf("\rAD Pot #1: %d", ADpotValue);
            }
            */
    
            // if (abs(previousPotValue - potValue) > 0) {
            if (previousPotValue != potValue) {
                previousPotValue = potValue;
                mLED_2_On();
                if (mode == RECORD && !USBHandleBusy(USBTxHandle)) {
                    convertDATAtoMIDI(boardNumber, servoNumber, (unsigned short) potValue, arrMIDIdata);
                    USBTxHandle = USBTxOnePacket(MIDI_EP, arrMIDIdata, 4);
                    sentNoteOff = false;
                } else if (mode == STANDBY && XBEETxLength == 0) {
                    convertDATAtoPOLU(boardNumber, servoNumber, (unsigned short) potValue, arrPOLUdata);

                    while (!UARTTransmitterIsReady(XBEEuart));
                    UARTSendDataByte(XBEEuart, arrPOLUdata[0]); // Send SSC Servo start character
                    XBEETxBuffer[0] = arrPOLUdata[1];
                    XBEETxBuffer[1] = arrPOLUdata[2];
                    XBEETxBuffer[2] = arrPOLUdata[3];
                    XBEETxBuffer[3] = arrPOLUdata[4];
                    XBEETxBuffer[4] = arrPOLUdata[5];
                    XBEETxLength = 5;

                    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                    if (displayMode) printf("\r#%d %d", servoNumber, potValue);
                }
            } else mLED_2_Off();
        } // end else
    } // end if (frameFlag)
}//end ProcessUSB

void ProcessMIDI(void) {
    static short potValue = 0;
    static short previousPotValue = 0;
    static unsigned char LEDflag = true;
    static unsigned short LEDcounter = 0;
    static unsigned char lowOut = 0, highOut = 0;
    unsigned short outServoValue = 0;
    static unsigned short MIDIdataPointer = 0;
    static unsigned short MIDIdataIndex = 0;
    static unsigned char sentNoteOff = true;
    static unsigned char velocity = 0;

    static unsigned char arrPOLUdata[8];
    static unsigned char arrMIDIdata[8];
    unsigned short servoData = 0;
    unsigned char ReceivedServoNumber;



    if (MIDIbufferFull) {
        MIDIbufferFull = false;

        MIDIdataPointer = MIDIStateMachine();
        while (MIDIdataIndex != MIDIdataPointer) {

            if (mode != STANDBY && XBEETxLength == 0) {
                if (displayMode) printf("\rCH%d POT: %d => MIDI: %d", arrServo[MIDIdataIndex].ID, potValue, arrServo[MIDIdataIndex].position);

                convertDATAtoPOLU(boardNumber, arrServo[MIDIdataIndex].ID, arrServo[MIDIdataIndex].position, arrPOLUdata);
                while (!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte(XBEEuart, arrPOLUdata[0]); // Send SSC Servo start character        
                XBEETxBuffer[0] = arrPOLUdata[1];
                XBEETxBuffer[1] = arrPOLUdata[2];
                XBEETxBuffer[2] = arrPOLUdata[3];
                XBEETxBuffer[3] = arrPOLUdata[4];
                XBEETxBuffer[4] = arrPOLUdata[5];
                XBEETxLength = 5;

                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);

                MIDIdataIndex++;
                if (MIDIdataIndex >= MAXBUFFER) MIDIdataIndex = 0;
            }
        }
    } // end if (MIDIbufferFull)

    if (frameFlag) {
        frameFlag = false;
        if (MIDITxLength == 0) {
            if (sentNoteOff == false) {
                while (!UARTTransmitterIsReady(MIDIuart));
                // UARTSendDataByte(MIDIuart, 0x80 | (servoNumber - 1));
                UARTSendDataByte(MIDIuart, 0x80 | servoNumber);
                MIDITxLength = 2;
                INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_ENABLED);
                sentNoteOff = true;
            } else {
                short rawValue = (short) (ADresult[0]);
                short windowValue = windowFilter(rawValue);
                potValue = lowPassFilter(windowValue);
                if (potValue > MAXPOTVALUE) potValue = MAXPOTVALUE;
                ConfigAd();

                if (abs(previousPotValue - potValue) > 4) {
                    previousPotValue = potValue;
                    mLED_2_On();
                    if (mode == RECORD) {
                        convertDATAtoMIDI(boardNumber, servoNumber, (unsigned short) potValue, arrMIDIdata);
                        MIDITxBuffer[0] = arrMIDIdata[2];
                        MIDITxBuffer[1] = arrMIDIdata[3];
                        MIDITxLength = 2;
                        while (!UARTTransmitterIsReady(MIDIuart));
                        // UARTSendDataByte(MIDIuart, 0x90 | (servoNumber - 1));
                        UARTSendDataByte(MIDIuart, 0x90 | servoNumber);
                        INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_ENABLED);
                        sentNoteOff = false;
                    } else if (mode == STANDBY && XBEETxLength == 0) {
                        convertDATAtoPOLU(boardNumber, servoNumber, (unsigned short) potValue, arrPOLUdata);

                        while (!UARTTransmitterIsReady(XBEEuart));
                        UARTSendDataByte(XBEEuart, arrPOLUdata[0]); // Send SSC Servo start character

                        XBEETxBuffer[0] = arrPOLUdata[1];
                        XBEETxBuffer[1] = arrPOLUdata[2];
                        XBEETxBuffer[2] = arrPOLUdata[3];
                        XBEETxBuffer[3] = arrPOLUdata[4];
                        XBEETxBuffer[4] = arrPOLUdata[5];
                        XBEETxLength = 5;

                        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                        if (displayMode) printf("\r#%d %d", servoNumber, potValue);
                    }
                } // end else
            } // end else
        } // end if (MIDITxLength == 0)
    }// end if (frameFlag)
} // end ProcessMIDI()

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    unsigned short PORTin;
    static unsigned short previousPORTin = 0x0000;


    // Step #1 - always clear the mismatch condition first
    PORTin = PORTC & 0x6000;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    if ((previousPORTin == 0x0000) && (PORTin == 0x4000))
        EncoderCounter++;
    else if ((previousPORTin == 0x4000) && (PORTin == 0x6000))
        EncoderCounter++;
    else if ((previousPORTin == 0x6000) && (PORTin == 0x2000))
        EncoderCounter++;
    else if ((previousPORTin == 0x2000) && (PORTin == 0x0000))
        EncoderCounter++;
    else EncoderCounter--;

    previousPORTin = PORTin;

    if (EncoderCounter > MAXPOTVALUE) EncoderCounter = MAXPOTVALUE;
    else if (EncoderCounter < 0) EncoderCounter = 0;

}


// To store 10 bit AD result as MIDI data:
// xxxxxx98 76543210 => x9876543 210xxxxx

unsigned char convertDATAtoMIDI(unsigned char boardNumber, unsigned char servoNumber, unsigned short intTenBit, unsigned char *arrMIDIdata) {
    unsigned char MIDIhigh, MIDIlow;

    if (arrMIDIdata == NULL) return (false);
    if (servoNumber > MAXSERVO) return (false);

    convert.integer = intTenBit << 2;
    MIDIhigh = convert.highByte + 12; // Make sure MSB of both bytes is zero
    MIDIlow = (convert.lowByte >> 1) | 0x01;

    arrMIDIdata[0] = MIDI_CIN_NOTE_ON;
    arrMIDIdata[1] = 0x90 | (boardNumber - 1);
    arrMIDIdata[2] = MIDIhigh | (servoNumber << 4);
    arrMIDIdata[3] = MIDIlow;

    return (true);
}

unsigned char convertMIDItoData(unsigned char *boardNumber, unsigned char *arrMIDIdata, unsigned short *servoData, unsigned char *servoNumber) {
    unsigned char arrMIDI0, arrMIDI1, arrMIDI2, arrMIDI3;

    if ((arrMIDIdata[1] & 0xF0) != 0x90)
        return (false);

    arrMIDI0 = arrMIDIdata[0];
    arrMIDI1 = arrMIDIdata[1];
    *boardNumber = (arrMIDI1 & 0x0F) + 1;
    arrMIDI2 = arrMIDIdata[2] & 0x0F;
    *servoNumber = (arrMIDIdata[2] & 0xF0) >> 4;
    arrMIDI3 = arrMIDIdata[3];

    // *servoNumber = (arrMIDI1 & 0x0F) + 1;

    convert.highByte = arrMIDI2;
    convert.lowByte = arrMIDI3 << 1;

    *servoData = convert.integer >> 2;
    return (true);
}

/*
#define SERVO_PULSE_OFFSET 93  // was 375
unsigned char convertDATAtoPOLU(unsigned char boardNumber, unsigned char servoNumber, unsigned short tenBitValue, unsigned char *arrPOLUdata) {
    unsigned char POLUlow, POLUhigh;
    unsigned long offsetValue;
    unsigned long lngValue;

    if (arrPOLUdata == NULL) return (false);
    if (servoNumber == 255) return (false);

    offsetValue = (unsigned long) (tenBitValue + SERVO_PULSE_OFFSET);

    // lngValue = (offsetValue * 64) / 5;
    lngValue = (offsetValue * 256) / 5;
    if (lngValue > 0xFFFF) lngValue = 0xFFFF;
    convert.integer = (unsigned short) lngValue;

    // convert.integer = (offsetValue * 64) / 5;
    POLUhigh = convert.highByte; // to make high byte a multiple of 128 
    POLUlow = (convert.lowByte >> 1) & 0b01111111; // Shift low byte down one to clear MSB

    // Now assemble Pololu command string:
    arrPOLUdata[0] = 0x84;
    arrPOLUdata[1] = servoNumber - 1;
    arrPOLUdata[2] = POLUlow;
    arrPOLUdata[3] = POLUhigh;

    return (true);
}
 */

#define SERVO_PULSE_OFFSET 93  

unsigned char convertDATAtoPOLU(unsigned char boardNumber, unsigned char servoNumber, unsigned short tenBitValue, unsigned char *arrPOLUdata) {
    unsigned char POLUlow, POLUhigh;
    unsigned long offsetValue;
    unsigned long lngValue;

    if (arrPOLUdata == NULL) return (false);
    if (servoNumber == 255) return (false);
    if (boardNumber > MAXBOARD) return (false);

    offsetValue = (unsigned long) (tenBitValue + SERVO_PULSE_OFFSET);

    // lngValue = (offsetValue * 64) / 5;
    lngValue = (offsetValue * 256) / 5;
    if (lngValue > 0xFFFF) lngValue = 0xFFFF;
    convert.integer = (unsigned short) lngValue;

    // convert.integer = (offsetValue * 64) / 5;
    POLUhigh = convert.highByte; // to make high byte a multiple of 128 
    POLUlow = (convert.lowByte >> 1) & 0b01111111; // Shift low byte down one to clear MSB

    // Now assemble Pololu command string:
    arrPOLUdata[0] = 0xAA; // Start character 0x84;
    arrPOLUdata[1] = boardNumber;
    arrPOLUdata[2] = 0x04; // Command
    arrPOLUdata[3] = servoNumber;
    arrPOLUdata[4] = POLUlow;
    arrPOLUdata[5] = POLUhigh;

    return (true);
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


    PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    mCNOpen(CN_ON, CN0_ENABLE | CN1_ENABLE, CN0_PULLUP_ENABLE | CN1_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);


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

    // Set up MIDI at 31250 baud
    UARTConfigure(MIDIuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MIDIuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MIDIuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(MIDIuart, SYS_FREQ, 31250);
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI Interrupts
    INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up DMX512 UART @ 25000 baud   	
	UARTConfigure(DMXuart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);   
	UARTSetFifoMode(DMXuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);  	  	
	UARTSetLineControl(DMXuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);	
	UARTSetDataRate(DMXuart, SYS_FREQ, 250000);  
    UARTEnable(DMXuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
   	
	// Configure DMX interrupts
	INTEnable(INT_SOURCE_UART_RX(DMXuart), INT_ENABLED);
	INTEnable(INT_SOURCE_UART_TX(DMXuart), INT_DISABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(DMXuart), INT_PRIORITY_LEVEL_1);
   	INTSetVectorSubPriority(INT_VECTOR_UART(DMXuart), INT_SUB_PRIORITY_LEVEL_0);  
    
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    PORTClearBits(IOPORT_G, BIT_0);  // RS485 control pin should be set to receive
    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTSetBits(IOPORT_B, BIT_2);
    
    ConfigAd();

    // Set up Timer 2 for 100 microsecond roll-over rate
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);
    
    
    // Set up DMX Timer 3 for 44 Hz roll-over rate to start with:
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, 7102);
    // set up the core timer interrupt with a priority of 3 and zero sub-priority
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);    

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit




void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) {
    static unsigned short frameDelay = 0;

    mT2ClearIntFlag(); // clear the interrupt flag

    milliSecondCounter++;
    frameDelay++;
    if (frameDelay >= FRAME_DELAY) {
        frameDelay = 0;
        frameFlag = true;
    }

    if (MIDItimeout) {
        MIDItimeout--;
        if (!MIDItimeout) MIDIbufferFull = true;
    }
}

void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void) {

    mT3ClearIntFlag();

    if (DMXstate == DMX_STANDBY) {
        DMXstate = DMX_BREAK; // This is the beginning of the BREAK
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 9600);
        PORTClearBits(IOPORT_B, BIT_2);
    } else if (DMXstate == DMX_BREAK) { // MARK before transmitting data:
        DMXstate = DMX_MARK;
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 960);
        PORTSetBits(IOPORT_B, BIT_2); // Release transmit line, let it go high to mark end of break
    } else if (DMXstate == DMX_MARK) {
        DMXstate = DMX_DATA;
        INTEnable(INT_U1TX, INT_ENABLED);
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, 7102);
        while (!UARTTransmitterIsReady(DMXuart));
        UARTSendDataByte(DMXuart, 0x00);            // Send start byte
    } else {
        DMXstate = DMX_BREAK; // This is the beginning of the BREAK
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 9600);
        PORTClearBits(IOPORT_D, BIT_2);
    }
}

//	DMX512 interrupt handler for receiving and transmitting DMX512 data
// 	Priority level 1
void __ISR(DMX_VECTOR, ipl1) IntDMXHandler(void){
static unsigned short 	DMXTxPtr=0;
static unsigned short	DMXRxPtr=0;
static unsigned char 	dummyCounter=1;
unsigned char 			ch, dummy;
	
	// RX interrupts
	if(INTGetFlag(INT_SOURCE_UART_RX(DMXuart))){
		// Clear the RX interrupt Flag
	   INTClearFlag(INT_SOURCE_UART_RX(DMXuart));
	   
   		if (DMXbits.OERR)				// If DMX overrun occurs, clear overrun flag
				DMXbits.OERR=0;										   
	   
		if (DMXbits.FERR){					
   			dummy=UARTGetDataByte(DMXuart); 	
 			dummyCounter=1;
			DMXRxPtr=0;																									
		}			
		else if (UARTReceivedDataIsAvailable(DMXuart)){
			ch=UARTGetDataByte(DMXuart);	
			if(dummyCounter)
				dummyCounter--;			
			else if (DMXRxPtr<DMXLENGTH){	
				DMXRxBuffer[DMXRxPtr]=ch;			 
				DMXRxPtr++;
				if (DMXRxPtr==DMXLENGTH){
					DMXflag=true;					
				}					
			}	
		}			
	}	
	// TX interrupts: 
	if (INTGetFlag(INT_SOURCE_UART_TX(DMXuart))){
		INTClearFlag(INT_SOURCE_UART_TX(DMXuart));        
		if (DMXstate>=DMX_START){
			if (DMXTxPtr<DMXLENGTH){
				if (DMXstate==DMX_START){			
					ch=0x00;
					DMXstate++;
				}
				else {
					ch=DMXTxBuffer[DMXTxPtr];	
					DMXTxPtr++;
				}
				while (!UARTTransmitterIsReady(DMXuart));
				UARTSendDataByte(DMXuart, ch);								
			}									
			else {
				DMXstate=DMX_STANDBY;	
				INTEnable(INT_SOURCE_UART_TX(DMXuart), INT_DISABLED);
				DMXTxPtr=0;					
			}		
		}			        
	}		
}

/** EOF main.c *************************************************/
#endif

