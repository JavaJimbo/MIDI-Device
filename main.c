/***********************************************************************************************************
 * PROJECT:     MIDI Demo
 * FileName:    main.c
 * 5-28-17:     Stripped all non-PIC795 code.
 * 5-30-17:     Records and plays back on LMMS. Servo number is built in to velocity.
 * 6-1-17:      Fixed NOTE OFF bug. Works well at 16 ms sampling rate with servo 0.
 * 6-1-17:      Assigns channel numbers to servos. Implemented HOST keyboard menu to enable/disable servos.
 * 6-2-17:      Added low pass and window filtering.
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

#define FRAME_DELAY 17
#define MAXFILTER 8
#define MAXWINDOW 5

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

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR

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

unsigned short MIDITxLength = 0;
unsigned char MIDITxBuffer[MAXBUFFER];
unsigned char MIDIRxBuffer[MAXBUFFER];
unsigned short RxIndex = 0;

#define MAXSERVO 11
#define MAXPOTS 1
unsigned short ADresult[MAXPOTS]; // read the result of channel 0 conversion from the idle buffer

void ConfigAd(void);

unsigned char frameFlag = FALSE;
unsigned short milliSecondCounter = 0;

unsigned char runMode = FALSE;
unsigned char displayMode = FALSE;
short servoNumber = 0;

unsigned char controlCommand = 0;
short getInteger(unsigned char *ptrString);
unsigned char getCommand(unsigned char *ptrString);
short windowFilter (short newValue);
unsigned char swap(short *ptrDataA, short *ptrDataB);
unsigned char sortData(short *arrSortData);
short lowPassFilter (short newValue);

#define SET_DISPLAY 4 // CTL-D
#define SET_SERVO 19 // CTL-S
#define SET_RUN 18 // CTL-R
#define NUMBER_ERROR 32767

int main(void) {
    short value = 0;
    unsigned char command = 0;
    short dummy = 0, filter = 0;
    short i = 0;
    unsigned char ch;

    InitializeSystem();
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();

    printf("\r\rTESTING MIDI IO:");

#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif

    while (1) {
        while(i != RxIndex){
            if (i >= MAXBUFFER) i = 0;
            ch = MIDIRxBuffer[i++];
            if (ch & 0x80) printf("\r%X ", ch);
            else printf("%X ", ch);
        }        
            
        if (controlCommand) {
            printf("\rCONTROL: %d", controlCommand);
            switch (controlCommand) {
                case SET_SERVO:
                    runMode = displayMode = FALSE;
                    printf("\rSET SERVO");
                    break;
                case SET_DISPLAY:
                    if (displayMode) {
                        displayMode = FALSE;
                        printf("\rDISPLAY OFF");
                    } else {
                        displayMode = TRUE;
                        printf("\rDISPLAY ON");
                    }
                    break;                    
                case SET_RUN:
                    if (runMode) {
                        runMode = FALSE;
                        printf("\rSTANDBY MODE");
                    } else {
                        runMode = TRUE;
                        printf("\rRUN MODE");
                    }
                    break;
                default:
                    break;
            }
            controlCommand = 0;
        } else if (HOSTRxBufferFull) {
            HOSTRxBufferFull = FALSE;
            if (!runMode){
                command = getCommand(HOSTRxBuffer);
                switch(command){
                    case 'S':
                        value = getInteger(HOSTRxBuffer);
                        if (value == NUMBER_ERROR) printf("\rNo number entered");
                        else if (value < 0 || value > MAXSERVO) printf("\rInvalid Servo");
                        else {
                            servoNumber = (unsigned char) value;
                            printf("\rSERVO: %d", servoNumber);
                        }
                        break;
                    default: printf("\rNO COMMAND");
                        break;
                }
                if (command) {
                    value = getInteger(HOSTRxBuffer);
                    printf("\rValue: %d", value);
                } else printf("\rNO COMMAND");
            }
        }

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

    milliSecondCounter++;
    frameDelay++;
    if (frameDelay >= FRAME_DELAY) {
        frameDelay = 0;
        frameFlag = TRUE;
    }

}

#define SEND_SERVO 0x90
#define NOTE_OFF 0x80

void ProcessIO(void) {
    // static BYTE pitch = 0x3C;
    short potValue = 0;    
    static short previousPotValue = 0;
    static unsigned char sentNoteOff = TRUE;
    static unsigned char LEDflag = TRUE;
    unsigned char MIDIreceivedCommand;
    unsigned char RxServoNumber;    
    static unsigned short LEDcounter = 0;
    static unsigned char lowOut = 0, highOut = 0;
    static unsigned char lowIn = 0, highIn = 0;
    unsigned char inValue;
    static unsigned short outServoValue = 0;

    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (!USBHandleBusy(USBRxHandle)) {
        USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
        LEDcounter++;
        if (LEDcounter > 10) {
            LEDcounter = 0;
            if (LEDflag) {
                mLED_4_On();
                LEDflag = FALSE;
                LATBbits.LATB2 = 1;
            } else {
                mLED_4_Off();
                LEDflag = TRUE;
                LATBbits.LATB2 = 0;
            }
        }

        MIDIreceivedCommand = ReceivedDataBuffer[1] & 0xF0;
        RxServoNumber = (ReceivedDataBuffer[1] & 0x0F);
        
        highIn = ReceivedDataBuffer[2];
        lowIn = ReceivedDataBuffer[3] & 0x01;            
        inValue = ((highIn << 1) & 0xFE) | lowIn;
        if (inValue > 254) inValue--;

        if (MIDIreceivedCommand == 0x90 && ReceivedDataBuffer[3]) {
            if (displayMode) printf("\rCH%d: %X", RxServoNumber, inValue);
            if (XBEETxLength == 0) {
                XBEETxBuffer[0] = RxServoNumber;                
                XBEETxBuffer[1] = inValue; 
                XBEETxLength = 2;
                while (!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte(XBEEuart, 0xFF);
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
            }
        }
    }
    if (frameFlag) {
        frameFlag = FALSE;
        if (sentNoteOff == FALSE && !USBHandleBusy(USBTxHandle)) {
            midiData.Val = 0; // must set all unused values to 0 
            midiData.CableNumber = 0;
            midiData.CodeIndexNumber = MIDI_CIN_NOTE_OFF;
            midiData.DATA_0 = 0x80 | servoNumber; //Note off
            // midiData.DATA_1 = pitch; //pitch
            midiData.DATA_1 = highOut; //pitch
            
            midiData.DATA_2 = 0x60 | (lowOut + 1);
            if (midiData.DATA_2 > 127) midiData.DATA_2 = 0x61;                        
            
            USBTxHandle = USBTxOnePacket(MIDI_EP, (BYTE*) & midiData, 4);
            sentNoteOff = TRUE;
        } else {  
            short rawValue = (short) (ADresult[0]);
            short windowValue = windowFilter (rawValue);
            potValue = lowPassFilter(windowValue);
            ConfigAd();
            outServoValue = (unsigned short) (potValue/4);
            if (outServoValue > 254) outServoValue = 254;
            if (abs(previousPotValue - potValue) > 1) {
                previousPotValue = potValue;
                mLED_2_On();
                if (runMode && !USBHandleBusy(USBTxHandle)) {
                    midiData.Val = 0; //must set all unused values to 0
                    midiData.CableNumber = 0;
                    midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
                    midiData.DATA_0 = 0x90 | servoNumber; //Note on
                    lowOut = outServoValue & 0x01;
                    highOut = (outServoValue >> 1) & 0x007F;
                    midiData.DATA_1 = highOut;
                    midiData.DATA_2 = 0x60 | (lowOut + 1);
                    if (midiData.DATA_2 > 127) midiData.DATA_2 = 0x61;
                    USBTxHandle = USBTxOnePacket(MIDI_EP, (BYTE*) & midiData, 4);
                    sentNoteOff = FALSE;
                }                 
                else if (!runMode && XBEETxLength == 0) {
                    XBEETxBuffer[0] = servoNumber;
                    XBEETxBuffer[1] = (unsigned char) outServoValue;
                    XBEETxLength = 2;
                    while (!UARTTransmitterIsReady(XBEEuart));
                    UARTSendDataByte(XBEEuart, 0xFF); // Send SSC Servo start character
                    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                    if (displayMode) printf("\r%d", potValue);
                }                
            } else mLED_2_Off();             
        } // end else
    } // end if (frameFlag)
}//end ProcessIO

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
                HOSTRxBufferFull = TRUE;
                HOSTRxIndex = 0;
            }
            else if (ch < 27) controlCommand = ch;
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
    unsigned char negativeFlag = FALSE;
    short i = 0, j = 0, value = 32767;

    while (ptrString[i] != NULL && j < MAXNUMLENGTH) {
        ch = ptrString[i];
        if (ch == '\0') {
            strNumber[j] = '\0';
            break;
        }
        if (ch == '-') negativeFlag = TRUE;
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



unsigned char swap(short *ptrDataA, short *ptrDataB){
short tempVal;
    if (ptrDataA == NULL || ptrDataB == NULL) return(FALSE);
    tempVal = *ptrDataA;
    *ptrDataA = *ptrDataB;
    *ptrDataB = tempVal;
    return(TRUE);
}

unsigned char sortData(short *arrSortData){
short i, j; 

    if (arrSortData == NULL) return(FALSE);
    for(i = 0; i < (MAXWINDOW-1); i++){   
        for (j = i+1; j < MAXWINDOW; j++){
            if (arrSortData[i] > arrSortData[j]){
                swap(&arrSortData[i], &arrSortData[j]);
            }
        }       
    }
    return(TRUE);
}

#define CENTERINDEX ((MAXWINDOW-1)/2)

short windowFilter (short newValue){
short centerValue;
static short arrInData[MAXWINDOW]; // = {103, 102, 101, 108, 100, 105, 107, 106, 104};
short arrFilterData[MAXWINDOW];
static unsigned short i = 0, j = 0, k = 0;
static unsigned char startFlag = TRUE;

    if (startFlag){
        startFlag = FALSE;
        for (i = 0; i < MAXWINDOW; i++) arrInData[i] = 0;
        i = 0;
    } 
    if (i >= MAXWINDOW) i = 0;
    arrInData[i++] = newValue;
    
    for (j = 0; j < MAXWINDOW; j++) arrFilterData[j] = arrInData[j];
    if (sortData(arrFilterData) == FALSE) printf("\rERROR");

    // printf("\rSORT ARRAY: ");
    // for (k = 0; k < MAXWINDOW; k++) printf ("%d ", arrFilterData[k]);
    centerValue = arrFilterData[CENTERINDEX];
    return(centerValue);
}

short lowPassFilter (short newValue){
static short arrInData[MAXFILTER]; // = {103, 102, 101, 108, 100, 105, 107, 106, 104};
static unsigned short i = 0, j = 0;
static unsigned char startFlag = TRUE;
short result;
long sum;

    if (startFlag){
        startFlag = FALSE;
        for (i = 0; i < MAXFILTER; i++) arrInData[i] = 0;
        i = 0;
    } 
    if (i >= MAXFILTER) i = 0;
    arrInData[i++] = newValue;
    
    sum = 0;
    for (j = 0; j < MAXFILTER; j++) sum = sum + (long) arrInData[j];

    result = (short) (sum / MAXFILTER);
    return(result);
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
            if (ch != 0xfe){
                if (RxIndex >= MAXBUFFER) RxIndex = 0;
                MIDIRxBuffer[RxIndex++] = ch;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
        if (MIDITxLength) {
            if (TxIndex < MAXBUFFER) ch = MIDITxBuffer[TxIndex++];
            while (!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte(MIDIuart, ch);
            if (TxIndex >= MIDITxLength) {
                INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
                MIDITxLength = 0;
                TxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    }

}

/** EOF main.c *************************************************/
#endif

