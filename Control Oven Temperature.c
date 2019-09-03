/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16F15385
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"
#include "MCP9600.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
/*
                         Main application
 */

unsigned int refPWM;
unsigned char Status_ZCD;
unsigned int Counter_ZCD,Counter_ZCD_OFF;
int Duty_SSR;
uint8_t  BufSetTempH,BufSetTempL;
int32_t BufCal;
char Toggle_SSR,Cal_5S,firs,Move_Setpoint;
uint8_t ChackStart,ChackDiff;
uint8_t FlashHoleData;
//==============================================================================
//============== Value PIC Controller ==========================================
//==============================================================================
volatile union{
    struct
    {
        int Ref;
        int FB;
        int Error;
        int Error_1;
        int Kp;
        int Ki;
        int Kc;
        int Kd;
        int Up;
        int Up1;
        int Ui;
        int Uc;
        int Ud;
        int Ud_1;
        int OutPreSat;
        int Out;
        int Max;
        int Min;
        int SatErr;
        
    };
}PIControl;
//==============================================================================
volatile union{
    struct
    {
        unsigned int Temp;
        unsigned int Temp_Read;
        unsigned int Temp_Read_1;
        unsigned int Set_Temp;
        unsigned int Set_Temp_Slide;
        unsigned char Temp_one;
        unsigned char Temp_two;
        unsigned char Temp_three;
        unsigned char Set_one;
        unsigned char Set_two;
        unsigned char Set_three;
        unsigned int CounterTime;
        unsigned int CounterDigit;
        unsigned char FlagShow;
        
    };
}Show7Seg;

volatile union{
    struct
    {
        unsigned char Timer10ms;
        unsigned char Flag10ms;
        unsigned char Timer50ms;
        unsigned char Flag50ms;
        unsigned char Timer100ms;
        unsigned char Flag100ms;
        unsigned char Timer500ms;
        unsigned char Flag500ms;
        unsigned char Timer1s;
        unsigned char Flag1s;
    };
}CounterTimer;

volatile union{
    struct
    {
        unsigned int DebounceUP;
        unsigned int DebounceUP2S;
        unsigned char FlagUp;
        unsigned int DebounceDown;
        unsigned char FlagDown;
        unsigned int DebounceSet;
        unsigned char FlagSet;
        unsigned char Counter_FlagSet;
    };
}ChackSW;


//==============================================================================
//== Prototype Function ========================================================
void interrupt INTERRUPT_InterruptManager (void);
unsigned char ConverData2Hex (unsigned char data);
void Analyze_Data(void);
void Data_to_7Segmen(char digit);
void ChackSWPush(void);
//==============================================================================

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:
    
    
    
    //==========================================================================
    BufSetTempH = DATAEE_ReadByte(0x1FFE);
    BufSetTempL = DATAEE_ReadByte(0x1FFF);
    Show7Seg.Set_Temp = (BufSetTempH<<8)|(BufSetTempL);
    if(Show7Seg.Set_Temp>1000){Show7Seg.Set_Temp = 0;}
    //==========================================================================
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    Analyze_Data();
    refPWM = ADC_GetConversion(REF_PWM);
    refPWM = refPWM>>2;
    PWM1_LoadDutyValue(refPWM);
    
    Duty_SSR = 0;
    
    PIControl.Kp = 128; //10.6
    PIControl.Ki = 5; //1.15
    PIControl.Kc = 0; //10.6
    PIControl.Kd = 0; //10.6
    
    PIControl.Max = 32767;
    PIControl.Min = 4096;
    ChackStart = 1;
    
    Show7Seg.Set_Temp = 100;
    
    FlashHoleData = 0;
    while (1)
    {
        ChackSWPush();
        if(CounterTimer.Flag100ms)
        {
            CounterTimer.Flag100ms = 0;                  
            refPWM = ADC_GetConversion(REF_PWM);
            refPWM = refPWM*0.389;
            PWM1_LoadDutyValue(refPWM);   
        }
        
        if(Cal_5S>=1)
        {
            Cal_5S = 0;
            //==================================================================
            PIControl.Ref = Show7Seg.Set_Temp*64;  // F10.6
            PIControl.FB = Show7Seg.Temp*64;       // F10.6
                  
            // Compute the error
            PIControl.Error = PIControl.Ref - PIControl.FB;     //F10.6 + F10.6 = F10.6

            // Compute the proportional output
            PIControl.Up = ((uint32_t)PIControl.Kp*(uint32_t)PIControl.Error)>>6;      

            // Compute the integral output
            PIControl.Up1 = (((uint32_t)PIControl.Ki*(uint32_t)PIControl.Error)>>6);     
            PIControl.Ui =  PIControl.Ui + PIControl.Up1;  
            

            // Compute the pre-saturated output
            PIControl.OutPreSat = PIControl.Up + PIControl.Ui ;

            // Saturate the output
            if(PIControl.OutPreSat>PIControl.Max)
            {
                    PIControl.Out = PIControl.Max;
            }
            else if(PIControl.OutPreSat<PIControl.Min)
            {
                PIControl.Out = PIControl.Min;
            }
            else
            {
                PIControl.Out = PIControl.OutPreSat;
            }

            // Calculator Duty for PI Controller
            Duty_SSR = ((uint32_t)PIControl.Out)/328;  
            if(Duty_SSR >= 97)
            {
                Duty_SSR = 97;
            }

            
        }
        

        if(ChackSW.FlagUp)
        {
            ChackSW.FlagUp = 0;
            Show7Seg.Set_Temp++;
            if(Show7Seg.Set_Temp>999){Show7Seg.Set_Temp = 0;} 
        }
        
        if(ChackSW.FlagDown)
        {
            ChackSW.FlagDown = 0;
            Show7Seg.Set_Temp--;
            if(Show7Seg.Set_Temp>999){Show7Seg.Set_Temp = 999;} 
        }
        
       
        

        if(ChackSW.FlagSet)
        {
            ChackSW.FlagSet = 0;
            //==========================================================================
            FLASH_EraseBlock(0x1FFE);
            BufSetTempH = Show7Seg.Set_Temp>>8;
            BufSetTempL = Show7Seg.Set_Temp;
            DATAEE_WriteByte(0x1FFE,BufSetTempH);
            DATAEE_WriteByte(0x1FFF,BufSetTempL);
            //==========================================================================
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
            LED_Toggle();
            __delay_ms(100);
        }
        else
        {
            if((Show7Seg.Temp > Show7Seg.Set_Temp))
            {    
                LED_SetLow();
            }
            else if((Show7Seg.Temp < Show7Seg.Set_Temp))
            {
                LED_SetHigh();
            }
        }
        
        

    }
}
/**
 End of File
*/

void interrupt INTERRUPT_InterruptManager (void)
{
    // interrupt handler
    if(PIE0bits.INTE == 1 && PIR0bits.INTF == 1)
    {
        if(INTCONbits.INTEDG == 1)
        {
            INTCONbits.INTEDG = 0;
        }
        else
        {
            INTCONbits.INTEDG = 1;
        }
        INT_ISR();
        Counter_ZCD = 0;
        Status_ZCD = 1;
    }
    if(PIE0bits.TMR0IE == 1 && PIR0bits.TMR0IF == 1)
    {  
        if(Duty_SSR<=25){Duty_SSR = 0;}
        if(Status_ZCD)
        {
            if((++Counter_ZCD>=(100 - Duty_SSR)))
            {
                SSR_SetHigh();
                if(++Counter_ZCD_OFF>=5)
                {
                    Counter_ZCD = 0;
                    Counter_ZCD_OFF = 0;
                    SSR_SetLow();
                    Status_ZCD = 0;
                }
            }  
        }
        
        //======================================================================
        // Show Data to 7Segment================================================
        //======================================================================
        Show7Seg.CounterDigit++;
        if(Show7Seg.CounterDigit>4){Show7Seg.CounterDigit = 1;}
        Data_to_7Segmen(Show7Seg.CounterDigit);
        //======================================================================
        
        if(++CounterTimer.Timer10ms>=100)
        {
            CounterTimer.Timer10ms = 0;
            CounterTimer.Flag10ms = 1;
            MCP9600_Read(MCP9600_TH,&MCP9600_Buff,2);
            if((MCP9600_Buff[0]&0x80)==0x80)
            {
                Show7Seg.Temp_Read = ((MCP9600_Buff[0]*16)+(MCP9600_Buff[1]>>4))-4096;
            }
            else
            {
                Show7Seg.Temp_Read = ((MCP9600_Buff[0]*16)+(MCP9600_Buff[1]>>4));
            }
            Show7Seg.Temp = (Show7Seg.Temp_Read>>1) + (Show7Seg.Temp_Read_1>>1);
            Show7Seg.Temp_Read_1 = Show7Seg.Temp; 
            if(++CounterTimer.Timer50ms>=5)
            {
                CounterTimer.Timer50ms = 0;
                CounterTimer.Flag50ms = 1;
                if(++CounterTimer.Timer100ms>=2)
                {
                    CounterTimer.Timer100ms = 0;
                    CounterTimer.Flag100ms = 1;  
                    Analyze_Data();
                    
                    if(++CounterTimer.Timer500ms>=5)
                    {
                        CounterTimer.Timer500ms = 0;
                        if(++CounterTimer.Timer1s>=10)
                        {
                            CounterTimer.Timer1s = 0;
                            CounterTimer.Flag1s = 1;
                            Cal_5S++;
                            if(Toggle_SSR)
                            {
                                Toggle_SSR = 0;
                            }
                            else
                            {
                                Toggle_SSR = 1;
                            }
                            
                            
                        }
                    }
                }
            }
            
        }
        TMR0_ISR();
    }
    if(INTCONbits.PEIE == 1 && PIE3bits.SSP1IE == 1 && PIR3bits.SSP1IF == 1)
    {
        I2C1_ISR();
    }
    else if(INTCONbits.PEIE == 1 && PIE3bits.BCL1IE == 1 && PIR3bits.BCL1IF == 1)
    {
        I2C1_BusCollisionISR();
    }
    else
    {
        //Unhandled Interrupt
    }
}

unsigned char ConverData2Hex (unsigned char data)
{
    switch(data)
    {
        case 0:
        {
            return 0xC0;
        }break;
        case 1:
        {
            return 0xF9;
        }break;
        case 2:
        {
            return 0xA4;
        }break;
        case 3:
        {
            return 0xB0;
        }break;
        case 4:
        {
            return 0x99;
        }break;
        case 5:
        {
            return 0x92;
        }break;
        case 6:
        {
            return 0x82;
        }break;
        case 7:
        {
            return 0xF8;
        }break;
        case 8:
        {
            return 0x80;
        }break;
        case 9:
        {
            return 0x98;
        }break;

    }
}

void Analyze_Data(void)
{
    if(FlashHoleData == 0)
    {
        Show7Seg.Temp_three = Show7Seg.Temp/100;
        Show7Seg.Temp_two = (Show7Seg.Temp - (Show7Seg.Temp_three*100))/10;
        Show7Seg.Temp_one = Show7Seg.Temp - ((Show7Seg.Temp_three*100)+(Show7Seg.Temp_two*10));
    }
    else if(FlashHoleData == 1)
    {
        Show7Seg.Temp_three = Show7Seg.Set_three;
        Show7Seg.Temp_two = Show7Seg.Set_two;
        Show7Seg.Temp_one = Show7Seg.Set_one;
    }

    
    Show7Seg.Set_three = Show7Seg.Set_Temp/100;
    Show7Seg.Set_two = (Show7Seg.Set_Temp - (Show7Seg.Set_three*100))/10;
    Show7Seg.Set_one = Show7Seg.Set_Temp - ((Show7Seg.Set_three*100)+(Show7Seg.Set_two*10));
}

void Data_to_7Segmen(char digit)
{
    if(++Show7Seg.CounterTime > 8)  // 50
    {
        Show7Seg.CounterTime = 0;
        Show7Seg.FlagShow = ~Show7Seg.FlagShow;
    }
    
    switch(digit)
    {
        case 1:
        {
            if(Show7Seg.FlagShow)
            {
                PORTD = 0xF7;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Temp_three);
            }
            else
            {
                PORTD = 0x7F;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Set_three);
            }
        }break;
        case 2:
        { 
            if(Show7Seg.FlagShow)
            {
                PORTD = 0xFB;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Temp_two);
            }
            else
            {
                PORTD = 0xBF;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Set_two);
            }
        }break;
        case 3:
        {
            if(Show7Seg.FlagShow)
            {
                PORTD = 0xFD;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Temp_one);
            }
            else
            {
                PORTD = 0xDF;
                PORTA = 0xFF;
                PORTA = ConverData2Hex(Show7Seg.Set_one);
            }
        }break;
        case 4:
        {
            if(Show7Seg.FlagShow)
            {
                PORTD = 0xEF;
                PORTA = 0xFF;
                PORTA = 0xC6;   
            }
            else
            {
                PORTD = 0xFE;
                PORTA = 0xFF;
                PORTA = 0xC6;
            }

        }break;
    }
            
}

void ChackSWPush(void)
{
    //==========================================================================
    ///===== Chack Switch Counter UP ===========================================
    //==========================================================================
    if(!UP_GetValue())
    {
        if(++ChackSW.DebounceUP>10000&&!ChackSW.FlagUp)
        {
            ChackSW.DebounceUP = 0;
            ChackSW.FlagUp = 1;
        }
    }
    else
    {
        if(--ChackSW.DebounceUP<=1)
        {
            ChackSW.DebounceUP = 0;
        }
    }
    //==========================================================================
    ///===== Chack Switch Counter Down =========================================
    //==========================================================================
    if(!DOWN_GetValue())
    {
        if(++ChackSW.DebounceDown>10000&&!ChackSW.FlagDown)
        {
            ChackSW.DebounceDown = 0;
            ChackSW.FlagDown = 1;
        }
    }
    else
    {
        if(--ChackSW.DebounceDown<=1)
        {
            ChackSW.DebounceDown = 0;
        }
    }
    //==========================================================================
    ///===== Chack Switch Counter SET ==========================================
    //==========================================================================
    if(!SET_GetValue())
    {
        if(++ChackSW.DebounceSet>10000&&!ChackSW.FlagSet)
        {
            ChackSW.DebounceSet = 0;
            ChackSW.FlagSet = 1;
        }
    }
    else
    {
        if(--ChackSW.DebounceSet<=1)
        {
            ChackSW.DebounceSet = 0;
        }
    }
    //==========================================================================
}