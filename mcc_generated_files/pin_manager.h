/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.26.1
        Device            :  PIC16F15385
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set A aliases
#define A_TRIS               TRISAbits.TRISA0
#define A_LAT                LATAbits.LATA0
#define A_PORT               PORTAbits.RA0
#define A_WPU                WPUAbits.WPUA0
#define A_OD                ODCONAbits.ODCA0
#define A_ANS                ANSELAbits.ANSA0
#define A_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define A_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define A_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define A_GetValue()           PORTAbits.RA0
#define A_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define A_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define A_SetPullup()      do { WPUAbits.WPUA0 = 1; } while(0)
#define A_ResetPullup()    do { WPUAbits.WPUA0 = 0; } while(0)
#define A_SetPushPull()    do { ODCONAbits.ODCA0 = 1; } while(0)
#define A_SetOpenDrain()   do { ODCONAbits.ODCA0 = 0; } while(0)
#define A_SetAnalogMode()  do { ANSELAbits.ANSA0 = 1; } while(0)
#define A_SetDigitalMode() do { ANSELAbits.ANSA0 = 0; } while(0)

// get/set B aliases
#define B_TRIS               TRISAbits.TRISA1
#define B_LAT                LATAbits.LATA1
#define B_PORT               PORTAbits.RA1
#define B_WPU                WPUAbits.WPUA1
#define B_OD                ODCONAbits.ODCA1
#define B_ANS                ANSELAbits.ANSA1
#define B_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define B_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define B_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define B_GetValue()           PORTAbits.RA1
#define B_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define B_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define B_SetPullup()      do { WPUAbits.WPUA1 = 1; } while(0)
#define B_ResetPullup()    do { WPUAbits.WPUA1 = 0; } while(0)
#define B_SetPushPull()    do { ODCONAbits.ODCA1 = 1; } while(0)
#define B_SetOpenDrain()   do { ODCONAbits.ODCA1 = 0; } while(0)
#define B_SetAnalogMode()  do { ANSELAbits.ANSA1 = 1; } while(0)
#define B_SetDigitalMode() do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set C aliases
#define C_TRIS               TRISAbits.TRISA2
#define C_LAT                LATAbits.LATA2
#define C_PORT               PORTAbits.RA2
#define C_WPU                WPUAbits.WPUA2
#define C_OD                ODCONAbits.ODCA2
#define C_ANS                ANSELAbits.ANSA2
#define C_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define C_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define C_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define C_GetValue()           PORTAbits.RA2
#define C_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define C_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define C_SetPullup()      do { WPUAbits.WPUA2 = 1; } while(0)
#define C_ResetPullup()    do { WPUAbits.WPUA2 = 0; } while(0)
#define C_SetPushPull()    do { ODCONAbits.ODCA2 = 1; } while(0)
#define C_SetOpenDrain()   do { ODCONAbits.ODCA2 = 0; } while(0)
#define C_SetAnalogMode()  do { ANSELAbits.ANSA2 = 1; } while(0)
#define C_SetDigitalMode() do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set D aliases
#define D_TRIS               TRISAbits.TRISA3
#define D_LAT                LATAbits.LATA3
#define D_PORT               PORTAbits.RA3
#define D_WPU                WPUAbits.WPUA3
#define D_OD                ODCONAbits.ODCA3
#define D_ANS                ANSELAbits.ANSA3
#define D_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define D_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define D_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define D_GetValue()           PORTAbits.RA3
#define D_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define D_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define D_SetPullup()      do { WPUAbits.WPUA3 = 1; } while(0)
#define D_ResetPullup()    do { WPUAbits.WPUA3 = 0; } while(0)
#define D_SetPushPull()    do { ODCONAbits.ODCA3 = 1; } while(0)
#define D_SetOpenDrain()   do { ODCONAbits.ODCA3 = 0; } while(0)
#define D_SetAnalogMode()  do { ANSELAbits.ANSA3 = 1; } while(0)
#define D_SetDigitalMode() do { ANSELAbits.ANSA3 = 0; } while(0)

// get/set E aliases
#define E_TRIS               TRISAbits.TRISA4
#define E_LAT                LATAbits.LATA4
#define E_PORT               PORTAbits.RA4
#define E_WPU                WPUAbits.WPUA4
#define E_OD                ODCONAbits.ODCA4
#define E_ANS                ANSELAbits.ANSA4
#define E_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define E_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define E_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define E_GetValue()           PORTAbits.RA4
#define E_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define E_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define E_SetPullup()      do { WPUAbits.WPUA4 = 1; } while(0)
#define E_ResetPullup()    do { WPUAbits.WPUA4 = 0; } while(0)
#define E_SetPushPull()    do { ODCONAbits.ODCA4 = 1; } while(0)
#define E_SetOpenDrain()   do { ODCONAbits.ODCA4 = 0; } while(0)
#define E_SetAnalogMode()  do { ANSELAbits.ANSA4 = 1; } while(0)
#define E_SetDigitalMode() do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set F aliases
#define F_TRIS               TRISAbits.TRISA5
#define F_LAT                LATAbits.LATA5
#define F_PORT               PORTAbits.RA5
#define F_WPU                WPUAbits.WPUA5
#define F_OD                ODCONAbits.ODCA5
#define F_ANS                ANSELAbits.ANSA5
#define F_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define F_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define F_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define F_GetValue()           PORTAbits.RA5
#define F_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define F_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define F_SetPullup()      do { WPUAbits.WPUA5 = 1; } while(0)
#define F_ResetPullup()    do { WPUAbits.WPUA5 = 0; } while(0)
#define F_SetPushPull()    do { ODCONAbits.ODCA5 = 1; } while(0)
#define F_SetOpenDrain()   do { ODCONAbits.ODCA5 = 0; } while(0)
#define F_SetAnalogMode()  do { ANSELAbits.ANSA5 = 1; } while(0)
#define F_SetDigitalMode() do { ANSELAbits.ANSA5 = 0; } while(0)

// get/set G aliases
#define G_TRIS               TRISAbits.TRISA6
#define G_LAT                LATAbits.LATA6
#define G_PORT               PORTAbits.RA6
#define G_WPU                WPUAbits.WPUA6
#define G_OD                ODCONAbits.ODCA6
#define G_ANS                ANSELAbits.ANSA6
#define G_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define G_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define G_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define G_GetValue()           PORTAbits.RA6
#define G_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define G_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define G_SetPullup()      do { WPUAbits.WPUA6 = 1; } while(0)
#define G_ResetPullup()    do { WPUAbits.WPUA6 = 0; } while(0)
#define G_SetPushPull()    do { ODCONAbits.ODCA6 = 1; } while(0)
#define G_SetOpenDrain()   do { ODCONAbits.ODCA6 = 0; } while(0)
#define G_SetAnalogMode()  do { ANSELAbits.ANSA6 = 1; } while(0)
#define G_SetDigitalMode() do { ANSELAbits.ANSA6 = 0; } while(0)

// get/set DP aliases
#define DP_TRIS               TRISAbits.TRISA7
#define DP_LAT                LATAbits.LATA7
#define DP_PORT               PORTAbits.RA7
#define DP_WPU                WPUAbits.WPUA7
#define DP_OD                ODCONAbits.ODCA7
#define DP_ANS                ANSELAbits.ANSA7
#define DP_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define DP_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define DP_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define DP_GetValue()           PORTAbits.RA7
#define DP_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define DP_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define DP_SetPullup()      do { WPUAbits.WPUA7 = 1; } while(0)
#define DP_ResetPullup()    do { WPUAbits.WPUA7 = 0; } while(0)
#define DP_SetPushPull()    do { ODCONAbits.ODCA7 = 1; } while(0)
#define DP_SetOpenDrain()   do { ODCONAbits.ODCA7 = 0; } while(0)
#define DP_SetAnalogMode()  do { ANSELAbits.ANSA7 = 1; } while(0)
#define DP_SetDigitalMode() do { ANSELAbits.ANSA7 = 0; } while(0)

// get/set ZCD aliases
#define ZCD_TRIS               TRISBbits.TRISB0
#define ZCD_LAT                LATBbits.LATB0
#define ZCD_PORT               PORTBbits.RB0
#define ZCD_WPU                WPUBbits.WPUB0
#define ZCD_OD                ODCONBbits.ODCB0
#define ZCD_ANS                ANSELBbits.ANSB0
#define ZCD_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define ZCD_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define ZCD_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define ZCD_GetValue()           PORTBbits.RB0
#define ZCD_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define ZCD_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define ZCD_SetPullup()      do { WPUBbits.WPUB0 = 1; } while(0)
#define ZCD_ResetPullup()    do { WPUBbits.WPUB0 = 0; } while(0)
#define ZCD_SetPushPull()    do { ODCONBbits.ODCB0 = 1; } while(0)
#define ZCD_SetOpenDrain()   do { ODCONBbits.ODCB0 = 0; } while(0)
#define ZCD_SetAnalogMode()  do { ANSELBbits.ANSB0 = 1; } while(0)
#define ZCD_SetDigitalMode() do { ANSELBbits.ANSB0 = 0; } while(0)

// get/set SSR aliases
#define SSR_TRIS               TRISBbits.TRISB1
#define SSR_LAT                LATBbits.LATB1
#define SSR_PORT               PORTBbits.RB1
#define SSR_WPU                WPUBbits.WPUB1
#define SSR_OD                ODCONBbits.ODCB1
#define SSR_ANS                ANSELBbits.ANSB1
#define SSR_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define SSR_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define SSR_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define SSR_GetValue()           PORTBbits.RB1
#define SSR_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define SSR_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define SSR_SetPullup()      do { WPUBbits.WPUB1 = 1; } while(0)
#define SSR_ResetPullup()    do { WPUBbits.WPUB1 = 0; } while(0)
#define SSR_SetPushPull()    do { ODCONBbits.ODCB1 = 1; } while(0)
#define SSR_SetOpenDrain()   do { ODCONBbits.ODCB1 = 0; } while(0)
#define SSR_SetAnalogMode()  do { ANSELBbits.ANSB1 = 1; } while(0)
#define SSR_SetDigitalMode() do { ANSELBbits.ANSB1 = 0; } while(0)

// get/set SET aliases
#define SET_TRIS               TRISBbits.TRISB2
#define SET_LAT                LATBbits.LATB2
#define SET_PORT               PORTBbits.RB2
#define SET_WPU                WPUBbits.WPUB2
#define SET_OD                ODCONBbits.ODCB2
#define SET_ANS                ANSELBbits.ANSB2
#define SET_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define SET_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define SET_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define SET_GetValue()           PORTBbits.RB2
#define SET_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define SET_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define SET_SetPullup()      do { WPUBbits.WPUB2 = 1; } while(0)
#define SET_ResetPullup()    do { WPUBbits.WPUB2 = 0; } while(0)
#define SET_SetPushPull()    do { ODCONBbits.ODCB2 = 1; } while(0)
#define SET_SetOpenDrain()   do { ODCONBbits.ODCB2 = 0; } while(0)
#define SET_SetAnalogMode()  do { ANSELBbits.ANSB2 = 1; } while(0)
#define SET_SetDigitalMode() do { ANSELBbits.ANSB2 = 0; } while(0)

// get/set UP aliases
#define UP_TRIS               TRISBbits.TRISB3
#define UP_LAT                LATBbits.LATB3
#define UP_PORT               PORTBbits.RB3
#define UP_WPU                WPUBbits.WPUB3
#define UP_OD                ODCONBbits.ODCB3
#define UP_ANS                ANSELBbits.ANSB3
#define UP_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define UP_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define UP_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define UP_GetValue()           PORTBbits.RB3
#define UP_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define UP_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define UP_SetPullup()      do { WPUBbits.WPUB3 = 1; } while(0)
#define UP_ResetPullup()    do { WPUBbits.WPUB3 = 0; } while(0)
#define UP_SetPushPull()    do { ODCONBbits.ODCB3 = 1; } while(0)
#define UP_SetOpenDrain()   do { ODCONBbits.ODCB3 = 0; } while(0)
#define UP_SetAnalogMode()  do { ANSELBbits.ANSB3 = 1; } while(0)
#define UP_SetDigitalMode() do { ANSELBbits.ANSB3 = 0; } while(0)

// get/set DOWN aliases
#define DOWN_TRIS               TRISBbits.TRISB4
#define DOWN_LAT                LATBbits.LATB4
#define DOWN_PORT               PORTBbits.RB4
#define DOWN_WPU                WPUBbits.WPUB4
#define DOWN_OD                ODCONBbits.ODCB4
#define DOWN_ANS                ANSELBbits.ANSB4
#define DOWN_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define DOWN_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define DOWN_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define DOWN_GetValue()           PORTBbits.RB4
#define DOWN_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define DOWN_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define DOWN_SetPullup()      do { WPUBbits.WPUB4 = 1; } while(0)
#define DOWN_ResetPullup()    do { WPUBbits.WPUB4 = 0; } while(0)
#define DOWN_SetPushPull()    do { ODCONBbits.ODCB4 = 1; } while(0)
#define DOWN_SetOpenDrain()   do { ODCONBbits.ODCB4 = 0; } while(0)
#define DOWN_SetAnalogMode()  do { ANSELBbits.ANSB4 = 1; } while(0)
#define DOWN_SetDigitalMode() do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set LED aliases
#define LED_TRIS               TRISBbits.TRISB5
#define LED_LAT                LATBbits.LATB5
#define LED_PORT               PORTBbits.RB5
#define LED_WPU                WPUBbits.WPUB5
#define LED_OD                ODCONBbits.ODCB5
#define LED_ANS                ANSELBbits.ANSB5
#define LED_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define LED_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define LED_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define LED_GetValue()           PORTBbits.RB5
#define LED_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define LED_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define LED_SetPullup()      do { WPUBbits.WPUB5 = 1; } while(0)
#define LED_ResetPullup()    do { WPUBbits.WPUB5 = 0; } while(0)
#define LED_SetPushPull()    do { ODCONBbits.ODCB5 = 1; } while(0)
#define LED_SetOpenDrain()   do { ODCONBbits.ODCB5 = 0; } while(0)
#define LED_SetAnalogMode()  do { ANSELBbits.ANSB5 = 1; } while(0)
#define LED_SetDigitalMode() do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set REF_PWM aliases
#define REF_PWM_TRIS               TRISCbits.TRISC0
#define REF_PWM_LAT                LATCbits.LATC0
#define REF_PWM_PORT               PORTCbits.RC0
#define REF_PWM_WPU                WPUCbits.WPUC0
#define REF_PWM_OD                ODCONCbits.ODCC0
#define REF_PWM_ANS                ANSELCbits.ANSC0
#define REF_PWM_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define REF_PWM_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define REF_PWM_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define REF_PWM_GetValue()           PORTCbits.RC0
#define REF_PWM_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define REF_PWM_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define REF_PWM_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define REF_PWM_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define REF_PWM_SetPushPull()    do { ODCONCbits.ODCC0 = 1; } while(0)
#define REF_PWM_SetOpenDrain()   do { ODCONCbits.ODCC0 = 0; } while(0)
#define REF_PWM_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define REF_PWM_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set Relay aliases
#define Relay_TRIS               TRISCbits.TRISC1
#define Relay_LAT                LATCbits.LATC1
#define Relay_PORT               PORTCbits.RC1
#define Relay_WPU                WPUCbits.WPUC1
#define Relay_OD                ODCONCbits.ODCC1
#define Relay_ANS                ANSELCbits.ANSC1
#define Relay_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define Relay_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define Relay_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define Relay_GetValue()           PORTCbits.RC1
#define Relay_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define Relay_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define Relay_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define Relay_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define Relay_SetPushPull()    do { ODCONCbits.ODCC1 = 1; } while(0)
#define Relay_SetOpenDrain()   do { ODCONCbits.ODCC1 = 0; } while(0)
#define Relay_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define Relay_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set RC2 procedures
#define RC2_SetHigh()    do { LATCbits.LATC2 = 1; } while(0)
#define RC2_SetLow()   do { LATCbits.LATC2 = 0; } while(0)
#define RC2_Toggle()   do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define RC2_GetValue()         PORTCbits.RC2
#define RC2_SetDigitalInput()   do { TRISCbits.TRISC2 = 1; } while(0)
#define RC2_SetDigitalOutput()  do { TRISCbits.TRISC2 = 0; } while(0)
#define RC2_SetPullup()     do { WPUCbits.WPUC2 = 1; } while(0)
#define RC2_ResetPullup()   do { WPUCbits.WPUC2 = 0; } while(0)
#define RC2_SetAnalogMode() do { ANSELCbits.ANSC2 = 1; } while(0)
#define RC2_SetDigitalMode()do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set SCL1 aliases
#define SCL1_TRIS               TRISCbits.TRISC3
#define SCL1_LAT                LATCbits.LATC3
#define SCL1_PORT               PORTCbits.RC3
#define SCL1_WPU                WPUCbits.WPUC3
#define SCL1_OD                ODCONCbits.ODCC3
#define SCL1_ANS                ANSELCbits.ANSC3
#define SCL1_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define SCL1_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define SCL1_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define SCL1_GetValue()           PORTCbits.RC3
#define SCL1_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define SCL1_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define SCL1_SetPullup()      do { WPUCbits.WPUC3 = 1; } while(0)
#define SCL1_ResetPullup()    do { WPUCbits.WPUC3 = 0; } while(0)
#define SCL1_SetPushPull()    do { ODCONCbits.ODCC3 = 1; } while(0)
#define SCL1_SetOpenDrain()   do { ODCONCbits.ODCC3 = 0; } while(0)
#define SCL1_SetAnalogMode()  do { ANSELCbits.ANSC3 = 1; } while(0)
#define SCL1_SetDigitalMode() do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set SDA1 aliases
#define SDA1_TRIS               TRISCbits.TRISC4
#define SDA1_LAT                LATCbits.LATC4
#define SDA1_PORT               PORTCbits.RC4
#define SDA1_WPU                WPUCbits.WPUC4
#define SDA1_OD                ODCONCbits.ODCC4
#define SDA1_ANS                ANSELCbits.ANSC4
#define SDA1_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define SDA1_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define SDA1_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define SDA1_GetValue()           PORTCbits.RC4
#define SDA1_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define SDA1_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define SDA1_SetPullup()      do { WPUCbits.WPUC4 = 1; } while(0)
#define SDA1_ResetPullup()    do { WPUCbits.WPUC4 = 0; } while(0)
#define SDA1_SetPushPull()    do { ODCONCbits.ODCC4 = 1; } while(0)
#define SDA1_SetOpenDrain()   do { ODCONCbits.ODCC4 = 0; } while(0)
#define SDA1_SetAnalogMode()  do { ANSELCbits.ANSC4 = 1; } while(0)
#define SDA1_SetDigitalMode() do { ANSELCbits.ANSC4 = 0; } while(0)

// get/set D4_1 aliases
#define D4_1_TRIS               TRISDbits.TRISD0
#define D4_1_LAT                LATDbits.LATD0
#define D4_1_PORT               PORTDbits.RD0
#define D4_1_WPU                WPUDbits.WPUD0
#define D4_1_OD                ODCONDbits.ODCD0
#define D4_1_ANS                ANSELDbits.ANSD0
#define D4_1_SetHigh()            do { LATDbits.LATD0 = 1; } while(0)
#define D4_1_SetLow()             do { LATDbits.LATD0 = 0; } while(0)
#define D4_1_Toggle()             do { LATDbits.LATD0 = ~LATDbits.LATD0; } while(0)
#define D4_1_GetValue()           PORTDbits.RD0
#define D4_1_SetDigitalInput()    do { TRISDbits.TRISD0 = 1; } while(0)
#define D4_1_SetDigitalOutput()   do { TRISDbits.TRISD0 = 0; } while(0)
#define D4_1_SetPullup()      do { WPUDbits.WPUD0 = 1; } while(0)
#define D4_1_ResetPullup()    do { WPUDbits.WPUD0 = 0; } while(0)
#define D4_1_SetPushPull()    do { ODCONDbits.ODCD0 = 1; } while(0)
#define D4_1_SetOpenDrain()   do { ODCONDbits.ODCD0 = 0; } while(0)
#define D4_1_SetAnalogMode()  do { ANSELDbits.ANSD0 = 1; } while(0)
#define D4_1_SetDigitalMode() do { ANSELDbits.ANSD0 = 0; } while(0)

// get/set D3_1 aliases
#define D3_1_TRIS               TRISDbits.TRISD1
#define D3_1_LAT                LATDbits.LATD1
#define D3_1_PORT               PORTDbits.RD1
#define D3_1_WPU                WPUDbits.WPUD1
#define D3_1_OD                ODCONDbits.ODCD1
#define D3_1_ANS                ANSELDbits.ANSD1
#define D3_1_SetHigh()            do { LATDbits.LATD1 = 1; } while(0)
#define D3_1_SetLow()             do { LATDbits.LATD1 = 0; } while(0)
#define D3_1_Toggle()             do { LATDbits.LATD1 = ~LATDbits.LATD1; } while(0)
#define D3_1_GetValue()           PORTDbits.RD1
#define D3_1_SetDigitalInput()    do { TRISDbits.TRISD1 = 1; } while(0)
#define D3_1_SetDigitalOutput()   do { TRISDbits.TRISD1 = 0; } while(0)
#define D3_1_SetPullup()      do { WPUDbits.WPUD1 = 1; } while(0)
#define D3_1_ResetPullup()    do { WPUDbits.WPUD1 = 0; } while(0)
#define D3_1_SetPushPull()    do { ODCONDbits.ODCD1 = 1; } while(0)
#define D3_1_SetOpenDrain()   do { ODCONDbits.ODCD1 = 0; } while(0)
#define D3_1_SetAnalogMode()  do { ANSELDbits.ANSD1 = 1; } while(0)
#define D3_1_SetDigitalMode() do { ANSELDbits.ANSD1 = 0; } while(0)

// get/set D2_1 aliases
#define D2_1_TRIS               TRISDbits.TRISD2
#define D2_1_LAT                LATDbits.LATD2
#define D2_1_PORT               PORTDbits.RD2
#define D2_1_WPU                WPUDbits.WPUD2
#define D2_1_OD                ODCONDbits.ODCD2
#define D2_1_ANS                ANSELDbits.ANSD2
#define D2_1_SetHigh()            do { LATDbits.LATD2 = 1; } while(0)
#define D2_1_SetLow()             do { LATDbits.LATD2 = 0; } while(0)
#define D2_1_Toggle()             do { LATDbits.LATD2 = ~LATDbits.LATD2; } while(0)
#define D2_1_GetValue()           PORTDbits.RD2
#define D2_1_SetDigitalInput()    do { TRISDbits.TRISD2 = 1; } while(0)
#define D2_1_SetDigitalOutput()   do { TRISDbits.TRISD2 = 0; } while(0)
#define D2_1_SetPullup()      do { WPUDbits.WPUD2 = 1; } while(0)
#define D2_1_ResetPullup()    do { WPUDbits.WPUD2 = 0; } while(0)
#define D2_1_SetPushPull()    do { ODCONDbits.ODCD2 = 1; } while(0)
#define D2_1_SetOpenDrain()   do { ODCONDbits.ODCD2 = 0; } while(0)
#define D2_1_SetAnalogMode()  do { ANSELDbits.ANSD2 = 1; } while(0)
#define D2_1_SetDigitalMode() do { ANSELDbits.ANSD2 = 0; } while(0)

// get/set D1_1 aliases
#define D1_1_TRIS               TRISDbits.TRISD3
#define D1_1_LAT                LATDbits.LATD3
#define D1_1_PORT               PORTDbits.RD3
#define D1_1_WPU                WPUDbits.WPUD3
#define D1_1_OD                ODCONDbits.ODCD3
#define D1_1_ANS                ANSELDbits.ANSD3
#define D1_1_SetHigh()            do { LATDbits.LATD3 = 1; } while(0)
#define D1_1_SetLow()             do { LATDbits.LATD3 = 0; } while(0)
#define D1_1_Toggle()             do { LATDbits.LATD3 = ~LATDbits.LATD3; } while(0)
#define D1_1_GetValue()           PORTDbits.RD3
#define D1_1_SetDigitalInput()    do { TRISDbits.TRISD3 = 1; } while(0)
#define D1_1_SetDigitalOutput()   do { TRISDbits.TRISD3 = 0; } while(0)
#define D1_1_SetPullup()      do { WPUDbits.WPUD3 = 1; } while(0)
#define D1_1_ResetPullup()    do { WPUDbits.WPUD3 = 0; } while(0)
#define D1_1_SetPushPull()    do { ODCONDbits.ODCD3 = 1; } while(0)
#define D1_1_SetOpenDrain()   do { ODCONDbits.ODCD3 = 0; } while(0)
#define D1_1_SetAnalogMode()  do { ANSELDbits.ANSD3 = 1; } while(0)
#define D1_1_SetDigitalMode() do { ANSELDbits.ANSD3 = 0; } while(0)

// get/set D4_2 aliases
#define D4_2_TRIS               TRISDbits.TRISD4
#define D4_2_LAT                LATDbits.LATD4
#define D4_2_PORT               PORTDbits.RD4
#define D4_2_WPU                WPUDbits.WPUD4
#define D4_2_OD                ODCONDbits.ODCD4
#define D4_2_ANS                ANSELDbits.ANSD4
#define D4_2_SetHigh()            do { LATDbits.LATD4 = 1; } while(0)
#define D4_2_SetLow()             do { LATDbits.LATD4 = 0; } while(0)
#define D4_2_Toggle()             do { LATDbits.LATD4 = ~LATDbits.LATD4; } while(0)
#define D4_2_GetValue()           PORTDbits.RD4
#define D4_2_SetDigitalInput()    do { TRISDbits.TRISD4 = 1; } while(0)
#define D4_2_SetDigitalOutput()   do { TRISDbits.TRISD4 = 0; } while(0)
#define D4_2_SetPullup()      do { WPUDbits.WPUD4 = 1; } while(0)
#define D4_2_ResetPullup()    do { WPUDbits.WPUD4 = 0; } while(0)
#define D4_2_SetPushPull()    do { ODCONDbits.ODCD4 = 1; } while(0)
#define D4_2_SetOpenDrain()   do { ODCONDbits.ODCD4 = 0; } while(0)
#define D4_2_SetAnalogMode()  do { ANSELDbits.ANSD4 = 1; } while(0)
#define D4_2_SetDigitalMode() do { ANSELDbits.ANSD4 = 0; } while(0)

// get/set D3_2 aliases
#define D3_2_TRIS               TRISDbits.TRISD5
#define D3_2_LAT                LATDbits.LATD5
#define D3_2_PORT               PORTDbits.RD5
#define D3_2_WPU                WPUDbits.WPUD5
#define D3_2_OD                ODCONDbits.ODCD5
#define D3_2_ANS                ANSELDbits.ANSD5
#define D3_2_SetHigh()            do { LATDbits.LATD5 = 1; } while(0)
#define D3_2_SetLow()             do { LATDbits.LATD5 = 0; } while(0)
#define D3_2_Toggle()             do { LATDbits.LATD5 = ~LATDbits.LATD5; } while(0)
#define D3_2_GetValue()           PORTDbits.RD5
#define D3_2_SetDigitalInput()    do { TRISDbits.TRISD5 = 1; } while(0)
#define D3_2_SetDigitalOutput()   do { TRISDbits.TRISD5 = 0; } while(0)
#define D3_2_SetPullup()      do { WPUDbits.WPUD5 = 1; } while(0)
#define D3_2_ResetPullup()    do { WPUDbits.WPUD5 = 0; } while(0)
#define D3_2_SetPushPull()    do { ODCONDbits.ODCD5 = 1; } while(0)
#define D3_2_SetOpenDrain()   do { ODCONDbits.ODCD5 = 0; } while(0)
#define D3_2_SetAnalogMode()  do { ANSELDbits.ANSD5 = 1; } while(0)
#define D3_2_SetDigitalMode() do { ANSELDbits.ANSD5 = 0; } while(0)

// get/set D2_2 aliases
#define D2_2_TRIS               TRISDbits.TRISD6
#define D2_2_LAT                LATDbits.LATD6
#define D2_2_PORT               PORTDbits.RD6
#define D2_2_WPU                WPUDbits.WPUD6
#define D2_2_OD                ODCONDbits.ODCD6
#define D2_2_ANS                ANSELDbits.ANSD6
#define D2_2_SetHigh()            do { LATDbits.LATD6 = 1; } while(0)
#define D2_2_SetLow()             do { LATDbits.LATD6 = 0; } while(0)
#define D2_2_Toggle()             do { LATDbits.LATD6 = ~LATDbits.LATD6; } while(0)
#define D2_2_GetValue()           PORTDbits.RD6
#define D2_2_SetDigitalInput()    do { TRISDbits.TRISD6 = 1; } while(0)
#define D2_2_SetDigitalOutput()   do { TRISDbits.TRISD6 = 0; } while(0)
#define D2_2_SetPullup()      do { WPUDbits.WPUD6 = 1; } while(0)
#define D2_2_ResetPullup()    do { WPUDbits.WPUD6 = 0; } while(0)
#define D2_2_SetPushPull()    do { ODCONDbits.ODCD6 = 1; } while(0)
#define D2_2_SetOpenDrain()   do { ODCONDbits.ODCD6 = 0; } while(0)
#define D2_2_SetAnalogMode()  do { ANSELDbits.ANSD6 = 1; } while(0)
#define D2_2_SetDigitalMode() do { ANSELDbits.ANSD6 = 0; } while(0)

// get/set D1_2 aliases
#define D1_2_TRIS               TRISDbits.TRISD7
#define D1_2_LAT                LATDbits.LATD7
#define D1_2_PORT               PORTDbits.RD7
#define D1_2_WPU                WPUDbits.WPUD7
#define D1_2_OD                ODCONDbits.ODCD7
#define D1_2_ANS                ANSELDbits.ANSD7
#define D1_2_SetHigh()            do { LATDbits.LATD7 = 1; } while(0)
#define D1_2_SetLow()             do { LATDbits.LATD7 = 0; } while(0)
#define D1_2_Toggle()             do { LATDbits.LATD7 = ~LATDbits.LATD7; } while(0)
#define D1_2_GetValue()           PORTDbits.RD7
#define D1_2_SetDigitalInput()    do { TRISDbits.TRISD7 = 1; } while(0)
#define D1_2_SetDigitalOutput()   do { TRISDbits.TRISD7 = 0; } while(0)
#define D1_2_SetPullup()      do { WPUDbits.WPUD7 = 1; } while(0)
#define D1_2_ResetPullup()    do { WPUDbits.WPUD7 = 0; } while(0)
#define D1_2_SetPushPull()    do { ODCONDbits.ODCD7 = 1; } while(0)
#define D1_2_SetOpenDrain()   do { ODCONDbits.ODCD7 = 0; } while(0)
#define D1_2_SetAnalogMode()  do { ANSELDbits.ANSD7 = 1; } while(0)
#define D1_2_SetDigitalMode() do { ANSELDbits.ANSD7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/