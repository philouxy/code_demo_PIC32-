/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
 * ETML-ES 
 *          Auteur : Ph. Bovey (Philou)
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 * 
 * Modification Date : 17.08.2020
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32DriverLcd.h"      // to use the functionality for the LCD 
                                // BC_2004AB
#include "Mc32DriverAdc.h"      // to user the AD Converter Chanel of PIC

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;           // global value using just in this file
                            // and allows to update also the state






// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h. => this function is called in the system_init.c 
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 * 
 * Auteur : - microchip 
 *          - Ph. Bovey (Philou) 
 * 
 * Modification date : le 17.08.2020
 * 
 * Description : 1) Add the new stat in the switch case
 *               2) complete the "init" state, with the initialization of the 
 *                  LCD functions 
 * 
 * version : 0.2 
 ******************************************************************************/
void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            //-- initialization of the LCD --//
            lcd_init(); 
            //-- backlight activation --//
            lcd_bl_on(); 
            
            //-- place the pointer on first character of the LCD dispay--// 
            lcd_gotoxy(1,1); 
            //-- display the user msg one first line --// 
            printf_lcd("TP0 LED + AD "); 
            
            //-- place the pointer on the second line --// 
            lcd_gotoxy(1,2); 
            printf_lcd("CODE BY Philou"); 
            
            //--place the pointer on the third line --// 
            lcd_gotoxy(1,3); 
            printf_lcd("VERSION 0.1");
            
            //-- AD input initialization -> see the Mc32DriverADC.h --// 
            BSP_InitADC10(); 
            
            //-- Light all the LED (LED0 to LED7) with the PLIB --//
            //-- use the Harmony Help : PLIB_PORTS_Write function --//
            //-- writing on the all port => help with schematic => to light the 
            //   led, write 0 => the port A has 16pin  
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_A, 0x7F0C); 
            //-- writing on the one pin => use the harmony help : 
            //   PLIB_PORTS_PinWrite function
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_10,0); 
            
            //-- active the timer1 --// 
            //DRV_TMR0_Start(); 
            
            //-- update of the state machine --// 
            appData.state = APP_STATE_WAIT; 
            
            
            break; 
        }

        /* TODO: implement your application state machine.*/
        case APP_STATE_SERVICE_TASKS:
        {
        
            break;
        }
        
        //-- state without activities --//
        case APP_STATE_WAIT: 
        {
            break; 
        }

       
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            appData.state = APP_STATE_WAIT; 
            break;
        }
    }
}

/*******************************************************************************
 * Function Name                : APP_UpdateStates 
 * IN/OUT/INOUT Parameters      : APP_STATE New / - / -
 * Description                  : Allow to change de state at each call which 
 *                                will be in an another file .c 
 *                                In this file (.c), you must declare a global 
 *                                value just for this file, for this point use 
 *                                the global done for this point "appData"
 * 
 * Creation date                : 17.08.2020
 * Modification date            : xx.xx.xxxx
 * 
 * Auteur                       : Ph. Bovey (philou) 
 * Version                      : 0.1 
*******************************************************************************/
void APP_UpdateStates(APP_STATES newState)
{
    appData.state = newState;    
}

/*******************************************************************************
 End of File
 */
