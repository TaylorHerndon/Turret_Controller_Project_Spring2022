/******************************************************************************/
//                          Nerf Turret Project
//                       Created By: Taylor Herndon
//                         Created On: 02/23/22
/*******************************************************************************
Summary: 
  This project is the controller code for the nerf turret. This program
  controlls 2 servo motors for the x and y axis of the turret, interfaces
  with two speed controllers to control the launcher motor and ball feed motor.
  Additionally, this controller interfaces with another board through UART to
  receive instructions for controlling the turret.
********************************************************************************
Serial Data : UART : Baud = 115.2k
 Byte 0 : Handshake : 0xFE
  
 Byte 1 : Buttons 1 : 0xFF - 0x00 : Holds button data from N64 controller
 * [   7   ] : [   6  ] : [   5  ] : [  4 ] : [   3   ] : [ 2 ] : [ 1 ] : [ 0 ]
 * [ Right ] : [ Left ] : [ Down ] : [ Up ] : [ Start ] : [ Z ] : [ B ] : [ A ]
  
 Byte 2 : Buttons 2 : 0xFF - 0x00 : Holds button data from N64 controller
 * [    7    ] : [    6   ] : [    5   ] : [   4  ] : [ 3 ] : [ 2 ] : [ 1 ] : [ 0 ]
 * [ C-Right ] : [ C-Left ] : [ C-Down ] : [ C-Up ] : [ R ] : [ L ] : [ - ] : [ - ]
  
 Byte 3 : X-Axis : 0xFF - 0x00 : Holds joystick value for X Axis
 * [  7 ] : [  6 ] : [  5 ] : [  4 ] : [  3 ] : [  2 ] : [  1 ] : [ 0 ]
 * [ D1 ] : [ D2 ] : [ D3 ] : [ D4 ] : [ D5 ] : [ D6 ] : [ D7 ] : [ P ]
 * D :   Data   : These are the data bits of the joystick value.
 * P : Polarity : This is the polarity of the data (1 = Negative, 0 = Positive)
 * This byte is similar to a signed byte, only the entire byte is reversed.
 * D7 is indicating the MSB of the data byte. If the byte is reversed the data
 * can be interpreted as a typical signed byte. 
  
  Byte 3 : Y-Axis : 0xFF - 0x00 : Holds joystick value for Y Axis
 * [  7 ] : [  6 ] : [  5 ] : [  4 ] : [  3 ] : [  2 ] : [  1 ] : [ 0 ]
 * [ D1 ] : [ D2 ] : [ D3 ] : [ D4 ] : [ D5 ] : [ D6 ] : [ D7 ] : [ P ]
 * D :   Data   : These are the data bits of the joystick value.
 * P : Polarity : This is the polarity of the data (1 = Negative, 0 = Positive)
 * This byte is similar to a signed byte, only the entire byte is reversed.
 * D7 is indicating the MSB of the data byte. If the byte is reversed the data
 * can be interpreted as a typical signed byte.  
 ******************************************************************************/

//Configuration Bits and XC8 Header File Start

#include <xc.h> //MPLAB XC8 compiler header file
#include <proc/pic16f1788.h> //PIC16F1788 header file

// PIC16F1788 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//Configuration Bits and XC8 Header File End
/******************************************************************************/
//Main Code Start
char pFlag = 0; //Personal flag to process data in main
char homeSequence = 0; //Personal flag to stop processing user input data on home sequence
char RCTemp = 0; //Holds received data byte
char RCIndex = 0; //Holds which byte of the UART data packet is next

char Buttons1, Buttons2, XAxis, YAxis; //Specific received data variables
char YAxisReversed, XAxisReversed; //Variables for joystick value processing

int shootCount = 0; //Holds how long the trigger has been pressed, controls ESC Duty Cycle
char shootPermission = 0; //Variable to allow for the A button to feed ammo
/******************************************************************************/
//Config Start

void configOSC (void)
{
    //Using INT OSC 16MHz, PLL Disabled, PLLMUX Output
    
    OSCCONbits.IRCF0 = 1; //Set ICRF to 1111 to select 16MHZ Internal OSC
    OSCCONbits.IRCF1 = 1; /////
    OSCCONbits.IRCF2 = 1; ////
    OSCCONbits.IRCF3 = 1; ///
    
    OSCCONbits.SCS0 = 0; //Set SCS to use PLLMUX Output
    OSCCONbits.SCS1 = 0; ///
}

void configTMR1(void)
{
    //Fosc/4 & 1:2 Prescaler = 32.768ms / Tick
    T1CONbits.TMR1CS = 0b00; //Use Fosc/4
    T1CONbits.T1CKPS = 0b01; //1:2 Prescaler
    T1GCONbits.TMR1GE = 0;   //Disable gate functions
    
    PIE1bits.TMR1IE = 1; //Enable Timer1 Interrupts
    PIR1bits.TMR1IF = 0; //Clear any unwanted TMR1 flags
    
    T1CONbits.TMR1ON = 1; //Turn TMR1 on
}

void configTMR2(void)
{
    //125ns = Fosc/4, Post = 1:10, Pre = 1:16, PR2 = 250, T/Tick = 5ms
    T2CONbits.T2OUTPS = 0b1001; //Set T2 Post Scaler to 1:10
    T2CONbits.T2CKPS = 0b10;    //Set T2 Pre Scaler to 1:16
    PR2 = 250;
    
    PIR1bits.TMR2IF = 0; //Clear any unwanted TMR2 Flags
    PIE1bits.TMR2IE = 1; //Enable Timer 2 Interrupts
    
    T2CONbits.TMR2ON = 0; //Disable T2 on startup
}

void configPorts(void)
{
    LATB = 0x00; //Clear PORT B outputs on startup
    LATC = 0x00; //Clear PORT C outputs on startup
    
    TRISB = 0b11000000; //Set serial ports as inputs, required
    ANSELB = 0x00;      //Set serial ports as digital IOs
    TRISC = 0x00;       //Set PORTC as all outputs to drive PWM outputs
    ANSELC = 0x00;      //Set PORTC as digital IOs
}

void configInterrupts(void)
{
    INTCONbits.PEIE = 1; //Enable Peripheral Interrupts
    INTCONbits.GIE = 1; //Enable Global Interrupts
}

void configPWM(void)
{
    //0.5ms DC => PSMCxDC = 1000;
    //1.5ms DC => PSMCxDC = 3000;
    //2.5ms DC => PSMCxDC = 5000;
    
    //PWM 1 : PSMC1 Setup, Output A (RC0), 20ms Period, 1.5ms DC
    PSMC1CLKbits.P1CSRC = 0b00; //PSMC clock = Fosc (16MHz)
    PSMC1CLKbits.P1CPRE = 0b11; //Set prescaler = /8
    PSMC1PR = 40000;            //Set period to 20ms
    PSMC1DC = 3000;             //Set period to 1.5ms
    PSMC1STR0bits.P1STRA = 1;   //Enable output on PSMC1A (RC0)
    PSMC1POLbits.P1POLA = 0;    //Polarity is active high
    PSMC1OENbits.P1OEA = 1;     //Activate PSMC1A Output
    PSMC1PRSbits.P1PRST = 1;    //Period event will occur and PSMCxTMR will reset when PSMCxTMR = PSMCxPR
    PSMC1PHSbits.P1PHST = 1;    //PSMCx Rising Edge Event occurs on Time Base match
    PSMC1DCSbits.P1DCST = 1;    //PSMCx Falling Edge Event occurs on Time Base match
    PSMC1CON = 0b11000000;      //Enable PSMC module and load buffer.
    
    //PWM 2 : PSMC4 Setup, Output A (RC3), 20ms Period, 1.5ms DC
    PSMC4CLKbits.P4CSRC = 0b00; //PSMC clock = Fosc (16MHz)
    PSMC4CLKbits.P4CPRE = 0b11; //Set prescaler = /8
    PSMC4PR = 40000;            //Set period to 20ms
    PSMC4DC = 3000;             //Set period to 1.5ms
    PSMC4STR0bits.P4STRA = 1;   //Enable output on PSMC1A (RC0)
    PSMC4POLbits.P4POLA = 0;    //Polarity is active high
    PSMC4OENbits.P4OEA = 1;     //Activate PSMC1A Output
    PSMC4PRSbits.P4PRST = 1;    //Period event will occur and PSMCxTMR will reset when PSMCxTMR = PSMCxPR
    PSMC4PHSbits.P4PHST = 1;    //PSMCx Rising Edge Event occurs on Time Base match
    PSMC4DCSbits.P4DCST = 1;    //PSMCx Falling Edge Event occurs on Time Base match
    PSMC4CON = 0b11000000;      //Enable PSMC module and load buffer.
    
    //PWM 3 : PSMC3 Setup, Output A (RC5), 20ms Period, 1.5ms DC
    PSMC3CLKbits.P3CSRC = 0b00; //PSMC clock = Fosc (16MHz)
    PSMC3CLKbits.P3CPRE = 0b11; //Set prescaler = /8
    PSMC3PR = 40000;            //Set period to 20ms
    PSMC3DC = 3000;             //Set period to 1.5ms
    PSMC3STR0bits.P3STRA = 1;   //Enable output on PSMC1A (RC0)
    PSMC3POLbits.P3POLA = 0;    //Polarity is active high
    PSMC3OENbits.P3OEA = 1;     //Activate PSMC1A Output
    PSMC3PRSbits.P3PRST = 1;    //Period event will occur and PSMCxTMR will reset when PSMCxTMR = PSMCxPR
    PSMC3PHSbits.P3PHST = 1;    //PSMCx Rising Edge Event occurs on Time Base match
    PSMC3DCSbits.P3DCST = 1;    //PSMCx Falling Edge Event occurs on Time Base match
    PSMC3CON = 0b11000000;      //Enable PSMC module and load buffer.
    
    //PWM 4 : PSMC2 Setup, Output A (RC6), 20ms Period, 1.5ms DC
    PSMC2CLKbits.P2CSRC = 0b00; //PSMC clock = Fosc (16MHz)
    PSMC2CLKbits.P2CPRE = 0b11; //Set prescaler = /8
    PSMC2PR = 40000;            //Set period to 20ms
    PSMC2DC = 3000;             //Set period to 1.5ms
    PSMC2STR0bits.P2STRA = 1;   //Enable output on PSMC1A (RC0)
    PSMC2POLbits.P2POLA = 0;    //Polarity is active high
    PSMC2OENbits.P2OEA = 1;     //Activate PSMC1A Output
    PSMC2PRSbits.P2PRST = 1;    //Period event will occur and PSMCxTMR will reset when PSMCxTMR = PSMCxPR
    PSMC2PHSbits.P2PHST = 1;    //PSMCx Rising Edge Event occurs on Time Base match
    PSMC2DCSbits.P2DCST = 1;    //PSMCx Falling Edge Event occurs on Time Base match
    PSMC2CON = 0b11000000;      //Enable PSMC module and load buffer.
}

void configUART(void)
{
    APFCON1bits.TXSEL = 1; //Set TX port on RB6
    APFCON1bits.RXSEL = 1; //Set RX port on RB7
    
    BAUDCONbits.SCKP = 0;  //Normal Polarity
    BAUDCONbits.BRG16 = 1; //Use 16 baud rate generator
    
    RCSTAbits.SPEN = 1; //Enable serial port, configures pins as serial ports
    RCSTAbits.CREN = 1; //Enable continuous data reception (Enables Receiver)
    
    TXSTAbits.TXEN = 0; //Disable UART Transmission (Data is not sent to other board)
    TXSTAbits.SYNC = 0; //Disable Synchronous mode
    TXSTAbits.BRGH = 1; //Enable High baud rate control to achieve 115.2k baud
    
    SPBRG = 34; //Set baud rate to 115.2k baud
    
    PIE1bits.RCIE = 1; //Enable UART receive interrupts
    PIR1bits.RCIF = 0; //Clear unwanted serial receive flags
}

//Config End
/******************************************************************************/
//Functions Start

/*This function is used to reverse the bits in a byte, this is used to interpret
 *the data sent from the N64 interface board. */
unsigned char reverseBits(unsigned char num)
{
    unsigned char  NO_OF_BITS = sizeof(num) * 8;
    unsigned char reverse_num = 0, i, temp;
  
    for (i = 0; i < NO_OF_BITS; i++)
    {
        temp = (num & (1 << i));
        if(temp)
            reverse_num |= (1 << ((NO_OF_BITS - 1) - i));
    }
   
    return reverse_num;
}

void ProcessUART(void)
{
    switch (RCIndex)
    {
        case 0: //Check for handshake (0xFE)
            if(RCTemp == 0xFE) {RCIndex++;}
            break;
        case 1: //Second byte contains the first 8 buttons (Info in header)
            Buttons1 = RCTemp;
            RCIndex++;
            break;
        case 2: //Third byte contains the second 8 buttons (Info in header)
            Buttons2 = RCTemp;
            RCIndex++;
            break;
        case 3: //Fourth byte contains joystick X Axis value (More info in header)
            XAxis = RCTemp;
            RCIndex++;
            break;
        case 4: //Fifth byte contains joystick Y Axis value (More info in header)
            YAxis = RCTemp;
            RCIndex = 0; //Reset count now that all values have been received
            pFlag |= 0b00000010; //Set PFLAG to determine next action to take.
            break;
    }
}

void DetermineAction(void)
{
    /*Bit 00000100 is the trigger button, start increasing the shoot count to
    record how long the trigger button has been pressed, PWM output is
    set in main for the shooter motor. More info on the packet in header*/
    if(Buttons1 & 0b00000100)
    {
        if(shootCount != 65535) {shootCount++;}
    }
    else
    {
        shootCount = 0;
    }
    
    /*If the 'A' button is pressed and the trigger has been pressed for more than
    2 seconds then turn on the feed motor, otherwise set the PWM cycle to 1.5ms*/
    if(shootPermission == 0xFF && Buttons1 & 0b00000001)
    {
        PSMC2DC = 3900; 
        PSMC2CONbits.PSMC2LD = 1;
    }
    else
    {
        PSMC2DC = 3000; 
        PSMC2CONbits.PSMC2LD = 1;
    }
    
    //Move XAxis Servo based on XAxis variable
    XAxisReversed = reverseBits(XAxis) >> 4; //Get the 4 MSBs of the X Axis value
    
    if(XAxisReversed & 0b00001000) //The indicated bit tells if the value is negative
    {
        //Decrease the PSMCDC value by a magnitude determined by the X Axis value.
        //The '<< 1' will change how fast the turret moves.
        PSMC1DC = PSMC1DC + (((~XAxisReversed) & 0b00000111) << 1);
    }
    else //Value is positive
    {
        /*Increase the PSMCDC value by a magnitude determined by the X Axis value.
        The '<< 1' will change how fast the turret moves.*/
        PSMC1DC = PSMC1DC - (XAxisReversed << 1); 
    }
    
    //Limit duty cycle to 2ms and 1ms
    if(PSMC1DC > 4000) {PSMC1DC = 4000;}
    if(PSMC1DC < 2000) {PSMC1DC = 2000;}
    
    PSMC1CONbits.PSMC1LD = 1; //Load the new value into the PSMCDC register
    
    //Move YAxis Servo based on YAxis variable
    YAxisReversed = reverseBits(YAxis) >> 4; //Get the 4 MSBs of the Y Axis value
    if(YAxisReversed & 0b00001000) //Indicated bit tells if the value is negative
    {
        /*Decrease the PSMCDC value by a magnitude determined by the X Axis value.
        The '<< 1' will change how fast the turret moves.*/
        PSMC4DC = PSMC4DC + (((~YAxisReversed) & 0b00000111) << 1);
    }
    else //Value is positive
    {
        /*Increase the PSMCDC value by a magnitude determined by the X Axis value.
        The '<< 1' will change how fast the turret moves.*/
        PSMC4DC = PSMC4DC - (YAxisReversed << 1);
    }
    
    //Limit duty cycle to 2ms and 1ms
    if(PSMC4DC > 4000) {PSMC4DC = 4000;}
    if(PSMC4DC < 2000) {PSMC4DC = 2000;}
    
    PSMC4CONbits.PSMC4LD = 1; //Load the new PSMCDC value to the active register
    
    //If start button was pressed, start home sequence
    if(Buttons1 & 0b00001000) //Indicated bit is the start button on the N64 controller
    {
        homeSequence = 0xFF;  //This stops the program from responding to user input (In Main)
        T2CONbits.TMR2ON = 1; //Activate T2 to move X & Y Servos to home pos
        shootCount = 0; //Stop shooting on home sequence (DC set to 1.5ms in Main)
    }
}
//Functions End
/******************************************************************************/
//Start Interrupts
void TMR2ISR(void)
{
    //Move PSMC1 and PSMC4 closer to 1.5ms duty cycle
    if(PSMC1DC > 3000) {PSMC1DC -= 10;}
    if(PSMC1DC < 3000) {PSMC1DC += 10;}
    if(PSMC4DC > 3000) {PSMC4DC -= 10;}
    if(PSMC4DC < 3000) {PSMC4DC += 10;}
    
    //Lock PSMC1 and PSMC4 to 1.5ms duty cycle if they are close enough
    if(PSMC1DC > 2990 && PSMC1DC < 3010) {PSMC1DC = 3000;}
    if(PSMC4DC > 2990 && PSMC4DC < 3010) {PSMC4DC = 3000;}
    
    PSMC1CONbits.PSMC1LD = 1; //Update PSMC1 to new duty cycle
    PSMC4CONbits.PSMC4LD = 1; //Update PSMC4 to new duty cycle
    
    //If home pos is reached allow user control again and turn of T2
    if(PSMC1DC == 3000 && PSMC4DC == 3000) {homeSequence = 0x00; T2CONbits.TMR2ON = 0;}
    
    PIR1bits.TMR2IF = 0;
}

void RC1ISR(void)
{
    RCTemp = RCREG; //Save received data to variable, this also clears the RC flag
    pFlag |= 0b00000001; //Set personal flag to process the received data
    TMR1 = 0; //Clear TMR1 count now that UART has been received
    LATB = 0x00; //Indicate that UART Data has been received
}

void TMR1ISR(void)
{
    PIR1bits.TMR1IF = 0; //Clear flag
    LATB = 0xFF; //Indicate that UART Communication has been lost
    
    //Clear control variables to stop robot operation with communication loss
    shootCount = 0x00;
    Buttons1 = 0;
    Buttons2 = 0;
    XAxis = 0;
    YAxis = 0;
}

void __interrupt() ISR(void)
{
    if(PIR1bits.TMR2IF == 1) {TMR2ISR();} //Handle home sequence interrupts
    if(PIR1bits.RCIF == 1) {RC1ISR();} //Handle received data interrupts
    if(PIR1bits.TMR1IF == 1) {TMR1ISR();} //Handle UART COMM loss interrupts 
}
//End Interrupts
/******************************************************************************/
int main(void) //Main Program Loop
{
    //Configure modules and interrupts
    configOSC();
    configPorts();
    configUART();
    configPWM();
    configTMR1();
    configTMR2();
    configInterrupts();
    
    while (1)
    {
        if(pFlag & 0b00000001) //UART Data received personal flag
        {
            ProcessUART();
            pFlag ^= 0b00000001; //Clear received data personal flag
        }
        
        //Determine next action if pFlag is high and home sequence is not active
        if(pFlag & 0b00000010 && homeSequence == 0x00) //Determine action personal flag
        {
            DetermineAction();
            pFlag ^= 0b00000010; //Clear determine action personal flag
        }
     
        /*The top PSMCDC value will change the duty cycle value when the trigger
         * is pressed (1.95ms here). The lower value will change the duty cycle
         * when the trigger is released or UART communication has been lost. (1.5ms)*/
        
        //If trigger has been pressed for more than 0s, start flywheel motor (PWM3)
        if(shootCount > 0) {PSMC3DC = 3900; PSMC3CONbits.PSMC3LD = 1;}
        else {PSMC3DC = 3000; PSMC3CONbits.PSMC3LD = 1;}
        
        //If trigger has been pressed for more than 2s, start feed motor (PWM4)
        if(shootCount > 400) {shootPermission = 0xFF;}
        else {shootPermission = 0x00; PSMC2DC = 3000; PSMC2CONbits.PSMC2LD = 1;}
    }
}

//Main Code End
/******************************************************************************/