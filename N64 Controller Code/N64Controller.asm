;====Kendall Callister Dog Trainer Box==========================================

    LIST  P=16F1788

;====PIC CONFIG=================================================================
    
    #include "p16f1788.inc"

; CONFIG1
; __config 0xC9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xDEFF
 __CONFIG _CONFIG2, _WRT_OFF & _VCAPEN_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

 ;====END PIC CONFIG============================================================
 
 ;====VARIABLES=================================================================
ZEROS	EQU 0X20    ;Counter to send the needed zeros for a poll
ONES	EQU 0X21    ;Counter to send the needed ones for poll
REGFULL	EQU 0X22    ;Variable to check if a byte has been filled
	
HANDSHAKE EQU	0X4F	
BUTTON1	EQU 0X50    ;Holds A,B,Z,Start,Up,Down,Left,and Right button controls
BUTTON2 EQU 0X51    ;Holds L,R,C-Up,C-Down,C-Left,and C-Right Button controls
XAXIS	EQU 0X52    ;Holds X-Axis data of the joystick
YAXIS	EQU 0X53    ;Holds Y-Axis data of the joystick
 ;====END VARIABLES=============================================================
 
 ;====MEMORY ORG================================================================
 	    ORG	    0X00	
	    GOTO    SETUP	;Setup Vector 0X00
	    ORG	    0X004	
	    GOTO    INTERRUPT	;Interrupt Vector 0X04
	    GOTO    MAINBEGIN	;Main Vector 0X05
 ;====END MEMORY ORG============================================================
 
 ;====SETUP=====================================================================
 SETUP
 ;========= OSCILLATOR SETUP ===================================================
    BANKSEL OSCCON	    ;INTERNAL OSCILATOR CONFIGURED TO 32MHZ
    BSF	    OSCCON,SPLLEN   ;/      
    BSF	    OSCCON,IRCF3    ;/
    BSF	    OSCCON,IRCF2    ;/
    BSF	    OSCCON,IRCF1    ;/
    BSF	    OSCCON,IRCF0    ;/
    BCF	    OSCCON,SCS1	    ;/
    BCF	    OSCCON,SCS0	    ;/
;========= OSCILLATOR SETUP ====================================================
    
    BANKSEL BUTTON1	    ;Clear Data Byte
    CLRF    BUTTON1	    ;////
    CLRF    BUTTON2	    ;///
    CLRF    XAXIS	    ;//
    CLRF    YAXIS	    ;/
    BANKSEL HANDSHAKE
    MOVLW   0XFE	    ;Set Handshake
    MOVWF   HANDSHAKE	    ;/
    
    BANKSEL PORTB
    CLRF    PORTB	    ;Clear Ports
    CLRF    PORTC	    ;/
    BANKSEL TRISC
    MOVLW   0X00	    ;Set Pin C3 as input and all other pin outputs
    MOVWF   TRISC	    ;/
    CLRF    TRISB	    ;Set PortB as Digital outputs
    BANKSEL ANSELC
    CLRF    ANSELC	    ;Set PortC as Digital I/O
    CLRF    ANSELB	    ;Set PortB as Digital outputs
    BANKSEL IOCCN
    BSF	    IOCCN,0	    ;Interrupt on Change negative transistion PortC pin0
    BANKSEL PORTC
    BSF	    PORTC,0	    ;Set Data Line to default High State
    
    BANKSEL APFCON1
    BCF	    APFCON1,TXSEL
    BCF	    APFCON1,RXSEL
    
;======== EUSART SETUP =========================================================
    BANKSEL RCSTA
    BSF	    RCSTA,SPEN
    BCF	    RCSTA,RX9
    BCF	    RCSTA,CREN
    BANKSEL TXSTA
    BCF	    TXSTA,TX9
    BSF	    TXSTA,TXEN
    BCF	    TXSTA,SYNC
    BSF	    TXSTA,BRGH
    BANKSEL BAUDCON
    BSF	    BAUDCON,BRG16
    BANKSEL SPBRGL
    MOVLW   D'68'
    MOVWF   SPBRGL
    BANKSEL SPBRGH
    CLRF    SPBRGH  
    
;======== EUSART SETUP END======================================================
    
    
;======== TIMER 2 SETUP ========================================================
    BANKSEL PIE1	    ;SELECT BANK  FOR PERIPHERAL INT REGISTER
    BSF	    PIE1,TMR2IE	    ;TURN ON TIMER 2 INTERRUPT ENABLE
    BANKSEL T2CON	    ;SELECT T2CON FOR If TIMER 2 CONTROLS
    BCF	    T2CON,T2OUTPS3  ;POST SCALER SET TO 8
    BSF	    T2CON,T2OUTPS2  ;///
    BCF	    T2CON,T2OUTPS1  ;//
    BCF	    T2CON,T2OUTPS0  ;/
    BSF	    T2CON,T2CKPS1   ;PRESCALER SET TO 1
    BSF	    T2CON,T2CKPS0   ;/	
    BANKSEL PR2		    ;SELECT BANK FOR PR2
    MOVLW   D'125'	    ;250 LOADED INTO PR2 SO INTERRUPT
    MOVWF   PR2		    ;OCURRS WHEN TIMER2 EQUALS 250 	
    BANKSEL PIR1	    ;SELECT BANK FOR PERIPHIAL INTERRUPT REG
    BCF	    PIR1,TMR2IF	    ;CLEAR TIMER 2 INTERRUPT FLAG	
;======== END TIMER 2 SETUP ====================================================
    
;====INTERRUPT SETUP============================================================
    BANKSEL T2CON		;Select Bank 0
    BSF	    T2CON,TMR2ON	;TURN ON TIMER2
    BSF	    INTCON,IOCIE	;Enable interrupt on change interrupts
    BSF	    INTCON,PEIE	    ;Enable Interrupts
    BSF	    INTCON,GIE	    ;/
;====INTERRUPT SETUP END========================================================
    
    GOTO    MAINBEGIN	    ;Go to Main
;====SETUP END==================================================================
    
;====INTERRUPT==================================================================
INTERRUPT    
    BANKSEL IOCCF
    BTFSC   IOCCF,0	    ;Check to see if PortC had a negative transition
    GOTO    READ	    ;Read the input     Note GOTO is used to get out of interrupt faster by using RETFIE
    BANKSEL PIR1	    
    BTFSC   PIR1,TMR2IF	    ;Check if timer has overflowed
    GOTO    TIMERRESET	    ;Reset timer and call a poll	Note GOTO is used to get out of interrupt faster by using RETFIE
    BTFSC   PIR1,TXIF
    CALL    TRANSMIT
    RETFIE
;====INTERRUPT END==============================================================
    
;====Timer Reset================================================================
TIMERRESET
    BCF	    PIR1,TMR2IF	;Clear Timer2 Flag
    BANKSEL PORTC
    CALL    POLL	;Asks the controller for data
    BANKSEL TRISC
    BSF	    TRISC,0	;Change pin to input
    RETFIE
    
    
;====POLL=======================================================================
POLL
    BANKSEL TRISC	
    BCF	    TRISC,0	;Take Control of the Data Line
    BANKSEL PORTC
    BSF	    PORTC,0
    MOVLW   D'7'	;Start Bit plus 6 Zero bits
    MOVWF   ZEROS	;/
    MOVLW   D'2'	;2 One bits
    MOVWF   ONES	;/
    BANKSEL FSR0L_SHAD
    MOVLW   LOW(BUTTON1)
    MOVWF   FSR0L_SHAD	;Indirect Address Button1 Write to Shadow Register so data isn't lost
    BANKSEL REGFULL
    MOVLW   0X08
    MOVWF   REGFULL	;Reset Counter for Byte
    
ZERO
    NOP			;Timing Control
    NOP
    NOP
    NOP
    BCF	    PORTC,0	;Start Pulse Space 3탎
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BSF	    PORTC,0	;Start Pulse Width 1탎
    DECFSZ  ZEROS,1	;Count down zeros
    GOTO    ZERO	;Repeat zeros until they have all been sent
    NOP
    NOP
    NOP
    NOP
    NOP
    
ONE
    BCF	    PORTC,0	;Start Pulse Space 1탎
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    BSF	    PORTC,0	;Start Pulse Width 3탎
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    DECFSZ  ONES,1	;Count Down ones
    GOTO    ONE		;Repeat Ones until done
    RETURN
;====POLL END===================================================================
    
;====READ CONTROLLER============================================================
READ
    BCF	    IOCCF,0	    ;Test pulse at 2 탎
    NOP			    ;//
    BANKSEL PORTC	    ;/
    LSRF    PORTC,0	    ;Place PortC bit 0 into carry bit
    RRF	    INDF0,1	    ;Shift the Carry Bit into MSB Data Byte
    DECFSZ  REGFULL,1	    ;Check to see if the Byte has been filled
    RETFIE		    ;If byte is not full return to main this needs to be fast
    MOVLW   0X08	    ;Reset bit counter
    MOVWF   REGFULL	    ;/
    BANKSEL FSR0L_SHAD
    INCF    FSR0L_SHAD,1    ;Increment indirect addressing byte to shadow register so data isn't lost
    RETFIE		
;====READ CONTROLLER END========================================================
    
;====TRANSMIT===================================================================
TRANSMIT
    BANKSEL TXREG	    
    MOVF    INDF0,0	    ;Move Handshake or Data Byte into Tx Reg to transmit
    MOVWF   TXREG	    ;/
    BANKSEL FSR0L_SHAD	    
    INCF    FSR0L_SHAD,1    ;Select the next byte of data to be addressed
    BANKSEL PIR1
    BCF	    PIR1,TXIF	    ;Clear the TXIF FLAG
    RETURN
    
;====TRANSMIT END===============================================================
    
;====XORTX======================================================================
XORTX
    BANKSEL HANDSHAKE
    MOVLW   0XFE	    ;Set Handshake
    MOVWF   HANDSHAKE	    ;/
    BANKSEL PIE1
    MOVLW   LOW(HANDSHAKE)  ;Set the indirect addressing register to Handshake
    MOVWF   FSR0L	    ;/
    MOVLW   0X10	    ;Enable TXIF or Disable TXIF
    XORWF   PIE1,1	    ;/
    RETURN
      
;====XORTX END================================================================== 
    
;====MAIN=======================================================================
MAINBEGIN
    BANKSEL PORTC
    MOVF    YAXIS,0	    ;Move recieved data to PortB LEDs
    MOVWF   PORTB	    ;/
    MOVLW   D'1'	    ;Runs a test to see if the all the data bytes have been
    ADDLW   LOW(YAXIS)	    ;filled or read.
    SUBWF   FSR0L,0	    ;//
    BTFSC   STATUS,C	    ;/
    CALL    XORTX	    ;A sub that will start or ends a transmit.
    GOTO    MAINBEGIN	    ;Main Loop
    END
;====MAIN END===================================================================
    
    
