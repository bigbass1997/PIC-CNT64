    include "p18lf4550.inc"
    include "macros.inc"
    processor 18lf4550
    
    CONFIG WDT = OFF
    CONFIG MCLRE = OFF
    CONFIG DEBUG = OFF
    CONFIG LVP = OFF
    
    CONFIG PBADEN = OFF
    
    CONFIG PLLDIV = 2
    ;CONFIG FOSC = INTOSC_EC
    CONFIG FOSC = HSPLL_HS
    CONFIG CPUDIV = OSC1_PLL2
    CONFIG USBDIV = 2
    
    CONFIG XINST = ON
    
ResVec      code	0x0000
    goto    Setup

; UNUSED AT THE MOMENT ;
HighInt     code    0x0008 ; High Priority Interrupt Vector
    ;pulse   PORTA, 1
    ;btfsc   INTCON3, INT0IF
    ;goto    HandleN64CommandInterrupt
    bcf     INTCON3, INT0IF ; clear interrupt flag
    retfie 1
;;;;;;;;;;;;;;;;;;;;;;;;

LowInt      code    0x0018 ; Low Priority Interrupt Vector
    pulse   PORTA, 0
    btfsc   INTCON, RBIF
    goto    HandleXAInterrupt
    ;movf    PORTB, 1
    
    btfsc   INTCON3, INT2IF
    goto    HandleYAInterrupt
    btfsc   INTCON3, INT1IF
    goto    HandleYAInterrupt
    
    retfie 1

; === DEFINE PINS (text substitutions) ===
; refer to pinout documentation for more information
; reminder: all button input pins should be pulled-DOWN (default state is cleared aka 0)
;           connect pin to power to set button to a "pressed" state

#define     PIN_DEBUG_1     PORTA, 1
#define     PIN_DEBUG_0     PORTA, 0

#define     PIN_ASTICK_XA   PORTB, 4
#define     PIN_ASTICK_YA   PORTB, 1

#define     PIN_cU          PORTC, 7
#define     PIN_cD          PORTC, 6
#define     PIN_cL          PORTC, 5
#define     PIN_cR          PORTC, 4
#define     PIN_LT          PORTC, 2
#define     PIN_RT          PORTC, 1
#define     PIN_START       PORTC, 0

#define     PIN_ASTICK_XB   PORTD, 7
#define     PIN_ASTICK_YB   PORTD, 6
#define     PIN_dU          PORTD, 5
#define     PIN_dD          PORTD, 4
#define     PIN_dL          PORTD, 3
#define     PIN_dR          PORTD, 2
#define     PIN_DATAIN      PORTD, 0
#define     PIN_DATAOUT     LATD, 0     ; LAT register is used for writing data out
#define     TRIS_DATAIO     TRISD, 0

#define     PIN_A           PORTE, 2
#define     PIN_B           PORTE, 1
#define     PIN_Z           PORTE, 0

; === REGISTERS ===
ZEROS_REG       equ H'00'
ONES_REG        equ H'01'

N64_BIT_REG     equ H'02' ; This register is used to temporarily store 3-4us of data
                          ; to determine if the data recieved is a 0 or 1 or a stop bit.
N64_CMD_REG     equ H'03'

N64_ASTICKX_REG equ H'04' ; Used to store x-axis data
N64_ASTICKY_REG equ H'05' ; Used to store y-axis data

N64_STATE_REG1  equ H'08'
N64_STATE_REG2  equ H'09'
N64_STATE_REG3  equ H'0A' ; Analog Stick X-Axis ; -127 to +128
N64_STATE_REG4  equ H'0B' ; Analog Stick Y-Axis ; -127 to +128

; === CONSTANT BYTES ===
N64_CMD_RESET   equ H'FF'
N64_CMD_INFO    equ H'00'
N64_CMD_STATE   equ H'01'

; https://sites.google.com/site/consoleprotocols/home/nintendo-joy-bus-documentation
N64_BIT_ZERO    equ B'11110001'
N64_BIT_ONE     equ B'11110111'
N64_BIT_CONSSTP equ B'11110111' ; bit #0 is not technically used, but for ease of programming, it is set to 1
N64_BIT_CONTSTP equ B'11110011'

Setup:
    movlw   B'00000000'
    movwf   FSR2L ; sets access bank start location to 0x00
    
    bsf     RCON, IPEN      ; enable interrupt feature
    bsf     INTCON, GIEH    ; enable high priority interrupts
    bsf     INTCON, GIEL    ; enable low priority interrupts
    
    ; Connected on RB4 ; Detects changing edge of stick XA
    bcf     INTCON, RBIF    ; Clear the PORTB Interrupt flag, for safety
    bsf     INTCON, RBIE    ; Set the PORTB Interrupt enable bit
    bcf     INTCON2, RBIP   ; Clear the PORTB Interrupt priority bit (low priority)
    
    ; INT2 == RB2 ; Detect the rising edge of stick YA
    bcf     INTCON3, INT2IF ; Clear the INT2 Interrupt flag, for safety
    bsf     INTCON3, INT2IE ; Set the INT2 Interrupt enable bit
    bcf     INTCON3, INT2IP ; Clear the INT2 Interrupt priority bit (low priority)
    bsf     INTCON2, INTEDG2; Set INT2 Interrupt to detect on rising edge
    
    ; INT1 == RB1 ; Detect the falling edge of stick YA
    bcf     INTCON3, INT1IF ; Clear the INT1 Interrupt flag, for safety
    bsf     INTCON3, INT1IE ; Set the INT1 Interrupt enable bit
    bcf     INTCON3, INT1IP ; Clear the INT1 Interrupt priority bit (low priority)
    bcf     INTCON2, INTEDG1; Clear INT1 Interrupt to detect on falling edge
    
    ; INT0 == RB0 ; Detect the falling edge of console command
    ;bcf     INTCON, INT0IF ; Clear the Interrupt 0 flag, for safety
    ;bsf     INTCON, INT0IE ; Set the Interrupt 0 enable bit
                            ; Interrupt 0 is always high priority
    ;bcf     INTCON2, INTEDG0; Clear INT0 Interrupt to detect on falling edge
    
    movlw   B'00000000'
    movwf   ZEROS_REG
    movlw   B'11111111'
    movwf   ONES_REG
    
    bsf     ADCON1, PCFG3
    
    ; configure I/O ports ; refer to pinout spreadsheet/docs for how these are mapped
    ; 0 is output, 1 is input
    movlw   B'00000000'
    movwf   TRISA
    
    movlw   B'11111111' ; interrupts, leave all inputs
    movwf   TRISB
    
    movlw   B'11110111'
    movwf   TRISC
    
    movlw   B'11111111'
    movwf   TRISD
    
    movlw   B'00000111'
    movwf   TRISE
    
    bcf     PIN_DEBUG_0 ; clear debug pin
    bcf     PIN_DEBUG_1 ; clear debug pin
    
    movlw   B'00000000'
    movwf   N64_STATE_REG1
    movlw   B'00000000'
    movwf   N64_STATE_REG2
    movlw   B'00000000'
    movwf   N64_STATE_REG3
    movlw   B'00000000'
    movwf   N64_STATE_REG4
    
    bsf     PIN_ASTICK_XA
    
Start:
    call    ListenForN64
    goto    Start

; SUBROUTINES / INTERRUPT LABELS ;

HandleN64CommandInterrupt:
    ; currently unused
    bcf     INTCON3, INT0IF ; clear interrupt flag
    retfie 1
    
HandleYAInterrupt:
    ; PIN_ASTICK_YA, PIN_ASTICK_YB
    btfsc   PIN_ASTICK_YA
    goto    YA_IS_ONE
YA_IS_ZERO:
    btfsc   PIN_ASTICK_YB
    goto    YA_NOTEQUALS_YB
    goto    YA_EQUALS_YB
YA_IS_ONE:
    btfsc   PIN_ASTICK_YB
    goto    YA_EQUALS_YB
YA_NOTEQUALS_YB:
    decf    N64_STATE_REG4, 1, 1
    decf    N64_STATE_REG4, 1, 1
    goto    HYAI_Finish
YA_EQUALS_YB:
    incf    N64_STATE_REG4, 1, 1
    incf    N64_STATE_REG4, 1, 1
HYAI_Finish:
    bcf     INTCON3, INT2IF ; clear interrupt flag
    bcf     INTCON3, INT1IF ; clear interrupt flag
    retfie 1
    
HandleXAInterrupt:
    ; PIN_ASTICK_XA, PIN_ASTICK_XB
    btfsc   PIN_ASTICK_XA
    goto    XA_IS_ONE
XA_IS_ZERO:
    btfsc   PIN_ASTICK_XB
    goto    XA_NOTEQUALS_XB
    goto    XA_EQUALS_XB
XA_IS_ONE:
    btfsc   PIN_ASTICK_XB
    goto    XA_EQUALS_XB
XA_NOTEQUALS_XB:
    decf    N64_STATE_REG3, 1, 1
    decf    N64_STATE_REG3, 1, 1
    goto    HXAI_Finish
XA_EQUALS_XB:
    incf    N64_STATE_REG3, 1, 1
    incf    N64_STATE_REG3, 1, 1
HXAI_Finish:
    movf    PORTB, 1
    bcf     INTCON, RBIF ; clear interrupt flag
    retfie 1
    
    
    
SetIfDataLow    macro bitIndex
    btfss   PIN_DATAIN
    bcf     N64_BIT_REG, bitIndex
    endm
    
DetermineDataToBit  macro cmdBitIndex ; loads 4us of data from data pin and determines what type of protocol bit it is
    movlw   B'11111111'
    movwf   N64_BIT_REG             ; reset N64_BIT_REG
    
    SetIfDataLow 3;bcf     N64_BIT_REG, 3          ; first microsecond of each bit is always 0
    wait    D'8'
    
    SetIfDataLow 2
    wait    D'10'
    
    SetIfDataLow 1
    wait    D'10'
    
    SetIfDataLow 0                  ; 2/12
    
    ;movff   N64_BIT_REG, PORTB
    ;movff   ZEROS_REG, PORTB
    
    movf    N64_BIT_REG, 0             ; 3/12
    xorlw   N64_BIT_ZERO            ; 4/12
    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
    bcf     N64_CMD_REG, cmdBitIndex; 6/12
    
    movf    N64_BIT_REG, 0          ; 7/12
    xorlw   N64_BIT_ONE             ; 8/12
    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
    bsf     N64_CMD_REG, cmdBitIndex;10/12
    
    wait    D'3'                    ;12/12
    endm
    
ListenForN64:
    ;ledoff                          ; DEBUG
    
    bsf     TRIS_DATAIO ; set to input
    wait    D'255'                  ; waits long enough to be sure we are not inside a signal command/response
    wait    D'255'
    wait    D'255'
    wait    D'255'
    wait    D'255'
    wait    D'255'
    wait    D'255'
    wait    D'231'
    
ListenForN64Loop:
    btfsc   PIN_DATAIN
    goto    ListenForN64Loop        ; wait until datapin goes LOW
    
    DetermineDataToBit 7
    DetermineDataToBit 6
    DetermineDataToBit 5
    DetermineDataToBit 4
    DetermineDataToBit 3
    DetermineDataToBit 2
    DetermineDataToBit 1
    DetermineDataToBit 0
    ; N64_CMD_REG is now set with command from N64 console
    ; Below is where N64_CMD_REG will be checked against each Protocol command
    ; (in order of most to least common command)
    
    bcf     TRIS_DATAIO ; set to output
    bsf     PIN_DATAOUT
    
    movf    N64_CMD_REG, 0
    xorlw   N64_CMD_STATE
    btfsc   STATUS, Z
    goto N64Loop01
    
    movf    N64_CMD_REG, 0
    xorlw   N64_CMD_INFO
    btfsc   STATUS, Z
    goto N64Loop00
    
    
N64LoopFF:  ; Do 0xFF (reset/info) command here
    movff   ZEROS_REG, N64_STATE_REG3 ; resets x-axis
    movff   ZEROS_REG, N64_STATE_REG4 ; resets y-axis
    
    ; continue to N64Loop00...
    
N64Loop00:  ; Do 0x00 (info) command here
    wait D'27'  ; this assumes console stop bit occured
    
    TransmitByte 0x05, PIN_DATAOUT, 1
    TransmitByte 0x00, PIN_DATAOUT, 1
    TransmitByte 0x02, PIN_DATAOUT, 1
    wait D'5'
    TransmitContStopBit PIN_DATAOUT, 0
    
    goto ContinueLFNL
    
N64Loop01:  ; Do 0x01 (state) command here
    ;wait D'27'  ; this assumes console stop bit occured ; this wait is no longer necessary
    
    ; Update input button states to state registers
    CopyRegBitToRegBit  PIN_A,      N64_STATE_REG1, 7
    CopyRegBitToRegBit  PIN_B,      N64_STATE_REG1, 6
    CopyRegBitToRegBit  PIN_Z,      N64_STATE_REG1, 5
    CopyRegBitToRegBit  PIN_START,  N64_STATE_REG1, 4
    CopyRegBitToRegBit  PIN_dU,     N64_STATE_REG1, 3
    CopyRegBitToRegBit  PIN_dD,     N64_STATE_REG1, 2
    CopyRegBitToRegBit  PIN_dL,     N64_STATE_REG1, 1
    CopyRegBitToRegBit  PIN_dR,     N64_STATE_REG1, 0
    
    bcf     N64_STATE_REG2, 7 ; reset RST bit
    btfss   PIN_LT      ; if LT is pressed
    goto    ContAfterRstCheck
    btfss   PIN_RT      ; and RT is pressed
    goto    ContAfterRstCheck
    btfss   PIN_START   ; and START is pressed
    goto    ContAfterRstCheck
    bsf     N64_STATE_REG2, 7 ; then set RST bit
    movff   ZEROS_REG, N64_STATE_REG3 ; and reset x-axis
    movff   ZEROS_REG, N64_STATE_REG4 ; and reset y-axis
ContAfterRstCheck:
    CopyRegBitToRegBit  PIN_LT,     N64_STATE_REG2, 5
    CopyRegBitToRegBit  PIN_RT,     N64_STATE_REG2, 4
    CopyRegBitToRegBit  PIN_cU,     N64_STATE_REG2, 3
    CopyRegBitToRegBit  PIN_cD,     N64_STATE_REG2, 2
    CopyRegBitToRegBit  PIN_cL,     N64_STATE_REG2, 1
    CopyRegBitToRegBit  PIN_cR,     N64_STATE_REG2, 0
    
    ; 60-68 "instruction" cycles will have passed by now
    
    ; Transmit bytes to console
    TransmitByte N64_STATE_REG1, PIN_DATAOUT, 0
    TransmitByte N64_STATE_REG2, PIN_DATAOUT, 0
    TransmitByte N64_STATE_REG3, PIN_DATAOUT, 0
    TransmitByte N64_STATE_REG4, PIN_DATAOUT, 0
    wait D'5'
    TransmitContStopBit PIN_DATAOUT, 0
    
    goto ContinueLFNL
    
ContinueLFNL:
    ;movff   N64_CMD_REG, PORTB
    ;movlw   B'00000000'
    ;movwf   PORTB
    
    return  
    
    end