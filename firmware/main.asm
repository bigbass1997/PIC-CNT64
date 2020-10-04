    include "p18lf47k42.inc"
    include "macros.inc"
    processor 18lf47k42
    
    CONFIG WDTE = OFF
    CONFIG DEBUG = ON
    CONFIG LVP = ON
    CONFIG MCLRE = EXTMCLR
    CONFIG MVECEN = OFF
    
    CONFIG RSTOSC = HFINTOSC_64MHZ
    
    CONFIG XINST = ON
    
ResVec      code	0x0000
    goto    Setup
    
IntVec      code    0x0008
    goto    HandleIOCInterrupt
    
    
; === DEFINE PINS (text substitutions) ===
; refer to pinout documentation for more information
; reminder: all button input pins should be pulled-DOWN (default state is cleared aka 0)
;           connect pin to power to set button to a "pressed" state

#define     PIN_cR          PORTA, 5
#define     PIN_cL          PORTA, 4
#define     PIN_cD          PORTA, 3
#define     PIN_cU          PORTA, 2
#define     PIN_B           PORTA, 1
#define     PIN_A           PORTA, 0

#define     PIN_ASTICK_XA   PORTB, 5
#define     PIN_ASTICK_XB   PORTB, 4
#define     PIN_ASTICK_YB   PORTB, 3
#define     PIN_ASTICK_YA   PORTB, 2

#define     PIN_MEM_RW      LATC,  7
#define     PIN_IO_OE1      LATC,  6
#define     PIN_IO_SER_IN   PORTC, 5
#define     PIN_IO_S1       LATC,  4
#define     PIN_dR          PORTC, 3
#define     PIN_dL          PORTC, 2
#define     PIN_dD          PORTC, 1
#define     PIN_dU          PORTC, 0

#define     PIN_ADDR_CLK    LATD,  5
#define     PIN_ADDR_SER    LATD,  4
#define     PIN_IO_SER_OUT  LATD,  3
#define     PIN_IO_CLK      LATD,  2
#define     PIN_DATAIN      PORTD, 1
#define     PIN_DATAOUT     LATD,  1     ; LAT register is used for writing data out
#define     TRIS_DATAIO     TRISD, 1
#define     PIN_Z           PORTD, 0
    
#define     PIN_START       PORTE, 2
#define     PIN_RT          PORTE, 1
#define     PIN_LT          PORTE, 0

; === REGISTERS ===
; ACCESS BANK  (0x00 - 0x5F)
ZEROS_REG       equ H'00' ; Always 0x00
ONES_REG        equ H'01' ; Always 0xFF

N64_BIT_REG     equ H'02' ; This register is used to temporarily store 3-4us of data
                          ; to determine if the data recieved is a 0 or 1 or a stop bit.
N64_CMD_REG     equ H'03'

N64_ASTICKX_REG equ H'04' ; Used to store x-axis data
N64_ASTICKY_REG equ H'05' ; Used to store y-axis data

N64_STATE_REG1  equ H'08' ; Buffer:  Button states
N64_STATE_REG2  equ H'09' ; Buffer:  Button states
N64_STATE_REG3  equ H'0A' ; Buffer:  Analog Stick X-Axis ; -127 to +128
N64_STATE_REG4  equ H'0B' ; Buffer:  Analog Stick Y-Axis ; -127 to +128

TX_DATA         equ H'0C' ; Data to be transmitted to the console

UTIL_FLAGS      equ H'0D' ; Utility Flags, initalized with 0x00
; <7> If set, determined byte is invalid, decoding should halt
; <6:0> Unused

; Pause Clock
PAUSE_REG_0     equ H'10'
PAUSE_REG_1     equ H'11'
PAUSE_REG_2     equ H'12'

PAUSE_TMP_0     equ H'13'
PAUSE_TMP_1     equ H'14'
PAUSE_TMP_2     equ H'15'

; Auxillary Loop Counters
LOOP_COUNT_0    equ H'16'
LOOP_COUNT_1    equ H'17'

; BANK 0  (0x60 - 0xFF)
; These bytes are used for temporary storage of data coming from or going to the N64 console.
; While only 4 addresses are defined, this whole section of memory is dedicated for this purpose.
; Remember to specify in every instruction to use the BSR instead of Access memory.
N64_DATA_DETER  equ H'60'
N64_DATA_TMP0   equ H'61'
N64_DATA_TMP1   equ H'62'
N64_DATA_TMP2   equ H'63'
N64_DATA_TMP3   equ H'64'

; === CONSTANT BYTES ===
N64_CMD_RESET       equ H'FF'
N64_CMD_INFO        equ H'00'
N64_CMD_STATE       equ H'01'
N64_CMD_READACCES   equ H'02'
N64_CMD_WRITEACCES  equ H'03'

; https://n64brew.dev/wiki/Joybus_Protocol
N64_BIT_ZERO    equ B'11110001'
N64_BIT_ONE     equ B'11110111'
N64_BIT_CONSSTP equ B'11110111' ; bit #0 is not technically used, but for ease of programming, it is set to 1
N64_BIT_CONTSTP equ B'11110011'

Setup:
    movlb   B'00000000' ; sets current GPR bank to bank 0
    
    clrf    FSR2H
    movlw   B'00000000'
    movwf   FSR2L ; sets access bank start location to 0x00
    
    
    ; === Peripheral Pin Select ===
    BANKSEL RB0PPS
    movlw   B'00010110'
    movwf   RB0PPS      ; Set U2TX to pin RB0
    
    movlb   B'00000000'
    ; === Register Setup ===
    clrf    ZEROS_REG
    setf    ONES_REG
    clrf    UTIL_FLAGS
    
    ; configure I/O ports ; refer to pinout spreadsheet/docs for how these are mapped
    
    ; enable digitial input buffers
    BANKSEL ANSELA
    clrf    ANSELA
    clrf    ANSELB
    clrf    ANSELC
    clrf    ANSELD
    clrf    ANSELE
    
    ; 0 is output, 1 is input
    movlw   B'00111111'
    movwf   TRISA
    
    movlw   B'00111100'
    movwf   TRISB
    
    movlw   B'00101111'
    movwf   TRISC
    
    movlw   B'00000011'
    movwf   TRISD
    
    movlw   B'00000111'
    movwf   TRISE
    
    BANKSEL N64_STATE_REG1
    clrf    N64_STATE_REG1
    clrf    N64_STATE_REG2
    clrf    N64_STATE_REG3
    clrf    N64_STATE_REG4
    
    bsf     PIN_ASTICK_XA
    
    bcf     PIN_ADDR_CLK
    bcf     PIN_IO_CLK
    bsf     PIN_IO_OE1
    bsf     PIN_MEM_RW
    
    ShiftAddrDualByte ZEROS_REG, ZEROS_REG
    ShiftIoOutByte ZEROS_REG
    
    movlw   B'00000000'
    movwf   H'20'
    movlw   B'00000000'
    movwf   H'21'
    
    ; === Enable UART ===
    BANKSEL U2CON0
    bsf     U2CON0, U2TXEN  ; enable TX
                            ; MODE is 0000 by default, which sets UART to Async 8-bit
    bcf     U2CON0, U2BRGS  ; normal baud rate formula
    clrf    U2BRGH
    movlw   D'15'
    movwf   U2BRGL          ; set baud rate to 250,000
    bsf     U2CON1, U2ON    ; enable UART2
    
    ; === Enable CRC ===
    BANKSEL CRCCON0
    bsf     CRCCON0, CRCEN  ; enable CRC module
    clrf    CRCACCH
    clrf    CRCACCL ; seed 0x00
    clrf    CRCXORH
    movlw   0x85
    movwf   CRCXORL ; set polynomial/mask to 0x85
    clrf    CRCDATH ; not strictly necessary, clear upper 8bits of CRC data
    movlw   B'01110111'
    movwf   CRCCON1 ; set bit length to 8
    bsf     CRCCON0, ACCM   ; enable concat data with zeros when shifting
    ;bsf     CRCCON0, SHIFTM ; shift order
    
    
    ; === Interrupts ===
    BANKSEL PIE0
    bsf     PIE0, IOCIE     ; enable Interrupt-On-Change feature
    
    BANKSEL IOCBP
    bsf     IOCBP, 5        ; enable IOC rising-edge on RB5
    bsf     IOCBN, 5        ; enable IOC falling-edge on RB5
    bsf     IOCBP, 2        ; enable IOC rising-edge on RB2
    bsf     IOCBN, 2        ; enable IOC falling-edge on RB2
    
    BANKSEL INTCON0
    bsf     INTCON0, GIE    ; enable interrupt feature
    
    
    movlb   B'00000000'
    
    ; === Begin Main Loop ===
Start:
    call    ListenForN64
    goto    Start
    
; SUBROUTINES ;
    include "sub-utilities.inc"
    
ListenForN64:
    bsf     TRIS_DATAIO ; set to input
    
    ;btfsc   PIN_START
    ;call    PakDump
ListenForN64Loop:
    btfsc   PIN_DATAIN
    goto    ListenForN64Loop        ; wait until datapin goes LOW
    
    call    DetermineDataToByte2
    movff   N64_DATA_DETER, N64_CMD_REG
    
    lfsr    1, N64_DATA_TMP0
LFNL_DecodeLoop:
    btfsc   PIN_DATAIN
    goto    LFNL_DecodeLoop         ; wait until datapin goes LOW, if not already
    
    call    DetermineDataToByte2    ; will have 11 cycles left over
    movffl  N64_DATA_DETER, POSTINC1
    wait    D'4'
    btfss   UTIL_FLAGS, 7
    goto    LFNL_DecodeLoop         ; if not skipped, 7 cycles will have been consumed after jumping
    
    bcf     UTIL_FLAGS, 7
    
    lfsr    1, N64_DATA_TMP0        ; reset FSR for command usage as needed
    
    ; N64_CMD_REG is now set with command from N64 console
    ; Below is where N64_CMD_REG will be checked against each Protocol command
    ; (in order of most to least common command)
    
    bcf     TRIS_DATAIO ; set to output
    bsf     PIN_DATAOUT
    
    movffl  N64_CMD_REG, U2TXB
    
    movf    N64_CMD_REG, 0
    xorlw   N64_CMD_STATE
    btfsc   STATUS, Z
    goto N64Loop01
    
    movf    N64_CMD_REG, 0
    xorlw   N64_CMD_INFO
    btfsc   STATUS, Z
    goto N64Loop00
    
    ;movf    N64_CMD_REG, 0
    ;xorlw   N64_CMD_READACCES
    ;btfsc   STATUS, Z
    ;goto N64Loop02
    
    movf    N64_CMD_REG, 0
    xorlw   N64_CMD_WRITEACCES
    btfsc   STATUS, Z
    goto N64Loop03
    
    ; if this point is reached, no commands were identified. Wait a short time in case of any additional data
    movlw   D'224'
    movwf   PAUSE_REG_0
    movlw   D'4'
    movwf   PAUSE_REG_1
    call    Pause2D
    return
    
N64LoopFF:  ; Do 0xFF (reset/info) command here
    BANKSEL ZEROS_REG
    clrf    N64_STATE_REG3 ; resets x-axis
    clrf    N64_STATE_REG4 ; resets y-axis
    
    ; continue to N64Loop00...
    
N64Loop00:  ; Do 0x00 (info) command here
    BANKSEL ZEROS_REG
    
    TransmitByte 0x05, PIN_DATAOUT, 1
    TransmitByte 0x00, PIN_DATAOUT, 1
    TransmitByte 0x01, PIN_DATAOUT, 1
    wait D'5'
    TransmitContStopBit PIN_DATAOUT, 0
    
    movlw   D'220'
    movwf   PAUSE_REG_0
    movlw   D'1'
    movwf   PAUSE_REG_1
    
    movlw   0x05
    movffl  WREG, U2TXB
    call    Pause2D
    movlw   0x00
    movffl  WREG, U2TXB
    call    Pause2D
    movlw   0x01
    movffl  WREG, U2TXB
    
    goto ContinueLFNL
    
N64Loop01:  ; Do 0x01 (state) command here
    BANKSEL ZEROS_REG
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
    clrf    N64_STATE_REG3 ; and reset x-axis
    clrf    N64_STATE_REG4 ; and reset y-axis
    bcf     N64_STATE_REG1, 4 ; clear START from response
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
    
    movlw   D'220'
    movwf   PAUSE_REG_0
    movlw   D'1'
    movwf   PAUSE_REG_1
    
    movffl  N64_STATE_REG1, U2TXB
    call    Pause2D
    movffl  N64_STATE_REG2, U2TXB
    call    Pause2D
    movffl  N64_STATE_REG3, U2TXB
    call    Pause2D
    movffl  N64_STATE_REG4, U2TXB
    
    goto ContinueLFNL
    
N64Loop02: ; Do 0x02 (read accessory port) command here
    ; CRC is currently ignored, TODO: verify CRC is valid
    bcf     N64_DATA_TMP1, 4
    bcf     N64_DATA_TMP1, 3
    bcf     N64_DATA_TMP1, 2
    bcf     N64_DATA_TMP1, 1
    bcf     N64_DATA_TMP1, 0
    ShiftAddrDualByte N64_DATA_TMP0, N64_DATA_TMP1
    
    movf    N64_DATA_TMP0, 0
    xorlw   B'10000000'
    btfsc   STATUS, Z
    goto    File8000
    goto    File0020
    
File8000:
i = 0
    movlw   B'11111110' ; 0xFE
    while i < 32
    movwf   TX_DATA
    call    TransmitByteRoutine
i += 1
    endw
    
    goto    FileEND
File0020:
    TXByTmp 0x00
    TXByTmp 0x2D
    TXByTmp 0x00
    TXByTmp 0x00
    
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x3B
    TXByTmp 0x74
    ;
    TXByTmp 0x00
    TXByTmp 0x40
    TXByTmp 0xBF
    TXByTmp 0x67
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    ;
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    TXByTmp 0x00
    ;
    TXByTmp 0x00
    TXByTmp 0x01
    
    TXByTmp 0x01
    
    TXByTmp 0x00
    
    TXByTmp 0xFC
    TXByTmp 0x49
    
    TXByTmp 0x03
    TXByTmp 0xA9
    ;
    
FileEND:
    wait D'5'
    TransmitContStopBit PIN_DATAOUT, 0
    
    goto ContinueLFNL
    
N64Loop03: ; Do 0x03 (write accessory port) command here
    BANKSEL ZEROS_REG
    
    call    CRCHardware
    
    movffl  WREG, TX_DATA
    call    TransmitByteRoutine
    wait D'5'
    TransmitContStopBit PIN_DATAOUT, 0
    
    movffl  WREG, U2TXB
    
;    lfsr    1, N64_DATA_TMP0
;    movlw   D'34'
;    movwf   LOOP_COUNT_0
;    movlw   D'220'
;    movwf   PAUSE_REG_0
;    movlw   D'1'
;    movwf   PAUSE_REG_1
;N64Loop03_DebugLoop:
;    movffl  POSTINC1, U2TXB
;    call    Pause2D
;    decfsz  LOOP_COUNT_0
;    goto    N64Loop03_DebugLoop
    
    goto ContinueLFNL ; not strictly necessary to have this goto right now, but will be as more commands are supported
    
    
ContinueLFNL:
    return
    
; INTERRUPT SUBROUTINES ;

HandleIOCInterrupt:
    BANKSEL IOCBF
    btfsc   IOCBF, 5
    goto    HandleXAInterrupt
    
    btfsc   IOCBF, 2
    goto    HandleYAInterrupt
    
    retfie 1
    
HandleXAInterrupt:
    BANKSEL ZEROS_REG
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
    BANKSEL IOCBF
    bcf     IOCBF, 5 ; clear interrupt flag
    retfie 1
    
HandleYAInterrupt:
    BANKSEL ZEROS_REG
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
    BANKSEL IOCBF
    bcf     IOCBF, 2 ; clear interrupt flag
    retfie 1
    
    end