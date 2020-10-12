wait macro cycles
    local three = cycles / 3
    local two = (cycles % 3) / 2
    local one = (cycles % 3) % 2

    while three > 0
        goto $+2
three--
    endw

    if two
        goto $+4
    endif

    if one
        nop
    endif
    endm
    
; 4 cycles
CopyRegBitToRegBit  macro fromPort, fromBit, toPort, toBit
    btfsc   fromPort, fromBit
    bsf     toPort, toBit
    btfss   fromPort, fromBit
    bcf     toPort, toBit
    endm
    
; same as CopyRegBitToRegBit, except that it inverts the output ; 1 -> 0, 0 -> 1
CopyRegBitToInvertRegBit  macro fromPort, fromBit, toPort, toBit
    btfsc   fromPort, fromBit
    bcf     toPort, toBit
    btfss   fromPort, fromBit
    bsf     toPort, toBit
    endm
    
; LOW, LOW, LOW, HIGH
TransmitZeroBit	macro port, bit, cutOvertime
    local overtime = D'15' - cutOvertime
    
    bcf	    port, bit
    wait D'47'
    bsf	    port, bit
    wait overtime
    endm
    
; LOW, HIGH, HIGH, HIGH
TransmitOneBit	macro port, bit, cutOvertime
    local overtime = D'47' - cutOvertime
    
    bcf	    port, bit
    wait D'15'
    bsf	    port, bit
    wait overtime
    endm
    
; LOW, LOW, HIGH, HIGH
TransmitContStopBit macro port, bit, cutOvertime
    local overtime = D'31' - cutOvertime
    
    bcf	    port, bit
    wait D'31'
    bsf	    port, bit
    wait overtime
    endm
    
    
; shifts one byte from reg, into SIPO shift register
ShiftAddrByte macro reg
    movf    reg, W
    
    ;LOCAL   i = 0
    ;WHILE   i < 9
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
;i += 1
    ;ENDW
    
    endm
    
; shifts two bytes from reg1 and reg2, into SIPO shift register
ShiftAddrDualByte macro reg1, reg2
    movf    reg1, W
    
    ;LOCAL   i = 0
    ;WHILE   i < 8
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
;i += 1
    ;ENDW
    
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    
    ShiftAddrByte reg2
    
    endm
    
; shift data from reg out to IO pins
ShiftIoOutByte	macro reg
    movf    reg, W
    bcf     PIN_IO_OE1
    bcf	    PIN_IO_S1
    
    LOCAL   i = 0
    WHILE   i < 8
    
    rlcf    WREG
    bcf	    PIN_IO_SER_OUT
    btfsc   STATUS, C
    bsf	    PIN_IO_SER_OUT
    ;CopyRegBitToRegBit STATUS, C, PIN_IO_SER_OUT
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
i += 1
    ENDW
    
    bcf	    PIN_IO_SER_OUT
    bsf     PIN_IO_OE1
    
    endm
    
WriteToMem macro
    bcf     PIN_IO_OE1
    
    bcf     PIN_MEM_RW
    nop
    nop
    bsf     PIN_MEM_RW
    
    bsf     PIN_IO_OE1
    
    endm
    
; shifts data from IO pins into reg
ShiftIoInByte	macro reg
    movlw   B'00000000'
    bcf     PIN_IO_OE1
    bsf	    PIN_IO_S1
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    PIN_IO_S1
    
    ;LOCAL   i = 0
    ;WHILE   i < 8
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
;i += 1
    ;ENDW
    
    bsf     PIN_IO_OE1
    movwf   reg
    
    endm
    
; temporary macro to transmit literals to console
TXByTmp	    macro lit
    movlw   lit
    movwf   TX_DATA
    nop
    call    SendN64Byte
    
    endm
    
; Macros for special byte read / transmission
    
IncAddrBytes macro ; 2 cycles
    infsnz  N64_DATA_TMP1
    incf    N64_DATA_TMP0
    
    endm
    
ShiftPulseAddr macro ; 6 cycles
    rlcf    WREG
    bcf	    PIN_ADDR_SER
    btfsc   STATUS, C
    bsf	    PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
    endm
    
PrepareIoRead macro ; 6 cycles
    movlw   B'00000000'
    bcf     PIN_IO_OE1
    bsf	    PIN_IO_S1
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    bcf	    PIN_IO_S1
    
    endm
    
ShiftPulseIo macro ; 6 cycles
    bcf	    STATUS, C
    btfsc   PIN_IO_SER_IN
    bsf	    STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
    endm
    
SendFinalByte macro
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_7
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_7
NL02_SendOne_32_7:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_7:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_6
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_6
NL02_SendOne_32_6:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_6:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_5
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_5
NL02_SendOne_32_5:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_5:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_4
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_4
NL02_SendOne_32_4:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_4:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_3
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_3
NL02_SendOne_32_3:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_3:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_2
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_2
NL02_SendOne_32_2:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_2:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_1
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    wait    D'9' ; 15 - 2 - 4 = 9
    goto    NL02_SendAfter_32_1
NL02_SendOne_32_1:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'43' ; 47 - 4 = 43
NL02_SendAfter_32_1:
    
    rlcf    TX_DATA1, 1
    btfsc   STATUS, C, ACCESS
    goto    NL02_SendOne_32_0
    nop
    ; Send zero bit
    bcf	    PIN_DATAOUT
    wait    D'47'
    bsf	    PIN_DATAOUT
    ;wait    D'9' ; 15 - 2 - 4 - 9 = 0
    goto    NL02_SendAfter_32_0
NL02_SendOne_32_0:
    ; Send one bit
    bcf	    PIN_DATAOUT
    wait    D'15'
    bsf	    PIN_DATAOUT
    wait    D'34' ; 47 - 4 - 9 = 34
NL02_SendAfter_32_0:
    
    endm