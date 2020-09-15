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
    local overtime = D'11' - cutOvertime
    
    bcf	    port, bit
    wait D'35'
    bsf	    port, bit
    wait overtime
    endm
    
; LOW, HIGH, HIGH, HIGH
TransmitOneBit	macro port, bit, cutOvertime
    local overtime = D'35' - cutOvertime
    
    bcf	    port, bit
    wait D'11'
    bsf	    port, bit
    wait overtime
    endm
    
; LOW, LOW, HIGH, HIGH
TransmitContStopBit macro port, bit, cutOvertime
    local overtime = D'23' - cutOvertime
    
    bcf	    port, bit
    wait D'23'
    bsf	    port, bit
    wait overtime
    endm
    
; must give exactly 4 cycles lead time
; byteReg is either a byte literal or a register address
;
; if setW == 1, then byteReg should be a literal
;   otherwise, byteReg should be a register address
TransmitByte	macro byteReg, port, bit, setW
    local i = 7
    
    if setW == 1
    movlw   byteReg		; 1 cycle
    else
    movf    byteReg, 0		; 1 cycle
    endif
    
    while i >= 0
    local Bit#v(i)_0, Bit#v(i)_Cont
    btfss   WREG, #v(i)		; 1 cycle
    goto Bit#v(i)_0			; 1 or 2 cycles
    ; 3 cycles total here
    wait D'1'
    if i == 0
    TransmitOneBit port, bit, D'7'
    else
    TransmitOneBit port, bit, D'5'
    endif
    goto Bit#v(i)_Cont
Bit#v(i)_0:
    ; 4 cycles total here
    if i == 0
    TransmitZeroBit port, bit, D'5'
    else
    TransmitZeroBit port, bit, D'3'
    endif
Bit#v(i)_Cont:
i -= 1
    endw
    
    endm
    
SetIfDataLow    macro bitIndex
    btfss   PIN_DATAIN
    bcf     N64_BIT_REG, bitIndex
    endm
    
; shifts one byte from reg, into SIPO shift register
ShiftAddrByte macro reg
    movff   reg, WREG
    
    LOCAL   i = 0
    WHILE   i < 9
    
    rlcf    WREG
    CopyRegBitToRegBit	STATUS, C, PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
i += 1
    ENDW
    
    endm
    
; shifts two bytes from reg1 and reg2, into SIPO shift register
ShiftAddrDualByte macro reg1, reg2
    movff   reg1, WREG
    
    LOCAL   i = 0
    WHILE   i < 8
    
    rlcf    WREG
    CopyRegBitToRegBit	STATUS, C, PIN_ADDR_SER
    bsf	    PIN_ADDR_CLK
    bcf	    PIN_ADDR_CLK
    
i += 1
    ENDW
    
    rlcf    WREG
    CopyRegBitToRegBit	STATUS, C, PIN_ADDR_SER
    
    ShiftAddrByte reg2
    
    endm
    
; shift data from reg out to IO pins
ShiftIoOutByte	macro reg
    movff   reg, WREG
    bcf     PIN_IO_OE1
    bcf	    PIN_IO_S1
    
    LOCAL   i = 0
    WHILE   i < 8
    
    rlcf    WREG
    CopyRegBitToRegBit STATUS, C, PIN_IO_SER_OUT
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
    
    LOCAL   i = 0
    WHILE   i < 8
    
    CopyRegBitToRegBit PIN_IO_SER_IN, STATUS, C
    rlcf    WREG
    bsf	    PIN_IO_CLK
    bcf	    PIN_IO_CLK
    
i += 1
    ENDW
    
    bsf     PIN_IO_OE1
    movwf   reg
    
    endm
    
; temporary macro to transmit literals to console
TXByTmp	    macro lit
    movlw   lit
    movwf   TX_DATA
    call TransmitByteRoutine
    
    endm