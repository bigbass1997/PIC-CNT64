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
    
ledon       macro
    bsf     PORTA,0
    endm
ledoff      macro
    bcf     PORTA,0
    endm
pulse	    macro port, bit
    bsf     port, bit
    bcf     port, bit
    endm

SetPinHigh	macro port, bit
    bsf	    port, bit
    endm
    
SetPinLow	macro port, bit
    bcf	    port, bit
    endm
    
; LOW, LOW, LOW, HIGH
TransmitZeroBit	macro port, bit, cutOvertime
    local overtime = D'11' - cutOvertime
    
    SetPinLow port, bit
    wait D'35'
    SetPinHigh port, bit
    wait overtime
    endm
    
; LOW, HIGH, HIGH, HIGH
TransmitOneBit	macro port, bit, cutOvertime
    local overtime = D'35' - cutOvertime
    
    SetPinLow port, bit
    wait D'11'
    SetPinHigh port, bit
    wait overtime
    endm
    
; LOW, LOW, HIGH, HIGH
TransmitContStopBit macro port, bit, cutOvertime
    local overtime = D'23' - cutOvertime
    
    SetPinLow port, bit
    wait D'23'
    SetPinHigh port, bit
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