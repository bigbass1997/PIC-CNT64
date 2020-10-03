; uses PAUSE_REG_0 and PAUSE_REG_1 to set the intervals of each loop
; cycles = ((PAUSE_REG_0 * 3) - 1) + ?
Pause2D:
    BANKSEL PAUSE_REG_0
    movff   PAUSE_REG_0, PAUSE_TMP_0
    movff   PAUSE_REG_1, PAUSE_TMP_1
    
Pause2D_SecondLoop:
    movff   PAUSE_REG_0, PAUSE_TMP_0
    
Pause2D_FirstLoop:
    decfsz  PAUSE_TMP_0
    goto    Pause2D_FirstLoop
    
    decfsz  PAUSE_TMP_1
    goto    Pause2D_SecondLoop
    
    return
    
    
TransmitByteRoutine:
    ; Utilizes TX_DATA register and PIN_DATAOUT to transmit a byte via the Joybus Protocol to the console
    TransmitByte TX_DATA, PIN_DATAOUT, 0
    return
    
    
PakDump:
    movlw   D'103'
    movwf   PAUSE_REG_0
    movlw   D'1'
    movwf   PAUSE_REG_1
    
PakDumpLoop:
    
    ShiftAddrDualByte H'20', H'21' ; set address
    ShiftIoInByte H'23' ; load byte
    movffl  H'23', U2TXB ; transmit byte
    
    call    Pause2D
    
    ; increase address by 1
    incf    H'21'
    btfss   STATUS, Z
    goto    PakDumpLoop
    
    incf    H'20'
    btfss   STATUS, Z
    goto    PakDumpLoop
    
    return
    
    
; Has the same result as DetermineDataToByte but calculates it entirely differently
; Instead of sampling every 1us, it only checks the 1st and 3rd microsecond.
; If first sample is HIGH, the byte is invalid and reading should stop.
; If both are LOW, bit is Zero
; If first is LOW, second is HIGH, bit is One
DetermineDataToByte2:
    movlw   D'8'
    movwf   LOOP_COUNT_0
    
DDTB_BitLoop:
    ; 4/16
    setf    N64_BIT_REG		; 1, 5/16
    
    bsf	    STATUS, C		; 1, 6/16
    btfss   PIN_DATAIN		; 1, 7/16
    bcf     STATUS, C
    rlcf    N64_BIT_REG, 1	; 1, 9/16
    
    wait    D'7'		; 1, 16/16
    wait    D'16'		; 2, 16/16
    wait    D'2'		; 3, 2/16
    
    bsf	    STATUS, C
    btfss   PIN_DATAIN		; 3, 4/16
    bcf     STATUS, C
    rlcf    N64_BIT_REG, 1	; 3, 6/16
    
    btfsc   N64_BIT_REG, 1
    goto    DDTB_ByteInvalid
    ; 3, 8/16
    
    bsf	    STATUS, C
    btfss   N64_BIT_REG, 0
    bcf	    STATUS, C
    rlcf    N64_DATA_DETER	; 3, 12/16
    wait    D'4'		; 3, 16/16
    
    dcfsnz  LOOP_COUNT_0
    goto    DDTB_End
    ; 4, 2/16
    
    wait    D'15'
    goto    DDTB_BitLoop
    
DDTB_ByteInvalid:
    bsf	    UTIL_FLAGS, 7	; byte is invalid, no bit was detected
    
DDTB_End:
    return  ; If byte was valid, cycle should be: 4, 5/16, leaving 11 cycles free
    
    
; This reads 32 bytes starting at N64_DATA_TMP2 register,
; and generates a 1 byte CRC to send back to the console.
; The byte is stored in the W register.
CRCHardware:
    BANKSEL CRCCON0
    bsf	    CRCCON0, CRCGO
    
    BANKSEL N64_DATA_TMP2
    
    lfsr    1, N64_DATA_TMP2
    movlw   D'32'
    movwf   LOOP_COUNT_0
CRCH_Loop:
    BANKSEL CRCCON0
    btfsc   CRCCON0, FULL
    goto    CRCH_Loop
    
    movffl  POSTINC1, CRCDATL
    BANKSEL LOOP_COUNT_0
    decfsz  LOOP_COUNT_0
    goto    CRCH_Loop
    
    BANKSEL CRCCON0
CRCH_Wait:
    btfsc   CRCCON0, BUSY_CRCCON0
    goto    CRCH_Wait
    movffl  CRCACCL, WREG
    
    bcf	    CRCCON0, CRCGO
    clrf    CRCACCL
    
    BANKSEL ZEROS_REG
    
    return