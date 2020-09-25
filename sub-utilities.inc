CalculateCRC:
    
    
    
    return
    
    
; uses PAUSE_REG_0 and PAUSE_REG_1 to set the intervals of each loop
; cycles = ((PAUSE_REG_0 * 3) - 1) + ?
Pause2D:
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
    
    
#ifdef USING_EUSART
PakDump:
    movlw   D'103'
    movwf   PAUSE_REG_0
    movlw   D'1'
    movwf   PAUSE_REG_1
    
PakDumpLoop:
    
    ShiftAddrDualByte H'20', H'21' ; set address
    ShiftIoInByte H'23' ; load byte
    movff   H'23', TXREG ; transmit byte
    
    call    Pause2D
    
    ; increase address by 1
    incf    H'21'
    btfss   STATUS, Z
    goto    PakDumpLoop
    
    incf    H'20'
    btfss   STATUS, Z
    goto    PakDumpLoop
    
    return
#endif
    
; loads 4us of data from data pin and determines what type of protocol bit it is
;DetermineDataToByte:
;    ; ITERATION #0
;    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
;    
;    SetIfDataLow 3
;    wait    D'7'		    ; 2 less than normal due to 'call' instruction
;    
;    SetIfDataLow 2
;    wait    D'10'
;    
;    SetIfDataLow 1
;    wait    D'10'
;    
;    SetIfDataLow 0                  ; 2/12
;    
;    movf    N64_BIT_REG, 0          ; 3/12
;    xorlw   N64_BIT_ZERO            ; 4/12
;    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
;    bcf     N64_DATA_DETER, 7    ; 6/12
;    
;    movf    N64_BIT_REG, 0          ; 7/12
;    xorlw   N64_BIT_ONE             ; 8/12
;    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
;    bsf     N64_DATA_DETER, 7    ;10/12
;    
;    wait    D'3'                    ;12/12
;    
;    ; ITERATION #1-6
;    local i = 6
;    while i >= 1
;    
;    ;movlw   B'11111111'
;    ;movwf   N64_BIT_REG             ; reset N64_BIT_REG
;    
;    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
;    
;    SetIfDataLow 3
;    wait    D'9'
;    
;    SetIfDataLow 2
;    wait    D'10'
;    
;    SetIfDataLow 1
;    wait    D'10'
;    
;    SetIfDataLow 0                  ; 2/12
;    
;    movf    N64_BIT_REG, 0          ; 3/12
;    xorlw   N64_BIT_ZERO            ; 4/12
;    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
;    bcf     N64_DATA_DETER, i    ; 6/12
;    
;    movf    N64_BIT_REG, 0          ; 7/12
;    xorlw   N64_BIT_ONE             ; 8/12
;    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
;    bsf     N64_DATA_DETER, i    ;10/12
;    
;    wait    D'3'                    ;12/12
;    
;i -= 1
;    endw
;    
;    ; ITERATION #7
;    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
;    
;    SetIfDataLow 3
;    wait    D'9'
;    
;    SetIfDataLow 2
;    wait    D'10'
;    
;    SetIfDataLow 1
;    wait    D'10'
;    
;    SetIfDataLow 0                  ; 2/12
;    
;    movf    N64_BIT_REG, 0          ; 3/12
;    xorlw   N64_BIT_ZERO            ; 4/12
;    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
;    bcf     N64_DATA_DETER, 0    ; 6/12
;    
;    movf    N64_BIT_REG, 0          ; 7/12
;    xorlw   N64_BIT_ONE             ; 8/12
;    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
;    bsf     N64_DATA_DETER, 0    ;10/12
;    
;    ; 'return' instruction uses last 2 cycles
;    wait D'1'
;    
;    return
    
    
; Has the same result as DetermineDataToByte but calculates it entirely differently
; Instead of sampling every 1us, it only checks the 1st and 3rd microsecond.
; If first sample is HIGH, the byte is invalid and reading should stop.
; If both are LOW, bit is Zero
; If first is LOW, second is HIGH, bit is One
DetermineDataToByte2:
    movlw   D'8'
    movwf   LOOP_COUNT_0
    
DDTB_BitLoop:
    ; 4/12
    setf    N64_BIT_REG		; 1, 5/12
    
    bsf	    STATUS, C		; 1, 6/12
    btfss   PIN_DATAIN		; 1, 7/12
    bcf     STATUS, C
    rlcf    N64_BIT_REG, 1	; 1, 9/12
    
    wait    D'3'		; 1, 12/12
    wait    D'12'		; 2, 12/12
    wait    D'2'		; 3, 3/12
    
    bsf	    STATUS, C
    btfss   PIN_DATAIN		; 3, 4/12
    bcf     STATUS, C
    rlcf    N64_BIT_REG, 1	; 3, 6/12
    
    btfsc   N64_BIT_REG, 1
    goto    DDTB_ByteInvalid
    ; 3, 8/12
    
    bsf	    STATUS, C
    btfss   N64_BIT_REG, 0
    bcf	    STATUS, C
    rlcf    N64_DATA_DETER	; 3, 12/12
    
    dcfsnz  LOOP_COUNT_0
    goto    DDTB_End
    ; 4, 2/12
    
    wait    D'11'
    goto    DDTB_BitLoop
    
DDTB_ByteInvalid:
    bsf	    UTIL_FLAGS, 7	; byte is invalid, no bit was detected
    
DDTB_End:
    return  ; If byte was valid, cycle should be: 4, 5/12, leaving 7 cycles free
    
    
; This reads 32 bytes starting at N64_DATA_TMP2 register,
; and generates a 1 byte CRC to send back to the console.
; The byte is stored in the W register.
CRC32Bytes:
    movlw   D'32'
    movwf   LOOP_COUNT_0
    clrf    LOOP_COUNT_1
    bsf	    LOOP_COUNT_1, 3
    clrf    WREG
    
    lfsr    2, N64_DATA_TMP1
CRC_ByteLoop:
    movff   PREINC2, N64_DATA_DETER ; junk move to increment FSR
    bsf	    LOOP_COUNT_1, 3
CRC_BitLoop:
    rlcf    INDF2
    rlcf    WREG
    btfsc   STATUS, C
    xorlw   B'10000101'
    
    decfsz  LOOP_COUNT_1
    goto    CRC_BitLoop
    
    decfsz  LOOP_COUNT_0
    goto    CRC_ByteLoop
    
    bsf	    LOOP_COUNT_1, 3
CRC_FinalLoop:
    bcf	    STATUS, C
    rlcf    WREG
    btfsc   STATUS, C
    xorlw   B'10000101'
    
    decfsz  LOOP_COUNT_1
    goto    CRC_FinalLoop
    
    return