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
DetermineDataToByte:
    ; ITERATION #0
    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
    
    SetIfDataLow 3
    wait    D'7'		    ; 2 less than normal due to 'call' instruction
    
    SetIfDataLow 2
    wait    D'10'
    
    SetIfDataLow 1
    wait    D'10'
    
    SetIfDataLow 0                  ; 2/12
    
    movf    N64_BIT_REG, 0          ; 3/12
    xorlw   N64_BIT_ZERO            ; 4/12
    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
    bcf     N64_DATA_DETER, 7    ; 6/12
    
    movf    N64_BIT_REG, 0          ; 7/12
    xorlw   N64_BIT_ONE             ; 8/12
    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
    bsf     N64_DATA_DETER, 7    ;10/12
    
    wait    D'3'                    ;12/12
    
    ; ITERATION #1-6
    local i = 6
    while i >= 1
    
    ;movlw   B'11111111'
    ;movwf   N64_BIT_REG             ; reset N64_BIT_REG
    
    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
    
    SetIfDataLow 3
    wait    D'9'
    
    SetIfDataLow 2
    wait    D'10'
    
    SetIfDataLow 1
    wait    D'10'
    
    SetIfDataLow 0                  ; 2/12
    
    movf    N64_BIT_REG, 0          ; 3/12
    xorlw   N64_BIT_ZERO            ; 4/12
    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
    bcf     N64_DATA_DETER, i    ; 6/12
    
    movf    N64_BIT_REG, 0          ; 7/12
    xorlw   N64_BIT_ONE             ; 8/12
    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
    bsf     N64_DATA_DETER, i    ;10/12
    
    wait    D'3'                    ;12/12
    
i -= 1
    endw
    
    ; ITERATION #7
    setf    N64_BIT_REG		    ; reset N64_BIT_REG ; 1 cycle
    
    SetIfDataLow 3
    wait    D'9'
    
    SetIfDataLow 2
    wait    D'10'
    
    SetIfDataLow 1
    wait    D'10'
    
    SetIfDataLow 0                  ; 2/12
    
    movf    N64_BIT_REG, 0          ; 3/12
    xorlw   N64_BIT_ZERO            ; 4/12
    btfsc   STATUS, Z               ; 5/12  if N64_BIT_REG != N64_BIT_ZERO then skip the next instruction
    bcf     N64_DATA_DETER, 0    ; 6/12
    
    movf    N64_BIT_REG, 0          ; 7/12
    xorlw   N64_BIT_ONE             ; 8/12
    btfsc   STATUS, Z               ; 9/12  if N64_BIT_REG != N64_BIT_ONE then skip the next instruction
    bsf     N64_DATA_DETER, 0    ;10/12
    
    ; 'return' instruction uses last 2 cycles
    wait D'1'
    
    return