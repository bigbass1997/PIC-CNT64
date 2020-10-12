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
    
; Utilizes TX_DATA register and PIN_DATAOUT
; 'BANKSEL TX_DATA' must be run BEFORE this subroutine!
; Provides 3 cycles of lead time (after considering the return and next call instructions)
SendN64Byte:
    lfsr    2, 0x00
    movlw   D'7'
    movwf   LOOP_COUNT_0
    
SNB_Loop:
    rlcf    [TX_DATA], 1
    btfsc   STATUS, C, ACCESS
    goto    SNB_One
    TransmitZeroBit PIN_DATAOUT, 9
    goto    SNB_After
SNB_One:
    TransmitOneBit PIN_DATAOUT, 7
SNB_After:
    decfsz  [LOOP_COUNT_0], 1
    goto    SNB_Loop
    
    ; last iteration has different timing to allow for another byte after
    rlcf    [TX_DATA], 1
    btfsc   STATUS, C, ACCESS
    goto    SNB_LastOne
    TransmitZeroBit PIN_DATAOUT, 15
    goto    SNB_End
SNB_LastOne:
    TransmitOneBit PIN_DATAOUT, 13
SNB_End:
    return
    
    
; Intended to be called immediately after the last `SendN64Byte` call
SendN64StopBit:
    wait D'9'
    bcf	    PIN_DATAOUT
    wait D'31'
    bsf	    PIN_DATAOUT
    return
    
    
PakDump:
    movlw   D'220'
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
    
CRCHardwareSpeed:
    movlb   B'111001'
    bsf	    CRCCON0, CRCGO
    
;CRCH_Loop:
    ;btfsc   CRCCON0, FULL
    ;goto    CRCH_Loop
    
    movffl  H'63', CRCDATL
    ;
    movffl  H'64', CRCDATL
    ;
    movffl  H'65', CRCDATL
    ;
    movffl  H'66', CRCDATL
    ;
    movffl  H'67', CRCDATL
    ;
    movffl  H'68', CRCDATL
    ;
    movffl  H'69', CRCDATL
    ;
    movffl  H'6A', CRCDATL
    ;
    
    movffl  H'6B', CRCDATL
    ;
    movffl  H'6C', CRCDATL
    ;
    movffl  H'6D', CRCDATL
    ;
    movffl  H'6E', CRCDATL
    ;
    movffl  H'6F', CRCDATL
    ;
    movffl  H'70', CRCDATL
    ;
    movffl  H'71', CRCDATL
    ;
    movffl  H'72', CRCDATL
    ;
    
    movffl  H'73', CRCDATL
    ;
    movffl  H'74', CRCDATL
    ;
    movffl  H'75', CRCDATL
    ;
    movffl  H'76', CRCDATL
    ;
    movffl  H'77', CRCDATL
    ;
    movffl  H'78', CRCDATL
    ;
    movffl  H'79', CRCDATL
    ;
    movffl  H'7A', CRCDATL
    ;
    
    movffl  H'7B', CRCDATL
    ;
    movffl  H'7C', CRCDATL
    ;
    movffl  H'7D', CRCDATL
    ;
    movffl  H'7E', CRCDATL
    ;
    movffl  H'7F', CRCDATL
    ;
    movffl  H'80', CRCDATL
    ;
    movffl  H'81', CRCDATL
    ;
    movffl  H'82', CRCDATL
    nop
    
    movffl  CRCACCL, WREG
    
    bcf	    CRCCON0, CRCGO
    clrf    CRCACCL
    
    movlb   B'000000'
    
    return