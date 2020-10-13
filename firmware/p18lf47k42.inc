        LIST

;==========================================================================
; Build date : Jan 21 2019
;  MPASM PIC18LF47K42 processor include
; 
;  (c) Copyright 1999-2019 Microchip Technology, All rights reserved
;==========================================================================

        NOLIST

;==========================================================================
;  This header file defines configurations, registers, and other useful
;  bits of information for the PIC18LF47K42 microcontroller.  These names
;  are taken to match the data sheets as closely as possible.
;
;  Note that the processor must be selected before this file is included.
;  The processor may be selected the following ways:
;
;       1. Command line switch:
;               C:\MPASM MYFILE.ASM /PIC18LF47K42
;       2. LIST directive in the source file
;               LIST   P=PIC18LF47K42
;       3. Processor Type entry in the MPASM full-screen interface
;       4. Setting the processor in the MPLAB Project Dialog
;==========================================================================

;==========================================================================
;
;       Verify Processor
;
;==========================================================================
        IFNDEF __18LF47K42
           MESSG "Processor-header file mismatch.  Verify selected processor."
        ENDIF

;==========================================================================
;       18xxxx Family        EQUates
;==========================================================================
FSR0             EQU  0
FSR1             EQU  1
FSR2             EQU  2

FAST             EQU  1

W                EQU  0
A                EQU  0
ACCESS           EQU  0
BANKED           EQU  1
;==========================================================================

;==========================================================================
;       16Cxxx/17Cxxx Substitutions
;==========================================================================
  #define DDRA  TRISA      ; PIC17Cxxx SFR substitution
  #define DDRB  TRISB      ; PIC17Cxxx SFR substitution
  #define DDRC  TRISC      ; PIC17Cxxx SFR substitution
  #define DDRD  TRISD      ; PIC17Cxxx SFR substitution
  #define DDRE  TRISE      ; PIC17Cxxx SFR substitution

;==========================================================================
;
;       Register Definitions
;
;==========================================================================

;----- Register Files -----------------------------------------------------
STATUS_CSHAD     EQU  H'3880'
WREG_CSHAD       EQU  H'3881'
BSR_CSHAD        EQU  H'3882'
SHADCON          EQU  H'3883'
STATUS_SHAD      EQU  H'3884'
WREG_SHAD        EQU  H'3885'
BSR_SHAD         EQU  H'3886'
PCLATH_SHAD      EQU  H'3887'
PCLATU_SHAD      EQU  H'3888'
FSR0L_SHAD       EQU  H'3889'
FSR0SH           EQU  H'3889'
FSR0H_SHAD       EQU  H'388A'
FSR1L_SHAD       EQU  H'388B'
FSR1SH           EQU  H'388B'
FSR1H_SHAD       EQU  H'388C'
FSR2L_SHAD       EQU  H'388D'
FSR2SH           EQU  H'388D'
FSR2H_SHAD       EQU  H'388E'
PRODL_SHAD       EQU  H'388F'
PRODSH           EQU  H'388F'
PRODH_SHAD       EQU  H'3890'
IVTADL           EQU  H'389D'
IVTADH           EQU  H'389E'
IVTADU           EQU  H'389F'
WDTCON0          EQU  H'395B'
WDTCON1          EQU  H'395C'
WDTPSL           EQU  H'395D'
WDTPSH           EQU  H'395E'
WDTTMR           EQU  H'395F'
CRCDATA          EQU  H'3960'
CRCDATL          EQU  H'3960'
CRCDATH          EQU  H'3961'
CRCACC           EQU  H'3962'
CRCACCL          EQU  H'3962'
CRCACCH          EQU  H'3963'
CRCSHFT          EQU  H'3964'
CRCSHIFTL        EQU  H'3964'
CRCSHIFTH        EQU  H'3965'
CRCXOR           EQU  H'3966'
CRCXORL          EQU  H'3966'
CRCXORH          EQU  H'3967'
CRCCON0          EQU  H'3968'
CRCCON1          EQU  H'3969'
SCANLADR         EQU  H'3976'
SCANLADRL        EQU  H'3976'
SCANLADRH        EQU  H'3977'
SCANLADRU        EQU  H'3978'
SCANHADR         EQU  H'3979'
SCANHADRL        EQU  H'3979'
SCANHADRH        EQU  H'397A'
SCANHADRU        EQU  H'397B'
SCANCON0         EQU  H'397C'
SCANTRIG         EQU  H'397D'
IPR0             EQU  H'3980'
IPR1             EQU  H'3981'
IPR2             EQU  H'3982'
IPR3             EQU  H'3983'
IPR4             EQU  H'3984'
IPR5             EQU  H'3985'
IPR6             EQU  H'3986'
IPR7             EQU  H'3987'
IPR8             EQU  H'3988'
IPR9             EQU  H'3989'
IPR10            EQU  H'398A'
PIE0             EQU  H'3990'
PIE1             EQU  H'3991'
PIE2             EQU  H'3992'
PIE3             EQU  H'3993'
PIE4             EQU  H'3994'
PIE5             EQU  H'3995'
PIE6             EQU  H'3996'
PIE7             EQU  H'3997'
PIE8             EQU  H'3998'
PIE9             EQU  H'3999'
PIE10            EQU  H'399A'
PIR0             EQU  H'39A0'
PIR1             EQU  H'39A1'
PIR2             EQU  H'39A2'
PIR3             EQU  H'39A3'
PIR4             EQU  H'39A4'
PIR5             EQU  H'39A5'
PIR6             EQU  H'39A6'
PIR7             EQU  H'39A7'
PIR8             EQU  H'39A8'
PIR9             EQU  H'39A9'
PIR10            EQU  H'39AA'
PMD0             EQU  H'39C0'
PMD1             EQU  H'39C1'
PMD2             EQU  H'39C2'
PMD3             EQU  H'39C3'
PMD4             EQU  H'39C4'
PMD5             EQU  H'39C5'
PMD6             EQU  H'39C6'
PMD7             EQU  H'39C7'
BORCON           EQU  H'39D0'
CPUDOZE          EQU  H'39D8'
OSCCON1          EQU  H'39D9'
OSCCON2          EQU  H'39DA'
OSCCON3          EQU  H'39DB'
OSCSTAT          EQU  H'39DC'
OSCSTAT1         EQU  H'39DC'
OSCEN            EQU  H'39DD'
OSCTUNE          EQU  H'39DE'
OSCFRQ           EQU  H'39DF'
NVMADRL          EQU  H'39E0'
NVMADRH          EQU  H'39E1'
NVMDAT           EQU  H'39E3'
NVMCON1          EQU  H'39E5'
NVMCON2          EQU  H'39E6'
PRLOCK           EQU  H'39EF'
ISRPR            EQU  H'39F1'
MAINPR           EQU  H'39F2'
DMA1PR           EQU  H'39F3'
DMA2PR           EQU  H'39F4'
SCANPR           EQU  H'39F7'
RA0PPS           EQU  H'3A00'
RA1PPS           EQU  H'3A01'
RA2PPS           EQU  H'3A02'
RA3PPS           EQU  H'3A03'
RA4PPS           EQU  H'3A04'
RA5PPS           EQU  H'3A05'
RA6PPS           EQU  H'3A06'
RA7PPS           EQU  H'3A07'
RB0PPS           EQU  H'3A08'
RB1PPS           EQU  H'3A09'
RB2PPS           EQU  H'3A0A'
RB3PPS           EQU  H'3A0B'
RB4PPS           EQU  H'3A0C'
RB5PPS           EQU  H'3A0D'
RB6PPS           EQU  H'3A0E'
RB7PPS           EQU  H'3A0F'
RC0PPS           EQU  H'3A10'
RC1PPS           EQU  H'3A11'
RC2PPS           EQU  H'3A12'
RC3PPS           EQU  H'3A13'
RC4PPS           EQU  H'3A14'
RC5PPS           EQU  H'3A15'
RC6PPS           EQU  H'3A16'
RC7PPS           EQU  H'3A17'
RD0PPS           EQU  H'3A18'
RD1PPS           EQU  H'3A19'
RD2PPS           EQU  H'3A1A'
RD3PPS           EQU  H'3A1B'
RD4PPS           EQU  H'3A1C'
RD5PPS           EQU  H'3A1D'
RD6PPS           EQU  H'3A1E'
RD7PPS           EQU  H'3A1F'
RE0PPS           EQU  H'3A20'
RE1PPS           EQU  H'3A21'
RE2PPS           EQU  H'3A22'
ANSELA           EQU  H'3A40'
WPUA             EQU  H'3A41'
ODCONA           EQU  H'3A42'
SLRCONA          EQU  H'3A43'
INLVLA           EQU  H'3A44'
IOCAP            EQU  H'3A45'
IOCAN            EQU  H'3A46'
IOCAF            EQU  H'3A47'
ANSELB           EQU  H'3A50'
WPUB             EQU  H'3A51'
ODCONB           EQU  H'3A52'
SLRCONB          EQU  H'3A53'
INLVLB           EQU  H'3A54'
IOCBP            EQU  H'3A55'
IOCBN            EQU  H'3A56'
IOCBF            EQU  H'3A57'
RB1I2C           EQU  H'3A5A'
RB2I2C           EQU  H'3A5B'
ANSELC           EQU  H'3A60'
WPUC             EQU  H'3A61'
ODCONC           EQU  H'3A62'
SLRCONC          EQU  H'3A63'
INLVLC           EQU  H'3A64'
IOCCP            EQU  H'3A65'
IOCCN            EQU  H'3A66'
IOCCF            EQU  H'3A67'
RC3I2C           EQU  H'3A6A'
RC4I2C           EQU  H'3A6B'
ANSELD           EQU  H'3A70'
WPUD             EQU  H'3A71'
ODCOND           EQU  H'3A72'
SLRCOND          EQU  H'3A73'
INLVLD           EQU  H'3A74'
RD0I2C           EQU  H'3A7A'
RD1I2C           EQU  H'3A7B'
ANSELE           EQU  H'3A80'
WPUE             EQU  H'3A81'
ODCONE           EQU  H'3A82'
SLRCONE          EQU  H'3A83'
INLVLE           EQU  H'3A84'
IOCEP            EQU  H'3A85'
IOCEN            EQU  H'3A86'
IOCEF            EQU  H'3A87'
PPSLOCK          EQU  H'3ABF'
INT0PPS          EQU  H'3AC0'
INT1PPS          EQU  H'3AC1'
INT2PPS          EQU  H'3AC2'
T0CKIPPS         EQU  H'3AC3'
T1CKIPPS         EQU  H'3AC4'
T1GPPS           EQU  H'3AC5'
T3CKIPPS         EQU  H'3AC6'
T3GPPS           EQU  H'3AC7'
T5CKIPPS         EQU  H'3AC8'
T5GPPS           EQU  H'3AC9'
T2INPPS          EQU  H'3ACA'
T4INPPS          EQU  H'3ACB'
T6INPPS          EQU  H'3ACC'
CCP1PPS          EQU  H'3ACD'
CCP2PPS          EQU  H'3ACE'
CCP3PPS          EQU  H'3ACF'
CCP4PPS          EQU  H'3AD0'
SMT1WINPPS       EQU  H'3AD1'
SMT1SIGPPS       EQU  H'3AD2'
CWG1INPPS        EQU  H'3AD3'
CWG2INPPS        EQU  H'3AD4'
CWG3INPPS        EQU  H'3AD5'
MD1CARLPPS       EQU  H'3AD6'
MD1CARHPPS       EQU  H'3AD7'
MD1SRCPPS        EQU  H'3AD8'
CLCIN0PPS        EQU  H'3AD9'
CLCIN1PPS        EQU  H'3ADA'
CLCIN2PPS        EQU  H'3ADB'
CLCIN3PPS        EQU  H'3ADC'
ADACTPPS         EQU  H'3ADD'
SPI1SCKPPS       EQU  H'3ADE'
SPI1SDIPPS       EQU  H'3ADF'
SPI1SSPPS        EQU  H'3AE0'
I2C1SCLPPS       EQU  H'3AE1'
I2C1SDAPPS       EQU  H'3AE2'
I2C2SCLPPS       EQU  H'3AE3'
I2C2SDAPPS       EQU  H'3AE4'
U1RXPPS          EQU  H'3AE5'
U1CTSPPS         EQU  H'3AE6'
U2RXPPS          EQU  H'3AE8'
U2CTSPPS         EQU  H'3AE9'
DMA2BUF          EQU  H'3BC9'
DMA2DCNT         EQU  H'3BCA'
DMA2DCNTL        EQU  H'3BCA'
DMA2DCNTH        EQU  H'3BCB'
DMA2DPTR         EQU  H'3BCC'
DMA2DPTRL        EQU  H'3BCC'
DMA2DPTRH        EQU  H'3BCD'
DMA2DSZ          EQU  H'3BCE'
DMA2DSZL         EQU  H'3BCE'
DMA2DSZH         EQU  H'3BCF'
DMA2DSA          EQU  H'3BD0'
DMA2DSAL         EQU  H'3BD0'
DMA2DSAH         EQU  H'3BD1'
DMA2SCNT         EQU  H'3BD2'
DMA2SCNTL        EQU  H'3BD2'
DMA2SCNTH        EQU  H'3BD3'
DMA2SPTR         EQU  H'3BD4'
DMA2SPTRL        EQU  H'3BD4'
DMA2SPTRH        EQU  H'3BD5'
DMA2SPTRU        EQU  H'3BD6'
DMA2SSZ          EQU  H'3BD7'
DMA2SSZL         EQU  H'3BD7'
DMA2SSZH         EQU  H'3BD8'
DMA2SSA          EQU  H'3BD9'
DMA2SSAL         EQU  H'3BD9'
DMA2SSAH         EQU  H'3BDA'
DMA2SSAU         EQU  H'3BDB'
DMA2CON0         EQU  H'3BDC'
DMA2CON1         EQU  H'3BDD'
DMA2AIRQ         EQU  H'3BDE'
DMA2SIRQ         EQU  H'3BDF'
DMA1BUF          EQU  H'3BE9'
DMA1DCNT         EQU  H'3BEA'
DMA1DCNTL        EQU  H'3BEA'
DMA1DCNTH        EQU  H'3BEB'
DMA1DPTR         EQU  H'3BEC'
DMA1DPTRL        EQU  H'3BEC'
DMA1DPTRH        EQU  H'3BED'
DMA1DSZ          EQU  H'3BEE'
DMA1DSZL         EQU  H'3BEE'
DMA1DSZH         EQU  H'3BEF'
DMA1DSA          EQU  H'3BF0'
DMA1DSAL         EQU  H'3BF0'
DMA1DSAH         EQU  H'3BF1'
DMA1SCNT         EQU  H'3BF2'
DMA1SCNTL        EQU  H'3BF2'
DMA1SCNTH        EQU  H'3BF3'
DMA1SPTR         EQU  H'3BF4'
DMA1SPTRL        EQU  H'3BF4'
DMA1SPTRH        EQU  H'3BF5'
DMA1SPTRU        EQU  H'3BF6'
DMA1SSZ          EQU  H'3BF7'
DMA1SSZL         EQU  H'3BF7'
DMA1SSZH         EQU  H'3BF8'
DMA1SSA          EQU  H'3BF9'
DMA1SSAL         EQU  H'3BF9'
DMA1SSAH         EQU  H'3BFA'
DMA1SSAU         EQU  H'3BFB'
DMA1CON0         EQU  H'3BFC'
DMA1CON1         EQU  H'3BFD'
DMA1AIRQ         EQU  H'3BFE'
DMA1SIRQ         EQU  H'3BFF'
CLC4CON          EQU  H'3C56'
CLC4POL          EQU  H'3C57'
CLC4SEL0         EQU  H'3C58'
CLC4SEL1         EQU  H'3C59'
CLC4SEL2         EQU  H'3C5A'
CLC4SEL3         EQU  H'3C5B'
CLC4GLS0         EQU  H'3C5C'
CLC4GLS1         EQU  H'3C5D'
CLC4GLS2         EQU  H'3C5E'
CLC4GLS3         EQU  H'3C5F'
CLC3CON          EQU  H'3C60'
CLC3POL          EQU  H'3C61'
CLC3SEL0         EQU  H'3C62'
CLC3SEL1         EQU  H'3C63'
CLC3SEL2         EQU  H'3C64'
CLC3SEL3         EQU  H'3C65'
CLC3GLS0         EQU  H'3C66'
CLC3GLS1         EQU  H'3C67'
CLC3GLS2         EQU  H'3C68'
CLC3GLS3         EQU  H'3C69'
CLC2CON          EQU  H'3C6A'
CLC2POL          EQU  H'3C6B'
CLC2SEL0         EQU  H'3C6C'
CLC2SEL1         EQU  H'3C6D'
CLC2SEL2         EQU  H'3C6E'
CLC2SEL3         EQU  H'3C6F'
CLC2GLS0         EQU  H'3C70'
CLC2GLS1         EQU  H'3C71'
CLC2GLS2         EQU  H'3C72'
CLC2GLS3         EQU  H'3C73'
CLC1CON          EQU  H'3C74'
CLC1POL          EQU  H'3C75'
CLC1SEL0         EQU  H'3C76'
CLC1SEL1         EQU  H'3C77'
CLC1SEL2         EQU  H'3C78'
CLC1SEL3         EQU  H'3C79'
CLC1GLS0         EQU  H'3C7A'
CLC1GLS1         EQU  H'3C7B'
CLC1GLS2         EQU  H'3C7C'
CLC1GLS3         EQU  H'3C7D'
CLCDATA0         EQU  H'3C7E'
CLKRCON          EQU  H'3CE5'
CLKRCLK          EQU  H'3CE6'
MD1CON0          EQU  H'3CFA'
MD1CON1          EQU  H'3CFB'
MD1SRC           EQU  H'3CFC'
MD1CARL          EQU  H'3CFD'
MD1CARH          EQU  H'3CFE'
SPI1RXB          EQU  H'3D10'
SPI1TXB          EQU  H'3D11'
SPI1TCNT         EQU  H'3D12'
SPI1TCNTL        EQU  H'3D12'
SPI1TCNTH        EQU  H'3D13'
SPI1CON0         EQU  H'3D14'
SPI1CON1         EQU  H'3D15'
SPI1CON2         EQU  H'3D16'
SPI1STATUS       EQU  H'3D17'
SPI1TWIDTH       EQU  H'3D18'
SPI1BAUD         EQU  H'3D19'
SPI1INTF         EQU  H'3D1A'
SPI1INTE         EQU  H'3D1B'
SPI1CLK          EQU  H'3D1C'
I2C2RXB          EQU  H'3D54'
I2C2TXB          EQU  H'3D55'
I2C2CNT          EQU  H'3D56'
I2C2ADB0         EQU  H'3D57'
I2C2ADB1         EQU  H'3D58'
I2C2ADR0         EQU  H'3D59'
I2C2ADR1         EQU  H'3D5A'
I2C2ADR2         EQU  H'3D5B'
I2C2ADR3         EQU  H'3D5C'
I2C2CON0         EQU  H'3D5D'
I2C2CON1         EQU  H'3D5E'
I2C2CON2         EQU  H'3D5F'
I2C2ERR          EQU  H'3D60'
I2C2STAT0        EQU  H'3D61'
I2C2STAT1        EQU  H'3D62'
I2C2PIR          EQU  H'3D63'
I2C2PIE          EQU  H'3D64'
I2C2CLK          EQU  H'3D65'
I2C2BTO          EQU  H'3D66'
I2C1RXB          EQU  H'3D6A'
I2C1TXB          EQU  H'3D6B'
I2C1CNT          EQU  H'3D6C'
I2C1ADB0         EQU  H'3D6D'
I2C1ADB1         EQU  H'3D6E'
I2C1ADR0         EQU  H'3D6F'
I2C1ADR1         EQU  H'3D70'
I2C1ADR2         EQU  H'3D71'
I2C1ADR3         EQU  H'3D72'
I2C1CON0         EQU  H'3D73'
I2C1CON1         EQU  H'3D74'
I2C1CON2         EQU  H'3D75'
I2C1ERR          EQU  H'3D76'
I2C1STAT0        EQU  H'3D77'
I2C1STAT1        EQU  H'3D78'
I2C1PIR          EQU  H'3D79'
I2C1PIE          EQU  H'3D7A'
I2C1CLK          EQU  H'3D7B'
I2C1BTO          EQU  H'3D7C'
U2RXB            EQU  H'3DD0'
U2RXBL           EQU  H'3DD0'
U2TXB            EQU  H'3DD2'
U2TXBL           EQU  H'3DD2'
U2P1             EQU  H'3DD4'
U2P1L            EQU  H'3DD4'
U2P2             EQU  H'3DD6'
U2P2L            EQU  H'3DD6'
U2P3             EQU  H'3DD8'
U2P3L            EQU  H'3DD8'
U2CON0           EQU  H'3DDA'
U2CON1           EQU  H'3DDB'
U2CON2           EQU  H'3DDC'
U2BRG            EQU  H'3DDD'
U2BRGL           EQU  H'3DDD'
U2BRGH           EQU  H'3DDE'
U2FIFO           EQU  H'3DDF'
U2UIR            EQU  H'3DE0'
U2ERRIR          EQU  H'3DE1'
U2ERRIE          EQU  H'3DE2'
U1RXB            EQU  H'3DE8'
U1RXBL           EQU  H'3DE8'
U1RXCHK          EQU  H'3DE9'
U1TXB            EQU  H'3DEA'
U1TXBL           EQU  H'3DEA'
U1TXCHK          EQU  H'3DEB'
U1P1             EQU  H'3DEC'
U1P1L            EQU  H'3DEC'
U1P1H            EQU  H'3DED'
U1P2             EQU  H'3DEE'
U1P2L            EQU  H'3DEE'
U1P2H            EQU  H'3DEF'
U1P3             EQU  H'3DF0'
U1P3L            EQU  H'3DF0'
U1P3H            EQU  H'3DF1'
U1CON0           EQU  H'3DF2'
U1CON1           EQU  H'3DF3'
U1CON2           EQU  H'3DF4'
U1BRG            EQU  H'3DF5'
U1BRGL           EQU  H'3DF5'
U1BRGH           EQU  H'3DF6'
U1FIFO           EQU  H'3DF7'
U1UIR            EQU  H'3DF8'
U1ERRIR          EQU  H'3DF9'
U1ERRIE          EQU  H'3DFA'
DAC1CON1         EQU  H'3E9C'
DAC1CON0         EQU  H'3E9E'
CM2CON0          EQU  H'3EB8'
CM2CON1          EQU  H'3EB9'
CM2NCH           EQU  H'3EBA'
CM2PCH           EQU  H'3EBB'
CM1CON0          EQU  H'3EBC'
CM1CON1          EQU  H'3EBD'
CM1NCH           EQU  H'3EBE'
CM1PCH           EQU  H'3EBF'
CMOUT            EQU  H'3EC0'
FVRCON           EQU  H'3EC1'
ZCDCON           EQU  H'3EC3'
HLVDCON0         EQU  H'3EC9'
HLVDCON1         EQU  H'3ECA'
ADCP             EQU  H'3ED7'
ADLTH            EQU  H'3EDE'
ADLTHL           EQU  H'3EDE'
ADLTHH           EQU  H'3EDF'
ADUTH            EQU  H'3EE0'
ADUTHL           EQU  H'3EE0'
ADUTHH           EQU  H'3EE1'
ADERR            EQU  H'3EE2'
ADERRL           EQU  H'3EE2'
ADERRH           EQU  H'3EE3'
ADSTPT           EQU  H'3EE4'
ADSTPTL          EQU  H'3EE4'
ADSTPTH          EQU  H'3EE5'
ADFLTR           EQU  H'3EE6'
ADFLTRL          EQU  H'3EE6'
ADFLTRH          EQU  H'3EE7'
ADACC            EQU  H'3EE8'
ADACCL           EQU  H'3EE8'
ADACCH           EQU  H'3EE9'
ADACCU           EQU  H'3EEA'
ADCNT            EQU  H'3EEB'
ADRPT            EQU  H'3EEC'
ADPREV           EQU  H'3EED'
ADPREVL          EQU  H'3EED'
ADPREVH          EQU  H'3EEE'
ADRES            EQU  H'3EEF'
ADRESL           EQU  H'3EEF'
ADRESH           EQU  H'3EF0'
ADPCH            EQU  H'3EF1'
ADACQ            EQU  H'3EF3'
ADACQL           EQU  H'3EF3'
ADACQH           EQU  H'3EF4'
ADCAP            EQU  H'3EF5'
ADPRE            EQU  H'3EF6'
ADPREL           EQU  H'3EF6'
ADPREH           EQU  H'3EF7'
ADCON0           EQU  H'3EF8'
ADCON1           EQU  H'3EF9'
ADCON2           EQU  H'3EFA'
ADCON3           EQU  H'3EFB'
ADSTAT           EQU  H'3EFC'
ADREF            EQU  H'3EFD'
ADACT            EQU  H'3EFE'
ADCLK            EQU  H'3EFF'
SMT1TMR          EQU  H'3F12'
SMT1TMRL         EQU  H'3F12'
SMT1TMRH         EQU  H'3F13'
SMT1TMRU         EQU  H'3F14'
SMT1CPR          EQU  H'3F15'
SMT1CPRL         EQU  H'3F15'
SMT1CPRH         EQU  H'3F16'
SMT1CPRU         EQU  H'3F17'
SMT1CPW          EQU  H'3F18'
SMT1CPWL         EQU  H'3F18'
SMT1CPWH         EQU  H'3F19'
SMT1CPWU         EQU  H'3F1A'
SMT1PR           EQU  H'3F1B'
SMT1PRL          EQU  H'3F1B'
SMT1PRH          EQU  H'3F1C'
SMT1PRU          EQU  H'3F1D'
SMT1CON0         EQU  H'3F1E'
SMT1CON1         EQU  H'3F1F'
SMT1STAT         EQU  H'3F20'
SMT1CLK          EQU  H'3F21'
SMT1SIG          EQU  H'3F22'
SMT1WIN          EQU  H'3F23'
NCO1ACC          EQU  H'3F38'
NCO1ACCL         EQU  H'3F38'
NCO1ACCH         EQU  H'3F39'
NCO1ACCU         EQU  H'3F3A'
NCO1INC          EQU  H'3F3B'
NCO1INCL         EQU  H'3F3B'
NCO1INCH         EQU  H'3F3C'
NCO1INCU         EQU  H'3F3D'
NCO1CON          EQU  H'3F3E'
NCO1CLK          EQU  H'3F3F'
CWG3CLK          EQU  H'3F40'
CWG3CLKCON       EQU  H'3F40'
CWG3ISM          EQU  H'3F41'
CWG3DBR          EQU  H'3F42'
CWG3DBF          EQU  H'3F43'
CWG3CON0         EQU  H'3F44'
CWG3CON1         EQU  H'3F45'
CWG3AS0          EQU  H'3F46'
CWG3AS1          EQU  H'3F47'
CWG3STR          EQU  H'3F48'
CWG2CLK          EQU  H'3F49'
CWG2CLKCON       EQU  H'3F49'
CWG2ISM          EQU  H'3F4A'
CWG2DBR          EQU  H'3F4B'
CWG2DBF          EQU  H'3F4C'
CWG2CON0         EQU  H'3F4D'
CWG2CON1         EQU  H'3F4E'
CWG2AS0          EQU  H'3F4F'
CWG2AS1          EQU  H'3F50'
CWG2STR          EQU  H'3F51'
CWG1CLK          EQU  H'3F52'
CWG1CLKCON       EQU  H'3F52'
CWG1ISM          EQU  H'3F53'
CWG1DBR          EQU  H'3F54'
CWG1DBF          EQU  H'3F55'
CWG1CON0         EQU  H'3F56'
CWG1CON1         EQU  H'3F57'
CWG1AS0          EQU  H'3F58'
CWG1AS1          EQU  H'3F59'
CWG1STR          EQU  H'3F5A'
CCPTMRS0         EQU  H'3F5E'
CCPTMRS1         EQU  H'3F5F'
PWM8DC           EQU  H'3F60'
PWM8DCL          EQU  H'3F60'
PWM8DCH          EQU  H'3F61'
PWM8CON          EQU  H'3F62'
PWM7DC           EQU  H'3F64'
PWM7DCL          EQU  H'3F64'
PWM7DCH          EQU  H'3F65'
PWM7CON          EQU  H'3F66'
PWM6DC           EQU  H'3F68'
PWM6DCL          EQU  H'3F68'
PWM6DCH          EQU  H'3F69'
PWM6CON          EQU  H'3F6A'
PWM5DC           EQU  H'3F6C'
PWM5DCL          EQU  H'3F6C'
PWM5DCH          EQU  H'3F6D'
PWM5CON          EQU  H'3F6E'
CCPR4            EQU  H'3F70'
CCPR4L           EQU  H'3F70'
CCPR4H           EQU  H'3F71'
CCP4CON          EQU  H'3F72'
CCP4CAP          EQU  H'3F73'
CCPR3            EQU  H'3F74'
CCPR3L           EQU  H'3F74'
CCPR3H           EQU  H'3F75'
CCP3CON          EQU  H'3F76'
CCP3CAP          EQU  H'3F77'
CCPR2            EQU  H'3F78'
CCPR2L           EQU  H'3F78'
CCPR2H           EQU  H'3F79'
CCP2CON          EQU  H'3F7A'
CCP2CAP          EQU  H'3F7B'
CCPR1            EQU  H'3F7C'
CCPR1L           EQU  H'3F7C'
CCPR1H           EQU  H'3F7D'
CCP1CON          EQU  H'3F7E'
CCP1CAP          EQU  H'3F7F'
T6TMR            EQU  H'3F92'
TMR6             EQU  H'3F92'
PR6              EQU  H'3F93'
T6PR             EQU  H'3F93'
T6CON            EQU  H'3F94'
T6HLT            EQU  H'3F95'
T6CLK            EQU  H'3F96'
T6CLKCON         EQU  H'3F96'
T6RST            EQU  H'3F97'
TMR5L            EQU  H'3F98'
TMR5H            EQU  H'3F99'
T5CON            EQU  H'3F9A'
TMR5CON          EQU  H'3F9A'
T5GCON           EQU  H'3F9B'
TMR5GCON         EQU  H'3F9B'
T5GATE           EQU  H'3F9C'
TMR5GATE         EQU  H'3F9C'
T5CLK            EQU  H'3F9D'
TMR5CLK          EQU  H'3F9D'
T4TMR            EQU  H'3F9E'
TMR4             EQU  H'3F9E'
PR4              EQU  H'3F9F'
T4PR             EQU  H'3F9F'
T4CON            EQU  H'3FA0'
T4HLT            EQU  H'3FA1'
T4CLK            EQU  H'3FA2'
T4CLKCON         EQU  H'3FA2'
T4RST            EQU  H'3FA3'
TMR3L            EQU  H'3FA4'
TMR3H            EQU  H'3FA5'
T3CON            EQU  H'3FA6'
TMR3CON          EQU  H'3FA6'
T3GCON           EQU  H'3FA7'
TMR3GCON         EQU  H'3FA7'
T3GATE           EQU  H'3FA8'
TMR3GATE         EQU  H'3FA8'
T3CLK            EQU  H'3FA9'
TMR3CLK          EQU  H'3FA9'
T2TMR            EQU  H'3FAA'
TMR2             EQU  H'3FAA'
PR2              EQU  H'3FAB'
T2PR             EQU  H'3FAB'
T2CON            EQU  H'3FAC'
T2HLT            EQU  H'3FAD'
T2CLK            EQU  H'3FAE'
T2CLKCON         EQU  H'3FAE'
T2RST            EQU  H'3FAF'
TMR1L            EQU  H'3FB0'
TMR1H            EQU  H'3FB1'
T1CON            EQU  H'3FB2'
TMR1CON          EQU  H'3FB2'
T1GCON           EQU  H'3FB3'
TMR1GCON         EQU  H'3FB3'
T1GATE           EQU  H'3FB4'
TMR1GATE         EQU  H'3FB4'
T1CLK            EQU  H'3FB5'
TMR1CLK          EQU  H'3FB5'
TMR0             EQU  H'3FB6'
TMR0L            EQU  H'3FB6'
PR0              EQU  H'3FB7'
TMR0H            EQU  H'3FB7'
T0CON0           EQU  H'3FB8'
T0CON1           EQU  H'3FB9'
LATA             EQU  H'3FBA'
LATB             EQU  H'3FBB'
LATC             EQU  H'3FBC'
LATD             EQU  H'3FBD'
LATE             EQU  H'3FBE'
TRISA            EQU  H'3FC2'
TRISB            EQU  H'3FC3'
TRISC            EQU  H'3FC4'
TRISD            EQU  H'3FC5'
TRISE            EQU  H'3FC6'
PORTA            EQU  H'3FCA'
PORTB            EQU  H'3FCB'
PORTC            EQU  H'3FCC'
PORTD            EQU  H'3FCD'
PORTE            EQU  H'3FCE'
INTCON0          EQU  H'3FD2'
INTCON1          EQU  H'3FD3'
IVTLOCK          EQU  H'3FD4'
IVTBASE          EQU  H'3FD5'
IVTBASEL         EQU  H'3FD5'
IVTBASEH         EQU  H'3FD6'
IVTBASEU         EQU  H'3FD7'
STATUS           EQU  H'3FD8'
FSR2L            EQU  H'3FD9'
FSR2H            EQU  H'3FDA'
PLUSW2           EQU  H'3FDB'
PREINC2          EQU  H'3FDC'
POSTDEC2         EQU  H'3FDD'
POSTINC2         EQU  H'3FDE'
INDF2            EQU  H'3FDF'
BSR              EQU  H'3FE0'
FSR1L            EQU  H'3FE1'
FSR1H            EQU  H'3FE2'
PLUSW1           EQU  H'3FE3'
PREINC1          EQU  H'3FE4'
POSTDEC1         EQU  H'3FE5'
POSTINC1         EQU  H'3FE6'
INDF1            EQU  H'3FE7'
WREG             EQU  H'3FE8'
FSR0L            EQU  H'3FE9'
FSR0H            EQU  H'3FEA'
PLUSW0           EQU  H'3FEB'
PREINC0          EQU  H'3FEC'
POSTDEC0         EQU  H'3FED'
POSTINC0         EQU  H'3FEE'
INDF0            EQU  H'3FEF'
PCON0            EQU  H'3FF0'
PCON1            EQU  H'3FF1'
PROD             EQU  H'3FF3'
PRODL            EQU  H'3FF3'
PRODH            EQU  H'3FF4'
TABLAT           EQU  H'3FF5'
TBLPTR           EQU  H'3FF6'
TBLPTRL          EQU  H'3FF6'
TBLPTRH          EQU  H'3FF7'
TBLPTRU          EQU  H'3FF8'
PC               EQU  H'3FF9'
PCL              EQU  H'3FF9'
PCLATH           EQU  H'3FFA'
PCLATU           EQU  H'3FFB'
STKPTR           EQU  H'3FFC'
TOS              EQU  H'3FFD'
TOSL             EQU  H'3FFD'
TOSH             EQU  H'3FFE'
TOSU             EQU  H'3FFF'

;----- STATUS_CSHAD Bits -----------------------------------------------------
C                EQU  H'0000'
DC               EQU  H'0001'
Z                EQU  H'0002'
OV_STATUS_CSHAD  EQU  H'0003'
N                EQU  H'0004'
NOT_PD           EQU  H'0005'
NOT_TO           EQU  H'0006'

PD               EQU  H'0005'
TO               EQU  H'0006'


;----- SHADCON Bits -----------------------------------------------------
SHADLO           EQU  H'0000'


;----- STATUS_SHAD Bits -----------------------------------------------------
C                EQU  H'0000'
DC               EQU  H'0001'
Z                EQU  H'0002'
OV_STATUS_SHAD   EQU  H'0003'
N                EQU  H'0004'
NOT_PD           EQU  H'0005'
NOT_TO           EQU  H'0006'

PD               EQU  H'0005'
TO               EQU  H'0006'


;----- WDTCON0 Bits -----------------------------------------------------
SEN_WDTCON0      EQU  H'0000'

SWDTEN           EQU  H'0000'

WDTSEN           EQU  H'0000'

WDTPS0           EQU  H'0001'
WDTPS1           EQU  H'0002'
WDTPS2           EQU  H'0003'
WDTPS3           EQU  H'0004'
WDTPS4           EQU  H'0005'


;----- WDTCON1 Bits -----------------------------------------------------
WINDOW0          EQU  H'0000'
WINDOW1          EQU  H'0001'
WINDOW2          EQU  H'0002'


WDTWINDOW0       EQU  H'0000'
WDTWINDOW1       EQU  H'0001'
WDTWINDOW2       EQU  H'0002'
WDTCS0           EQU  H'0004'
WDTCS1           EQU  H'0005'
WDTCS2           EQU  H'0006'


;----- WDTPSL Bits -----------------------------------------------------
PSCNT0           EQU  H'0000'
PSCNT1           EQU  H'0001'
PSCNT2           EQU  H'0002'
PSCNT3           EQU  H'0003'
PSCNT4           EQU  H'0004'
PSCNT5           EQU  H'0005'
PSCNT6           EQU  H'0006'
PSCNT7           EQU  H'0007'


WDTPSCNT0        EQU  H'0000'
WDTPSCNT1        EQU  H'0001'
WDTPSCNT2        EQU  H'0002'
WDTPSCNT3        EQU  H'0003'
WDTPSCNT4        EQU  H'0004'
WDTPSCNT5        EQU  H'0005'
WDTPSCNT6        EQU  H'0006'
WDTPSCNT7        EQU  H'0007'


;----- WDTPSH Bits -----------------------------------------------------
PSCNT8           EQU  H'0000'
PSCNT9           EQU  H'0001'
PSCNT10          EQU  H'0002'
PSCNT11          EQU  H'0003'
PSCNT12          EQU  H'0004'
PSCNT13          EQU  H'0005'
PSCNT14          EQU  H'0006'
PSCNT15          EQU  H'0007'


WDTPSCNT8        EQU  H'0000'
WDTPSCNT9        EQU  H'0001'
WDTPSCNT10       EQU  H'0002'
WDTPSCNT11       EQU  H'0003'
WDTPSCNT12       EQU  H'0004'
WDTPSCNT13       EQU  H'0005'
WDTPSCNT14       EQU  H'0006'
WDTPSCNT15       EQU  H'0007'


;----- WDTTMR Bits -----------------------------------------------------
PSCNT16          EQU  H'0000'
PSCNT17          EQU  H'0001'
STATE            EQU  H'0002'

WDTPSCNT16       EQU  H'0000'
WDTPSCNT17       EQU  H'0001'
WDTSTATE         EQU  H'0002'
WDTTMR0          EQU  H'0003'
WDTTMR1          EQU  H'0004'
WDTTMR2          EQU  H'0005'
WDTTMR3          EQU  H'0006'
WDTTMR4          EQU  H'0007'


;----- CRCDATL Bits -----------------------------------------------------
DATA0            EQU  H'0000'
DATA1            EQU  H'0001'
DATA2            EQU  H'0002'
DATA3            EQU  H'0003'
DATA4            EQU  H'0004'
DATA5            EQU  H'0005'
DATA6            EQU  H'0006'
DATA7            EQU  H'0007'


;----- CRCDATH Bits -----------------------------------------------------
DATA8            EQU  H'0000'
DATA9            EQU  H'0001'
DATA10           EQU  H'0002'
DATA11           EQU  H'0003'
DATA12           EQU  H'0004'
DATA13           EQU  H'0005'
DATA14           EQU  H'0006'
DATA15           EQU  H'0007'


;----- CRCACCL Bits -----------------------------------------------------
ACC0             EQU  H'0000'
ACC1             EQU  H'0001'
ACC2             EQU  H'0002'
ACC3             EQU  H'0003'
ACC4             EQU  H'0004'
ACC5             EQU  H'0005'
ACC6             EQU  H'0006'
ACC7             EQU  H'0007'


;----- CRCACCH Bits -----------------------------------------------------
ACC8             EQU  H'0000'
ACC9             EQU  H'0001'
ACC10            EQU  H'0002'
ACC11            EQU  H'0003'
ACC12            EQU  H'0004'
ACC13            EQU  H'0005'
ACC14            EQU  H'0006'
ACC15            EQU  H'0007'


;----- CRCSHIFTL Bits -----------------------------------------------------
SHFT0            EQU  H'0000'
SHFT1            EQU  H'0001'
SHFT2            EQU  H'0002'
SHFT3            EQU  H'0003'
SHFT4            EQU  H'0004'
SHFT5            EQU  H'0005'
SHFT6            EQU  H'0006'
SHFT7            EQU  H'0007'


;----- CRCSHIFTH Bits -----------------------------------------------------
SHFT8            EQU  H'0000'
SHFT9            EQU  H'0001'
SHFT10           EQU  H'0002'
SHFT11           EQU  H'0003'
SHFT12           EQU  H'0004'
SHFT13           EQU  H'0005'
SHFT14           EQU  H'0006'
SHFT15           EQU  H'0007'


;----- CRCXORL Bits -----------------------------------------------------
X1               EQU  H'0001'
X2               EQU  H'0002'
X3               EQU  H'0003'
X4               EQU  H'0004'
X5               EQU  H'0005'
X6               EQU  H'0006'
X7               EQU  H'0007'


;----- CRCXORH Bits -----------------------------------------------------
X8               EQU  H'0000'
X9               EQU  H'0001'
X10              EQU  H'0002'
X11              EQU  H'0003'
X12              EQU  H'0004'
X13              EQU  H'0005'
X14              EQU  H'0006'
X15              EQU  H'0007'


;----- CRCCON0 Bits -----------------------------------------------------
FULL             EQU  H'0000'
SHIFTM           EQU  H'0001'
ACCM             EQU  H'0004'
BUSY_CRCCON0     EQU  H'0005'
CRCGO            EQU  H'0006'
EN               EQU  H'0007'

CRCEN            EQU  H'0007'


;----- CRCCON1 Bits -----------------------------------------------------
PLEN0            EQU  H'0000'
PLEN1            EQU  H'0001'
PLEN2            EQU  H'0002'
PLEN3            EQU  H'0003'
DLEN0            EQU  H'0004'
DLEN1            EQU  H'0005'
DLEN2            EQU  H'0006'
DLEN3            EQU  H'0007'


;----- SCANLADRL Bits -----------------------------------------------------
LADR0            EQU  H'0000'
LADR1            EQU  H'0001'
LADR2            EQU  H'0002'
LADR3            EQU  H'0003'
LADR4            EQU  H'0004'
LADR5            EQU  H'0005'
LADR6            EQU  H'0006'
LADR7            EQU  H'0007'


SCANLADR0        EQU  H'0000'
SCANLADR1        EQU  H'0001'
SCANLADR2        EQU  H'0002'
SCANLADR3        EQU  H'0003'
SCANLADR4        EQU  H'0004'
SCANLADR5        EQU  H'0005'
SCANLADR6        EQU  H'0006'
SCANLADR7        EQU  H'0007'


;----- SCANLADRH Bits -----------------------------------------------------
LADR8            EQU  H'0000'
LADR9            EQU  H'0001'
LADR10           EQU  H'0002'
LADR11           EQU  H'0003'
LADR12           EQU  H'0004'
LADR13           EQU  H'0005'
LADR14           EQU  H'0006'
LADR15           EQU  H'0007'


SCANLADR8        EQU  H'0000'
SCANLADR9        EQU  H'0001'
SCANLADR10       EQU  H'0002'
SCANLADR11       EQU  H'0003'
SCANLADR12       EQU  H'0004'
SCANLADR13       EQU  H'0005'
SCANLADR14       EQU  H'0006'
SCANLADR15       EQU  H'0007'


;----- SCANLADRU Bits -----------------------------------------------------
LADR16           EQU  H'0000'
LADR17           EQU  H'0001'
LADR18           EQU  H'0002'
LADR19           EQU  H'0003'
LADR20           EQU  H'0004'
LADR21           EQU  H'0005'


SCANLADR16       EQU  H'0000'
SCANLADR17       EQU  H'0001'
SCANLADR18       EQU  H'0002'
SCANLADR19       EQU  H'0003'
SCANLADR20       EQU  H'0004'
SCANLADR21       EQU  H'0005'


;----- SCANHADRL Bits -----------------------------------------------------
HADR0            EQU  H'0000'
HADR1            EQU  H'0001'
HADR2            EQU  H'0002'
HADR3            EQU  H'0003'
HADR4            EQU  H'0004'
HADR5            EQU  H'0005'
HADR6            EQU  H'0006'
HADR7            EQU  H'0007'


SCANHADR0        EQU  H'0000'
SCANHADR1        EQU  H'0001'
SCANHADR2        EQU  H'0002'
SCANHADR3        EQU  H'0003'
SCANHADR4        EQU  H'0004'
SCANHADR5        EQU  H'0005'
SCANHADR6        EQU  H'0006'
SCANHADR7        EQU  H'0007'


;----- SCANHADRH Bits -----------------------------------------------------
HADR8            EQU  H'0000'
HADR9            EQU  H'0001'
HADR10           EQU  H'0002'
HADR11           EQU  H'0003'
HADR12           EQU  H'0004'
HADR13           EQU  H'0005'
HADR14           EQU  H'0006'
HADR15           EQU  H'0007'


SCANHADR8        EQU  H'0000'
SCANHADR9        EQU  H'0001'
SCANHADR10       EQU  H'0002'
SCANHADR11       EQU  H'0003'
SCANHADR12       EQU  H'0004'
SCANHADR13       EQU  H'0005'
SCANHADR14       EQU  H'0006'
SCANHADR15       EQU  H'0007'


;----- SCANHADRU Bits -----------------------------------------------------
HADR16           EQU  H'0000'
HADR17           EQU  H'0001'
HADR18           EQU  H'0002'
HADR19           EQU  H'0003'
HADR20           EQU  H'0004'
HADR21           EQU  H'0005'


SCANHADR16       EQU  H'0000'
SCANHADR17       EQU  H'0001'
SCANHADR18       EQU  H'0002'
SCANHADR19       EQU  H'0003'
SCANHADR20       EQU  H'0004'
SCANHADR21       EQU  H'0005'


;----- SCANCON0 Bits -----------------------------------------------------
BUSY_SCANCON0    EQU  H'0000'
BURSTMD          EQU  H'0001'
MREG             EQU  H'0002'
SGO              EQU  H'0005'
TRIGEN           EQU  H'0006'
EN               EQU  H'0007'


;----- IPR0 Bits -----------------------------------------------------
SWIP             EQU  H'0000'
HLVDIP           EQU  H'0001'
OSFIP            EQU  H'0002'
CSWIP            EQU  H'0003'
NVMIP            EQU  H'0004'
SCANIP           EQU  H'0005'
CRCIP            EQU  H'0006'
IOCIP            EQU  H'0007'


;----- IPR1 Bits -----------------------------------------------------
INT0IP           EQU  H'0000'
ZCDIP            EQU  H'0001'
ADIP             EQU  H'0002'
ADTIP            EQU  H'0003'
C1IP             EQU  H'0004'
SMT1IP           EQU  H'0005'
SMT1PRAIP        EQU  H'0006'
SMT1PWAIP        EQU  H'0007'


;----- IPR2 Bits -----------------------------------------------------
DMA1SCNTIP       EQU  H'0000'
DMA1DCNTIP       EQU  H'0001'
DMA1ORIP         EQU  H'0002'
DMA1AIP          EQU  H'0003'
SPI1RXIP         EQU  H'0004'
SPI1TXIP         EQU  H'0005'
SPI1IP           EQU  H'0006'
I2C1RXIP         EQU  H'0007'


;----- IPR3 Bits -----------------------------------------------------
I2C1TXIP         EQU  H'0000'
I2C1IP           EQU  H'0001'
I2C1EIP          EQU  H'0002'
U1RXIP           EQU  H'0003'
U1TXIP           EQU  H'0004'
U1EIP            EQU  H'0005'
U1IP             EQU  H'0006'
TMR0IP           EQU  H'0007'


;----- IPR4 Bits -----------------------------------------------------
TMR1IP           EQU  H'0000'
TMR1GIP          EQU  H'0001'
TMR2IP           EQU  H'0002'
CCP1IP           EQU  H'0003'
NCO1IP           EQU  H'0005'
CWG1IP           EQU  H'0006'
CLC1IP           EQU  H'0007'


;----- IPR5 Bits -----------------------------------------------------
INT1IP           EQU  H'0000'
C2IP             EQU  H'0001'
DMA2SCNTIP       EQU  H'0002'
DMA2DCNTIP       EQU  H'0003'
DMA2ORIP         EQU  H'0004'
DMA2AIP          EQU  H'0005'
I2C2RXIP         EQU  H'0006'
I2C2TXIP         EQU  H'0007'


;----- IPR6 Bits -----------------------------------------------------
I2C2IP           EQU  H'0000'
I2C2EIP          EQU  H'0001'
U2RXIP           EQU  H'0002'
U2TXIP           EQU  H'0003'
U2EIP            EQU  H'0004'
U2IP             EQU  H'0005'
TMR3IP           EQU  H'0006'
TMR3GIP          EQU  H'0007'


;----- IPR7 Bits -----------------------------------------------------
TMR4IP           EQU  H'0000'
CCP2IP           EQU  H'0001'
CWG2IP           EQU  H'0003'
CLC2IP           EQU  H'0004'
INT2IP           EQU  H'0005'


;----- IPR8 Bits -----------------------------------------------------
TMR5IP           EQU  H'0006'
TMR5GIP          EQU  H'0007'


;----- IPR9 Bits -----------------------------------------------------
TMR6IP           EQU  H'0000'
CCP3IP           EQU  H'0001'
CWG3IP           EQU  H'0002'
CLC3IP           EQU  H'0003'


;----- IPR10 Bits -----------------------------------------------------
CCP4IP           EQU  H'0000'
CLC4IP           EQU  H'0001'


;----- PIE0 Bits -----------------------------------------------------
SWIE             EQU  H'0000'
HLVDIE           EQU  H'0001'
OSFIE            EQU  H'0002'
CSWIE            EQU  H'0003'
NVMIE            EQU  H'0004'
SCANIE           EQU  H'0005'
CRCIE            EQU  H'0006'
IOCIE            EQU  H'0007'


;----- PIE1 Bits -----------------------------------------------------
INT0IE           EQU  H'0000'
ZCDIE            EQU  H'0001'
ADIE             EQU  H'0002'
ADTIE            EQU  H'0003'
C1IE             EQU  H'0004'
SMT1IE           EQU  H'0005'
SMT1PRAIE        EQU  H'0006'
SMT1PWAIE        EQU  H'0007'


;----- PIE2 Bits -----------------------------------------------------
DMA1SCNTIE       EQU  H'0000'
DMA1DCNTIE       EQU  H'0001'
DMA1ORIE         EQU  H'0002'
DMA1AIE          EQU  H'0003'
SPI1RXIE         EQU  H'0004'
SPI1TXIE         EQU  H'0005'
SPI1IE           EQU  H'0006'
I2C1RXIE         EQU  H'0007'


;----- PIE3 Bits -----------------------------------------------------
I2C1TXIE         EQU  H'0000'
I2C1IE           EQU  H'0001'
I2C1EIE          EQU  H'0002'
U1RXIE           EQU  H'0003'
U1TXIE           EQU  H'0004'
U1EIE            EQU  H'0005'
U1IE             EQU  H'0006'
TMR0IE           EQU  H'0007'


;----- PIE4 Bits -----------------------------------------------------
TMR1IE           EQU  H'0000'
TMR1GIE          EQU  H'0001'
TMR2IE           EQU  H'0002'
CCP1IE           EQU  H'0003'
NCO1IE           EQU  H'0005'
CWG1IE           EQU  H'0006'
CLC1IE           EQU  H'0007'


;----- PIE5 Bits -----------------------------------------------------
INT1IE           EQU  H'0000'
C2IE             EQU  H'0001'
DMA2SCNTIE       EQU  H'0002'
DMA2DCNTIE       EQU  H'0003'
DMA2ORIE         EQU  H'0004'
DMA2AIE          EQU  H'0005'
I2C2RXIE         EQU  H'0006'
I2C2TXIE         EQU  H'0007'


;----- PIE6 Bits -----------------------------------------------------
I2C2IE           EQU  H'0000'
I2C2EIE          EQU  H'0001'
U2RXIE           EQU  H'0002'
U2TXIE           EQU  H'0003'
U2EIE            EQU  H'0004'
U2IE             EQU  H'0005'
TMR3IE           EQU  H'0006'
TMR3GIE          EQU  H'0007'


;----- PIE7 Bits -----------------------------------------------------
TMR4IE           EQU  H'0000'
CCP2IE           EQU  H'0001'
CWG2IE           EQU  H'0003'
CLC2IE           EQU  H'0004'
INT2IE           EQU  H'0005'


;----- PIE8 Bits -----------------------------------------------------
TMR5IE           EQU  H'0006'
TMR5GIE          EQU  H'0007'


;----- PIE9 Bits -----------------------------------------------------
TMR6IE           EQU  H'0000'
CCP3IE           EQU  H'0001'
CWG3IE           EQU  H'0002'
CLC3IE           EQU  H'0003'


;----- PIE10 Bits -----------------------------------------------------
CCP4IE           EQU  H'0000'
CLC4IE           EQU  H'0001'


;----- PIR0 Bits -----------------------------------------------------
SWIF             EQU  H'0000'
HLVDIF           EQU  H'0001'
OSFIF            EQU  H'0002'
CSWIF            EQU  H'0003'
NVMIF            EQU  H'0004'
SCANIF           EQU  H'0005'
CRCIF            EQU  H'0006'
IOCIF            EQU  H'0007'


;----- PIR1 Bits -----------------------------------------------------
INT0IF           EQU  H'0000'
ZCDIF            EQU  H'0001'
ADIF             EQU  H'0002'
ADTIF            EQU  H'0003'
C1IF             EQU  H'0004'
SMT1IF           EQU  H'0005'
SMT1PRAIF        EQU  H'0006'
SMT1PWAIF        EQU  H'0007'


;----- PIR2 Bits -----------------------------------------------------
DMA1SCNTIF       EQU  H'0000'
DMA1DCNTIF       EQU  H'0001'
DMA1ORIF         EQU  H'0002'
DMA1AIF          EQU  H'0003'
SPI1RXIF         EQU  H'0004'
SPI1TXIF         EQU  H'0005'
SPI1IF           EQU  H'0006'
I2C1RXIF         EQU  H'0007'


;----- PIR3 Bits -----------------------------------------------------
I2C1TXIF         EQU  H'0000'
I2C1IF           EQU  H'0001'
I2C1EIF          EQU  H'0002'
U1RXIF           EQU  H'0003'
U1TXIF           EQU  H'0004'
U1EIF            EQU  H'0005'
U1IF             EQU  H'0006'
TMR0IF           EQU  H'0007'


;----- PIR4 Bits -----------------------------------------------------
TMR1IF           EQU  H'0000'
TMR1GIF          EQU  H'0001'
TMR2IF           EQU  H'0002'
CCP1IF           EQU  H'0003'
NCO1IF           EQU  H'0005'
CWG1IF           EQU  H'0006'
CLC1IF           EQU  H'0007'


;----- PIR5 Bits -----------------------------------------------------
INT1IF           EQU  H'0000'
C2IF             EQU  H'0001'
DMA2SCNTIF       EQU  H'0002'
DMA2DCNTIF       EQU  H'0003'
DMA2ORIF         EQU  H'0004'
DMA2AIF          EQU  H'0005'
I2C2RXIF         EQU  H'0006'
I2C2TXIF         EQU  H'0007'


;----- PIR6 Bits -----------------------------------------------------
I2C2IF           EQU  H'0000'
I2C2EIF          EQU  H'0001'
U2RXIF           EQU  H'0002'
U2TXIF           EQU  H'0003'
U2EIF            EQU  H'0004'
U2IF             EQU  H'0005'
TMR3IF           EQU  H'0006'
TMR3GIF          EQU  H'0007'


;----- PIR7 Bits -----------------------------------------------------
TMR4IF           EQU  H'0000'
CCP2IF           EQU  H'0001'
CWG2IF           EQU  H'0003'
CLC2IF           EQU  H'0004'
INT2IF           EQU  H'0005'


;----- PIR8 Bits -----------------------------------------------------
TMR5IF           EQU  H'0006'
TMR5GIF          EQU  H'0007'


;----- PIR9 Bits -----------------------------------------------------
TMR6IF           EQU  H'0000'
CCP3IF           EQU  H'0001'
CWG3IF           EQU  H'0002'
CLC3IF           EQU  H'0003'


;----- PIR10 Bits -----------------------------------------------------
CCP4IF           EQU  H'0000'
CLC4IF           EQU  H'0001'


;----- PMD0 Bits -----------------------------------------------------
IOCMD            EQU  H'0000'
CLKRMD           EQU  H'0001'
NVMMD            EQU  H'0002'
SCANMD           EQU  H'0003'
CRCMD            EQU  H'0004'
HLVDMD           EQU  H'0005'
FVRMD            EQU  H'0006'
SYSCMD           EQU  H'0007'


;----- PMD1 Bits -----------------------------------------------------
TMR0MD           EQU  H'0000'
TMR1MD           EQU  H'0001'
TMR2MD           EQU  H'0002'
TMR3MD           EQU  H'0003'
TMR4MD           EQU  H'0004'
TMR5MD           EQU  H'0005'
TMR6MD           EQU  H'0006'
NCO1MD           EQU  H'0007'


;----- PMD2 Bits -----------------------------------------------------
ZCDMD            EQU  H'0000'
CMP1MD           EQU  H'0001'
CMP2MD           EQU  H'0002'
ADCMD            EQU  H'0005'
DACMD            EQU  H'0006'


;----- PMD3 Bits -----------------------------------------------------
CCP1MD           EQU  H'0000'
CCP2MD           EQU  H'0001'
CCP3MD           EQU  H'0002'
CCP4MD           EQU  H'0003'
PWM5MD           EQU  H'0004'
PWM6MD           EQU  H'0005'
PWM7MD           EQU  H'0006'
PWM8MD           EQU  H'0007'


;----- PMD4 Bits -----------------------------------------------------
CWG1MD           EQU  H'0005'
CWG2MD           EQU  H'0006'
CWG3MD           EQU  H'0007'


;----- PMD5 Bits -----------------------------------------------------
I2C1MD           EQU  H'0000'
I2C2MD           EQU  H'0001'
SPI1MD           EQU  H'0002'
U1MD             EQU  H'0004'
U2MD             EQU  H'0005'


;----- PMD6 Bits -----------------------------------------------------
DSMMD            EQU  H'0000'
CLC1MD           EQU  H'0001'
CLC2MD           EQU  H'0002'
CLC3MD           EQU  H'0003'
CLC4MD           EQU  H'0004'
SMT1MD           EQU  H'0005'


;----- PMD7 Bits -----------------------------------------------------
DMA1MD           EQU  H'0000'
DMA2MD           EQU  H'0001'


;----- BORCON Bits -----------------------------------------------------
BORRDY           EQU  H'0000'
SBOREN           EQU  H'0007'


;----- CPUDOZE Bits -----------------------------------------------------
DOE              EQU  H'0004'
ROI              EQU  H'0005'
DOZEN            EQU  H'0006'
IDLEN            EQU  H'0007'

DOZE0            EQU  H'0000'
DOZE1            EQU  H'0001'
DOZE2            EQU  H'0002'


;----- OSCCON1 Bits -----------------------------------------------------
NDIV0            EQU  H'0000'
NDIV1            EQU  H'0001'
NDIV2            EQU  H'0002'
NDIV3            EQU  H'0003'
NOSC0            EQU  H'0004'
NOSC1            EQU  H'0005'
NOSC2            EQU  H'0006'


;----- OSCCON2 Bits -----------------------------------------------------
CDIV0            EQU  H'0000'
CDIV1            EQU  H'0001'
CDIV2            EQU  H'0002'
CDIV3            EQU  H'0003'
COSC0            EQU  H'0004'
COSC1            EQU  H'0005'
COSC2            EQU  H'0006'


;----- OSCCON3 Bits -----------------------------------------------------
NOSCR            EQU  H'0003'
ORDY             EQU  H'0004'
SOSCPWR          EQU  H'0006'
CSWHOLD          EQU  H'0007'


;----- OSCSTAT Bits -----------------------------------------------------
PLLR             EQU  H'0000'
ADOR             EQU  H'0002'
SOR              EQU  H'0003'
LFOR             EQU  H'0004'
MFOR             EQU  H'0005'
HFOR             EQU  H'0006'
EXTOR            EQU  H'0007'


;----- OSCSTAT1 Bits -----------------------------------------------------
PLLR             EQU  H'0000'
ADOR             EQU  H'0002'
SOR              EQU  H'0003'
LFOR             EQU  H'0004'
MFOR             EQU  H'0005'
HFOR             EQU  H'0006'
EXTOR            EQU  H'0007'


;----- OSCEN Bits -----------------------------------------------------
ADOEN            EQU  H'0002'
SOSCEN           EQU  H'0003'
LFOEN            EQU  H'0004'
MFOEN            EQU  H'0005'
HFOEN            EQU  H'0006'
EXTOEN           EQU  H'0007'


;----- OSCTUNE Bits -----------------------------------------------------
TUN0             EQU  H'0000'
TUN1             EQU  H'0001'
TUN2             EQU  H'0002'
TUN3             EQU  H'0003'
TUN4             EQU  H'0004'
TUN5             EQU  H'0005'


HFTUN0           EQU  H'0000'
HFTUN1           EQU  H'0001'
HFTUN2           EQU  H'0002'
HFTUN3           EQU  H'0003'
HFTUN4           EQU  H'0004'
HFTUN5           EQU  H'0005'


;----- OSCFRQ Bits -----------------------------------------------------
FRQ0             EQU  H'0000'
FRQ1             EQU  H'0001'
FRQ2             EQU  H'0002'
FRQ3             EQU  H'0003'


HFFRQ0           EQU  H'0000'
HFFRQ1           EQU  H'0001'
HFFRQ2           EQU  H'0002'
HFFRQ3           EQU  H'0003'


;----- NVMADRL Bits -----------------------------------------------------
ADR0             EQU  H'0000'
ADR1             EQU  H'0001'
ADR2             EQU  H'0002'
ADR3             EQU  H'0003'
ADR4             EQU  H'0004'
ADR5             EQU  H'0005'
ADR6             EQU  H'0006'
ADR7             EQU  H'0007'


NVMADR0          EQU  H'0000'
NVMADR1          EQU  H'0001'
NVMADR2          EQU  H'0002'
NVMADR3          EQU  H'0003'
NVMADR4          EQU  H'0004'
NVMADR5          EQU  H'0005'
NVMADR6          EQU  H'0006'
NVMADR7          EQU  H'0007'


;----- NVMADRH Bits -----------------------------------------------------
ADR8             EQU  H'0000'
ADR9             EQU  H'0001'


NVMADR8          EQU  H'0000'
NVMADR9          EQU  H'0001'


;----- NVMDAT Bits -----------------------------------------------------
DAT0             EQU  H'0000'
DAT1             EQU  H'0001'
DAT2             EQU  H'0002'
DAT3             EQU  H'0003'
DAT4             EQU  H'0004'
DAT5             EQU  H'0005'
DAT6             EQU  H'0006'
DAT7             EQU  H'0007'


NVMDAT0          EQU  H'0000'
NVMDAT1          EQU  H'0001'
NVMDAT2          EQU  H'0002'
NVMDAT3          EQU  H'0003'
NVMDAT4          EQU  H'0004'
NVMDAT5          EQU  H'0005'
NVMDAT6          EQU  H'0006'
NVMDAT7          EQU  H'0007'


;----- NVMCON1 Bits -----------------------------------------------------
RD               EQU  H'0000'
WR               EQU  H'0001'
WREN             EQU  H'0002'
WRERR            EQU  H'0003'
FREE             EQU  H'0004'

REG0             EQU  H'0006'
REG1             EQU  H'0007'


NVMREG0          EQU  H'0006'
NVMREG1          EQU  H'0007'


;----- PRLOCK Bits -----------------------------------------------------
PRLOCKED         EQU  H'0000'


;----- ISRPR Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'


ISRPR0           EQU  H'0000'
ISRPR1           EQU  H'0001'
ISRPR2           EQU  H'0002'


;----- MAINPR Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'


MAINPR0          EQU  H'0000'
MAINPR1          EQU  H'0001'
MAINPR2          EQU  H'0002'


;----- DMA1PR Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'


DMA1PR0          EQU  H'0000'
DMA1PR1          EQU  H'0001'
DMA1PR2          EQU  H'0002'


;----- DMA2PR Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'


DMA2PR0          EQU  H'0000'
DMA2PR1          EQU  H'0001'
DMA2PR2          EQU  H'0002'


;----- SCANPR Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'


SCANPR0          EQU  H'0000'
SCANPR1          EQU  H'0001'
SCANPR2          EQU  H'0002'


;----- RA0PPS Bits -----------------------------------------------------
RA0PPS0          EQU  H'0000'
RA0PPS1          EQU  H'0001'
RA0PPS2          EQU  H'0002'
RA0PPS3          EQU  H'0003'
RA0PPS4          EQU  H'0004'
RA0PPS5          EQU  H'0005'


;----- RA1PPS Bits -----------------------------------------------------
RA1PPS0          EQU  H'0000'
RA1PPS1          EQU  H'0001'
RA1PPS2          EQU  H'0002'
RA1PPS3          EQU  H'0003'
RA1PPS4          EQU  H'0004'
RA1PPS5          EQU  H'0005'


;----- RA2PPS Bits -----------------------------------------------------
RA2PPS0          EQU  H'0000'
RA2PPS1          EQU  H'0001'
RA2PPS2          EQU  H'0002'
RA2PPS3          EQU  H'0003'
RA2PPS4          EQU  H'0004'
RA2PPS5          EQU  H'0005'


;----- RA3PPS Bits -----------------------------------------------------
RA3PPS0          EQU  H'0000'
RA3PPS1          EQU  H'0001'
RA3PPS2          EQU  H'0002'
RA3PPS3          EQU  H'0003'
RA3PPS4          EQU  H'0004'
RA3PPS5          EQU  H'0005'


;----- RA4PPS Bits -----------------------------------------------------
RA4PPS0          EQU  H'0000'
RA4PPS1          EQU  H'0001'
RA4PPS2          EQU  H'0002'
RA4PPS3          EQU  H'0003'
RA4PPS4          EQU  H'0004'
RA4PPS5          EQU  H'0005'


;----- RA5PPS Bits -----------------------------------------------------
RA5PPS0          EQU  H'0000'
RA5PPS1          EQU  H'0001'
RA5PPS2          EQU  H'0002'
RA5PPS3          EQU  H'0003'
RA5PPS4          EQU  H'0004'
RA5PPS5          EQU  H'0005'


;----- RA6PPS Bits -----------------------------------------------------
RA6PPS0          EQU  H'0000'
RA6PPS1          EQU  H'0001'
RA6PPS2          EQU  H'0002'
RA6PPS3          EQU  H'0003'
RA6PPS4          EQU  H'0004'
RA6PPS5          EQU  H'0005'


;----- RA7PPS Bits -----------------------------------------------------
RA7PPS0          EQU  H'0000'
RA7PPS1          EQU  H'0001'
RA7PPS2          EQU  H'0002'
RA7PPS3          EQU  H'0003'
RA7PPS4          EQU  H'0004'
RA7PPS5          EQU  H'0005'


;----- RB0PPS Bits -----------------------------------------------------
RB0PPS0          EQU  H'0000'
RB0PPS1          EQU  H'0001'
RB0PPS2          EQU  H'0002'
RB0PPS3          EQU  H'0003'
RB0PPS4          EQU  H'0004'
RB0PPS5          EQU  H'0005'


;----- RB1PPS Bits -----------------------------------------------------
RB1PPS0          EQU  H'0000'
RB1PPS1          EQU  H'0001'
RB1PPS2          EQU  H'0002'
RB1PPS3          EQU  H'0003'
RB1PPS4          EQU  H'0004'
RB1PPS5          EQU  H'0005'


;----- RB2PPS Bits -----------------------------------------------------
RB2PPS0          EQU  H'0000'
RB2PPS1          EQU  H'0001'
RB2PPS2          EQU  H'0002'
RB2PPS3          EQU  H'0003'
RB2PPS4          EQU  H'0004'
RB2PPS5          EQU  H'0005'


;----- RB3PPS Bits -----------------------------------------------------
RB3PPS0          EQU  H'0000'
RB3PPS1          EQU  H'0001'
RB3PPS2          EQU  H'0002'
RB3PPS3          EQU  H'0003'
RB3PPS4          EQU  H'0004'
RB3PPS5          EQU  H'0005'


;----- RB4PPS Bits -----------------------------------------------------
RB4PPS0          EQU  H'0000'
RB4PPS1          EQU  H'0001'
RB4PPS2          EQU  H'0002'
RB4PPS3          EQU  H'0003'
RB4PPS4          EQU  H'0004'
RB4PPS5          EQU  H'0005'


;----- RB5PPS Bits -----------------------------------------------------
RB5PPS0          EQU  H'0000'
RB5PPS1          EQU  H'0001'
RB5PPS2          EQU  H'0002'
RB5PPS3          EQU  H'0003'
RB5PPS4          EQU  H'0004'
RB5PPS5          EQU  H'0005'


;----- RB6PPS Bits -----------------------------------------------------
RB6PPS0          EQU  H'0000'
RB6PPS1          EQU  H'0001'
RB6PPS2          EQU  H'0002'
RB6PPS3          EQU  H'0003'
RB6PPS4          EQU  H'0004'
RB6PPS5          EQU  H'0005'


;----- RB7PPS Bits -----------------------------------------------------
RB7PPS0          EQU  H'0000'
RB7PPS1          EQU  H'0001'
RB7PPS2          EQU  H'0002'
RB7PPS3          EQU  H'0003'
RB7PPS4          EQU  H'0004'
RB7PPS5          EQU  H'0005'


;----- RC0PPS Bits -----------------------------------------------------
RC0PPS0          EQU  H'0000'
RC0PPS1          EQU  H'0001'
RC0PPS2          EQU  H'0002'
RC0PPS3          EQU  H'0003'
RC0PPS4          EQU  H'0004'
RC0PPS5          EQU  H'0005'


;----- RC1PPS Bits -----------------------------------------------------
RC1PPS0          EQU  H'0000'
RC1PPS1          EQU  H'0001'
RC1PPS2          EQU  H'0002'
RC1PPS3          EQU  H'0003'
RC1PPS4          EQU  H'0004'
RC1PPS5          EQU  H'0005'


;----- RC2PPS Bits -----------------------------------------------------
RC2PPS0          EQU  H'0000'
RC2PPS1          EQU  H'0001'
RC2PPS2          EQU  H'0002'
RC2PPS3          EQU  H'0003'
RC2PPS4          EQU  H'0004'
RC2PPS5          EQU  H'0005'


;----- RC3PPS Bits -----------------------------------------------------
RC3PPS0          EQU  H'0000'
RC3PPS1          EQU  H'0001'
RC3PPS2          EQU  H'0002'
RC3PPS3          EQU  H'0003'
RC3PPS4          EQU  H'0004'
RC3PPS5          EQU  H'0005'


;----- RC4PPS Bits -----------------------------------------------------
RC4PPS0          EQU  H'0000'
RC4PPS1          EQU  H'0001'
RC4PPS2          EQU  H'0002'
RC4PPS3          EQU  H'0003'
RC4PPS4          EQU  H'0004'
RC4PPS5          EQU  H'0005'


;----- RC5PPS Bits -----------------------------------------------------
RC5PPS0          EQU  H'0000'
RC5PPS1          EQU  H'0001'
RC5PPS2          EQU  H'0002'
RC5PPS3          EQU  H'0003'
RC5PPS4          EQU  H'0004'
RC5PPS5          EQU  H'0005'


;----- RC6PPS Bits -----------------------------------------------------
RC6PPS0          EQU  H'0000'
RC6PPS1          EQU  H'0001'
RC6PPS2          EQU  H'0002'
RC6PPS3          EQU  H'0003'
RC6PPS4          EQU  H'0004'
RC6PPS5          EQU  H'0005'


;----- RC7PPS Bits -----------------------------------------------------
RC7PPS0          EQU  H'0000'
RC7PPS1          EQU  H'0001'
RC7PPS2          EQU  H'0002'
RC7PPS3          EQU  H'0003'
RC7PPS4          EQU  H'0004'
RC7PPS5          EQU  H'0005'


;----- RD0PPS Bits -----------------------------------------------------
RD0PPS0          EQU  H'0000'
RD0PPS1          EQU  H'0001'
RD0PPS2          EQU  H'0002'
RD0PPS3          EQU  H'0003'
RD0PPS4          EQU  H'0004'
RD0PPS5          EQU  H'0005'


;----- RD1PPS Bits -----------------------------------------------------
RD1PPS0          EQU  H'0000'
RD1PPS1          EQU  H'0001'
RD1PPS2          EQU  H'0002'
RD1PPS3          EQU  H'0003'
RD1PPS4          EQU  H'0004'
RD1PPS5          EQU  H'0005'


;----- RD2PPS Bits -----------------------------------------------------
RD2PPS0          EQU  H'0000'
RD2PPS1          EQU  H'0001'
RD2PPS2          EQU  H'0002'
RD2PPS3          EQU  H'0003'
RD2PPS4          EQU  H'0004'
RD2PPS5          EQU  H'0005'


;----- RD3PPS Bits -----------------------------------------------------
RD3PPS0          EQU  H'0000'
RD3PPS1          EQU  H'0001'
RD3PPS2          EQU  H'0002'
RD3PPS3          EQU  H'0003'
RD3PPS4          EQU  H'0004'
RD3PPS5          EQU  H'0005'


;----- RD4PPS Bits -----------------------------------------------------
RD4PPS0          EQU  H'0000'
RD4PPS1          EQU  H'0001'
RD4PPS2          EQU  H'0002'
RD4PPS3          EQU  H'0003'
RD4PPS4          EQU  H'0004'
RD4PPS5          EQU  H'0005'


;----- RD5PPS Bits -----------------------------------------------------
RD5PPS0          EQU  H'0000'
RD5PPS1          EQU  H'0001'
RD5PPS2          EQU  H'0002'
RD5PPS3          EQU  H'0003'
RD5PPS4          EQU  H'0004'
RD5PPS5          EQU  H'0005'


;----- RD6PPS Bits -----------------------------------------------------
RD6PPS0          EQU  H'0000'
RD6PPS1          EQU  H'0001'
RD6PPS2          EQU  H'0002'
RD6PPS3          EQU  H'0003'
RD6PPS4          EQU  H'0004'
RD6PPS5          EQU  H'0005'


;----- RD7PPS Bits -----------------------------------------------------
RD7PPS0          EQU  H'0000'
RD7PPS1          EQU  H'0001'
RD7PPS2          EQU  H'0002'
RD7PPS3          EQU  H'0003'
RD7PPS4          EQU  H'0004'
RD7PPS5          EQU  H'0005'


;----- RE0PPS Bits -----------------------------------------------------
RE0PPS0          EQU  H'0000'
RE0PPS1          EQU  H'0001'
RE0PPS2          EQU  H'0002'
RE0PPS3          EQU  H'0003'
RE0PPS4          EQU  H'0004'
RE0PPS5          EQU  H'0005'


;----- RE1PPS Bits -----------------------------------------------------
RE1PPS0          EQU  H'0000'
RE1PPS1          EQU  H'0001'
RE1PPS2          EQU  H'0002'
RE1PPS3          EQU  H'0003'
RE1PPS4          EQU  H'0004'
RE1PPS5          EQU  H'0005'


;----- RE2PPS Bits -----------------------------------------------------
RE2PPS0          EQU  H'0000'
RE2PPS1          EQU  H'0001'
RE2PPS2          EQU  H'0002'
RE2PPS3          EQU  H'0003'
RE2PPS4          EQU  H'0004'
RE2PPS5          EQU  H'0005'


;----- ANSELA Bits -----------------------------------------------------
ANSELA0          EQU  H'0000'
ANSELA1          EQU  H'0001'
ANSELA2          EQU  H'0002'
ANSELA3          EQU  H'0003'
ANSELA4          EQU  H'0004'
ANSELA5          EQU  H'0005'
ANSELA6          EQU  H'0006'
ANSELA7          EQU  H'0007'


;----- WPUA Bits -----------------------------------------------------
WPUA0            EQU  H'0000'
WPUA1            EQU  H'0001'
WPUA2            EQU  H'0002'
WPUA3            EQU  H'0003'
WPUA4            EQU  H'0004'
WPUA5            EQU  H'0005'
WPUA6            EQU  H'0006'
WPUA7            EQU  H'0007'


;----- ODCONA Bits -----------------------------------------------------
ODCA0            EQU  H'0000'
ODCA1            EQU  H'0001'
ODCA2            EQU  H'0002'
ODCA3            EQU  H'0003'
ODCA4            EQU  H'0004'
ODCA5            EQU  H'0005'
ODCA6            EQU  H'0006'
ODCA7            EQU  H'0007'


;----- SLRCONA Bits -----------------------------------------------------
SLRA0            EQU  H'0000'
SLRA1            EQU  H'0001'
SLRA2            EQU  H'0002'
SLRA3            EQU  H'0003'
SLRA4            EQU  H'0004'
SLRA5            EQU  H'0005'
SLRA6            EQU  H'0006'
SLRA7            EQU  H'0007'


;----- INLVLA Bits -----------------------------------------------------
INLVLA0          EQU  H'0000'
INLVLA1          EQU  H'0001'
INLVLA2          EQU  H'0002'
INLVLA3          EQU  H'0003'
INLVLA4          EQU  H'0004'
INLVLA5          EQU  H'0005'
INLVLA6          EQU  H'0006'
INLVLA7          EQU  H'0007'


;----- IOCAP Bits -----------------------------------------------------
IOCAP0           EQU  H'0000'
IOCAP1           EQU  H'0001'
IOCAP2           EQU  H'0002'
IOCAP3           EQU  H'0003'
IOCAP4           EQU  H'0004'
IOCAP5           EQU  H'0005'
IOCAP6           EQU  H'0006'
IOCAP7           EQU  H'0007'


;----- IOCAN Bits -----------------------------------------------------
IOCAN0           EQU  H'0000'
IOCAN1           EQU  H'0001'
IOCAN2           EQU  H'0002'
IOCAN3           EQU  H'0003'
IOCAN4           EQU  H'0004'
IOCAN5           EQU  H'0005'
IOCAN6           EQU  H'0006'
IOCAN7           EQU  H'0007'


;----- IOCAF Bits -----------------------------------------------------
IOCAF0           EQU  H'0000'
IOCAF1           EQU  H'0001'
IOCAF2           EQU  H'0002'
IOCAF3           EQU  H'0003'
IOCAF4           EQU  H'0004'
IOCAF5           EQU  H'0005'
IOCAF6           EQU  H'0006'
IOCAF7           EQU  H'0007'


;----- ANSELB Bits -----------------------------------------------------
ANSELB0          EQU  H'0000'
ANSELB1          EQU  H'0001'
ANSELB2          EQU  H'0002'
ANSELB3          EQU  H'0003'
ANSELB4          EQU  H'0004'
ANSELB5          EQU  H'0005'
ANSELB6          EQU  H'0006'
ANSELB7          EQU  H'0007'


;----- WPUB Bits -----------------------------------------------------
WPUB0            EQU  H'0000'
WPUB1            EQU  H'0001'
WPUB2            EQU  H'0002'
WPUB3            EQU  H'0003'
WPUB4            EQU  H'0004'
WPUB5            EQU  H'0005'
WPUB6            EQU  H'0006'
WPUB7            EQU  H'0007'


;----- ODCONB Bits -----------------------------------------------------
ODCB0            EQU  H'0000'
ODCB1            EQU  H'0001'
ODCB2            EQU  H'0002'
ODCB3            EQU  H'0003'
ODCB4            EQU  H'0004'
ODCB5            EQU  H'0005'
ODCB6            EQU  H'0006'
ODCB7            EQU  H'0007'


;----- SLRCONB Bits -----------------------------------------------------
SLRB0            EQU  H'0000'
SLRB1            EQU  H'0001'
SLRB2            EQU  H'0002'
SLRB3            EQU  H'0003'
SLRB4            EQU  H'0004'
SLRB5            EQU  H'0005'
SLRB6            EQU  H'0006'
SLRB7            EQU  H'0007'


;----- INLVLB Bits -----------------------------------------------------
INLVLB0          EQU  H'0000'
INLVLB1          EQU  H'0001'
INLVLB2          EQU  H'0002'
INLVLB3          EQU  H'0003'
INLVLB4          EQU  H'0004'
INLVLB5          EQU  H'0005'
INLVLB6          EQU  H'0006'
INLVLB7          EQU  H'0007'


;----- IOCBP Bits -----------------------------------------------------
IOCBP0           EQU  H'0000'
IOCBP1           EQU  H'0001'
IOCBP2           EQU  H'0002'
IOCBP3           EQU  H'0003'
IOCBP4           EQU  H'0004'
IOCBP5           EQU  H'0005'
IOCBP6           EQU  H'0006'
IOCBP7           EQU  H'0007'


;----- IOCBN Bits -----------------------------------------------------
IOCBN0           EQU  H'0000'
IOCBN1           EQU  H'0001'
IOCBN2           EQU  H'0002'
IOCBN3           EQU  H'0003'
IOCBN4           EQU  H'0004'
IOCBN5           EQU  H'0005'
IOCBN6           EQU  H'0006'
IOCBN7           EQU  H'0007'


;----- IOCBF Bits -----------------------------------------------------
IOCBF0           EQU  H'0000'
IOCBF1           EQU  H'0001'
IOCBF2           EQU  H'0002'
IOCBF3           EQU  H'0003'
IOCBF4           EQU  H'0004'
IOCBF5           EQU  H'0005'
IOCBF6           EQU  H'0006'
IOCBF7           EQU  H'0007'


;----- RB1I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- RB2I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- ANSELC Bits -----------------------------------------------------
ANSELC0          EQU  H'0000'
ANSELC1          EQU  H'0001'
ANSELC2          EQU  H'0002'
ANSELC3          EQU  H'0003'
ANSELC4          EQU  H'0004'
ANSELC5          EQU  H'0005'
ANSELC6          EQU  H'0006'
ANSELC7          EQU  H'0007'


;----- WPUC Bits -----------------------------------------------------
WPUC0            EQU  H'0000'
WPUC1            EQU  H'0001'
WPUC2            EQU  H'0002'
WPUC3            EQU  H'0003'
WPUC4            EQU  H'0004'
WPUC5            EQU  H'0005'
WPUC6            EQU  H'0006'
WPUC7            EQU  H'0007'


;----- ODCONC Bits -----------------------------------------------------
ODCC0            EQU  H'0000'
ODCC1            EQU  H'0001'
ODCC2            EQU  H'0002'
ODCC3            EQU  H'0003'
ODCC4            EQU  H'0004'
ODCC5            EQU  H'0005'
ODCC6            EQU  H'0006'
ODCC7            EQU  H'0007'


;----- SLRCONC Bits -----------------------------------------------------
SLRC0            EQU  H'0000'
SLRC1            EQU  H'0001'
SLRC2            EQU  H'0002'
SLRC3            EQU  H'0003'
SLRC4            EQU  H'0004'
SLRC5            EQU  H'0005'
SLRC6            EQU  H'0006'
SLRC7            EQU  H'0007'


;----- INLVLC Bits -----------------------------------------------------
INLVLC0          EQU  H'0000'
INLVLC1          EQU  H'0001'
INLVLC2          EQU  H'0002'
INLVLC3          EQU  H'0003'
INLVLC4          EQU  H'0004'
INLVLC5          EQU  H'0005'
INLVLC6          EQU  H'0006'
INLVLC7          EQU  H'0007'


;----- IOCCP Bits -----------------------------------------------------
IOCCP0           EQU  H'0000'
IOCCP1           EQU  H'0001'
IOCCP2           EQU  H'0002'
IOCCP3           EQU  H'0003'
IOCCP4           EQU  H'0004'
IOCCP5           EQU  H'0005'
IOCCP6           EQU  H'0006'
IOCCP7           EQU  H'0007'


;----- IOCCN Bits -----------------------------------------------------
IOCCN0           EQU  H'0000'
IOCCN1           EQU  H'0001'
IOCCN2           EQU  H'0002'
IOCCN3           EQU  H'0003'
IOCCN4           EQU  H'0004'
IOCCN5           EQU  H'0005'
IOCCN6           EQU  H'0006'
IOCCN7           EQU  H'0007'


;----- IOCCF Bits -----------------------------------------------------
IOCCF0           EQU  H'0000'
IOCCF1           EQU  H'0001'
IOCCF2           EQU  H'0002'
IOCCF3           EQU  H'0003'
IOCCF4           EQU  H'0004'
IOCCF5           EQU  H'0005'
IOCCF6           EQU  H'0006'
IOCCF7           EQU  H'0007'


;----- RC3I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- RC4I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- ANSELD Bits -----------------------------------------------------
ANSELD0          EQU  H'0000'
ANSELD1          EQU  H'0001'
ANSELD2          EQU  H'0002'
ANSELD3          EQU  H'0003'
ANSELD4          EQU  H'0004'
ANSELD5          EQU  H'0005'
ANSELD6          EQU  H'0006'
ANSELD7          EQU  H'0007'


;----- WPUD Bits -----------------------------------------------------
WPUD0            EQU  H'0000'
WPUD1            EQU  H'0001'
WPUD2            EQU  H'0002'
WPUD3            EQU  H'0003'
WPUD4            EQU  H'0004'
WPUD5            EQU  H'0005'
WPUD6            EQU  H'0006'
WPUD7            EQU  H'0007'


;----- ODCOND Bits -----------------------------------------------------
ODCD0            EQU  H'0000'
ODCD1            EQU  H'0001'
ODCD2            EQU  H'0002'
ODCD3            EQU  H'0003'
ODCD4            EQU  H'0004'
ODCD5            EQU  H'0005'
ODCD6            EQU  H'0006'
ODCD7            EQU  H'0007'


;----- SLRCOND Bits -----------------------------------------------------
SLRD0            EQU  H'0000'
SLRD1            EQU  H'0001'
SLRD2            EQU  H'0002'
SLRD3            EQU  H'0003'
SLRD4            EQU  H'0004'
SLRD5            EQU  H'0005'
SLRD6            EQU  H'0006'
SLRD7            EQU  H'0007'


;----- INLVLD Bits -----------------------------------------------------
INLVLD0          EQU  H'0000'
INLVLD1          EQU  H'0001'
INLVLD2          EQU  H'0002'
INLVLD3          EQU  H'0003'
INLVLD4          EQU  H'0004'
INLVLD5          EQU  H'0005'
INLVLD6          EQU  H'0006'
INLVLD7          EQU  H'0007'


;----- RD0I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- RD1I2C Bits -----------------------------------------------------
SLEW             EQU  H'0006'

TH0              EQU  H'0000'
TH1              EQU  H'0001'
PU0              EQU  H'0004'
PU1              EQU  H'0005'

I2CSLEW          EQU  H'0006'

I2CTH0           EQU  H'0000'
I2CTH1           EQU  H'0001'
I2CPU0           EQU  H'0004'
I2CPU1           EQU  H'0005'


;----- ANSELE Bits -----------------------------------------------------
ANSELE0          EQU  H'0000'
ANSELE1          EQU  H'0001'
ANSELE2          EQU  H'0002'


;----- WPUE Bits -----------------------------------------------------
WPUE0            EQU  H'0000'
WPUE1            EQU  H'0001'
WPUE2            EQU  H'0002'
WPUE3            EQU  H'0003'


;----- ODCONE Bits -----------------------------------------------------
ODCE0            EQU  H'0000'
ODCE1            EQU  H'0001'
ODCE2            EQU  H'0002'


;----- SLRCONE Bits -----------------------------------------------------
SLRE0            EQU  H'0000'
SLRE1            EQU  H'0001'
SLRE2            EQU  H'0002'


;----- INLVLE Bits -----------------------------------------------------
INLVLE0          EQU  H'0000'
INLVLE1          EQU  H'0001'
INLVLE2          EQU  H'0002'
INLVLE3          EQU  H'0003'


;----- IOCEP Bits -----------------------------------------------------
IOCEP3           EQU  H'0003'


;----- IOCEN Bits -----------------------------------------------------
IOCEN3           EQU  H'0003'


;----- IOCEF Bits -----------------------------------------------------
IOCEF3           EQU  H'0003'


;----- PPSLOCK Bits -----------------------------------------------------
PPSLOCKED        EQU  H'0000'


;----- DMA2BUF Bits -----------------------------------------------------
BUF0             EQU  H'0000'
BUF1             EQU  H'0001'
BUF2             EQU  H'0002'
BUF3             EQU  H'0003'
BUF4             EQU  H'0004'
BUF5             EQU  H'0005'
BUF6             EQU  H'0006'
BUF7             EQU  H'0007'


DMA2BUF0         EQU  H'0000'
DMA2BUF1         EQU  H'0001'
DMA2BUF2         EQU  H'0002'
DMA2BUF3         EQU  H'0003'
DMA2BUF4         EQU  H'0004'
DMA2BUF5         EQU  H'0005'
DMA2BUF6         EQU  H'0006'
DMA2BUF7         EQU  H'0007'


;----- DMA2DCNTL Bits -----------------------------------------------------
DCNT0            EQU  H'0000'
DCNT1            EQU  H'0001'
DCNT2            EQU  H'0002'
DCNT3            EQU  H'0003'
DCNT4            EQU  H'0004'
DCNT5            EQU  H'0005'
DCNT6            EQU  H'0006'
DCNT7            EQU  H'0007'


DMA2DCNT0        EQU  H'0000'
DMA2DCNT1        EQU  H'0001'
DMA2DCNT2        EQU  H'0002'
DMA2DCNT3        EQU  H'0003'
DMA2DCNT4        EQU  H'0004'
DMA2DCNT5        EQU  H'0005'
DMA2DCNT6        EQU  H'0006'
DMA2DCNT7        EQU  H'0007'


;----- DMA2DCNTH Bits -----------------------------------------------------
DCNT8            EQU  H'0000'
DCNT9            EQU  H'0001'
DCNT10           EQU  H'0002'
DCNT11           EQU  H'0003'


DMA2DCNT8        EQU  H'0000'
DMA2DCNT9        EQU  H'0001'
DMA2DCNT10       EQU  H'0002'
DMA2DCNT11       EQU  H'0003'


;----- DMA2DPTRL Bits -----------------------------------------------------
DPTR0            EQU  H'0000'
DPTR1            EQU  H'0001'
DPTR2            EQU  H'0002'
DPTR3            EQU  H'0003'
DPTR4            EQU  H'0004'
DPTR5            EQU  H'0005'
DPTR6            EQU  H'0006'
DPTR7            EQU  H'0007'


DMA2DPTR0        EQU  H'0000'
DMA2DPTR1        EQU  H'0001'
DMA2DPTR2        EQU  H'0002'
DMA2DPTR3        EQU  H'0003'
DMA2DPTR4        EQU  H'0004'
DMA2DPTR5        EQU  H'0005'
DMA2DPTR6        EQU  H'0006'
DMA2DPTR7        EQU  H'0007'


;----- DMA2DPTRH Bits -----------------------------------------------------
DPTR8            EQU  H'0000'
DPTR9            EQU  H'0001'
DPTR10           EQU  H'0002'
DPTR11           EQU  H'0003'
DPTR12           EQU  H'0004'
DPTR13           EQU  H'0005'
DPTR14           EQU  H'0006'
DPTR15           EQU  H'0007'


DMA2DPTR8        EQU  H'0000'
DMA2DPTR9        EQU  H'0001'
DMA2DPTR10       EQU  H'0002'
DMA2DPTR11       EQU  H'0003'
DMA2DPTR12       EQU  H'0004'
DMA2DPTR13       EQU  H'0005'
DMA2DPTR14       EQU  H'0006'
DMA2DPTR15       EQU  H'0007'


;----- DMA2DSZL Bits -----------------------------------------------------
DSZ0             EQU  H'0000'
DSZ1             EQU  H'0001'
DSZ2             EQU  H'0002'
DSZ3             EQU  H'0003'
DSZ4             EQU  H'0004'
DSZ5             EQU  H'0005'
DSZ6             EQU  H'0006'
DSZ7             EQU  H'0007'

DMA2DSZ0         EQU  H'0000'
DMA2DSZ1         EQU  H'0001'
DMA2DSZ2         EQU  H'0002'
DMA2DSZ3         EQU  H'0003'
DMA2DSZ4         EQU  H'0004'
DMA2DSZ5         EQU  H'0005'
DMA2DSZ6         EQU  H'0006'
DMA2DSZ7         EQU  H'0007'


;----- DMA2DSZH Bits -----------------------------------------------------
DSZ8             EQU  H'0000'
DSZ9             EQU  H'0001'
DSZ10            EQU  H'0002'
DSZ11            EQU  H'0003'

DMA2DSZ8         EQU  H'0000'
DMA2DSZ9         EQU  H'0001'
DMA2DSZ10        EQU  H'0002'
DMA2DSZ11        EQU  H'0003'


;----- DMA2DSAL Bits -----------------------------------------------------
SSA0             EQU  H'0000'
SSA1             EQU  H'0001'
SSA2             EQU  H'0002'
SSA3             EQU  H'0003'
SSA4             EQU  H'0004'
SSA5             EQU  H'0005'
SSA6             EQU  H'0006'
SSA7             EQU  H'0007'


DMA2DSA0         EQU  H'0000'
DMA2DSA1         EQU  H'0001'
DMA2DSA2         EQU  H'0002'
DMA2DSA3         EQU  H'0003'
DMA2DSA4         EQU  H'0004'
DMA2DSA5         EQU  H'0005'
DMA2DSA6         EQU  H'0006'
DMA2DSA7         EQU  H'0007'


;----- DMA2DSAH Bits -----------------------------------------------------
DSA8             EQU  H'0000'
DSA9             EQU  H'0001'
DSA10            EQU  H'0002'
DSA11            EQU  H'0003'
DSA12            EQU  H'0004'
DSA13            EQU  H'0005'
DSA14            EQU  H'0006'
DSA15            EQU  H'0007'


DMA2DSA8         EQU  H'0000'
DMA2DSA9         EQU  H'0001'
DMA2DSA10        EQU  H'0002'
DMA2DSA11        EQU  H'0003'
DMA2DSA12        EQU  H'0004'
DMA2DSA13        EQU  H'0005'
DMA2DSA14        EQU  H'0006'
DMA2DSA15        EQU  H'0007'


;----- DMA2SCNTL Bits -----------------------------------------------------
SCNT0            EQU  H'0000'
SCNT1            EQU  H'0001'
SCNT2            EQU  H'0002'
SCNT3            EQU  H'0003'
SCNT4            EQU  H'0004'
SCNT5            EQU  H'0005'
SCNT6            EQU  H'0006'
SCNT7            EQU  H'0007'


DMA2SCNT0        EQU  H'0000'
DMA2SCNT1        EQU  H'0001'
DMA2SCNT2        EQU  H'0002'
DMA2SCNT3        EQU  H'0003'
DMA2SCNT4        EQU  H'0004'
DMA2SCNT5        EQU  H'0005'
DMA2SCNT6        EQU  H'0006'
DMA2SCNT7        EQU  H'0007'


;----- DMA2SCNTH Bits -----------------------------------------------------
SCNT8            EQU  H'0000'
SCNT9            EQU  H'0001'
SCNT10           EQU  H'0002'
SCNT11           EQU  H'0003'


DMA2SCNT8        EQU  H'0000'
DMA2SCNT9        EQU  H'0001'
DMA2SCNT10       EQU  H'0002'
DMA2SCNT11       EQU  H'0003'


;----- DMA2SPTRL Bits -----------------------------------------------------
SPTR0            EQU  H'0000'
SPTR1            EQU  H'0001'
SPTR2            EQU  H'0002'
SPTR3            EQU  H'0003'
SPTR4            EQU  H'0004'
SPTR5            EQU  H'0005'
SPTR6            EQU  H'0006'
SPTR7            EQU  H'0007'


DMA2SPTR0        EQU  H'0000'
DMA2SPTR1        EQU  H'0001'
DMA2SPTR2        EQU  H'0002'
DMA2SPTR3        EQU  H'0003'
DMA2SPTR4        EQU  H'0004'
DMA2SPTR5        EQU  H'0005'
DMA2SPTR6        EQU  H'0006'
DMA2SPTR7        EQU  H'0007'


;----- DMA2SPTRH Bits -----------------------------------------------------
SPTR8            EQU  H'0000'
SPTR9            EQU  H'0001'
SPTR10           EQU  H'0002'
SPTR11           EQU  H'0003'
SPTR12           EQU  H'0004'
SPTR13           EQU  H'0005'
SPTR14           EQU  H'0006'
SPTR15           EQU  H'0007'


DMA2SPTR8        EQU  H'0000'
DMA2SPTR9        EQU  H'0001'
DMA2SPTR10       EQU  H'0002'
DMA2SPTR11       EQU  H'0003'
DMA2SPTR12       EQU  H'0004'
DMA2SPTR13       EQU  H'0005'
DMA2SPTR14       EQU  H'0006'
DMA2SPTR15       EQU  H'0007'


;----- DMA2SPTRU Bits -----------------------------------------------------
SPTR16           EQU  H'0000'
SPTR17           EQU  H'0001'
SPTR18           EQU  H'0002'
SPTR19           EQU  H'0003'
SPTR20           EQU  H'0004'
SPTR21           EQU  H'0005'


DMA2SPTR16       EQU  H'0000'
DMA2SPTR17       EQU  H'0001'
DMA2SPTR18       EQU  H'0002'
DMA2SPTR19       EQU  H'0003'
DMA2SPTR20       EQU  H'0004'
DMA2SPTR21       EQU  H'0005'


;----- DMA2SSZL Bits -----------------------------------------------------
SSZ0             EQU  H'0000'
SSZ1             EQU  H'0001'
SSZ2             EQU  H'0002'
SSZ3             EQU  H'0003'
SSZ4             EQU  H'0004'
SSZ5             EQU  H'0005'
SSZ6             EQU  H'0006'
SSZ7             EQU  H'0007'

DMA2SSZ0         EQU  H'0000'
DMA2SSZ1         EQU  H'0001'
DMA2SSZ2         EQU  H'0002'
DMA2SSZ3         EQU  H'0003'
DMA2SSZ4         EQU  H'0004'
DMA2SSZ5         EQU  H'0005'
DMA2SSZ6         EQU  H'0006'
DMA2SSZ7         EQU  H'0007'


;----- DMA2SSZH Bits -----------------------------------------------------
SSZ8             EQU  H'0000'
SSZ9             EQU  H'0001'
SSZ10            EQU  H'0002'
SSZ11            EQU  H'0003'

DMA2SSZ8         EQU  H'0000'
DMA2SSZ9         EQU  H'0001'
DMA2SSZ10        EQU  H'0002'
DMA2SSZ11        EQU  H'0003'


;----- DMA2SSAL Bits -----------------------------------------------------
SSA0             EQU  H'0000'
SSA1             EQU  H'0001'
SSA2             EQU  H'0002'
SSA3             EQU  H'0003'
SSA4             EQU  H'0004'
SSA5             EQU  H'0005'
SSA6             EQU  H'0006'
SSA7             EQU  H'0007'


DMA2SSA0         EQU  H'0000'
DMA2SSA1         EQU  H'0001'
DMA2SSA2         EQU  H'0002'
DMA2SSA3         EQU  H'0003'
DMA2SSA4         EQU  H'0004'
DMA2SSA5         EQU  H'0005'
DMA2SSA6         EQU  H'0006'
DMA2SSA7         EQU  H'0007'


;----- DMA2SSAH Bits -----------------------------------------------------
SSA8             EQU  H'0000'
SSA9             EQU  H'0001'
SSA10            EQU  H'0002'
SSA11            EQU  H'0003'
SSA12            EQU  H'0004'
SSA13            EQU  H'0005'
SSA14            EQU  H'0006'
SSA15            EQU  H'0007'


DMA2SSA8         EQU  H'0000'
DMA2SSA9         EQU  H'0001'
DMA2SSA10        EQU  H'0002'
DMA2SSA11        EQU  H'0003'
DMA2SSA12        EQU  H'0004'
DMA2SSA13        EQU  H'0005'
DMA2SSA14        EQU  H'0006'
DMA2SSA15        EQU  H'0007'


;----- DMA2SSAU Bits -----------------------------------------------------
SSA16            EQU  H'0000'
SSA17            EQU  H'0001'
SSA18            EQU  H'0002'
SSA19            EQU  H'0003'
SSA20            EQU  H'0004'
SSA21            EQU  H'0005'


DMA2SSA16        EQU  H'0000'
DMA2SSA17        EQU  H'0001'
DMA2SSA18        EQU  H'0002'
DMA2SSA19        EQU  H'0003'
DMA2SSA20        EQU  H'0004'
DMA2SSA21        EQU  H'0005'


;----- DMA2CON0 Bits -----------------------------------------------------
XIP              EQU  H'0000'
AIRQEN           EQU  H'0002'
DGO              EQU  H'0005'
SIRQEN           EQU  H'0006'
EN               EQU  H'0007'

DMA2XIP          EQU  H'0000'
DMA2AIRQEN       EQU  H'0002'
DMA2DGO          EQU  H'0005'
DMA2SIRQEN       EQU  H'0006'
DMA2EN           EQU  H'0007'


;----- DMA2CON1 Bits -----------------------------------------------------
SSTP             EQU  H'0000'
DSTP             EQU  H'0005'


;----- DMA2AIRQ Bits -----------------------------------------------------
AIRQ0            EQU  H'0000'
AIRQ1            EQU  H'0001'
AIRQ2            EQU  H'0002'
AIRQ3            EQU  H'0003'
AIRQ4            EQU  H'0004'
AIRQ5            EQU  H'0005'
AIRQ6            EQU  H'0006'

DMA2AIRQ0        EQU  H'0000'
DMA2AIRQ1        EQU  H'0001'
DMA2AIRQ2        EQU  H'0002'
DMA2AIRQ3        EQU  H'0003'
DMA2AIRQ4        EQU  H'0004'
DMA2AIRQ5        EQU  H'0005'
DMA2AIRQ6        EQU  H'0006'


;----- DMA2SIRQ Bits -----------------------------------------------------
SIRQ0            EQU  H'0000'
SIRQ1            EQU  H'0001'
SIRQ2            EQU  H'0002'
SIRQ3            EQU  H'0003'
SIRQ4            EQU  H'0004'
SIRQ5            EQU  H'0005'
SIRQ6            EQU  H'0006'

DMA2SIRQ0        EQU  H'0000'
DMA2SIRQ1        EQU  H'0001'
DMA2SIRQ2        EQU  H'0002'
DMA2SIRQ3        EQU  H'0003'
DMA2SIRQ4        EQU  H'0004'
DMA2SIRQ5        EQU  H'0005'
DMA2SIRQ6        EQU  H'0006'


;----- DMA1BUF Bits -----------------------------------------------------
BUF0             EQU  H'0000'
BUF1             EQU  H'0001'
BUF2             EQU  H'0002'
BUF3             EQU  H'0003'
BUF4             EQU  H'0004'
BUF5             EQU  H'0005'
BUF6             EQU  H'0006'
BUF7             EQU  H'0007'


DMA1BUF0         EQU  H'0000'
DMA1BUF1         EQU  H'0001'
DMA1BUF2         EQU  H'0002'
DMA1BUF3         EQU  H'0003'
DMA1BUF4         EQU  H'0004'
DMA1BUF5         EQU  H'0005'
DMA1BUF6         EQU  H'0006'
DMA1BUF7         EQU  H'0007'


;----- DMA1DCNTL Bits -----------------------------------------------------
DCNT0            EQU  H'0000'
DCNT1            EQU  H'0001'
DCNT2            EQU  H'0002'
DCNT3            EQU  H'0003'
DCNT4            EQU  H'0004'
DCNT5            EQU  H'0005'
DCNT6            EQU  H'0006'
DCNT7            EQU  H'0007'


DMA1DCNT0        EQU  H'0000'
DMA1DCNT1        EQU  H'0001'
DMA1DCNT2        EQU  H'0002'
DMA1DCNT3        EQU  H'0003'
DMA1DCNT4        EQU  H'0004'
DMA1DCNT5        EQU  H'0005'
DMA1DCNT6        EQU  H'0006'
DMA1DCNT7        EQU  H'0007'


;----- DMA1DCNTH Bits -----------------------------------------------------
DCNT8            EQU  H'0000'
DCNT9            EQU  H'0001'
DCNT10           EQU  H'0002'
DCNT11           EQU  H'0003'


DMA1DCNT8        EQU  H'0000'
DMA1DCNT9        EQU  H'0001'
DMA1DCNT10       EQU  H'0002'
DMA1DCNT11       EQU  H'0003'


;----- DMA1DPTRL Bits -----------------------------------------------------
DPTR0            EQU  H'0000'
DPTR1            EQU  H'0001'
DPTR2            EQU  H'0002'
DPTR3            EQU  H'0003'
DPTR4            EQU  H'0004'
DPTR5            EQU  H'0005'
DPTR6            EQU  H'0006'
DPTR7            EQU  H'0007'


DMA1DPTR0        EQU  H'0000'
DMA1DPTR1        EQU  H'0001'
DMA1DPTR2        EQU  H'0002'
DMA1DPTR3        EQU  H'0003'
DMA1DPTR4        EQU  H'0004'
DMA1DPTR5        EQU  H'0005'
DMA1DPTR6        EQU  H'0006'
DMA1DPTR7        EQU  H'0007'


;----- DMA1DPTRH Bits -----------------------------------------------------
DPTR8            EQU  H'0000'
DPTR9            EQU  H'0001'
DPTR10           EQU  H'0002'
DPTR11           EQU  H'0003'
DPTR12           EQU  H'0004'
DPTR13           EQU  H'0005'
DPTR14           EQU  H'0006'
DPTR15           EQU  H'0007'


DMA1DPTR8        EQU  H'0000'
DMA1DPTR9        EQU  H'0001'
DMA1DPTR10       EQU  H'0002'
DMA1DPTR11       EQU  H'0003'
DMA1DPTR12       EQU  H'0004'
DMA1DPTR13       EQU  H'0005'
DMA1DPTR14       EQU  H'0006'
DMA1DPTR15       EQU  H'0007'


;----- DMA1DSZL Bits -----------------------------------------------------
DSZ0             EQU  H'0000'
DSZ1             EQU  H'0001'
DSZ2             EQU  H'0002'
DSZ3             EQU  H'0003'
DSZ4             EQU  H'0004'
DSZ5             EQU  H'0005'
DSZ6             EQU  H'0006'
DSZ7             EQU  H'0007'

DMA1DSZ0         EQU  H'0000'
DMA1DSZ1         EQU  H'0001'
DMA1DSZ2         EQU  H'0002'
DMA1DSZ3         EQU  H'0003'
DMA1DSZ4         EQU  H'0004'
DMA1DSZ5         EQU  H'0005'
DMA1DSZ6         EQU  H'0006'
DMA1DSZ7         EQU  H'0007'


;----- DMA1DSZH Bits -----------------------------------------------------
DSZ8             EQU  H'0000'
DSZ9             EQU  H'0001'
DSZ10            EQU  H'0002'
DSZ11            EQU  H'0003'

DMA1DSZ8         EQU  H'0000'
DMA1DSZ9         EQU  H'0001'
DMA1DSZ10        EQU  H'0002'
DMA1DSZ11        EQU  H'0003'


;----- DMA1DSAL Bits -----------------------------------------------------
SSA0             EQU  H'0000'
SSA1             EQU  H'0001'
SSA2             EQU  H'0002'
SSA3             EQU  H'0003'
SSA4             EQU  H'0004'
SSA5             EQU  H'0005'
SSA6             EQU  H'0006'
SSA7             EQU  H'0007'


DMA1DSA0         EQU  H'0000'
DMA1DSA1         EQU  H'0001'
DMA1DSA2         EQU  H'0002'
DMA1DSA3         EQU  H'0003'
DMA1DSA4         EQU  H'0004'
DMA1DSA5         EQU  H'0005'
DMA1DSA6         EQU  H'0006'
DMA1DSA7         EQU  H'0007'


;----- DMA1DSAH Bits -----------------------------------------------------
DSA8             EQU  H'0000'
DSA9             EQU  H'0001'
DSA10            EQU  H'0002'
DSA11            EQU  H'0003'
DSA12            EQU  H'0004'
DSA13            EQU  H'0005'
DSA14            EQU  H'0006'
DSA15            EQU  H'0007'


DMA1DSA8         EQU  H'0000'
DMA1DSA9         EQU  H'0001'
DMA1DSA10        EQU  H'0002'
DMA1DSA11        EQU  H'0003'
DMA1DSA12        EQU  H'0004'
DMA1DSA13        EQU  H'0005'
DMA1DSA14        EQU  H'0006'
DMA1DSA15        EQU  H'0007'


;----- DMA1SCNTL Bits -----------------------------------------------------
SCNT0            EQU  H'0000'
SCNT1            EQU  H'0001'
SCNT2            EQU  H'0002'
SCNT3            EQU  H'0003'
SCNT4            EQU  H'0004'
SCNT5            EQU  H'0005'
SCNT6            EQU  H'0006'
SCNT7            EQU  H'0007'


DMA1SCNT0        EQU  H'0000'
DMA1SCNT1        EQU  H'0001'
DMA1SCNT2        EQU  H'0002'
DMA1SCNT3        EQU  H'0003'
DMA1SCNT4        EQU  H'0004'
DMA1SCNT5        EQU  H'0005'
DMA1SCNT6        EQU  H'0006'
DMA1SCNT7        EQU  H'0007'


;----- DMA1SCNTH Bits -----------------------------------------------------
SCNT8            EQU  H'0000'
SCNT9            EQU  H'0001'
SCNT10           EQU  H'0002'
SCNT11           EQU  H'0003'


DMA1SCNT8        EQU  H'0000'
DMA1SCNT9        EQU  H'0001'
DMA1SCNT10       EQU  H'0002'
DMA1SCNT11       EQU  H'0003'


;----- DMA1SPTRL Bits -----------------------------------------------------
SPTR0            EQU  H'0000'
SPTR1            EQU  H'0001'
SPTR2            EQU  H'0002'
SPTR3            EQU  H'0003'
SPTR4            EQU  H'0004'
SPTR5            EQU  H'0005'
SPTR6            EQU  H'0006'
SPTR7            EQU  H'0007'


DMA1SPTR0        EQU  H'0000'
DMA1SPTR1        EQU  H'0001'
DMA1SPTR2        EQU  H'0002'
DMA1SPTR3        EQU  H'0003'
DMA1SPTR4        EQU  H'0004'
DMA1SPTR5        EQU  H'0005'
DMA1SPTR6        EQU  H'0006'
DMA1SPTR7        EQU  H'0007'


;----- DMA1SPTRH Bits -----------------------------------------------------
SPTR8            EQU  H'0000'
SPTR9            EQU  H'0001'
SPTR10           EQU  H'0002'
SPTR11           EQU  H'0003'
SPTR12           EQU  H'0004'
SPTR13           EQU  H'0005'
SPTR14           EQU  H'0006'
SPTR15           EQU  H'0007'


DMA1SPTR8        EQU  H'0000'
DMA1SPTR9        EQU  H'0001'
DMA1SPTR10       EQU  H'0002'
DMA1SPTR11       EQU  H'0003'
DMA1SPTR12       EQU  H'0004'
DMA1SPTR13       EQU  H'0005'
DMA1SPTR14       EQU  H'0006'
DMA1SPTR15       EQU  H'0007'


;----- DMA1SPTRU Bits -----------------------------------------------------
SPTR16           EQU  H'0000'
SPTR17           EQU  H'0001'
SPTR18           EQU  H'0002'
SPTR19           EQU  H'0003'
SPTR20           EQU  H'0004'
SPTR21           EQU  H'0005'


DMA1SPTR16       EQU  H'0000'
DMA1SPTR17       EQU  H'0001'
DMA1SPTR18       EQU  H'0002'
DMA1SPTR19       EQU  H'0003'
DMA1SPTR20       EQU  H'0004'
DMA1SPTR21       EQU  H'0005'


;----- DMA1SSZL Bits -----------------------------------------------------
SSZ0             EQU  H'0000'
SSZ1             EQU  H'0001'
SSZ2             EQU  H'0002'
SSZ3             EQU  H'0003'
SSZ4             EQU  H'0004'
SSZ5             EQU  H'0005'
SSZ6             EQU  H'0006'
SSZ7             EQU  H'0007'

DMA1SSZ0         EQU  H'0000'
DMA1SSZ1         EQU  H'0001'
DMA1SSZ2         EQU  H'0002'
DMA1SSZ3         EQU  H'0003'
DMA1SSZ4         EQU  H'0004'
DMA1SSZ5         EQU  H'0005'
DMA1SSZ6         EQU  H'0006'
DMA1SSZ7         EQU  H'0007'


;----- DMA1SSZH Bits -----------------------------------------------------
SSZ8             EQU  H'0000'
SSZ9             EQU  H'0001'
SSZ10            EQU  H'0002'
SSZ11            EQU  H'0003'

DMA1SSZ8         EQU  H'0000'
DMA1SSZ9         EQU  H'0001'
DMA1SSZ10        EQU  H'0002'
DMA1SSZ11        EQU  H'0003'


;----- DMA1SSAL Bits -----------------------------------------------------
SSA0             EQU  H'0000'
SSA1             EQU  H'0001'
SSA2             EQU  H'0002'
SSA3             EQU  H'0003'
SSA4             EQU  H'0004'
SSA5             EQU  H'0005'
SSA6             EQU  H'0006'
SSA7             EQU  H'0007'


DMA1SSA0         EQU  H'0000'
DMA1SSA1         EQU  H'0001'
DMA1SSA2         EQU  H'0002'
DMA1SSA3         EQU  H'0003'
DMA1SSA4         EQU  H'0004'
DMA1SSA5         EQU  H'0005'
DMA1SSA6         EQU  H'0006'
DMA1SSA7         EQU  H'0007'


;----- DMA1SSAH Bits -----------------------------------------------------
SSA8             EQU  H'0000'
SSA9             EQU  H'0001'
SSA10            EQU  H'0002'
SSA11            EQU  H'0003'
SSA12            EQU  H'0004'
SSA13            EQU  H'0005'
SSA14            EQU  H'0006'
SSA15            EQU  H'0007'


DMA1SSA8         EQU  H'0000'
DMA1SSA9         EQU  H'0001'
DMA1SSA10        EQU  H'0002'
DMA1SSA11        EQU  H'0003'
DMA1SSA12        EQU  H'0004'
DMA1SSA13        EQU  H'0005'
DMA1SSA14        EQU  H'0006'
DMA1SSA15        EQU  H'0007'


;----- DMA1SSAU Bits -----------------------------------------------------
SSA16            EQU  H'0000'
SSA17            EQU  H'0001'
SSA18            EQU  H'0002'
SSA19            EQU  H'0003'
SSA20            EQU  H'0004'
SSA21            EQU  H'0005'


DMA1SSA16        EQU  H'0000'
DMA1SSA17        EQU  H'0001'
DMA1SSA18        EQU  H'0002'
DMA1SSA19        EQU  H'0003'
DMA1SSA20        EQU  H'0004'
DMA1SSA21        EQU  H'0005'


;----- DMA1CON0 Bits -----------------------------------------------------
XIP              EQU  H'0000'
AIRQEN           EQU  H'0002'
DGO              EQU  H'0005'
SIRQEN           EQU  H'0006'
EN               EQU  H'0007'

DMA1XIP          EQU  H'0000'
DMA1AIRQEN       EQU  H'0002'
DMA1DGO          EQU  H'0005'
DMA1SIRQEN       EQU  H'0006'
DMA1EN           EQU  H'0007'


;----- DMA1CON1 Bits -----------------------------------------------------
SSTP             EQU  H'0000'
DSTP             EQU  H'0005'


;----- DMA1AIRQ Bits -----------------------------------------------------
AIRQ0            EQU  H'0000'
AIRQ1            EQU  H'0001'
AIRQ2            EQU  H'0002'
AIRQ3            EQU  H'0003'
AIRQ4            EQU  H'0004'
AIRQ5            EQU  H'0005'
AIRQ6            EQU  H'0006'

DMA1AIRQ0        EQU  H'0000'
DMA1AIRQ1        EQU  H'0001'
DMA1AIRQ2        EQU  H'0002'
DMA1AIRQ3        EQU  H'0003'
DMA1AIRQ4        EQU  H'0004'
DMA1AIRQ5        EQU  H'0005'
DMA1AIRQ6        EQU  H'0006'


;----- DMA1SIRQ Bits -----------------------------------------------------
SIRQ0            EQU  H'0000'
SIRQ1            EQU  H'0001'
SIRQ2            EQU  H'0002'
SIRQ3            EQU  H'0003'
SIRQ4            EQU  H'0004'
SIRQ5            EQU  H'0005'
SIRQ6            EQU  H'0006'

DMA1SIRQ0        EQU  H'0000'
DMA1SIRQ1        EQU  H'0001'
DMA1SIRQ2        EQU  H'0002'
DMA1SIRQ3        EQU  H'0003'
DMA1SIRQ4        EQU  H'0004'
DMA1SIRQ5        EQU  H'0005'
DMA1SIRQ6        EQU  H'0006'


;----- CLC4CON Bits -----------------------------------------------------
INTN_CLC4CON     EQU  H'0003'
INTP_CLC4CON     EQU  H'0004'
OUT_CLC4CON      EQU  H'0005'
OE               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

LC4INTN          EQU  H'0003'
LC4INTP          EQU  H'0004'
LC4OUT           EQU  H'0005'
LC4OE            EQU  H'0006'
LC4EN            EQU  H'0007'

LC4MODE0         EQU  H'0000'
LC4MODE1         EQU  H'0001'
LC4MODE2         EQU  H'0002'


;----- CLC4POL Bits -----------------------------------------------------
G1POL            EQU  H'0000'
G2POL            EQU  H'0001'
G3POL            EQU  H'0002'
G4POL            EQU  H'0003'
POL_CLC4POL      EQU  H'0007'

LC4G1POL         EQU  H'0000'
LC4G2POL         EQU  H'0001'
LC4G3POL         EQU  H'0002'
LC4G4POL         EQU  H'0003'
LC4POL           EQU  H'0007'


;----- CLC4SEL0 Bits -----------------------------------------------------
D1S0             EQU  H'0000'
D1S1             EQU  H'0001'
D1S2             EQU  H'0002'
D1S3             EQU  H'0003'
D1S4             EQU  H'0004'
D1S5             EQU  H'0005'
D1S6             EQU  H'0006'
D1S7             EQU  H'0007'

LC4D1S0          EQU  H'0000'
LC4D1S1          EQU  H'0001'
LC4D1S2          EQU  H'0002'
LC4D1S3          EQU  H'0003'
LC4D1S4          EQU  H'0004'
LC4D1S5          EQU  H'0005'
LC4D1S6          EQU  H'0006'
LC4D1S7          EQU  H'0007'


;----- CLC4SEL1 Bits -----------------------------------------------------
D2S0             EQU  H'0000'
D2S1             EQU  H'0001'
D2S2             EQU  H'0002'
D2S3             EQU  H'0003'
D2S4             EQU  H'0004'
D2S5             EQU  H'0005'
D2S6             EQU  H'0006'
D2S7             EQU  H'0007'

LC4D2S0          EQU  H'0000'
LC4D2S1          EQU  H'0001'
LC4D2S2          EQU  H'0002'
LC4D2S3          EQU  H'0003'
LC4D2S4          EQU  H'0004'
LC4D2S5          EQU  H'0005'
LC4D2S6          EQU  H'0006'
LC4D2S7          EQU  H'0007'


;----- CLC4SEL2 Bits -----------------------------------------------------
D3S0             EQU  H'0000'
D3S1             EQU  H'0001'
D3S2             EQU  H'0002'
D3S3             EQU  H'0003'
D3S4             EQU  H'0004'
D3S5             EQU  H'0005'
D3S6             EQU  H'0006'
D3S7             EQU  H'0007'

LC4D3S0          EQU  H'0000'
LC4D3S1          EQU  H'0001'
LC4D3S2          EQU  H'0002'
LC4D3S3          EQU  H'0003'
LC4D3S4          EQU  H'0004'
LC4D3S5          EQU  H'0005'
LC4D3S6          EQU  H'0006'
LC4D3S7          EQU  H'0007'


;----- CLC4SEL3 Bits -----------------------------------------------------
D4S0             EQU  H'0000'
D4S1             EQU  H'0001'
D4S2             EQU  H'0002'
D4S3             EQU  H'0003'
D4S4             EQU  H'0004'
D4S5             EQU  H'0005'
D4S6             EQU  H'0006'
D4S7             EQU  H'0007'

LC4D4S0          EQU  H'0000'
LC4D4S1          EQU  H'0001'
LC4D4S2          EQU  H'0002'
LC4D4S3          EQU  H'0003'
LC4D4S4          EQU  H'0004'
LC4D4S5          EQU  H'0005'
LC4D4S6          EQU  H'0006'
LC4D4S7          EQU  H'0007'


;----- CLC4GLS0 Bits -----------------------------------------------------
G1D1N            EQU  H'0000'
G1D1T            EQU  H'0001'
G1D2N            EQU  H'0002'
G1D2T            EQU  H'0003'
G1D3N            EQU  H'0004'
G1D3T            EQU  H'0005'
G1D4N            EQU  H'0006'
G1D4T            EQU  H'0007'

LC4G1D1N         EQU  H'0000'
LC4G1D1T         EQU  H'0001'
LC4G1D2N         EQU  H'0002'
LC4G1D2T         EQU  H'0003'
LC4G1D3N         EQU  H'0004'
LC4G1D3T         EQU  H'0005'
LC4G1D4N         EQU  H'0006'
LC4G1D4T         EQU  H'0007'


;----- CLC4GLS1 Bits -----------------------------------------------------
G2D1N            EQU  H'0000'
G2D1T            EQU  H'0001'
G2D2N            EQU  H'0002'
G2D2T            EQU  H'0003'
G2D3N            EQU  H'0004'
G2D3T            EQU  H'0005'
G2D4N            EQU  H'0006'
G2D4T            EQU  H'0007'

LC4G2D1N         EQU  H'0000'
LC4G2D1T         EQU  H'0001'
LC4G2D2N         EQU  H'0002'
LC4G2D2T         EQU  H'0003'
LC4G2D3N         EQU  H'0004'
LC4G2D3T         EQU  H'0005'
LC4G2D4N         EQU  H'0006'
LC4G2D4T         EQU  H'0007'


;----- CLC4GLS2 Bits -----------------------------------------------------
G3D1N            EQU  H'0000'
G3D1T            EQU  H'0001'
G3D2N            EQU  H'0002'
G3D2T            EQU  H'0003'
G3D3N            EQU  H'0004'
G3D3T            EQU  H'0005'
G3D4N            EQU  H'0006'
G3D4T            EQU  H'0007'

LC4G3D1N         EQU  H'0000'
LC4G3D1T         EQU  H'0001'
LC4G3D2N         EQU  H'0002'
LC4G3D2T         EQU  H'0003'
LC4G3D3N         EQU  H'0004'
LC4G3D3T         EQU  H'0005'
LC4G3D4N         EQU  H'0006'
LC4G3D4T         EQU  H'0007'


;----- CLC4GLS3 Bits -----------------------------------------------------
G4D1N            EQU  H'0000'
G4D1T            EQU  H'0001'
G4D2N            EQU  H'0002'
G4D2T            EQU  H'0003'
G4D3N            EQU  H'0004'
G4D3T            EQU  H'0005'
G4D4N            EQU  H'0006'
G4D4T            EQU  H'0007'

LC4G4D1N         EQU  H'0000'
LC4G4D1T         EQU  H'0001'
LC4G4D2N         EQU  H'0002'
LC4G4D2T         EQU  H'0003'
LC4G4D3N         EQU  H'0004'
LC4G4D3T         EQU  H'0005'
LC4G4D4N         EQU  H'0006'
LC4G4D4T         EQU  H'0007'


;----- CLC3CON Bits -----------------------------------------------------
INTN_CLC3CON     EQU  H'0003'
INTP_CLC3CON     EQU  H'0004'
OUT_CLC3CON      EQU  H'0005'
OE               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

LC3INTN          EQU  H'0003'
LC3INTP          EQU  H'0004'
LC3OUT           EQU  H'0005'
LC3OE            EQU  H'0006'
LC3EN            EQU  H'0007'

LC3MODE0         EQU  H'0000'
LC3MODE1         EQU  H'0001'
LC3MODE2         EQU  H'0002'


;----- CLC3POL Bits -----------------------------------------------------
G1POL            EQU  H'0000'
G2POL            EQU  H'0001'
G3POL            EQU  H'0002'
G4POL            EQU  H'0003'
POL_CLC3POL      EQU  H'0007'

LC3G1POL         EQU  H'0000'
LC3G2POL         EQU  H'0001'
LC3G3POL         EQU  H'0002'
LC3G4POL         EQU  H'0003'
LC3POL           EQU  H'0007'


;----- CLC3SEL0 Bits -----------------------------------------------------
D1S0             EQU  H'0000'
D1S1             EQU  H'0001'
D1S2             EQU  H'0002'
D1S3             EQU  H'0003'
D1S4             EQU  H'0004'
D1S5             EQU  H'0005'
D1S6             EQU  H'0006'
D1S7             EQU  H'0007'

LC3D1S0          EQU  H'0000'
LC3D1S1          EQU  H'0001'
LC3D1S2          EQU  H'0002'
LC3D1S3          EQU  H'0003'
LC3D1S4          EQU  H'0004'
LC3D1S5          EQU  H'0005'
LC3D1S6          EQU  H'0006'
LC3D1S7          EQU  H'0007'


;----- CLC3SEL1 Bits -----------------------------------------------------
D2S0             EQU  H'0000'
D2S1             EQU  H'0001'
D2S2             EQU  H'0002'
D2S3             EQU  H'0003'
D2S4             EQU  H'0004'
D2S5             EQU  H'0005'
D2S6             EQU  H'0006'
D2S7             EQU  H'0007'

LC3D2S0          EQU  H'0000'
LC3D2S1          EQU  H'0001'
LC3D2S2          EQU  H'0002'
LC3D2S3          EQU  H'0003'
LC3D2S4          EQU  H'0004'
LC3D2S5          EQU  H'0005'
LC3D2S6          EQU  H'0006'
LC3D2S7          EQU  H'0007'


;----- CLC3SEL2 Bits -----------------------------------------------------
D3S0             EQU  H'0000'
D3S1             EQU  H'0001'
D3S2             EQU  H'0002'
D3S3             EQU  H'0003'
D3S4             EQU  H'0004'
D3S5             EQU  H'0005'
D3S6             EQU  H'0006'
D3S7             EQU  H'0007'

LC3D3S0          EQU  H'0000'
LC3D3S1          EQU  H'0001'
LC3D3S2          EQU  H'0002'
LC3D3S3          EQU  H'0003'
LC3D3S4          EQU  H'0004'
LC3D3S5          EQU  H'0005'
LC3D3S6          EQU  H'0006'
LC3D3S7          EQU  H'0007'


;----- CLC3SEL3 Bits -----------------------------------------------------
D4S0             EQU  H'0000'
D4S1             EQU  H'0001'
D4S2             EQU  H'0002'
D4S3             EQU  H'0003'
D4S4             EQU  H'0004'
D4S5             EQU  H'0005'
D4S6             EQU  H'0006'
D4S7             EQU  H'0007'

LC3D4S0          EQU  H'0000'
LC3D4S1          EQU  H'0001'
LC3D4S2          EQU  H'0002'
LC3D4S3          EQU  H'0003'
LC3D4S4          EQU  H'0004'
LC3D4S5          EQU  H'0005'
LC3D4S6          EQU  H'0006'
LC3D4S7          EQU  H'0007'


;----- CLC3GLS0 Bits -----------------------------------------------------
G1D1N            EQU  H'0000'
G1D1T            EQU  H'0001'
G1D2N            EQU  H'0002'
G1D2T            EQU  H'0003'
G1D3N            EQU  H'0004'
G1D3T            EQU  H'0005'
G1D4N            EQU  H'0006'
G1D4T            EQU  H'0007'

LC3G1D1N         EQU  H'0000'
LC3G1D1T         EQU  H'0001'
LC3G1D2N         EQU  H'0002'
LC3G1D2T         EQU  H'0003'
LC3G1D3N         EQU  H'0004'
LC3G1D3T         EQU  H'0005'
LC3G1D4N         EQU  H'0006'
LC3G1D4T         EQU  H'0007'


;----- CLC3GLS1 Bits -----------------------------------------------------
G2D1N            EQU  H'0000'
G2D1T            EQU  H'0001'
G2D2N            EQU  H'0002'
G2D2T            EQU  H'0003'
G2D3N            EQU  H'0004'
G2D3T            EQU  H'0005'
G2D4N            EQU  H'0006'
G2D4T            EQU  H'0007'

LC3G2D1N         EQU  H'0000'
LC3G2D1T         EQU  H'0001'
LC3G2D2N         EQU  H'0002'
LC3G2D2T         EQU  H'0003'
LC3G2D3N         EQU  H'0004'
LC3G2D3T         EQU  H'0005'
LC3G2D4N         EQU  H'0006'
LC3G2D4T         EQU  H'0007'


;----- CLC3GLS2 Bits -----------------------------------------------------
G3D1N            EQU  H'0000'
G3D1T            EQU  H'0001'
G3D2N            EQU  H'0002'
G3D2T            EQU  H'0003'
G3D3N            EQU  H'0004'
G3D3T            EQU  H'0005'
G3D4N            EQU  H'0006'
G3D4T            EQU  H'0007'

LC3G3D1N         EQU  H'0000'
LC3G3D1T         EQU  H'0001'
LC3G3D2N         EQU  H'0002'
LC3G3D2T         EQU  H'0003'
LC3G3D3N         EQU  H'0004'
LC3G3D3T         EQU  H'0005'
LC3G3D4N         EQU  H'0006'
LC3G3D4T         EQU  H'0007'


;----- CLC3GLS3 Bits -----------------------------------------------------
G4D1N            EQU  H'0000'
G4D1T            EQU  H'0001'
G4D2N            EQU  H'0002'
G4D2T            EQU  H'0003'
G4D3N            EQU  H'0004'
G4D3T            EQU  H'0005'
G4D4N            EQU  H'0006'
G4D4T            EQU  H'0007'

LC3G4D1N         EQU  H'0000'
LC3G4D1T         EQU  H'0001'
LC3G4D2N         EQU  H'0002'
LC3G4D2T         EQU  H'0003'
LC3G4D3N         EQU  H'0004'
LC3G4D3T         EQU  H'0005'
LC3G4D4N         EQU  H'0006'
LC3G4D4T         EQU  H'0007'


;----- CLC2CON Bits -----------------------------------------------------
INTN_CLC2CON     EQU  H'0003'
INTP_CLC2CON     EQU  H'0004'
OUT_CLC2CON      EQU  H'0005'
OE               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

LC2INTN          EQU  H'0003'
LC2INTP          EQU  H'0004'
LC2OUT           EQU  H'0005'
LC2OE            EQU  H'0006'
LC2EN            EQU  H'0007'

LC2MODE0         EQU  H'0000'
LC2MODE1         EQU  H'0001'
LC2MODE2         EQU  H'0002'


;----- CLC2POL Bits -----------------------------------------------------
G1POL            EQU  H'0000'
G2POL            EQU  H'0001'
G3POL            EQU  H'0002'
G4POL            EQU  H'0003'
POL_CLC2POL      EQU  H'0007'

LC2G1POL         EQU  H'0000'
LC2G2POL         EQU  H'0001'
LC2G3POL         EQU  H'0002'
LC2G4POL         EQU  H'0003'
LC2POL           EQU  H'0007'


;----- CLC2SEL0 Bits -----------------------------------------------------
D1S0             EQU  H'0000'
D1S1             EQU  H'0001'
D1S2             EQU  H'0002'
D1S3             EQU  H'0003'
D1S4             EQU  H'0004'
D1S5             EQU  H'0005'
D1S6             EQU  H'0006'
D1S7             EQU  H'0007'

LC2D1S0          EQU  H'0000'
LC2D1S1          EQU  H'0001'
LC2D1S2          EQU  H'0002'
LC2D1S3          EQU  H'0003'
LC2D1S4          EQU  H'0004'
LC2D1S5          EQU  H'0005'
LC2D1S6          EQU  H'0006'
LC2D1S7          EQU  H'0007'


;----- CLC2SEL1 Bits -----------------------------------------------------
D2S0             EQU  H'0000'
D2S1             EQU  H'0001'
D2S2             EQU  H'0002'
D2S3             EQU  H'0003'
D2S4             EQU  H'0004'
D2S5             EQU  H'0005'
D2S6             EQU  H'0006'
D2S7             EQU  H'0007'

LC2D2S0          EQU  H'0000'
LC2D2S1          EQU  H'0001'
LC2D2S2          EQU  H'0002'
LC2D2S3          EQU  H'0003'
LC2D2S4          EQU  H'0004'
LC2D2S5          EQU  H'0005'
LC2D2S6          EQU  H'0006'
LC2D2S7          EQU  H'0007'


;----- CLC2SEL2 Bits -----------------------------------------------------
D3S0             EQU  H'0000'
D3S1             EQU  H'0001'
D3S2             EQU  H'0002'
D3S3             EQU  H'0003'
D3S4             EQU  H'0004'
D3S5             EQU  H'0005'
D3S6             EQU  H'0006'
D3S7             EQU  H'0007'

LC2D3S0          EQU  H'0000'
LC2D3S1          EQU  H'0001'
LC2D3S2          EQU  H'0002'
LC2D3S3          EQU  H'0003'
LC2D3S4          EQU  H'0004'
LC2D3S5          EQU  H'0005'
LC2D3S6          EQU  H'0006'
LC2D3S7          EQU  H'0007'


;----- CLC2SEL3 Bits -----------------------------------------------------
D4S0             EQU  H'0000'
D4S1             EQU  H'0001'
D4S2             EQU  H'0002'
D4S3             EQU  H'0003'
D4S4             EQU  H'0004'
D4S5             EQU  H'0005'
D4S6             EQU  H'0006'
D4S7             EQU  H'0007'

LC2D4S0          EQU  H'0000'
LC2D4S1          EQU  H'0001'
LC2D4S2          EQU  H'0002'
LC2D4S3          EQU  H'0003'
LC2D4S4          EQU  H'0004'
LC2D4S5          EQU  H'0005'
LC2D4S6          EQU  H'0006'
LC2D4S7          EQU  H'0007'


;----- CLC2GLS0 Bits -----------------------------------------------------
G1D1N            EQU  H'0000'
G1D1T            EQU  H'0001'
G1D2N            EQU  H'0002'
G1D2T            EQU  H'0003'
G1D3N            EQU  H'0004'
G1D3T            EQU  H'0005'
G1D4N            EQU  H'0006'
G1D4T            EQU  H'0007'

LC2G1D1N         EQU  H'0000'
LC2G1D1T         EQU  H'0001'
LC2G1D2N         EQU  H'0002'
LC2G1D2T         EQU  H'0003'
LC2G1D3N         EQU  H'0004'
LC2G1D3T         EQU  H'0005'
LC2G1D4N         EQU  H'0006'
LC2G1D4T         EQU  H'0007'


;----- CLC2GLS1 Bits -----------------------------------------------------
G2D1N            EQU  H'0000'
G2D1T            EQU  H'0001'
G2D2N            EQU  H'0002'
G2D2T            EQU  H'0003'
G2D3N            EQU  H'0004'
G2D3T            EQU  H'0005'
G2D4N            EQU  H'0006'
G2D4T            EQU  H'0007'

LC2G2D1N         EQU  H'0000'
LC2G2D1T         EQU  H'0001'
LC2G2D2N         EQU  H'0002'
LC2G2D2T         EQU  H'0003'
LC2G2D3N         EQU  H'0004'
LC2G2D3T         EQU  H'0005'
LC2G2D4N         EQU  H'0006'
LC2G2D4T         EQU  H'0007'


;----- CLC2GLS2 Bits -----------------------------------------------------
G3D1N            EQU  H'0000'
G3D1T            EQU  H'0001'
G3D2N            EQU  H'0002'
G3D2T            EQU  H'0003'
G3D3N            EQU  H'0004'
G3D3T            EQU  H'0005'
G3D4N            EQU  H'0006'
G3D4T            EQU  H'0007'

LC2G3D1N         EQU  H'0000'
LC2G3D1T         EQU  H'0001'
LC2G3D2N         EQU  H'0002'
LC2G3D2T         EQU  H'0003'
LC2G3D3N         EQU  H'0004'
LC2G3D3T         EQU  H'0005'
LC2G3D4N         EQU  H'0006'
LC2G3D4T         EQU  H'0007'


;----- CLC2GLS3 Bits -----------------------------------------------------
G4D1N            EQU  H'0000'
G4D1T            EQU  H'0001'
G4D2N            EQU  H'0002'
G4D2T            EQU  H'0003'
G4D3N            EQU  H'0004'
G4D3T            EQU  H'0005'
G4D4N            EQU  H'0006'
G4D4T            EQU  H'0007'

LC2G4D1N         EQU  H'0000'
LC2G4D1T         EQU  H'0001'
LC2G4D2N         EQU  H'0002'
LC2G4D2T         EQU  H'0003'
LC2G4D3N         EQU  H'0004'
LC2G4D3T         EQU  H'0005'
LC2G4D4N         EQU  H'0006'
LC2G4D4T         EQU  H'0007'


;----- CLC1CON Bits -----------------------------------------------------
INTN_CLC1CON     EQU  H'0003'
INTP_CLC1CON     EQU  H'0004'
OUT_CLC1CON      EQU  H'0005'
OE               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

LC1INTN          EQU  H'0003'
LC1INTP          EQU  H'0004'
LC1OUT           EQU  H'0005'
LC1OE            EQU  H'0006'
LC1EN            EQU  H'0007'

LC1MODE0         EQU  H'0000'
LC1MODE1         EQU  H'0001'
LC1MODE2         EQU  H'0002'


;----- CLC1POL Bits -----------------------------------------------------
G1POL            EQU  H'0000'
G2POL            EQU  H'0001'
G3POL            EQU  H'0002'
G4POL            EQU  H'0003'
POL_CLC1POL      EQU  H'0007'

LC1G1POL         EQU  H'0000'
LC1G2POL         EQU  H'0001'
LC1G3POL         EQU  H'0002'
LC1G4POL         EQU  H'0003'
LC1POL           EQU  H'0007'


;----- CLC1SEL0 Bits -----------------------------------------------------
D1S0             EQU  H'0000'
D1S1             EQU  H'0001'
D1S2             EQU  H'0002'
D1S3             EQU  H'0003'
D1S4             EQU  H'0004'
D1S5             EQU  H'0005'
D1S6             EQU  H'0006'
D1S7             EQU  H'0007'

LC1D1S0          EQU  H'0000'
LC1D1S1          EQU  H'0001'
LC1D1S2          EQU  H'0002'
LC1D1S3          EQU  H'0003'
LC1D1S4          EQU  H'0004'
LC1D1S5          EQU  H'0005'
LC1D1S6          EQU  H'0006'
LC1D1S7          EQU  H'0007'


;----- CLC1SEL1 Bits -----------------------------------------------------
D2S0             EQU  H'0000'
D2S1             EQU  H'0001'
D2S2             EQU  H'0002'
D2S3             EQU  H'0003'
D2S4             EQU  H'0004'
D2S5             EQU  H'0005'
D2S6             EQU  H'0006'
D2S7             EQU  H'0007'

LC1D2S0          EQU  H'0000'
LC1D2S1          EQU  H'0001'
LC1D2S2          EQU  H'0002'
LC1D2S3          EQU  H'0003'
LC1D2S4          EQU  H'0004'
LC1D2S5          EQU  H'0005'
LC1D2S6          EQU  H'0006'
LC1D2S7          EQU  H'0007'


;----- CLC1SEL2 Bits -----------------------------------------------------
D3S0             EQU  H'0000'
D3S1             EQU  H'0001'
D3S2             EQU  H'0002'
D3S3             EQU  H'0003'
D3S4             EQU  H'0004'
D3S5             EQU  H'0005'
D3S6             EQU  H'0006'
D3S7             EQU  H'0007'

LC1D3S0          EQU  H'0000'
LC1D3S1          EQU  H'0001'
LC1D3S2          EQU  H'0002'
LC1D3S3          EQU  H'0003'
LC1D3S4          EQU  H'0004'
LC1D3S5          EQU  H'0005'
LC1D3S6          EQU  H'0006'
LC1D3S7          EQU  H'0007'


;----- CLC1SEL3 Bits -----------------------------------------------------
D4S0             EQU  H'0000'
D4S1             EQU  H'0001'
D4S2             EQU  H'0002'
D4S3             EQU  H'0003'
D4S4             EQU  H'0004'
D4S5             EQU  H'0005'
D4S6             EQU  H'0006'
D4S7             EQU  H'0007'

LC1D4S0          EQU  H'0000'
LC1D4S1          EQU  H'0001'
LC1D4S2          EQU  H'0002'
LC1D4S3          EQU  H'0003'
LC1D4S4          EQU  H'0004'
LC1D4S5          EQU  H'0005'
LC1D4S6          EQU  H'0006'
LC1D4S7          EQU  H'0007'


;----- CLC1GLS0 Bits -----------------------------------------------------
G1D1N            EQU  H'0000'
G1D1T            EQU  H'0001'
G1D2N            EQU  H'0002'
G1D2T            EQU  H'0003'
G1D3N            EQU  H'0004'
G1D3T            EQU  H'0005'
G1D4N            EQU  H'0006'
G1D4T            EQU  H'0007'

LC1G1D1N         EQU  H'0000'
LC1G1D1T         EQU  H'0001'
LC1G1D2N         EQU  H'0002'
LC1G1D2T         EQU  H'0003'
LC1G1D3N         EQU  H'0004'
LC1G1D3T         EQU  H'0005'
LC1G1D4N         EQU  H'0006'
LC1G1D4T         EQU  H'0007'


;----- CLC1GLS1 Bits -----------------------------------------------------
G2D1N            EQU  H'0000'
G2D1T            EQU  H'0001'
G2D2N            EQU  H'0002'
G2D2T            EQU  H'0003'
G2D3N            EQU  H'0004'
G2D3T            EQU  H'0005'
G2D4N            EQU  H'0006'
G2D4T            EQU  H'0007'

LC1G2D1N         EQU  H'0000'
LC1G2D1T         EQU  H'0001'
LC1G2D2N         EQU  H'0002'
LC1G2D2T         EQU  H'0003'
LC1G2D3N         EQU  H'0004'
LC1G2D3T         EQU  H'0005'
LC1G2D4N         EQU  H'0006'
LC1G2D4T         EQU  H'0007'


;----- CLC1GLS2 Bits -----------------------------------------------------
G3D1N            EQU  H'0000'
G3D1T            EQU  H'0001'
G3D2N            EQU  H'0002'
G3D2T            EQU  H'0003'
G3D3N            EQU  H'0004'
G3D3T            EQU  H'0005'
G3D4N            EQU  H'0006'
G3D4T            EQU  H'0007'

LC1G3D1N         EQU  H'0000'
LC1G3D1T         EQU  H'0001'
LC1G3D2N         EQU  H'0002'
LC1G3D2T         EQU  H'0003'
LC1G3D3N         EQU  H'0004'
LC1G3D3T         EQU  H'0005'
LC1G3D4N         EQU  H'0006'
LC1G3D4T         EQU  H'0007'


;----- CLC1GLS3 Bits -----------------------------------------------------
G4D1N            EQU  H'0000'
G4D1T            EQU  H'0001'
G4D2N            EQU  H'0002'
G4D2T            EQU  H'0003'
G4D3N            EQU  H'0004'
G4D3T            EQU  H'0005'
G4D4N            EQU  H'0006'
G4D4T            EQU  H'0007'

LC1G4D1N         EQU  H'0000'
LC1G4D1T         EQU  H'0001'
LC1G4D2N         EQU  H'0002'
LC1G4D2T         EQU  H'0003'
LC1G4D3N         EQU  H'0004'
LC1G4D3T         EQU  H'0005'
LC1G4D4N         EQU  H'0006'
LC1G4D4T         EQU  H'0007'


;----- CLCDATA0 Bits -----------------------------------------------------
CLC1OUT          EQU  H'0000'
CLC2OUT          EQU  H'0001'
CLC3OUT          EQU  H'0002'
CLC4OUT          EQU  H'0003'


;----- CLKRCON Bits -----------------------------------------------------
EN               EQU  H'0007'

CLKRDIV0         EQU  H'0000'
CLKRDIV1         EQU  H'0001'
CLKRDIV2         EQU  H'0002'
CLKRDC0          EQU  H'0003'
CLKRDC1          EQU  H'0004'
CLKREN           EQU  H'0007'



;----- CLKRCLK Bits -----------------------------------------------------
CLK0             EQU  H'0000'
CLK1             EQU  H'0001'
CLK2             EQU  H'0002'
CLK3             EQU  H'0003'

CLKRCLK0         EQU  H'0000'
CLKRCLK1         EQU  H'0001'
CLKRCLK2         EQU  H'0002'
CLKRCLK3         EQU  H'0003'


;----- MD1CON0 Bits -----------------------------------------------------
BIT              EQU  H'0000'
OPOL             EQU  H'0004'
OUT_MD1CON0      EQU  H'0005'
EN               EQU  H'0007'

MD1BIT           EQU  H'0000'
MD1OPOL          EQU  H'0004'
MD1OUT           EQU  H'0005'
MD1EN            EQU  H'0007'


;----- MD1CON1 Bits -----------------------------------------------------
CLSYNC           EQU  H'0000'
CLPOL            EQU  H'0001'
CHSYNC           EQU  H'0004'
CHPOL            EQU  H'0005'

MD1CLSYNC        EQU  H'0000'
MD1CLPOL         EQU  H'0001'
MD1CHSYNC        EQU  H'0004'
MD1CHPOL         EQU  H'0005'


;----- MD1SRC Bits -----------------------------------------------------
MS0              EQU  H'0000'
MS1              EQU  H'0001'
MS2              EQU  H'0002'
MS3              EQU  H'0003'
MS4              EQU  H'0004'


MD1MS0           EQU  H'0000'
MD1MS1           EQU  H'0001'
MD1MS2           EQU  H'0002'
MD1MS3           EQU  H'0003'
MD1MS4           EQU  H'0004'


;----- MD1CARL Bits -----------------------------------------------------
CL0              EQU  H'0000'
CL1              EQU  H'0001'
CL2              EQU  H'0002'
CL3              EQU  H'0003'
CL4              EQU  H'0004'


MD1CL0           EQU  H'0000'
MD1CL1           EQU  H'0001'
MD1CL2           EQU  H'0002'
MD1CL3           EQU  H'0003'
MD1CL4           EQU  H'0004'


;----- MD1CARH Bits -----------------------------------------------------
CH0              EQU  H'0000'
CH1              EQU  H'0001'
CH2              EQU  H'0002'
CH3              EQU  H'0003'
CH4              EQU  H'0004'


MD1CH0           EQU  H'0000'
MD1CH1           EQU  H'0001'
MD1CH2           EQU  H'0002'
MD1CH3           EQU  H'0003'
MD1CH4           EQU  H'0004'


;----- SPI1RXB Bits -----------------------------------------------------
RXB0             EQU  H'0000'
RXB1             EQU  H'0001'
RXB2             EQU  H'0002'
RXB3             EQU  H'0003'
RXB4             EQU  H'0004'
RXB5             EQU  H'0005'
RXB6             EQU  H'0006'
RXB7             EQU  H'0007'


;----- SPI1TXB Bits -----------------------------------------------------
TXB0             EQU  H'0000'
TXB1             EQU  H'0001'
TXB2             EQU  H'0002'
TXB3             EQU  H'0003'
TXB4             EQU  H'0004'
TXB5             EQU  H'0005'
TXB6             EQU  H'0006'
TXB7             EQU  H'0007'


;----- SPI1CON0 Bits -----------------------------------------------------
BMODE            EQU  H'0000'
MST              EQU  H'0001'
LSBF             EQU  H'0002'
EN               EQU  H'0007'

SPI1BMODE        EQU  H'0000'
SPI1MST          EQU  H'0001'
SPI1LSBF         EQU  H'0002'
SPI1SPIEN        EQU  H'0007'


;----- SPI1CON1 Bits -----------------------------------------------------
SDOP             EQU  H'0000'
SDIP             EQU  H'0001'
SSP              EQU  H'0002'
FST              EQU  H'0004'
CKP              EQU  H'0005'
CKE              EQU  H'0006'
SMP              EQU  H'0007'

SPI1SDOP         EQU  H'0000'
SPI1SDIP         EQU  H'0001'
SPI1SSP          EQU  H'0002'
SPI1FST          EQU  H'0004'
SPI1CKP          EQU  H'0005'
SPI1CKE          EQU  H'0006'
SPI1SMP          EQU  H'0007'


;----- SPI1CON2 Bits -----------------------------------------------------
RXR              EQU  H'0000'
TXR              EQU  H'0001'
SSET             EQU  H'0002'
SSFLT            EQU  H'0006'
BUSY_SPI1CON2    EQU  H'0007'

SPI1RXR          EQU  H'0000'
SPI1TXR          EQU  H'0001'
SPI1SSET         EQU  H'0002'
SPI1SSFLT        EQU  H'0006'
SPI1BUSY         EQU  H'0007'


;----- SPI1STATUS Bits -----------------------------------------------------
RXBF             EQU  H'0000'
CLRBF            EQU  H'0002'
RXRE             EQU  H'0003'
TXBE             EQU  H'0005'
TXWE             EQU  H'0007'

SPI1RXBF         EQU  H'0000'
SPI1CLRBF        EQU  H'0002'
SPI1RXRE         EQU  H'0003'
SPI1TXBE         EQU  H'0005'
SPI1TXWE         EQU  H'0007'


;----- SPI1TWIDTH Bits -----------------------------------------------------
TWIDTH0          EQU  H'0000'
TWIDTH1          EQU  H'0001'
TWIDTH2          EQU  H'0002'


;----- SPI1BAUD Bits -----------------------------------------------------
BAUD0            EQU  H'0000'
BAUD1            EQU  H'0001'
BAUD2            EQU  H'0002'
BAUD3            EQU  H'0003'
BAUD4            EQU  H'0004'
BAUD5            EQU  H'0005'
BAUD6            EQU  H'0006'
BAUD7            EQU  H'0007'


;----- SPI1INTF Bits -----------------------------------------------------
TXUIF            EQU  H'0001'
RXOIF            EQU  H'0002'
EOSIF            EQU  H'0004'
SOSIF            EQU  H'0005'
TCZIF            EQU  H'0006'
SRMTIF           EQU  H'0007'

SPI1TXUIF        EQU  H'0001'
SPI1RXOIF        EQU  H'0002'
SPI1EOSIF        EQU  H'0004'
SPI1SOSIF        EQU  H'0005'
SPI1TCZIF        EQU  H'0006'
SPI1SRMTIF       EQU  H'0007'


;----- SPI1INTE Bits -----------------------------------------------------
TXUIE            EQU  H'0001'
RXOIE            EQU  H'0002'
EOSIE            EQU  H'0004'
SOSIE            EQU  H'0005'
TCZIE            EQU  H'0006'
SRMTIE           EQU  H'0007'

SPI1TXUIE        EQU  H'0001'
SPI1RXOIE        EQU  H'0002'
SPI1EOSIE        EQU  H'0004'
SPI1SOSIE        EQU  H'0005'
SPI1TCZIE        EQU  H'0006'
SPI1SRMTIE       EQU  H'0007'


;----- SPI1CLK Bits -----------------------------------------------------
CLKSEL0          EQU  H'0000'
CLKSEL1          EQU  H'0001'
CLKSEL2          EQU  H'0002'


SPI1CLKSEL0      EQU  H'0000'
SPI1CLKSEL1      EQU  H'0001'
SPI1CLKSEL2      EQU  H'0002'


;----- I2C2CNT Bits -----------------------------------------------------
CNT0             EQU  H'0000'
CNT1             EQU  H'0001'
CNT2             EQU  H'0002'
CNT3             EQU  H'0003'
CNT4             EQU  H'0004'
CNT5             EQU  H'0005'
CNT6             EQU  H'0006'
CNT7             EQU  H'0007'


;----- I2C2CON0 Bits -----------------------------------------------------
MDR              EQU  H'0003'
CSTR             EQU  H'0004'
S                EQU  H'0005'
RSEN             EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
I2CEN            EQU  H'0007'


;----- I2C2CON1 Bits -----------------------------------------------------
CSD              EQU  H'0000'
TXU              EQU  H'0001'
RXO              EQU  H'0002'
ACKT             EQU  H'0004'
ACKSTAT          EQU  H'0005'
ACKDT            EQU  H'0006'
ACKCNT           EQU  H'0007'


;----- I2C2CON2 Bits -----------------------------------------------------
ABD              EQU  H'0004'
FME              EQU  H'0005'
GCEN             EQU  H'0006'
ACNT             EQU  H'0007'

BFRET0           EQU  H'0000'
BFRET1           EQU  H'0001'
SDAHT0           EQU  H'0002'
SDAHT1           EQU  H'0003'


;----- I2C2ERR Bits -----------------------------------------------------
NACKIE           EQU  H'0000'
BCLIE            EQU  H'0001'
BTOIE            EQU  H'0002'
NACKIF           EQU  H'0004'
BCLIF            EQU  H'0005'
BTOIF            EQU  H'0006'

NACK2IE          EQU  H'0000'
BCL2IE           EQU  H'0001'
BTO2IE           EQU  H'0002'
NACK2IF          EQU  H'0004'
BCL2IF           EQU  H'0005'
BTO2IF           EQU  H'0006'


;----- I2C2STAT0 Bits -----------------------------------------------------
D                EQU  H'0003'
R                EQU  H'0004'
MMA              EQU  H'0005'
SMA              EQU  H'0006'
BFRE             EQU  H'0007'

; DATA is a reserved word
; DATA             EQU  H'0003'
READ             EQU  H'0004'

NOT_ADDRESS      EQU  H'0003'
NOT_WRITE        EQU  H'0004'

NOT_A            EQU  H'0003'
NOT_W            EQU  H'0004'


;----- I2C2STAT1 Bits -----------------------------------------------------
RXBF             EQU  H'0000'
CLRBF            EQU  H'0002'
RXRE             EQU  H'0003'
TXBE             EQU  H'0005'
TXWE             EQU  H'0007'


;----- I2C2PIR Bits -----------------------------------------------------
SCIF             EQU  H'0000'
RSCIF            EQU  H'0001'
PCIF             EQU  H'0002'
ADRIF            EQU  H'0003'
WRIF             EQU  H'0004'
ACKTIF           EQU  H'0006'
CNTIF            EQU  H'0007'

SC2IF            EQU  H'0000'
RSC2IF           EQU  H'0001'
PC2IF            EQU  H'0002'
ADR2IF           EQU  H'0003'
WR2IF            EQU  H'0004'
ACKT2IF          EQU  H'0006'
CNT2IF           EQU  H'0007'


;----- I2C2PIE Bits -----------------------------------------------------
SCIE             EQU  H'0000'
RSCIE            EQU  H'0001'
PCIE             EQU  H'0002'
ADRIE            EQU  H'0003'
WRIE             EQU  H'0004'
ACKTIE           EQU  H'0006'
CNTIE            EQU  H'0007'

SC2IE            EQU  H'0000'
RSC2IE           EQU  H'0001'
PC2IE            EQU  H'0002'
ADR2IE           EQU  H'0003'
WR2IE            EQU  H'0004'
ACKT2IE          EQU  H'0006'
CNT2IE           EQU  H'0007'


;----- I2C2CLK Bits -----------------------------------------------------
CLK0             EQU  H'0000'
CLK1             EQU  H'0001'
CLK2             EQU  H'0002'
CLK3             EQU  H'0003'

I2CCLK0          EQU  H'0000'
I2CCLK1          EQU  H'0001'
I2CCLK2          EQU  H'0002'
I2CCLK3          EQU  H'0003'


;----- I2C2BTO Bits -----------------------------------------------------
BTO0             EQU  H'0000'
BTO1             EQU  H'0001'
BTO2             EQU  H'0002'

I2CBTO0          EQU  H'0000'
I2CBTO1          EQU  H'0001'
I2CBTO2          EQU  H'0002'


;----- I2C1CNT Bits -----------------------------------------------------
CNT0             EQU  H'0000'
CNT1             EQU  H'0001'
CNT2             EQU  H'0002'
CNT3             EQU  H'0003'
CNT4             EQU  H'0004'
CNT5             EQU  H'0005'
CNT6             EQU  H'0006'
CNT7             EQU  H'0007'


;----- I2C1CON0 Bits -----------------------------------------------------
MDR              EQU  H'0003'
CSTR             EQU  H'0004'
S                EQU  H'0005'
RSEN             EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
I2CEN            EQU  H'0007'


;----- I2C1CON1 Bits -----------------------------------------------------
CSD              EQU  H'0000'
TXU              EQU  H'0001'
RXO              EQU  H'0002'
ACKT             EQU  H'0004'
ACKSTAT          EQU  H'0005'
ACKDT            EQU  H'0006'
ACKCNT           EQU  H'0007'


;----- I2C1CON2 Bits -----------------------------------------------------
ABD              EQU  H'0004'
FME              EQU  H'0005'
GCEN             EQU  H'0006'
ACNT             EQU  H'0007'

BFRET0           EQU  H'0000'
BFRET1           EQU  H'0001'
SDAHT0           EQU  H'0002'
SDAHT1           EQU  H'0003'


;----- I2C1ERR Bits -----------------------------------------------------
NACKIE           EQU  H'0000'
BCLIE            EQU  H'0001'
BTOIE            EQU  H'0002'
NACKIF           EQU  H'0004'
BCLIF            EQU  H'0005'
BTOIF            EQU  H'0006'

NACK1IE          EQU  H'0000'
BCL1IE           EQU  H'0001'
BTO1IE           EQU  H'0002'
NACK1IF          EQU  H'0004'
BCL1IF           EQU  H'0005'
BTO1IF           EQU  H'0006'


;----- I2C1STAT0 Bits -----------------------------------------------------
D                EQU  H'0003'
R                EQU  H'0004'
MMA              EQU  H'0005'
SMA              EQU  H'0006'
BFRE             EQU  H'0007'

; DATA is a reserved word
; DATA             EQU  H'0003'
READ             EQU  H'0004'

NOT_ADDRESS      EQU  H'0003'
NOT_WRITE        EQU  H'0004'

NOT_A            EQU  H'0003'
NOT_W            EQU  H'0004'


;----- I2C1STAT1 Bits -----------------------------------------------------
RXBF             EQU  H'0000'
CLRBF            EQU  H'0002'
RXRE             EQU  H'0003'
TXBE             EQU  H'0005'
TXWE             EQU  H'0007'


;----- I2C1PIR Bits -----------------------------------------------------
SCIF             EQU  H'0000'
RSCIF            EQU  H'0001'
PCIF             EQU  H'0002'
ADRIF            EQU  H'0003'
WRIF             EQU  H'0004'
ACKTIF           EQU  H'0006'
CNTIF            EQU  H'0007'

SC1IF            EQU  H'0000'
RSC1IF           EQU  H'0001'
PC1IF            EQU  H'0002'
ADR1IF           EQU  H'0003'
WR1IF            EQU  H'0004'
ACKT1IF          EQU  H'0006'
CNT1IF           EQU  H'0007'


;----- I2C1PIE Bits -----------------------------------------------------
SCIE             EQU  H'0000'
RSCIE            EQU  H'0001'
PCIE             EQU  H'0002'
ADRIE            EQU  H'0003'
WRIE             EQU  H'0004'
ACKTIE           EQU  H'0006'
CNTIE            EQU  H'0007'

SC1IE            EQU  H'0000'
RSC1IE           EQU  H'0001'
PC1IE            EQU  H'0002'
ADR1IE           EQU  H'0003'
WR1IE            EQU  H'0004'
ACKT1IE          EQU  H'0006'
CNT1IE           EQU  H'0007'


;----- I2C1CLK Bits -----------------------------------------------------
CLK0             EQU  H'0000'
CLK1             EQU  H'0001'
CLK2             EQU  H'0002'
CLK3             EQU  H'0003'

I2CCLK0          EQU  H'0000'
I2CCLK1          EQU  H'0001'
I2CCLK2          EQU  H'0002'
I2CCLK3          EQU  H'0003'


;----- I2C1BTO Bits -----------------------------------------------------
BTO0             EQU  H'0000'
BTO1             EQU  H'0001'
BTO2             EQU  H'0002'

I2CBTO0          EQU  H'0000'
I2CBTO1          EQU  H'0001'
I2CBTO2          EQU  H'0002'


;----- U2CON0 Bits -----------------------------------------------------
MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

U2RXEN           EQU  H'0004'
U2TXEN           EQU  H'0005'
U2ABDEN          EQU  H'0006'
U2BRGS           EQU  H'0007'

U2MODE0          EQU  H'0000'
U2MODE1          EQU  H'0001'
U2MODE2          EQU  H'0002'

RXEN             EQU  H'0004'
TXEN             EQU  H'0005'
ABDEN            EQU  H'0006'
BRGS             EQU  H'0007'


;----- U2CON1 Bits -----------------------------------------------------
SENDB            EQU  H'0000'
BRKOVR           EQU  H'0001'
RXBIMD           EQU  H'0003'
WUE              EQU  H'0004'
ON_U2CON1        EQU  H'0007'

U2SENDB          EQU  H'0000'
U2BRKOVR         EQU  H'0001'
U2RXBIMD         EQU  H'0003'
U2WUE            EQU  H'0004'
U2ON             EQU  H'0007'


;----- U2CON2 Bits -----------------------------------------------------
TXPOL            EQU  H'0002'
RXPOL            EQU  H'0006'
RUNOVF           EQU  H'0007'

FLO0             EQU  H'0000'
FLO1             EQU  H'0001'
STP0             EQU  H'0004'
STP1             EQU  H'0005'

U2TXPOL          EQU  H'0002'
U2RXPOL          EQU  H'0006'
U2RUNOVF         EQU  H'0007'

U2FLO0           EQU  H'0000'
U2FLO1           EQU  H'0001'
U2STP0           EQU  H'0004'
U2STP1           EQU  H'0005'


;----- U2FIFO Bits -----------------------------------------------------
RXBF             EQU  H'0000'
RXBE             EQU  H'0001'
XON              EQU  H'0002'
RXIDL            EQU  H'0003'
TXBF             EQU  H'0004'
TXBE             EQU  H'0005'
STPMD            EQU  H'0006'
TXWRE            EQU  H'0007'

U2RXBF           EQU  H'0000'
U2RXBE           EQU  H'0001'
U2XON            EQU  H'0002'
U2RXIDL          EQU  H'0003'
U2TXBF           EQU  H'0004'
U2TXBE           EQU  H'0005'
U2STPMD          EQU  H'0006'
U2TXWRE          EQU  H'0007'

U2RCIDL          EQU  H'0003'

RCIDL            EQU  H'0003'


;----- U2UIR Bits -----------------------------------------------------
ABDIE            EQU  H'0002'
ABDIF            EQU  H'0006'
WUIF             EQU  H'0007'

U2ABDIE          EQU  H'0002'
U2ABDIF          EQU  H'0006'
U2WUIF           EQU  H'0007'


;----- U2ERRIR Bits -----------------------------------------------------
TXCIF            EQU  H'0000'
RXFOIF           EQU  H'0001'
RXBKIF           EQU  H'0002'
FERIF            EQU  H'0003'
CERIF            EQU  H'0004'
ABDOVF           EQU  H'0005'
PERIF            EQU  H'0006'
TXMTIF           EQU  H'0007'

U2TXCIF          EQU  H'0000'
U2RXFOIF         EQU  H'0001'
U2RXBKIF         EQU  H'0002'
U2FERIF          EQU  H'0003'
U2CERIF          EQU  H'0004'
U2ABDOVF         EQU  H'0005'
U2PERIF          EQU  H'0006'
U2TXMTIF         EQU  H'0007'


;----- U2ERRIE Bits -----------------------------------------------------
TXCIE            EQU  H'0000'
RXFOIE           EQU  H'0001'
RXBKIE           EQU  H'0002'
FERIE            EQU  H'0003'
CERIE            EQU  H'0004'
ABDOVE           EQU  H'0005'
PERIE            EQU  H'0006'
TXMTIE           EQU  H'0007'

U2TXCIE          EQU  H'0000'
U2RXFOIE         EQU  H'0001'
U2RXBKIE         EQU  H'0002'
U2FERIE          EQU  H'0003'
U2CERIE          EQU  H'0004'
U2ABDOVE         EQU  H'0005'
U2PERIE          EQU  H'0006'
U2TXMTIE         EQU  H'0007'


;----- U1P1H Bits -----------------------------------------------------
P1H              EQU  H'0000'


;----- U1P2H Bits -----------------------------------------------------
P2H              EQU  H'0000'


;----- U1P3H Bits -----------------------------------------------------
P3H              EQU  H'0000'


;----- U1CON0 Bits -----------------------------------------------------
MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'

U1RXEN           EQU  H'0004'
U1TXEN           EQU  H'0005'
U1ABDEN          EQU  H'0006'
U1BRGS           EQU  H'0007'

U1MODE0          EQU  H'0000'
U1MODE1          EQU  H'0001'
U1MODE2          EQU  H'0002'

RXEN             EQU  H'0004'
TXEN             EQU  H'0005'
ABDEN            EQU  H'0006'
BRGS             EQU  H'0007'


;----- U1CON1 Bits -----------------------------------------------------
SENDB            EQU  H'0000'
BRKOVR           EQU  H'0001'
RXBIMD           EQU  H'0003'
WUE              EQU  H'0004'
ON_U1CON1        EQU  H'0007'

U1SENDB          EQU  H'0000'
U1BRKOVR         EQU  H'0001'
U1RXBIMD         EQU  H'0003'
U1WUE            EQU  H'0004'
U1ON             EQU  H'0007'


;----- U1CON2 Bits -----------------------------------------------------
TXPOL            EQU  H'0002'
C0EN             EQU  H'0003'
RXPOL            EQU  H'0006'
RUNOVF           EQU  H'0007'

FLO0             EQU  H'0000'
FLO1             EQU  H'0001'
STP0             EQU  H'0004'
STP1             EQU  H'0005'

U1TXPOL          EQU  H'0002'
U1C0EN           EQU  H'0003'
U1RXPOL          EQU  H'0006'
U1RUNOVF         EQU  H'0007'

U1FLO0           EQU  H'0000'
U1FLO1           EQU  H'0001'
U1STP0           EQU  H'0004'
U1STP1           EQU  H'0005'


;----- U1FIFO Bits -----------------------------------------------------
RXBF             EQU  H'0000'
RXBE             EQU  H'0001'
XON              EQU  H'0002'
RXIDL            EQU  H'0003'
TXBF             EQU  H'0004'
TXBE             EQU  H'0005'
STPMD            EQU  H'0006'
TXWRE            EQU  H'0007'

U1RXBF           EQU  H'0000'
U1RXBE           EQU  H'0001'
U1XON            EQU  H'0002'
U1RXIDL          EQU  H'0003'
U1TXBF           EQU  H'0004'
U1TXBE           EQU  H'0005'
U1STPMD          EQU  H'0006'
U1TXWRE          EQU  H'0007'

U1RCIDL          EQU  H'0003'

RCIDL            EQU  H'0003'


;----- U1UIR Bits -----------------------------------------------------
ABDIE            EQU  H'0002'
ABDIF            EQU  H'0006'
WUIF             EQU  H'0007'

U1ABDIE          EQU  H'0002'
U1ABDIF          EQU  H'0006'
U1WUIF           EQU  H'0007'


;----- U1ERRIR Bits -----------------------------------------------------
TXCIF            EQU  H'0000'
RXFOIF           EQU  H'0001'
RXBKIF           EQU  H'0002'
FERIF            EQU  H'0003'
CERIF            EQU  H'0004'
ABDOVF           EQU  H'0005'
PERIF            EQU  H'0006'
TXMTIF           EQU  H'0007'

U1TXCIF          EQU  H'0000'
U1RXFOIF         EQU  H'0001'
U1RXBKIF         EQU  H'0002'
U1FERIF          EQU  H'0003'
U1CERIF          EQU  H'0004'
U1ABDOVF         EQU  H'0005'
U1PERIF          EQU  H'0006'
U1TXMTIF         EQU  H'0007'


;----- U1ERRIE Bits -----------------------------------------------------
TXCIE            EQU  H'0000'
RXFOIE           EQU  H'0001'
RXBKIE           EQU  H'0002'
FERIE            EQU  H'0003'
CERIE            EQU  H'0004'
ABDOVE           EQU  H'0005'
PERIE            EQU  H'0006'
TXMTIE           EQU  H'0007'

U1TXCIE          EQU  H'0000'
U1RXFOIE         EQU  H'0001'
U1RXBKIE         EQU  H'0002'
U1FERIE          EQU  H'0003'
U1CERIE          EQU  H'0004'
U1ABDOVE         EQU  H'0005'
U1PERIE          EQU  H'0006'
U1TXMTIE         EQU  H'0007'


;----- DAC1CON1 Bits -----------------------------------------------------
DAC1R0           EQU  H'0000'
DAC1R1           EQU  H'0001'
DAC1R2           EQU  H'0002'
DAC1R3           EQU  H'0003'
DAC1R4           EQU  H'0004'



;----- DAC1CON0 Bits -----------------------------------------------------
NSS              EQU  H'0000'
OE2              EQU  H'0004'
OE1              EQU  H'0005'
EN               EQU  H'0007'

DAC1NSS          EQU  H'0000'
DAC1PSS0         EQU  H'0002'
DAC1PSS1         EQU  H'0003'
DAC1OE2          EQU  H'0004'
DAC1OE1          EQU  H'0005'
DAC1EN           EQU  H'0007'

PSS0             EQU  H'0002'
PSS1             EQU  H'0003'


;----- CM2CON0 Bits -----------------------------------------------------
SYNC             EQU  H'0000'
HYS              EQU  H'0001'
POL_CM2CON0      EQU  H'0004'
OUT_CM2CON0      EQU  H'0006'
EN               EQU  H'0007'

C2SYNC           EQU  H'0000'
C2HYS            EQU  H'0001'
C2POL            EQU  H'0004'
C2OUT_CM2CON0    EQU  H'0006'
C2EN             EQU  H'0007'


;----- CM2CON1 Bits -----------------------------------------------------
INTN_CM2CON1     EQU  H'0000'
INTP_CM2CON1     EQU  H'0001'

C2INTN           EQU  H'0000'
C2INTP           EQU  H'0001'


;----- CM2NCH Bits -----------------------------------------------------
NCH0             EQU  H'0000'
NCH1             EQU  H'0001'
NCH2             EQU  H'0002'

C2NCH0           EQU  H'0000'
C2NCH1           EQU  H'0001'
C2NCH2           EQU  H'0002'


;----- CM2PCH Bits -----------------------------------------------------
PCH0             EQU  H'0000'
PCH1             EQU  H'0001'
PCH2             EQU  H'0002'

C2PCH0           EQU  H'0000'
C2PCH1           EQU  H'0001'
C2PCH2           EQU  H'0002'


;----- CM1CON0 Bits -----------------------------------------------------
SYNC             EQU  H'0000'
HYS              EQU  H'0001'
POL_CM1CON0      EQU  H'0004'
OUT_CM1CON0      EQU  H'0006'
EN               EQU  H'0007'

C1SYNC           EQU  H'0000'
C1HYS            EQU  H'0001'
C1POL            EQU  H'0004'
C1OUT_CM1CON0    EQU  H'0006'
C1EN             EQU  H'0007'


;----- CM1CON1 Bits -----------------------------------------------------
INTN_CM1CON1     EQU  H'0000'
INTP_CM1CON1     EQU  H'0001'

C1INTN           EQU  H'0000'
C1INTP           EQU  H'0001'


;----- CM1NCH Bits -----------------------------------------------------
NCH0             EQU  H'0000'
NCH1             EQU  H'0001'
NCH2             EQU  H'0002'

C1NCH0           EQU  H'0000'
C1NCH1           EQU  H'0001'
C1NCH2           EQU  H'0002'


;----- CM1PCH Bits -----------------------------------------------------
PCH0             EQU  H'0000'
PCH1             EQU  H'0001'
PCH2             EQU  H'0002'

C1PCH0           EQU  H'0000'
C1PCH1           EQU  H'0001'
C1PCH2           EQU  H'0002'


;----- CMOUT Bits -----------------------------------------------------
C1OUT_CMOUT      EQU  H'0000'
C2OUT_CMOUT      EQU  H'0001'


;----- FVRCON Bits -----------------------------------------------------
TSRNG            EQU  H'0004'
TSEN             EQU  H'0005'
RDY_FVRCON       EQU  H'0006'
EN               EQU  H'0007'

ADFVR0           EQU  H'0000'
ADFVR1           EQU  H'0001'
CDAFVR0          EQU  H'0002'
CDAFVR1          EQU  H'0003'
FVRRDY           EQU  H'0006'
FVREN            EQU  H'0007'


;----- ZCDCON Bits -----------------------------------------------------
INTN_ZCDCON      EQU  H'0000'
INTP_ZCDCON      EQU  H'0001'
POL_ZCDCON       EQU  H'0004'
OUT_ZCDCON       EQU  H'0005'
SEN_ZCDCON       EQU  H'0007'

ZCDINTN          EQU  H'0000'
ZCDINTP          EQU  H'0001'
ZCDPOL           EQU  H'0004'
ZCDOUT           EQU  H'0005'
ZCDSEN           EQU  H'0007'


;----- HLVDCON0 Bits -----------------------------------------------------
INTL             EQU  H'0000'
INTH             EQU  H'0001'
RDY_HLVDCON0     EQU  H'0004'
OUT_HLVDCON0     EQU  H'0005'
EN               EQU  H'0007'

HLVDINTL         EQU  H'0000'
HLVDINTH         EQU  H'0001'
HLVDRDY          EQU  H'0004'
HLVDOUT          EQU  H'0005'
HLVDEN           EQU  H'0007'


;----- HLVDCON1 Bits -----------------------------------------------------
SEL0             EQU  H'0000'
SEL1             EQU  H'0001'
SEL2             EQU  H'0002'
SEL3             EQU  H'0003'

HLVDSEL0         EQU  H'0000'
HLVDSEL1         EQU  H'0001'
HLVDSEL2         EQU  H'0002'
HLVDSEL3         EQU  H'0003'


;----- ADCP Bits -----------------------------------------------------
CPRDY            EQU  H'0000'
ON_ADCP          EQU  H'0007'

ADCPRDY          EQU  H'0000'
CPON             EQU  H'0007'

ADCPON           EQU  H'0007'


;----- ADLTHL Bits -----------------------------------------------------
ADLTH0           EQU  H'0000'
ADLTH1           EQU  H'0001'
ADLTH2           EQU  H'0002'
ADLTH3           EQU  H'0003'
ADLTH4           EQU  H'0004'
ADLTH5           EQU  H'0005'
ADLTH6           EQU  H'0006'
ADLTH7           EQU  H'0007'


LTH0             EQU  H'0000'
LTH1             EQU  H'0001'
LTH2             EQU  H'0002'
LTH3             EQU  H'0003'
LTH4             EQU  H'0004'
LTH5             EQU  H'0005'
LTH6             EQU  H'0006'
LTH7             EQU  H'0007'


;----- ADLTHH Bits -----------------------------------------------------
ADLTH8           EQU  H'0000'
ADLTH9           EQU  H'0001'
ADLTH10          EQU  H'0002'
ADLTH11          EQU  H'0003'
ADLTH12          EQU  H'0004'
ADLTH13          EQU  H'0005'
ADLTH14          EQU  H'0006'
ADLTH15          EQU  H'0007'


LTH8             EQU  H'0000'
LTH9             EQU  H'0001'
LTH10            EQU  H'0002'
LTH11            EQU  H'0003'
LTH12            EQU  H'0004'
LTH13            EQU  H'0005'
LTH14            EQU  H'0006'
LTH15            EQU  H'0007'


;----- ADUTHL Bits -----------------------------------------------------
ADUTH0           EQU  H'0000'
ADUTH1           EQU  H'0001'
ADUTH2           EQU  H'0002'
ADUTH3           EQU  H'0003'
ADUTH4           EQU  H'0004'
ADUTH5           EQU  H'0005'
ADUTH6           EQU  H'0006'
ADUTH7           EQU  H'0007'


UTH0             EQU  H'0000'
UTH1             EQU  H'0001'
UTH2             EQU  H'0002'
UTH3             EQU  H'0003'
UTH4             EQU  H'0004'
UTH5             EQU  H'0005'
UTH6             EQU  H'0006'
UTH7             EQU  H'0007'


;----- ADUTHH Bits -----------------------------------------------------
ADUTH8           EQU  H'0000'
ADUTH9           EQU  H'0001'
ADUTH10          EQU  H'0002'
ADUTH11          EQU  H'0003'
ADUTH12          EQU  H'0004'
ADUTH13          EQU  H'0005'
ADUTH14          EQU  H'0006'
ADUTH15          EQU  H'0007'


UTH8             EQU  H'0000'
UTH9             EQU  H'0001'
UTH10            EQU  H'0002'
UTH11            EQU  H'0003'
UTH12            EQU  H'0004'
UTH13            EQU  H'0005'
UTH14            EQU  H'0006'
UTH15            EQU  H'0007'


;----- ADERRL Bits -----------------------------------------------------
ADERR0           EQU  H'0000'
ADERR1           EQU  H'0001'
ADERR2           EQU  H'0002'
ADERR3           EQU  H'0003'
ADERR4           EQU  H'0004'
ADERR5           EQU  H'0005'
ADERR6           EQU  H'0006'
ADERR7           EQU  H'0007'


ERR0             EQU  H'0000'
ERR1             EQU  H'0001'
ERR2             EQU  H'0002'
ERR3             EQU  H'0003'
ERR4             EQU  H'0004'
ERR5             EQU  H'0005'
ERR6             EQU  H'0006'
ERR7             EQU  H'0007'


;----- ADERRH Bits -----------------------------------------------------
ADERR8           EQU  H'0000'
ADERR9           EQU  H'0001'
ADERR10          EQU  H'0002'
ADERR11          EQU  H'0003'
ADERR12          EQU  H'0004'
ADERR13          EQU  H'0005'
ADERR14          EQU  H'0006'
ADERR15          EQU  H'0007'


ERR8             EQU  H'0000'
ERR9             EQU  H'0001'
ERR10            EQU  H'0002'
ERR11            EQU  H'0003'
ERR12            EQU  H'0004'
ERR13            EQU  H'0005'
ERR14            EQU  H'0006'
ERR15            EQU  H'0007'


;----- ADSTPTL Bits -----------------------------------------------------
ADSTPT0          EQU  H'0000'
ADSTPT1          EQU  H'0001'
ADSTPT2          EQU  H'0002'
ADSTPT3          EQU  H'0003'
ADSTPT4          EQU  H'0004'
ADSTPT5          EQU  H'0005'
ADSTPT6          EQU  H'0006'
ADSTPT7          EQU  H'0007'


STPT0            EQU  H'0000'
STPT1            EQU  H'0001'
STPT2            EQU  H'0002'
STPT3            EQU  H'0003'
STPT4            EQU  H'0004'
STPT5            EQU  H'0005'
STPT6            EQU  H'0006'
STPT7            EQU  H'0007'


;----- ADSTPTH Bits -----------------------------------------------------
ADSTPT8          EQU  H'0000'
ADSTPT9          EQU  H'0001'
ADSTPT10         EQU  H'0002'
ADSTPT11         EQU  H'0003'
ADSTPT12         EQU  H'0004'
ADSTPT13         EQU  H'0005'
ADSTPT14         EQU  H'0006'
ADSTPT15         EQU  H'0007'


STPT8            EQU  H'0000'
STPT9            EQU  H'0001'
STPT10           EQU  H'0002'
STPT11           EQU  H'0003'
STPT12           EQU  H'0004'
STPT13           EQU  H'0005'
STPT15           EQU  H'0006'
STPT16           EQU  H'0007'


;----- ADFLTRL Bits -----------------------------------------------------
ADFLTR0          EQU  H'0000'
ADFLTR1          EQU  H'0001'
ADFLTR2          EQU  H'0002'
ADFLTR3          EQU  H'0003'
ADFLTR4          EQU  H'0004'
ADFLTR5          EQU  H'0005'
ADFLTR6          EQU  H'0006'
ADFLTR7          EQU  H'0007'


FLTR0            EQU  H'0000'
FLTR1            EQU  H'0001'
FLTR2            EQU  H'0002'
FLTR3            EQU  H'0003'
FLTR4            EQU  H'0004'
FLTR5            EQU  H'0005'
FLTR6            EQU  H'0006'
FLTR7            EQU  H'0007'


;----- ADFLTRH Bits -----------------------------------------------------
ADFLTR8          EQU  H'0000'
ADFLTR9          EQU  H'0001'
ADFLTR10         EQU  H'0002'
ADFLTR11         EQU  H'0003'
ADFLTR12         EQU  H'0004'
ADFLTR13         EQU  H'0005'
ADFLTR14         EQU  H'0006'
ADFLTR15         EQU  H'0007'


FLTR8            EQU  H'0000'
FLTR9            EQU  H'0001'
FLTR10           EQU  H'0002'
FLTR11           EQU  H'0003'
FLTR12           EQU  H'0004'
FLTR13           EQU  H'0005'
FLTR14           EQU  H'0006'
FLTR15           EQU  H'0007'


;----- ADACCL Bits -----------------------------------------------------
ADACC0           EQU  H'0000'
ADACC1           EQU  H'0001'
ADACC2           EQU  H'0002'
ADACC3           EQU  H'0003'
ADACC4           EQU  H'0004'
ADACC5           EQU  H'0005'
ADACC6           EQU  H'0006'
ADACC7           EQU  H'0007'


ACC0             EQU  H'0000'
ACC1             EQU  H'0001'
ACC2             EQU  H'0002'
ACC3             EQU  H'0003'
ACC4             EQU  H'0004'
ACC5             EQU  H'0005'
ACC6             EQU  H'0006'
ACC7             EQU  H'0007'


;----- ADACCH Bits -----------------------------------------------------
ADACC8           EQU  H'0000'
ADACC9           EQU  H'0001'
ADACC10          EQU  H'0002'
ADACC11          EQU  H'0003'
ADACC12          EQU  H'0004'
ADACC13          EQU  H'0005'
ADACC14          EQU  H'0006'
ADACC15          EQU  H'0007'


ACC8             EQU  H'0000'
ACC9             EQU  H'0001'
ACC10            EQU  H'0002'
ACC11            EQU  H'0003'
ACC12            EQU  H'0004'
ACC13            EQU  H'0005'
ACC14            EQU  H'0006'
ACC15            EQU  H'0007'


;----- ADACCU Bits -----------------------------------------------------
ADACC16          EQU  H'0000'
ADACC17          EQU  H'0001'
ADACC18          EQU  H'0002'
ADACC19          EQU  H'0003'
ADACC20          EQU  H'0004'
ADACC21          EQU  H'0005'
ADACC22          EQU  H'0006'
ADACC23          EQU  H'0007'


ACC16            EQU  H'0000'
ACC17            EQU  H'0001'
ACC18            EQU  H'0002'
ACC19            EQU  H'0003'
ACC20            EQU  H'0004'
ACC21            EQU  H'0005'
ACC22            EQU  H'0006'
ACC23            EQU  H'0007'


;----- ADCNT Bits -----------------------------------------------------
ADCNT0           EQU  H'0000'
ADCNT1           EQU  H'0001'
ADCNT2           EQU  H'0002'
ADCNT3           EQU  H'0003'
ADCNT4           EQU  H'0004'
ADCNT5           EQU  H'0005'
ADCNT6           EQU  H'0006'
ADCNT7           EQU  H'0007'


CNT0             EQU  H'0000'
CNT1             EQU  H'0001'
CNT2             EQU  H'0002'
CNT3             EQU  H'0003'
CNT4             EQU  H'0004'
CNT5             EQU  H'0005'
CNT6             EQU  H'0006'
CNT7             EQU  H'0007'


;----- ADRPT Bits -----------------------------------------------------
ADRPT0           EQU  H'0000'
ADRPT1           EQU  H'0001'
ADRPT2           EQU  H'0002'
ADRPT3           EQU  H'0003'
ADRPT4           EQU  H'0004'
ADRPT5           EQU  H'0005'
ADRPT6           EQU  H'0006'
ADRPT7           EQU  H'0007'


RPT0             EQU  H'0000'
RPT1             EQU  H'0001'
RPT2             EQU  H'0002'
RPT3             EQU  H'0003'
RPT4             EQU  H'0004'
RPT5             EQU  H'0005'
RPT6             EQU  H'0006'
RPT7             EQU  H'0007'


;----- ADPREVL Bits -----------------------------------------------------
ADPREV0          EQU  H'0000'
ADPREV1          EQU  H'0001'
ADPREV2          EQU  H'0002'
ADPREV3          EQU  H'0003'
ADPREV4          EQU  H'0004'
ADPREV5          EQU  H'0005'
ADPREV6          EQU  H'0006'
ADPREV7          EQU  H'0007'


PREV0            EQU  H'0000'
PREV1            EQU  H'0001'
PREV2            EQU  H'0002'
PREV3            EQU  H'0003'
PREV4            EQU  H'0004'
PREV5            EQU  H'0005'
PREV6            EQU  H'0006'
PREV7            EQU  H'0007'


;----- ADPREVH Bits -----------------------------------------------------
ADPREV8          EQU  H'0000'
ADPREV9          EQU  H'0001'
ADPREV10         EQU  H'0002'
ADPREV11         EQU  H'0003'
ADPREV12         EQU  H'0004'
ADPREV13         EQU  H'0005'
ADPREV14         EQU  H'0006'
ADPREV15         EQU  H'0007'


PREV8            EQU  H'0000'
PREV9            EQU  H'0001'
PREV10           EQU  H'0002'
PREV11           EQU  H'0003'
PREV12           EQU  H'0004'
PREV13           EQU  H'0005'
PREV14           EQU  H'0006'
PREV15           EQU  H'0007'


;----- ADRESL Bits -----------------------------------------------------
ADRES0           EQU  H'0000'
ADRES1           EQU  H'0001'
ADRES2           EQU  H'0002'
ADRES3           EQU  H'0003'
ADRES4           EQU  H'0004'
ADRES5           EQU  H'0005'
ADRES6           EQU  H'0006'
ADRES7           EQU  H'0007'


RES0             EQU  H'0000'
RES1             EQU  H'0001'
RES2             EQU  H'0002'
RES3             EQU  H'0003'
RES4             EQU  H'0004'
RES5             EQU  H'0005'
RES6             EQU  H'0006'
RES7             EQU  H'0007'


;----- ADRESH Bits -----------------------------------------------------
ADRES8           EQU  H'0000'
ADRES9           EQU  H'0001'
ADRES10          EQU  H'0002'
ADRES11          EQU  H'0003'
ADRES12          EQU  H'0004'
ADRES13          EQU  H'0005'
ADRES14          EQU  H'0006'
ADRES15          EQU  H'0007'


RES8             EQU  H'0000'
RES9             EQU  H'0001'
RES10            EQU  H'0002'
RES11            EQU  H'0003'
RES12            EQU  H'0004'
RES13            EQU  H'0005'
RES14            EQU  H'0006'
RES15            EQU  H'0007'


;----- ADPCH Bits -----------------------------------------------------
ADPCH0           EQU  H'0000'
ADPCH1           EQU  H'0001'
ADPCH2           EQU  H'0002'
ADPCH3           EQU  H'0003'
ADPCH4           EQU  H'0004'
ADPCH5           EQU  H'0005'


;----- ADACQL Bits -----------------------------------------------------
ADACQ0           EQU  H'0000'
ADACQ1           EQU  H'0001'
ADACQ2           EQU  H'0002'
ADACQ3           EQU  H'0003'
ADACQ4           EQU  H'0004'
ADACQ5           EQU  H'0005'
ADACQ6           EQU  H'0006'
ADACQ7           EQU  H'0007'


ACQ0             EQU  H'0000'
ACQ1             EQU  H'0001'
ACQ2             EQU  H'0002'
ACQ3             EQU  H'0003'
ACQ4             EQU  H'0004'
ACQ5             EQU  H'0005'
ACQ6             EQU  H'0006'
ACQ7             EQU  H'0007'


;----- ADACQH Bits -----------------------------------------------------
ADACQ8           EQU  H'0000'
ADACQ9           EQU  H'0001'
ADACQ10          EQU  H'0002'
ADACQ11          EQU  H'0003'
ADACQ12          EQU  H'0004'


ACQ8             EQU  H'0000'
ACQ9             EQU  H'0001'
ACQ10            EQU  H'0002'
ACQ11            EQU  H'0003'
ACQ12            EQU  H'0004'


;----- ADCAP Bits -----------------------------------------------------
ADCAP0           EQU  H'0000'
ADCAP1           EQU  H'0001'
ADCAP2           EQU  H'0002'
ADCAP3           EQU  H'0003'
ADCAP4           EQU  H'0004'


;----- ADPREL Bits -----------------------------------------------------
PRE0             EQU  H'0000'
PRE1             EQU  H'0001'
PRE2             EQU  H'0002'
PRE3             EQU  H'0003'
PRE4             EQU  H'0004'
PRE5             EQU  H'0005'
PRE6             EQU  H'0006'
PRE7             EQU  H'0007'


ADPRE0           EQU  H'0000'
ADPRE1           EQU  H'0001'
ADPRE2           EQU  H'0002'
ADPRE3           EQU  H'0003'
ADPRE4           EQU  H'0004'
ADPRE5           EQU  H'0005'
ADPRE6           EQU  H'0006'
ADPRE7           EQU  H'0007'


;----- ADPREH Bits -----------------------------------------------------
PRE8             EQU  H'0000'
PRE9             EQU  H'0001'
PRE10            EQU  H'0002'
PRE11            EQU  H'0003'
PRE12            EQU  H'0004'


ADPRE8           EQU  H'0000'
ADPRE9           EQU  H'0001'
ADPRE10          EQU  H'0002'
ADPRE11          EQU  H'0003'
ADPRE12          EQU  H'0004'


;----- ADCON0 Bits -----------------------------------------------------
GO_ADCON0        EQU  H'0000'
CS_ADCON0        EQU  H'0004'
CONT             EQU  H'0006'
ON_ADCON0        EQU  H'0007'

ADGO             EQU  H'0000'
ADCS             EQU  H'0004'
ADCONT           EQU  H'0006'
ADON             EQU  H'0007'

DONE             EQU  H'0000'
FM0              EQU  H'0002'

GO_NOT_DONE      EQU  H'0000'
ADFM0            EQU  H'0002'


;----- ADCON1 Bits -----------------------------------------------------
DSEN             EQU  H'0000'
GPOL_ADCON1      EQU  H'0005'
IPEN_ADCON1      EQU  H'0006'
PPOL             EQU  H'0007'

ADDSEN           EQU  H'0000'
ADGPOL           EQU  H'0005'
ADIPEN           EQU  H'0006'
ADPPOL           EQU  H'0007'


;----- ADCON2 Bits -----------------------------------------------------
ACLR             EQU  H'0003'
PSIS             EQU  H'0007'

ADMD0            EQU  H'0000'
ADMD1            EQU  H'0001'
ADMD2            EQU  H'0002'
ADACLR           EQU  H'0003'
ADCRS0           EQU  H'0004'
ADCRS1           EQU  H'0005'
ADCRS2           EQU  H'0006'
ADPSIS           EQU  H'0007'


MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
CRS0             EQU  H'0004'
CRS1             EQU  H'0005'
CRS2             EQU  H'0006'


MD0              EQU  H'0000'
MD1              EQU  H'0001'
MD2              EQU  H'0002'



;----- ADCON3 Bits -----------------------------------------------------
SOI              EQU  H'0003'

ADTMD0           EQU  H'0000'
ADTMD1           EQU  H'0001'
ADTMD2           EQU  H'0002'
ADSOI            EQU  H'0003'
ADCALC0          EQU  H'0004'
ADCALC1          EQU  H'0005'
ADCALC2          EQU  H'0006'


TMD0             EQU  H'0000'
TMD1             EQU  H'0001'
TMD2             EQU  H'0002'
CALC0            EQU  H'0004'
CALC1            EQU  H'0005'
CALC2            EQU  H'0006'


;----- ADSTAT Bits -----------------------------------------------------
MATH             EQU  H'0004'
LTHR             EQU  H'0005'
UTHR             EQU  H'0006'
OV_ADSTAT        EQU  H'0007'

ADSTAT0          EQU  H'0000'
ADSTAT1          EQU  H'0001'
ADSTAT2          EQU  H'0002'
ADMATH           EQU  H'0004'
ADLTHR           EQU  H'0005'
ADUTHR           EQU  H'0006'
ADAOV            EQU  H'0007'

ADOV             EQU  H'0007'

STAT0_ADSTAT     EQU  H'0000'
STAT1_ADSTAT     EQU  H'0001'
STAT2            EQU  H'0002'


;----- ADREF Bits -----------------------------------------------------
PREF0            EQU  H'0000'
PREF1            EQU  H'0001'
NREF0            EQU  H'0004'

ADPREF0          EQU  H'0000'
ADPREF1          EQU  H'0001'
ADNREF0          EQU  H'0004'


;----- ADACT Bits -----------------------------------------------------
ADACT0           EQU  H'0000'
ADACT1           EQU  H'0001'
ADACT2           EQU  H'0002'
ADACT3           EQU  H'0003'
ADACT4           EQU  H'0004'


ACT0             EQU  H'0000'
ACT1             EQU  H'0001'
ACT2             EQU  H'0002'
ACT3             EQU  H'0003'
ACT4             EQU  H'0004'


;----- ADCLK Bits -----------------------------------------------------
ADCS0            EQU  H'0000'
ADCS1            EQU  H'0001'
ADCS2            EQU  H'0002'
ADCS3            EQU  H'0003'
ADCS4            EQU  H'0004'
ADCS5            EQU  H'0005'


CS0_ADCLK        EQU  H'0000'
CS1_ADCLK        EQU  H'0001'
CS2_ADCLK        EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'
CS5              EQU  H'0005'


;----- SMT1TMRL Bits -----------------------------------------------------
;TMR0             EQU  H'0000'
TMR1             EQU  H'0001'
;TMR2             EQU  H'0002'
TMR3             EQU  H'0003'
;TMR4             EQU  H'0004'
TMR5             EQU  H'0005'
;TMR6             EQU  H'0006'
TMR7             EQU  H'0007'


SMT1TMR0         EQU  H'0000'
SMT1TMR1         EQU  H'0001'
SMT1TMR2         EQU  H'0002'
SMT1TMR3         EQU  H'0003'
SMT1TMR4         EQU  H'0004'
SMT1TMR5         EQU  H'0005'
SMT1TMR6         EQU  H'0006'
SMT1TMR7         EQU  H'0007'


;----- SMT1TMRH Bits -----------------------------------------------------
TMR8             EQU  H'0000'
TMR9             EQU  H'0001'
TMR10            EQU  H'0002'
TMR11            EQU  H'0003'
TMR12            EQU  H'0004'
TMR13            EQU  H'0005'
TMR14            EQU  H'0006'
TMR15            EQU  H'0007'


SMT1TMR8         EQU  H'0000'
SMT1TMR9         EQU  H'0001'
SMT1TMR10        EQU  H'0002'
SMT1TMR11        EQU  H'0003'
SMT1TMR12        EQU  H'0004'
SMT1TMR13        EQU  H'0005'
SMT1TMR14        EQU  H'0006'
SMT1TMR15        EQU  H'0007'


;----- SMT1TMRU Bits -----------------------------------------------------
TMR16            EQU  H'0000'
TMR17            EQU  H'0001'
TMR18            EQU  H'0002'
TMR19            EQU  H'0003'
TMR20            EQU  H'0004'
TMR21            EQU  H'0005'
TMR22            EQU  H'0006'
TMR23            EQU  H'0007'


SMT1TMR16        EQU  H'0000'
SMT1TMR17        EQU  H'0001'
SMT1TMR18        EQU  H'0002'
SMT1TMR19        EQU  H'0003'
SMT1TMR20        EQU  H'0004'
SMT1TMR21        EQU  H'0005'
SMT1TMR22        EQU  H'0006'
SMT1TMR23        EQU  H'0007'


;----- SMT1CPRL Bits -----------------------------------------------------
CPR0             EQU  H'0000'
CPR1             EQU  H'0001'
CPR2             EQU  H'0002'
CPR3             EQU  H'0003'
CPR4             EQU  H'0004'
CPR5             EQU  H'0005'
CPR6             EQU  H'0006'
CPR7             EQU  H'0007'


SMT1CPR0         EQU  H'0000'
SMT1CPR1         EQU  H'0001'
SMT1CPR2         EQU  H'0002'
SMT1CPR3         EQU  H'0003'
SMT1CPR4         EQU  H'0004'
SMT1CPR5         EQU  H'0005'
SMT1CPR6         EQU  H'0006'
SMT1CPR7         EQU  H'0007'


;----- SMT1CPRH Bits -----------------------------------------------------
CPR8             EQU  H'0000'
CPR9             EQU  H'0001'
CPR10            EQU  H'0002'
CPR11            EQU  H'0003'
CPR12            EQU  H'0004'
CPR13            EQU  H'0005'
CPR14            EQU  H'0006'
CPR15            EQU  H'0007'

SMT1CPR8         EQU  H'0000'
SMT1CPR9         EQU  H'0001'
SMT1CPR10        EQU  H'0002'
SMT1CPR11        EQU  H'0003'
SMT1CPR12        EQU  H'0004'
SMT1CPR13        EQU  H'0005'
SMT1CPR14        EQU  H'0006'
SMT1CPR15        EQU  H'0007'



;----- SMT1CPRU Bits -----------------------------------------------------
CPR16            EQU  H'0000'
CPR17            EQU  H'0001'
CPR18            EQU  H'0002'
CPR19            EQU  H'0003'
CPR20            EQU  H'0004'
CPR21            EQU  H'0005'
CPR22            EQU  H'0006'
CPR23            EQU  H'0007'

SMT1CPR16        EQU  H'0000'
SMT1CPR17        EQU  H'0001'
SMT1CPR18        EQU  H'0002'
SMT1CPR19        EQU  H'0003'
SMT1CPR20        EQU  H'0004'
SMT1CPR21        EQU  H'0005'
SMT1CPR22        EQU  H'0006'
SMT1CPR23        EQU  H'0007'



;----- SMT1CPWL Bits -----------------------------------------------------
CPW0             EQU  H'0000'
CPW1             EQU  H'0001'
CPW2             EQU  H'0002'
CPW3             EQU  H'0003'
CPW4             EQU  H'0004'
CPW5             EQU  H'0005'
CPW6             EQU  H'0006'
CPW7             EQU  H'0007'


SMT1CPW0         EQU  H'0000'
SMT1CPW1         EQU  H'0001'
SMT1CPW2         EQU  H'0002'
SMT1CPW3         EQU  H'0003'
SMT1CPW4         EQU  H'0004'
SMT1CPW5         EQU  H'0005'
SMT1CPW6         EQU  H'0006'
SMT1CPW7         EQU  H'0007'


;----- SMT1CPWH Bits -----------------------------------------------------
CPW8             EQU  H'0000'
CPW9             EQU  H'0001'
CPW10            EQU  H'0002'
CPW11            EQU  H'0003'
CPW12            EQU  H'0004'
CPW13            EQU  H'0005'
CPW14            EQU  H'0006'
CPW15            EQU  H'0007'


SMT1CPW8         EQU  H'0000'
SMT1CPW9         EQU  H'0001'
SMT1CPW10        EQU  H'0002'
SMT1CPW11        EQU  H'0003'
SMT1CPW12        EQU  H'0004'
SMT1CPW13        EQU  H'0005'
SMT1CPW14        EQU  H'0006'
SMT1CPW15        EQU  H'0007'


;----- SMT1CPWU Bits -----------------------------------------------------
CPW16            EQU  H'0000'
CPW17            EQU  H'0001'
CPW18            EQU  H'0002'
CPW19            EQU  H'0003'
CPW20            EQU  H'0004'
CPW21            EQU  H'0005'
CPW22            EQU  H'0006'
CPW23            EQU  H'0007'


SMT1CPW16        EQU  H'0000'
SMT1CPW17        EQU  H'0001'
SMT1CPW18        EQU  H'0002'
SMT1CPW19        EQU  H'0003'
SMT1CPW20        EQU  H'0004'
SMT1CPW21        EQU  H'0005'
SMT1CPW22        EQU  H'0006'
SMT1CPW23        EQU  H'0007'


;----- SMT1PRL Bits -----------------------------------------------------
;PR0              EQU  H'0000'
PR1              EQU  H'0001'
;PR2              EQU  H'0002'
PR3              EQU  H'0003'
;PR4              EQU  H'0004'
PR5              EQU  H'0005'
;PR6              EQU  H'0006'
PR7              EQU  H'0007'


SMT1PR0          EQU  H'0000'
SMT1PR1          EQU  H'0001'
SMT1PR2          EQU  H'0002'
SMT1PR3          EQU  H'0003'
SMT1PR4          EQU  H'0004'
SMT1PR5          EQU  H'0005'
SMT1PR6          EQU  H'0006'
SMT1PR7          EQU  H'0007'


;----- SMT1PRH Bits -----------------------------------------------------
PR8              EQU  H'0000'
PR9              EQU  H'0001'
PR10             EQU  H'0002'
PR11             EQU  H'0003'
PR12             EQU  H'0004'
PR13             EQU  H'0005'
PR14             EQU  H'0006'
PR15             EQU  H'0007'


SMT1PR8          EQU  H'0000'
SMT1PR9          EQU  H'0001'
SMT1PR10         EQU  H'0002'
SMT1PR11         EQU  H'0003'
SMT1PR12         EQU  H'0004'
SMT1PR13         EQU  H'0005'
SMT1PR14         EQU  H'0006'
SMT1PR15         EQU  H'0007'


;----- SMT1PRU Bits -----------------------------------------------------
PR16             EQU  H'0000'
PR17             EQU  H'0001'
PR18             EQU  H'0002'
PR19             EQU  H'0003'
PR20             EQU  H'0004'
PR21             EQU  H'0005'
PR22             EQU  H'0006'
PR23             EQU  H'0007'


SMT1PR16         EQU  H'0000'
SMT1PR17         EQU  H'0001'
SMT1PR18         EQU  H'0002'
SMT1PR19         EQU  H'0003'
SMT1PR20         EQU  H'0004'
SMT1PR21         EQU  H'0005'
SMT1PR22         EQU  H'0006'
SMT1PR23         EQU  H'0007'


;----- SMT1CON0 Bits -----------------------------------------------------
CPOL             EQU  H'0002'
SPOL             EQU  H'0003'
WPOL             EQU  H'0004'
STP              EQU  H'0005'
EN               EQU  H'0007'

SMT1CPOL         EQU  H'0002'
SMT1SPOL         EQU  H'0003'
SMT1WOL          EQU  H'0004'
SMT1STP          EQU  H'0005'
SMT1EN           EQU  H'0007'

PS0              EQU  H'0000'
PS1              EQU  H'0001'

SMT1PS0          EQU  H'0000'
SMT1PS1          EQU  H'0001'


;----- SMT1CON1 Bits -----------------------------------------------------
REPEAT           EQU  H'0006'
GO_SMT1CON1      EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'
SMT1REPEAT       EQU  H'0006'
SMT1GO           EQU  H'0007'



;----- SMT1STAT Bits -----------------------------------------------------
AS               EQU  H'0000'
WS               EQU  H'0001'
TS               EQU  H'0002'
RST              EQU  H'0005'
CPWUP            EQU  H'0006'
CPRUP            EQU  H'0007'

SMT1AS           EQU  H'0000'
SMT1WS           EQU  H'0001'
SMT1TS           EQU  H'0002'
SMT1RESET        EQU  H'0005'
SMT1CPWUP        EQU  H'0006'
SMT1CPRUP        EQU  H'0007'

SMT1RST          EQU  H'0005'


;----- SMT1CLK Bits -----------------------------------------------------
CSEL0            EQU  H'0000'
CSEL1            EQU  H'0001'
CSEL2            EQU  H'0002'


SMT1CSEL0        EQU  H'0000'
SMT1CSEL1        EQU  H'0001'
SMT1CSEL2        EQU  H'0002'


;----- SMT1SIG Bits -----------------------------------------------------
SSEL0            EQU  H'0000'
SSEL1            EQU  H'0001'
SSEL2            EQU  H'0002'
SSEL3            EQU  H'0003'
SSEL4            EQU  H'0004'


SMT1SSEL0        EQU  H'0000'
SMT1SSEL1        EQU  H'0001'
SMT1SSEL2        EQU  H'0002'
SMT1SSEL3        EQU  H'0003'
SMT1SSEL4        EQU  H'0004'


;----- SMT1WIN Bits -----------------------------------------------------
WSEL0            EQU  H'0000'
WSEL1            EQU  H'0001'
WSEL2            EQU  H'0002'
WSEL3            EQU  H'0003'
WSEL4            EQU  H'0004'


SMT1WSEL0        EQU  H'0000'
SMT1WSEL1        EQU  H'0001'
SMT1WSEL2        EQU  H'0002'
SMT1WSEL3        EQU  H'0003'
SMT1WSEL4        EQU  H'0004'


;----- NCO1ACCL Bits -----------------------------------------------------
NCO1ACC0         EQU  H'0000'
NCO1ACC1         EQU  H'0001'
NCO1ACC2         EQU  H'0002'
NCO1ACC3         EQU  H'0003'
NCO1ACC4         EQU  H'0004'
NCO1ACC5         EQU  H'0005'
NCO1ACC6         EQU  H'0006'
NCO1ACC7         EQU  H'0007'

ACC0             EQU  H'0000'
ACC1             EQU  H'0001'
ACC2             EQU  H'0002'
ACC3             EQU  H'0003'
ACC4             EQU  H'0004'
ACC5             EQU  H'0005'
ACC6             EQU  H'0006'
ACC7             EQU  H'0007'



;----- NCO1ACCH Bits -----------------------------------------------------
NCO1ACC8         EQU  H'0000'
NCO1ACC9         EQU  H'0001'
NCO1ACC10        EQU  H'0002'
NCO1ACC11        EQU  H'0003'
NCO1ACC12        EQU  H'0004'
NCO1ACC13        EQU  H'0005'
NCO1ACC14        EQU  H'0006'
NCO1ACC15        EQU  H'0007'

ACC8             EQU  H'0000'
ACC9             EQU  H'0001'
ACC10            EQU  H'0002'
ACC11            EQU  H'0003'
ACC12            EQU  H'0004'
ACC13            EQU  H'0005'
ACC14            EQU  H'0006'
ACC15            EQU  H'0007'



;----- NCO1ACCU Bits -----------------------------------------------------
NCO1ACC16        EQU  H'0000'
NCO1ACC17        EQU  H'0001'
NCO1ACC18        EQU  H'0002'
NCO1ACC19        EQU  H'0003'
NCO1ACC20        EQU  H'0004'
NCO1ACC21        EQU  H'0005'
NCO1ACC22        EQU  H'0006'
NCO1ACC23        EQU  H'0007'

ACC16            EQU  H'0000'
ACC17            EQU  H'0001'
ACC18            EQU  H'0002'
ACC19            EQU  H'0003'
ACC20            EQU  H'0004'
ACC21            EQU  H'0005'
ACC22            EQU  H'0006'
ACC23            EQU  H'0007'



;----- NCO1INCL Bits -----------------------------------------------------
NCO1INC0         EQU  H'0000'
NCO1INC1         EQU  H'0001'
NCO1INC2         EQU  H'0002'
NCO1INC3         EQU  H'0003'
NCO1INC4         EQU  H'0004'
NCO1INC5         EQU  H'0005'
NCO1INC6         EQU  H'0006'
NCO1INC7         EQU  H'0007'

INC0             EQU  H'0000'
INC1             EQU  H'0001'
INC2             EQU  H'0002'
INC3             EQU  H'0003'
INC4             EQU  H'0004'
INC5             EQU  H'0005'
INC6             EQU  H'0006'
INC7             EQU  H'0007'



;----- NCO1INCH Bits -----------------------------------------------------
NCO1INC8         EQU  H'0000'
NCO1INC9         EQU  H'0001'
NCO1INC10        EQU  H'0002'
NCO1INC11        EQU  H'0003'
NCO1INC12        EQU  H'0004'
NCO1INC13        EQU  H'0005'
NCO1INC14        EQU  H'0006'
NCO1INC15        EQU  H'0007'

INC8             EQU  H'0000'
INC9             EQU  H'0001'
INC10            EQU  H'0002'
INC11            EQU  H'0003'
INC12            EQU  H'0004'
INC13            EQU  H'0005'
INC14            EQU  H'0006'
INC15            EQU  H'0007'



;----- NCO1INCU Bits -----------------------------------------------------
NCO1INC16        EQU  H'0000'
NCO1INC17        EQU  H'0001'
NCO1INC18        EQU  H'0002'
NCO1INC19        EQU  H'0003'
NCO1INC20        EQU  H'0004'
NCO1INC21        EQU  H'0005'
NCO1INC22        EQU  H'0006'
NCO1INC23        EQU  H'0007'

INC16            EQU  H'0000'
INC17            EQU  H'0001'
INC18            EQU  H'0002'
INC19            EQU  H'0003'
INC20            EQU  H'0004'
INC21            EQU  H'0005'
INC22            EQU  H'0006'
INC23            EQU  H'0007'



;----- NCO1CON Bits -----------------------------------------------------
PFM              EQU  H'0000'
POL_NCO1CON      EQU  H'0004'
OUT_NCO1CON      EQU  H'0005'
EN               EQU  H'0007'

NCO1PFM          EQU  H'0000'
NCO1POL          EQU  H'0004'
NCO1OUT          EQU  H'0005'
NCO1EN           EQU  H'0007'


;----- NCO1CLK Bits -----------------------------------------------------
CKS0             EQU  H'0000'
CKS1             EQU  H'0001'
CKS2             EQU  H'0002'
CKS3             EQU  H'0003'
PWS0             EQU  H'0005'
PWS1             EQU  H'0006'
PWS2             EQU  H'0007'


NCO1CKS0         EQU  H'0000'
NCO1CKS1         EQU  H'0001'
NCO1CKS2         EQU  H'0002'
NCO1CKS3         EQU  H'0003'
NCO1PWS0         EQU  H'0005'
NCO1PWS1         EQU  H'0006'
NCO1PWS2         EQU  H'0007'


;----- CWG3CLK Bits -----------------------------------------------------
CS_CWG3CLK       EQU  H'0000'

CWG3CS           EQU  H'0000'


;----- CWG3CLKCON Bits -----------------------------------------------------
CS_CWG3CLKCON    EQU  H'0000'

CWG3CS           EQU  H'0000'


;----- CWG3ISM Bits -----------------------------------------------------
CWG3ISM0         EQU  H'0000'
CWG3ISM1         EQU  H'0001'
CWG3ISM2         EQU  H'0002'
CWG3ISM3         EQU  H'0003'


;----- CWG3DBR Bits -----------------------------------------------------
DBR0             EQU  H'0000'
DBR1             EQU  H'0001'
DBR2             EQU  H'0002'
DBR3             EQU  H'0003'
DBR4             EQU  H'0004'
DBR5             EQU  H'0005'


CWG3DBR0         EQU  H'0000'
CWG3DBR1         EQU  H'0001'
CWG3DBR2         EQU  H'0002'
CWG3DBR3         EQU  H'0003'
CWG3DBR4         EQU  H'0004'
CWG3DBR5         EQU  H'0005'


;----- CWG3DBF Bits -----------------------------------------------------
DBF0             EQU  H'0000'
DBF1             EQU  H'0001'
DBF2             EQU  H'0002'
DBF3             EQU  H'0003'
DBF4             EQU  H'0004'
DBF5             EQU  H'0005'


CWG3DBF0         EQU  H'0000'
CWG3DBF1         EQU  H'0001'
CWG3DBF2         EQU  H'0002'
CWG3DBF3         EQU  H'0003'
CWG3DBF4         EQU  H'0004'
CWG3DBF5         EQU  H'0005'


;----- CWG3CON0 Bits -----------------------------------------------------
LD               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
G3EN             EQU  H'0007'

CWG3LD           EQU  H'0006'
CWG3EN           EQU  H'0007'

CWG3MODE0        EQU  H'0000'
CWG3MODE1        EQU  H'0001'
CWG3MODE2        EQU  H'0002'


;----- CWG3CON1 Bits -----------------------------------------------------
POLA             EQU  H'0000'
POLB             EQU  H'0001'
POLC             EQU  H'0002'
POLD             EQU  H'0003'
IN               EQU  H'0005'

CWG3POLA         EQU  H'0000'
CWG3POLB         EQU  H'0001'
CWG3POLC         EQU  H'0002'
CWG3POLD         EQU  H'0003'
CWG3IN           EQU  H'0005'


;----- CWG3AS0 Bits -----------------------------------------------------
REN              EQU  H'0006'
SHUTDOWN         EQU  H'0007'

LSAC0            EQU  H'0002'
LSAC1            EQU  H'0003'
LSBD0            EQU  H'0004'
LSBD1            EQU  H'0005'

CWG3REN          EQU  H'0006'
CWG3SHUTDOWN     EQU  H'0007'

CWG3LSAC0        EQU  H'0002'
CWG3LSAC1        EQU  H'0003'
CWG3LSBD0        EQU  H'0004'
CWG3LSBD1        EQU  H'0005'


;----- CWG3AS1 Bits -----------------------------------------------------
AS0E             EQU  H'0000'
AS1E             EQU  H'0001'
AS2E             EQU  H'0002'
AS3E             EQU  H'0003'
AS4E             EQU  H'0004'
AS5E             EQU  H'0005'
AS6E             EQU  H'0006'


;----- CWG3STR Bits -----------------------------------------------------
STRA             EQU  H'0000'
STRB             EQU  H'0001'
STRC             EQU  H'0002'
STRD             EQU  H'0003'
OVRA             EQU  H'0004'
OVRB             EQU  H'0005'
OVRC             EQU  H'0006'
OVRD             EQU  H'0007'

CWG3STRA         EQU  H'0000'
CWG3STRB         EQU  H'0001'
CWG3STRC         EQU  H'0002'
CWG3STRD         EQU  H'0003'
CWG3OVRA         EQU  H'0004'
CWG3OVRB         EQU  H'0005'
CWG3OVRC         EQU  H'0006'
CWG3OVRD         EQU  H'0007'


;----- CWG2CLK Bits -----------------------------------------------------
CS_CWG2CLK       EQU  H'0000'

CWG2CS           EQU  H'0000'


;----- CWG2CLKCON Bits -----------------------------------------------------
CS_CWG2CLKCON    EQU  H'0000'

CWG2CS           EQU  H'0000'


;----- CWG2ISM Bits -----------------------------------------------------
CWG2ISM0         EQU  H'0000'
CWG2ISM1         EQU  H'0001'
CWG2ISM2         EQU  H'0002'
CWG2ISM3         EQU  H'0003'


;----- CWG2DBR Bits -----------------------------------------------------
DBR0             EQU  H'0000'
DBR1             EQU  H'0001'
DBR2             EQU  H'0002'
DBR3             EQU  H'0003'
DBR4             EQU  H'0004'
DBR5             EQU  H'0005'


CWG2DBR0         EQU  H'0000'
CWG2DBR1         EQU  H'0001'
CWG2DBR2         EQU  H'0002'
CWG2DBR3         EQU  H'0003'
CWG2DBR4         EQU  H'0004'
CWG2DBR5         EQU  H'0005'


;----- CWG2DBF Bits -----------------------------------------------------
DBF0             EQU  H'0000'
DBF1             EQU  H'0001'
DBF2             EQU  H'0002'
DBF3             EQU  H'0003'
DBF4             EQU  H'0004'
DBF5             EQU  H'0005'


CWG2DBF0         EQU  H'0000'
CWG2DBF1         EQU  H'0001'
CWG2DBF2         EQU  H'0002'
CWG2DBF3         EQU  H'0003'
CWG2DBF4         EQU  H'0004'
CWG2DBF5         EQU  H'0005'


;----- CWG2CON0 Bits -----------------------------------------------------
LD               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
G2EN             EQU  H'0007'

CWG2LD           EQU  H'0006'
CWG2EN           EQU  H'0007'

CWG2MODE0        EQU  H'0000'
CWG2MODE1        EQU  H'0001'
CWG2MODE2        EQU  H'0002'


;----- CWG2CON1 Bits -----------------------------------------------------
POLA             EQU  H'0000'
POLB             EQU  H'0001'
POLC             EQU  H'0002'
POLD             EQU  H'0003'
IN               EQU  H'0005'

CWG2POLA         EQU  H'0000'
CWG2POLB         EQU  H'0001'
CWG2POLC         EQU  H'0002'
CWG2POLD         EQU  H'0003'
CWG2IN           EQU  H'0005'


;----- CWG2AS0 Bits -----------------------------------------------------
REN              EQU  H'0006'
SHUTDOWN         EQU  H'0007'

LSAC0            EQU  H'0002'
LSAC1            EQU  H'0003'
LSBD0            EQU  H'0004'
LSBD1            EQU  H'0005'

CWG2REN          EQU  H'0006'
CWG2SHUTDOWN     EQU  H'0007'

CWG2LSAC0        EQU  H'0002'
CWG2LSAC1        EQU  H'0003'
CWG2LSBD0        EQU  H'0004'
CWG2LSBD1        EQU  H'0005'


;----- CWG2AS1 Bits -----------------------------------------------------
AS0E             EQU  H'0000'
AS1E             EQU  H'0001'
AS2E             EQU  H'0002'
AS3E             EQU  H'0003'
AS4E             EQU  H'0004'
AS5E             EQU  H'0005'
AS6E             EQU  H'0006'


;----- CWG2STR Bits -----------------------------------------------------
STRA             EQU  H'0000'
STRB             EQU  H'0001'
STRC             EQU  H'0002'
STRD             EQU  H'0003'
OVRA             EQU  H'0004'
OVRB             EQU  H'0005'
OVRC             EQU  H'0006'
OVRD             EQU  H'0007'

CWG2STRA         EQU  H'0000'
CWG2STRB         EQU  H'0001'
CWG2STRC         EQU  H'0002'
CWG2STRD         EQU  H'0003'
CWG2OVRA         EQU  H'0004'
CWG2OVRB         EQU  H'0005'
CWG2OVRC         EQU  H'0006'
CWG2OVRD         EQU  H'0007'


;----- CWG1CLK Bits -----------------------------------------------------
CS_CWG1CLK       EQU  H'0000'

CWG1CS           EQU  H'0000'


;----- CWG1CLKCON Bits -----------------------------------------------------
CS_CWG1CLKCON    EQU  H'0000'

CWG1CS           EQU  H'0000'


;----- CWG1ISM Bits -----------------------------------------------------
CWG1ISM0         EQU  H'0000'
CWG1ISM1         EQU  H'0001'
CWG1ISM2         EQU  H'0002'
CWG1ISM3         EQU  H'0003'


;----- CWG1DBR Bits -----------------------------------------------------
DBR0             EQU  H'0000'
DBR1             EQU  H'0001'
DBR2             EQU  H'0002'
DBR3             EQU  H'0003'
DBR4             EQU  H'0004'
DBR5             EQU  H'0005'


CWG1DBR0         EQU  H'0000'
CWG1DBR1         EQU  H'0001'
CWG1DBR2         EQU  H'0002'
CWG1DBR3         EQU  H'0003'
CWG1DBR4         EQU  H'0004'
CWG1DBR5         EQU  H'0005'


;----- CWG1DBF Bits -----------------------------------------------------
DBF0             EQU  H'0000'
DBF1             EQU  H'0001'
DBF2             EQU  H'0002'
DBF3             EQU  H'0003'
DBF4             EQU  H'0004'
DBF5             EQU  H'0005'


CWG1DBF0         EQU  H'0000'
CWG1DBF1         EQU  H'0001'
CWG1DBF2         EQU  H'0002'
CWG1DBF3         EQU  H'0003'
CWG1DBF4         EQU  H'0004'
CWG1DBF5         EQU  H'0005'


;----- CWG1CON0 Bits -----------------------------------------------------
LD               EQU  H'0006'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
G1EN             EQU  H'0007'

CWG1LD           EQU  H'0006'
CWG1EN           EQU  H'0007'

CWG1MODE0        EQU  H'0000'
CWG1MODE1        EQU  H'0001'
CWG1MODE2        EQU  H'0002'


;----- CWG1CON1 Bits -----------------------------------------------------
POLA             EQU  H'0000'
POLB             EQU  H'0001'
POLC             EQU  H'0002'
POLD             EQU  H'0003'
IN               EQU  H'0005'

CWG1POLA         EQU  H'0000'
CWG1POLB         EQU  H'0001'
CWG1POLC         EQU  H'0002'
CWG1POLD         EQU  H'0003'
CWG1IN           EQU  H'0005'


;----- CWG1AS0 Bits -----------------------------------------------------
REN              EQU  H'0006'
SHUTDOWN         EQU  H'0007'

LSAC0            EQU  H'0002'
LSAC1            EQU  H'0003'
LSBD0            EQU  H'0004'
LSBD1            EQU  H'0005'

CWG1REN          EQU  H'0006'
CWG1SHUTDOWN     EQU  H'0007'

CWG1LSAC0        EQU  H'0002'
CWG1LSAC1        EQU  H'0003'
CWG1LSBD0        EQU  H'0004'
CWG1LSBD1        EQU  H'0005'


;----- CWG1AS1 Bits -----------------------------------------------------
AS0E             EQU  H'0000'
AS1E             EQU  H'0001'
AS2E             EQU  H'0002'
AS3E             EQU  H'0003'
AS4E             EQU  H'0004'
AS5E             EQU  H'0005'
AS6E             EQU  H'0006'


;----- CWG1STR Bits -----------------------------------------------------
STRA             EQU  H'0000'
STRB             EQU  H'0001'
STRC             EQU  H'0002'
STRD             EQU  H'0003'
OVRA             EQU  H'0004'
OVRB             EQU  H'0005'
OVRC             EQU  H'0006'
OVRD             EQU  H'0007'

CWG1STRA         EQU  H'0000'
CWG1STRB         EQU  H'0001'
CWG1STRC         EQU  H'0002'
CWG1STRD         EQU  H'0003'
CWG1OVRA         EQU  H'0004'
CWG1OVRB         EQU  H'0005'
CWG1OVRC         EQU  H'0006'
CWG1OVRD         EQU  H'0007'


;----- CCPTMRS0 Bits -----------------------------------------------------
C1TSEL0          EQU  H'0000'
C1TSEL1          EQU  H'0001'
C2TSEL0          EQU  H'0002'
C2TSEL1          EQU  H'0003'
C3TSEL0          EQU  H'0004'
C3TSEL1          EQU  H'0005'
C4TSEL0          EQU  H'0006'
C4TSEL1          EQU  H'0007'


;----- CCPTMRS1 Bits -----------------------------------------------------
P5TSEL0          EQU  H'0000'
P5TSEL1          EQU  H'0001'
P6TSEL0          EQU  H'0002'
P6TSEL1          EQU  H'0003'
P7TSEL0          EQU  H'0004'
P7TSEL1          EQU  H'0005'
P8TSEL0          EQU  H'0006'
P8TSEL1          EQU  H'0007'


;----- PWM8DCL Bits -----------------------------------------------------
DC0              EQU  H'0006'
DC1              EQU  H'0007'

PWM8DC0          EQU  H'0006'
PWM8DC1          EQU  H'0007'

PWMPW0           EQU  H'0006'
PWMPW1           EQU  H'0007'


;----- PWM8DCH Bits -----------------------------------------------------
DC2              EQU  H'0000'
DC3              EQU  H'0001'
DC4              EQU  H'0002'
DC5              EQU  H'0003'
DC6              EQU  H'0004'
DC7              EQU  H'0005'
DC8              EQU  H'0006'
DC9              EQU  H'0007'

PWM8DC2          EQU  H'0000'
PWM8DC3          EQU  H'0001'
PWM8DC4          EQU  H'0002'
PWM8DC5          EQU  H'0003'
PWM8DC6          EQU  H'0004'
PWM8DC7          EQU  H'0005'
PWM8DC8          EQU  H'0006'
PWM8DC9          EQU  H'0007'

PWMPW2           EQU  H'0000'
PWMPW3           EQU  H'0001'
PWMPW4           EQU  H'0002'
PWMPW5           EQU  H'0003'
PWMPW6           EQU  H'0004'
PWMPW7           EQU  H'0005'
PWMPW8           EQU  H'0006'
PWMPW9           EQU  H'0007'


;----- PWM8CON Bits -----------------------------------------------------
POL_PWM8CON      EQU  H'0004'
OUT_PWM8CON      EQU  H'0005'
EN               EQU  H'0007'

PWM8POL          EQU  H'0004'
PWM8OUT          EQU  H'0005'
PWM8EN           EQU  H'0007'


;----- PWM7DCL Bits -----------------------------------------------------
DC0              EQU  H'0006'
DC1              EQU  H'0007'

PWM7DC0          EQU  H'0006'
PWM7DC1          EQU  H'0007'

PWMPW0           EQU  H'0006'
PWMPW1           EQU  H'0007'


;----- PWM7DCH Bits -----------------------------------------------------
DC2              EQU  H'0000'
DC3              EQU  H'0001'
DC4              EQU  H'0002'
DC5              EQU  H'0003'
DC6              EQU  H'0004'
DC7              EQU  H'0005'
DC8              EQU  H'0006'
DC9              EQU  H'0007'

PWM7DC2          EQU  H'0000'
PWM7DC3          EQU  H'0001'
PWM7DC4          EQU  H'0002'
PWM7DC5          EQU  H'0003'
PWM7DC6          EQU  H'0004'
PWM7DC7          EQU  H'0005'
PWM7DC8          EQU  H'0006'
PWM7DC9          EQU  H'0007'

PWMPW2           EQU  H'0000'
PWMPW3           EQU  H'0001'
PWMPW4           EQU  H'0002'
PWMPW5           EQU  H'0003'
PWMPW6           EQU  H'0004'
PWMPW7           EQU  H'0005'
PWMPW8           EQU  H'0006'
PWMPW9           EQU  H'0007'


;----- PWM7CON Bits -----------------------------------------------------
POL_PWM7CON      EQU  H'0004'
OUT_PWM7CON      EQU  H'0005'
EN               EQU  H'0007'

PWM7POL          EQU  H'0004'
PWM7OUT          EQU  H'0005'
PWM7EN           EQU  H'0007'


;----- PWM6DCL Bits -----------------------------------------------------
DC0              EQU  H'0006'
DC1              EQU  H'0007'

PWM6DC0          EQU  H'0006'
PWM6DC1          EQU  H'0007'

PWMPW0           EQU  H'0006'
PWMPW1           EQU  H'0007'


;----- PWM6DCH Bits -----------------------------------------------------
DC2              EQU  H'0000'
DC3              EQU  H'0001'
DC4              EQU  H'0002'
DC5              EQU  H'0003'
DC6              EQU  H'0004'
DC7              EQU  H'0005'
DC8              EQU  H'0006'
DC9              EQU  H'0007'

PWM6DC2          EQU  H'0000'
PWM6DC3          EQU  H'0001'
PWM6DC4          EQU  H'0002'
PWM6DC5          EQU  H'0003'
PWM6DC6          EQU  H'0004'
PWM6DC7          EQU  H'0005'
PWM6DC8          EQU  H'0006'
PWM6DC9          EQU  H'0007'

PWMPW2           EQU  H'0000'
PWMPW3           EQU  H'0001'
PWMPW4           EQU  H'0002'
PWMPW5           EQU  H'0003'
PWMPW6           EQU  H'0004'
PWMPW7           EQU  H'0005'
PWMPW8           EQU  H'0006'
PWMPW9           EQU  H'0007'


;----- PWM6CON Bits -----------------------------------------------------
POL_PWM6CON      EQU  H'0004'
OUT_PWM6CON      EQU  H'0005'
EN               EQU  H'0007'

PWM6POL          EQU  H'0004'
PWM6OUT          EQU  H'0005'
PWM6EN           EQU  H'0007'


;----- PWM5DCL Bits -----------------------------------------------------
DC0              EQU  H'0006'
DC1              EQU  H'0007'

PWM5DC0          EQU  H'0006'
PWM5DC1          EQU  H'0007'

PWMPW0           EQU  H'0006'
PWMPW1           EQU  H'0007'


;----- PWM5DCH Bits -----------------------------------------------------
DC2              EQU  H'0000'
DC3              EQU  H'0001'
DC4              EQU  H'0002'
DC5              EQU  H'0003'
DC6              EQU  H'0004'
DC7              EQU  H'0005'
DC8              EQU  H'0006'
DC9              EQU  H'0007'

PWM5DC2          EQU  H'0000'
PWM5DC3          EQU  H'0001'
PWM5DC4          EQU  H'0002'
PWM5DC5          EQU  H'0003'
PWM5DC6          EQU  H'0004'
PWM5DC7          EQU  H'0005'
PWM5DC8          EQU  H'0006'
PWM5DC9          EQU  H'0007'

PWMPW2           EQU  H'0000'
PWMPW3           EQU  H'0001'
PWMPW4           EQU  H'0002'
PWMPW5           EQU  H'0003'
PWMPW6           EQU  H'0004'
PWMPW7           EQU  H'0005'
PWMPW8           EQU  H'0006'
PWMPW9           EQU  H'0007'


;----- PWM5CON Bits -----------------------------------------------------
POL_PWM5CON      EQU  H'0004'
OUT_PWM5CON      EQU  H'0005'
EN               EQU  H'0007'

PWM5POL          EQU  H'0004'
PWM5OUT          EQU  H'0005'
PWM5EN           EQU  H'0007'


;----- CCP4CON Bits -----------------------------------------------------
FMT              EQU  H'0004'
OUT_CCP4CON      EQU  H'0005'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'

CCP4FMT          EQU  H'0004'
CCP4OUT          EQU  H'0005'
CCP4EN           EQU  H'0007'

CCP4MODE0        EQU  H'0000'
CCP4MODE1        EQU  H'0001'
CCP4MODE2        EQU  H'0002'
CCP4MODE3        EQU  H'0003'


;----- CCP4CAP Bits -----------------------------------------------------
CTS0             EQU  H'0000'
CTS1             EQU  H'0001'
CTS2             EQU  H'0002'


CCP4CTS0         EQU  H'0000'
CCP4CTS1         EQU  H'0001'
CCP4CTS2         EQU  H'0002'


;----- CCP3CON Bits -----------------------------------------------------
FMT              EQU  H'0004'
OUT_CCP3CON      EQU  H'0005'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'

CCP3FMT          EQU  H'0004'
CCP3OUT          EQU  H'0005'
CCP3EN           EQU  H'0007'

CCP3MODE0        EQU  H'0000'
CCP3MODE1        EQU  H'0001'
CCP3MODE2        EQU  H'0002'
CCP3MODE3        EQU  H'0003'


;----- CCP3CAP Bits -----------------------------------------------------
CTS0             EQU  H'0000'
CTS1             EQU  H'0001'
CTS2             EQU  H'0002'


CCP3CTS0         EQU  H'0000'
CCP3CTS1         EQU  H'0001'
CCP3CTS2         EQU  H'0002'


;----- CCP2CON Bits -----------------------------------------------------
FMT              EQU  H'0004'
OUT_CCP2CON      EQU  H'0005'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'

CCP2FMT          EQU  H'0004'
CCP2OUT          EQU  H'0005'
CCP2EN           EQU  H'0007'

CCP2MODE0        EQU  H'0000'
CCP2MODE1        EQU  H'0001'
CCP2MODE2        EQU  H'0002'
CCP2MODE3        EQU  H'0003'


;----- CCP2CAP Bits -----------------------------------------------------
CTS0             EQU  H'0000'
CTS1             EQU  H'0001'
CTS2             EQU  H'0002'


CCP2CTS0         EQU  H'0000'
CCP2CTS1         EQU  H'0001'
CCP2CTS2         EQU  H'0002'


;----- CCP1CON Bits -----------------------------------------------------
FMT              EQU  H'0004'
OUT_CCP1CON      EQU  H'0005'
EN               EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'

CCP1FMT          EQU  H'0004'
CCP1OUT          EQU  H'0005'
CCP1EN           EQU  H'0007'

CCP1MODE0        EQU  H'0000'
CCP1MODE1        EQU  H'0001'
CCP1MODE2        EQU  H'0002'
CCP1MODE3        EQU  H'0003'


;----- CCP1CAP Bits -----------------------------------------------------
CTS0             EQU  H'0000'
CTS1             EQU  H'0001'
CTS2             EQU  H'0002'


CCP1CTS0         EQU  H'0000'
CCP1CTS1         EQU  H'0001'
CCP1CTS2         EQU  H'0002'


;----- T6CON Bits -----------------------------------------------------
ON_T6CON         EQU  H'0007'

T6ON             EQU  H'0007'

T6OUTPS0         EQU  H'0000'
T6OUTPS1         EQU  H'0001'
T6OUTPS2         EQU  H'0002'
T6OUTPS3         EQU  H'0003'
T6CKPS0          EQU  H'0004'
T6CKPS1          EQU  H'0005'
T6CKPS2          EQU  H'0006'

OUTPS0           EQU  H'0000'
OUTPS1           EQU  H'0001'
OUTPS2           EQU  H'0002'
OUTPS3           EQU  H'0003'
CKPS0_T6CON      EQU  H'0004'
CKPS1_T6CON      EQU  H'0005'
CKPS2_T6CON      EQU  H'0006'
TMR6ON           EQU  H'0007'


;----- T6HLT Bits -----------------------------------------------------
CKSYNC           EQU  H'0005'
CKPOL            EQU  H'0006'
PSYNC            EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'
MODE4            EQU  H'0004'

T6CKSYNC         EQU  H'0005'
T6CKPOL          EQU  H'0006'
T6PSYNC          EQU  H'0007'

T6MODE0          EQU  H'0000'
T6MODE1          EQU  H'0001'
T6MODE2          EQU  H'0002'
T6MODE3          EQU  H'0003'
T6MODE4          EQU  H'0004'


;----- T6CLK Bits -----------------------------------------------------
CS0_T6CLK        EQU  H'0000'
CS1_T6CLK        EQU  H'0001'
CS2_T6CLK        EQU  H'0002'
CS3              EQU  H'0003'


T6CS0            EQU  H'0000'
T6CS1            EQU  H'0001'
T6CS2            EQU  H'0002'
T6CS3            EQU  H'0003'


;----- T6CLKCON Bits -----------------------------------------------------
CS0_T6CLKCON     EQU  H'0000'
CS1_T6CLKCON     EQU  H'0001'
CS2_T6CLKCON     EQU  H'0002'
CS3              EQU  H'0003'


T6CS0            EQU  H'0000'
T6CS1            EQU  H'0001'
T6CS2            EQU  H'0002'
T6CS3            EQU  H'0003'


;----- T6RST Bits -----------------------------------------------------
RSEL0            EQU  H'0000'
RSEL1            EQU  H'0001'
RSEL2            EQU  H'0002'
RSEL3            EQU  H'0003'
RSEL4            EQU  H'0004'


T6RSEL0          EQU  H'0000'
T6RSEL1          EQU  H'0001'
T6RSEL2          EQU  H'0002'
T6RSEL3          EQU  H'0003'
T6RSEL4          EQU  H'0004'


;----- TMR5L Bits -----------------------------------------------------
TMR5L0           EQU  H'0000'
TMR5L1           EQU  H'0001'
TMR5L2           EQU  H'0002'
TMR5L3           EQU  H'0003'
TMR5L4           EQU  H'0004'
TMR5L5           EQU  H'0005'
TMR5L6           EQU  H'0006'
TMR5L7           EQU  H'0007'


;----- TMR5H Bits -----------------------------------------------------
TMR5H0           EQU  H'0000'
TMR5H1           EQU  H'0001'
TMR5H2           EQU  H'0002'
TMR5H3           EQU  H'0003'
TMR5H4           EQU  H'0004'
TMR5H5           EQU  H'0005'
TMR5H6           EQU  H'0006'
TMR5H7           EQU  H'0007'


;----- T5CON Bits -----------------------------------------------------
ON_T5CON         EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR5ON           EQU  H'0000'
T5RD16           EQU  H'0001'
NOT_T5SYNC       EQU  H'0002'
T5CKPS0          EQU  H'0004'
T5CKPS1          EQU  H'0005'

CKPS0_T5CON      EQU  H'0004'
CKPS1_T5CON      EQU  H'0005'


;----- TMR5CON Bits -----------------------------------------------------
ON_TMR5CON       EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR5ON           EQU  H'0000'
T5RD16           EQU  H'0001'
NOT_T5SYNC       EQU  H'0002'
T5CKPS0          EQU  H'0004'
T5CKPS1          EQU  H'0005'

CKPS0_TMR5CON    EQU  H'0004'
CKPS1_TMR5CON    EQU  H'0005'


;----- T5GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_T5GCON      EQU  H'0006'
GE               EQU  H'0007'

T5GVAL           EQU  H'0002'
T5GGO            EQU  H'0003'
T5GSPM           EQU  H'0004'
T5GTM            EQU  H'0005'
T5GPOL           EQU  H'0006'
T5GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T5DONE       EQU  H'0003'


;----- TMR5GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_TMR5GCON    EQU  H'0006'
GE               EQU  H'0007'

T5GVAL           EQU  H'0002'
T5GGO            EQU  H'0003'
T5GSPM           EQU  H'0004'
T5GTM            EQU  H'0005'
T5GPOL           EQU  H'0006'
T5GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T5DONE       EQU  H'0003'


;----- T5GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T5GSS0           EQU  H'0000'
T5GSS1           EQU  H'0001'
T5GSS2           EQU  H'0002'
T5GSS3           EQU  H'0003'
T5GSS4           EQU  H'0004'


;----- TMR5GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T5GSS0           EQU  H'0000'
T5GSS1           EQU  H'0001'
T5GSS2           EQU  H'0002'
T5GSS3           EQU  H'0003'
T5GSS4           EQU  H'0004'


;----- T5CLK Bits -----------------------------------------------------
T5CS0            EQU  H'0000'
T5CS1            EQU  H'0001'
T5CS2            EQU  H'0002'
T5CS3            EQU  H'0003'
T5CS4            EQU  H'0004'

CS0_T5CLK        EQU  H'0000'
CS1_T5CLK        EQU  H'0001'
CS2_T5CLK        EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- TMR5CLK Bits -----------------------------------------------------
T5CS0            EQU  H'0000'
T5CS1            EQU  H'0001'
T5CS2            EQU  H'0002'
T5CS3            EQU  H'0003'
T5CS4            EQU  H'0004'

CS0_TMR5CLK      EQU  H'0000'
CS1_TMR5CLK      EQU  H'0001'
CS2_TMR5CLK      EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- T4CON Bits -----------------------------------------------------
ON_T4CON         EQU  H'0007'

T4ON             EQU  H'0007'

T4OUTPS0         EQU  H'0000'
T4OUTPS1         EQU  H'0001'
T4OUTPS2         EQU  H'0002'
T4OUTPS3         EQU  H'0003'
T4CKPS0          EQU  H'0004'
T4CKPS1          EQU  H'0005'
T4CKPS2          EQU  H'0006'

OUTPS0           EQU  H'0000'
OUTPS1           EQU  H'0001'
OUTPS2           EQU  H'0002'
OUTPS3           EQU  H'0003'
CKPS0_T4CON      EQU  H'0004'
CKPS1_T4CON      EQU  H'0005'
CKPS2_T4CON      EQU  H'0006'
TMR4ON           EQU  H'0007'


;----- T4HLT Bits -----------------------------------------------------
CKSYNC           EQU  H'0005'
CKPOL            EQU  H'0006'
PSYNC            EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'
MODE4            EQU  H'0004'

T4CKSYNC         EQU  H'0005'
T4CKPOL          EQU  H'0006'
T4PSYNC          EQU  H'0007'

T4MODE0          EQU  H'0000'
T4MODE1          EQU  H'0001'
T4MODE2          EQU  H'0002'
T4MODE3          EQU  H'0003'
T4MODE4          EQU  H'0004'


;----- T4CLK Bits -----------------------------------------------------
CS0_T4CLK        EQU  H'0000'
CS1_T4CLK        EQU  H'0001'
CS2_T4CLK        EQU  H'0002'
CS3              EQU  H'0003'


T4CS0            EQU  H'0000'
T4CS1            EQU  H'0001'
T4CS2            EQU  H'0002'
T4CS3            EQU  H'0003'


;----- T4CLKCON Bits -----------------------------------------------------
CS0_T4CLKCON     EQU  H'0000'
CS1_T4CLKCON     EQU  H'0001'
CS2_T4CLKCON     EQU  H'0002'
CS3              EQU  H'0003'


T4CS0            EQU  H'0000'
T4CS1            EQU  H'0001'
T4CS2            EQU  H'0002'
T4CS3            EQU  H'0003'


;----- T4RST Bits -----------------------------------------------------
RSEL0            EQU  H'0000'
RSEL1            EQU  H'0001'
RSEL2            EQU  H'0002'
RSEL3            EQU  H'0003'
RSEL4            EQU  H'0004'


T4RSEL0          EQU  H'0000'
T4RSEL1          EQU  H'0001'
T4RSEL2          EQU  H'0002'
T4RSEL3          EQU  H'0003'
T4RSEL4          EQU  H'0004'


;----- TMR3L Bits -----------------------------------------------------
TMR3L0           EQU  H'0000'
TMR3L1           EQU  H'0001'
TMR3L2           EQU  H'0002'
TMR3L3           EQU  H'0003'
TMR3L4           EQU  H'0004'
TMR3L5           EQU  H'0005'
TMR3L6           EQU  H'0006'
TMR3L7           EQU  H'0007'


;----- TMR3H Bits -----------------------------------------------------
TMR3H0           EQU  H'0000'
TMR3H1           EQU  H'0001'
TMR3H2           EQU  H'0002'
TMR3H3           EQU  H'0003'
TMR3H4           EQU  H'0004'
TMR3H5           EQU  H'0005'
TMR3H6           EQU  H'0006'
TMR3H7           EQU  H'0007'


;----- T3CON Bits -----------------------------------------------------
ON_T3CON         EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR3ON           EQU  H'0000'
T3RD16           EQU  H'0001'
NOT_T3SYNC       EQU  H'0002'
T3CKPS0          EQU  H'0004'
T3CKPS1          EQU  H'0005'

CKPS0_T3CON      EQU  H'0004'
CKPS1_T3CON      EQU  H'0005'


;----- TMR3CON Bits -----------------------------------------------------
ON_TMR3CON       EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR3ON           EQU  H'0000'
T3RD16           EQU  H'0001'
NOT_T3SYNC       EQU  H'0002'
T3CKPS0          EQU  H'0004'
T3CKPS1          EQU  H'0005'

CKPS0_TMR3CON    EQU  H'0004'
CKPS1_TMR3CON    EQU  H'0005'


;----- T3GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_T3GCON      EQU  H'0006'
GE               EQU  H'0007'

T3GVAL           EQU  H'0002'
T3GGO            EQU  H'0003'
T3GSPM           EQU  H'0004'
T3GTM            EQU  H'0005'
T3GPOL           EQU  H'0006'
T3GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T3DONE       EQU  H'0003'


;----- TMR3GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_TMR3GCON    EQU  H'0006'
GE               EQU  H'0007'

T3GVAL           EQU  H'0002'
T3GGO            EQU  H'0003'
T3GSPM           EQU  H'0004'
T3GTM            EQU  H'0005'
T3GPOL           EQU  H'0006'
T3GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T3DONE       EQU  H'0003'


;----- T3GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T3GSS0           EQU  H'0000'
T3GSS1           EQU  H'0001'
T3GSS2           EQU  H'0002'
T3GSS3           EQU  H'0003'
T3GSS4           EQU  H'0004'


;----- TMR3GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T3GSS0           EQU  H'0000'
T3GSS1           EQU  H'0001'
T3GSS2           EQU  H'0002'
T3GSS3           EQU  H'0003'
T3GSS4           EQU  H'0004'


;----- T3CLK Bits -----------------------------------------------------
T3CS0            EQU  H'0000'
T3CS1            EQU  H'0001'
T3CS2            EQU  H'0002'
T3CS3            EQU  H'0003'
T3CS4            EQU  H'0004'

CS0_T3CLK        EQU  H'0000'
CS1_T3CLK        EQU  H'0001'
CS2_T3CLK        EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- TMR3CLK Bits -----------------------------------------------------
T3CS0            EQU  H'0000'
T3CS1            EQU  H'0001'
T3CS2            EQU  H'0002'
T3CS3            EQU  H'0003'
T3CS4            EQU  H'0004'

CS0_TMR3CLK      EQU  H'0000'
CS1_TMR3CLK      EQU  H'0001'
CS2_TMR3CLK      EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- T2CON Bits -----------------------------------------------------
ON_T2CON         EQU  H'0007'

T2ON             EQU  H'0007'

T2OUTPS0         EQU  H'0000'
T2OUTPS1         EQU  H'0001'
T2OUTPS2         EQU  H'0002'
T2OUTPS3         EQU  H'0003'
T2CKPS0          EQU  H'0004'
T2CKPS1          EQU  H'0005'
T2CKPS2          EQU  H'0006'

OUTPS0           EQU  H'0000'
OUTPS1           EQU  H'0001'
OUTPS2           EQU  H'0002'
OUTPS3           EQU  H'0003'
CKPS0_T2CON      EQU  H'0004'
CKPS1_T2CON      EQU  H'0005'
CKPS2_T2CON      EQU  H'0006'
TMR2ON           EQU  H'0007'


;----- T2HLT Bits -----------------------------------------------------
CKSYNC           EQU  H'0005'
CKPOL            EQU  H'0006'
PSYNC            EQU  H'0007'

MODE0            EQU  H'0000'
MODE1            EQU  H'0001'
MODE2            EQU  H'0002'
MODE3            EQU  H'0003'
MODE4            EQU  H'0004'

T2CKSYNC         EQU  H'0005'
T2CKPOL          EQU  H'0006'
T2PSYNC          EQU  H'0007'

T2MODE0          EQU  H'0000'
T2MODE1          EQU  H'0001'
T2MODE2          EQU  H'0002'
T2MODE3          EQU  H'0003'
T2MODE4          EQU  H'0004'


;----- T2CLK Bits -----------------------------------------------------
CS0_T2CLK        EQU  H'0000'
CS1_T2CLK        EQU  H'0001'
CS2_T2CLK        EQU  H'0002'
CS3              EQU  H'0003'


T2CS0            EQU  H'0000'
T2CS1            EQU  H'0001'
T2CS2            EQU  H'0002'
T2CS3            EQU  H'0003'


;----- T2CLKCON Bits -----------------------------------------------------
CS0_T2CLKCON     EQU  H'0000'
CS1_T2CLKCON     EQU  H'0001'
CS2_T2CLKCON     EQU  H'0002'
CS3              EQU  H'0003'


T2CS0            EQU  H'0000'
T2CS1            EQU  H'0001'
T2CS2            EQU  H'0002'
T2CS3            EQU  H'0003'


;----- T2RST Bits -----------------------------------------------------
RSEL0            EQU  H'0000'
RSEL1            EQU  H'0001'
RSEL2            EQU  H'0002'
RSEL3            EQU  H'0003'
RSEL4            EQU  H'0004'


T2RSEL0          EQU  H'0000'
T2RSEL1          EQU  H'0001'
T2RSEL2          EQU  H'0002'
T2RSEL3          EQU  H'0003'
T2RSEL4          EQU  H'0004'


;----- TMR1L Bits -----------------------------------------------------
TMR1L0           EQU  H'0000'
TMR1L1           EQU  H'0001'
TMR1L2           EQU  H'0002'
TMR1L3           EQU  H'0003'
TMR1L4           EQU  H'0004'
TMR1L5           EQU  H'0005'
TMR1L6           EQU  H'0006'
TMR1L7           EQU  H'0007'


;----- TMR1H Bits -----------------------------------------------------
TMR1H0           EQU  H'0000'
TMR1H1           EQU  H'0001'
TMR1H2           EQU  H'0002'
TMR1H3           EQU  H'0003'
TMR1H4           EQU  H'0004'
TMR1H5           EQU  H'0005'
TMR1H6           EQU  H'0006'
TMR1H7           EQU  H'0007'


;----- T1CON Bits -----------------------------------------------------
ON_T1CON         EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR1ON           EQU  H'0000'
T1RD16           EQU  H'0001'
NOT_T1SYNC       EQU  H'0002'
T1CKPS0          EQU  H'0004'
T1CKPS1          EQU  H'0005'

CKPS0_T1CON      EQU  H'0004'
CKPS1_T1CON      EQU  H'0005'


;----- TMR1CON Bits -----------------------------------------------------
ON_TMR1CON       EQU  H'0000'
RD16             EQU  H'0001'
NOT_SYNC         EQU  H'0002'

TMR1ON           EQU  H'0000'
T1RD16           EQU  H'0001'
NOT_T1SYNC       EQU  H'0002'
T1CKPS0          EQU  H'0004'
T1CKPS1          EQU  H'0005'

CKPS0_TMR1CON    EQU  H'0004'
CKPS1_TMR1CON    EQU  H'0005'


;----- T1GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_T1GCON      EQU  H'0006'
GE               EQU  H'0007'

T1GVAL           EQU  H'0002'
T1GGO            EQU  H'0003'
T1GSPM           EQU  H'0004'
T1GTM            EQU  H'0005'
T1GPOL           EQU  H'0006'
T1GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T1DONE       EQU  H'0003'


;----- TMR1GCON Bits -----------------------------------------------------
GVAL             EQU  H'0002'
GGO              EQU  H'0003'
GSPM             EQU  H'0004'
GTM              EQU  H'0005'
GPOL_TMR1GCON    EQU  H'0006'
GE               EQU  H'0007'

T1GVAL           EQU  H'0002'
T1GGO            EQU  H'0003'
T1GSPM           EQU  H'0004'
T1GTM            EQU  H'0005'
T1GPOL           EQU  H'0006'
T1GE             EQU  H'0007'

NOT_DONE         EQU  H'0003'

NOT_T1DONE       EQU  H'0003'


;----- T1GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T1GSS0           EQU  H'0000'
T1GSS1           EQU  H'0001'
T1GSS2           EQU  H'0002'
T1GSS3           EQU  H'0003'
T1GSS4           EQU  H'0004'


;----- TMR1GATE Bits -----------------------------------------------------
GSS0             EQU  H'0000'
GSS1             EQU  H'0001'
GSS2             EQU  H'0002'
GSS3             EQU  H'0003'
GSS4             EQU  H'0004'

T1GSS0           EQU  H'0000'
T1GSS1           EQU  H'0001'
T1GSS2           EQU  H'0002'
T1GSS3           EQU  H'0003'
T1GSS4           EQU  H'0004'


;----- T1CLK Bits -----------------------------------------------------
T1CS0            EQU  H'0000'
T1CS1            EQU  H'0001'
T1CS2            EQU  H'0002'
T1CS3            EQU  H'0003'
T1CS4            EQU  H'0004'

CS0_T1CLK        EQU  H'0000'
CS1_T1CLK        EQU  H'0001'
CS2_T1CLK        EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- TMR1CLK Bits -----------------------------------------------------
T1CS0            EQU  H'0000'
T1CS1            EQU  H'0001'
T1CS2            EQU  H'0002'
T1CS3            EQU  H'0003'
T1CS4            EQU  H'0004'

CS0_TMR1CLK      EQU  H'0000'
CS1_TMR1CLK      EQU  H'0001'
CS2_TMR1CLK      EQU  H'0002'
CS3              EQU  H'0003'
CS4              EQU  H'0004'


;----- TMR0 Bits -----------------------------------------------------
TMR0L0           EQU  H'0000'
TMR0L1           EQU  H'0001'
TMR0L2           EQU  H'0002'
TMR0L3           EQU  H'0003'
TMR0L4           EQU  H'0004'
TMR0L5           EQU  H'0005'
TMR0L6           EQU  H'0006'
TMR0L7           EQU  H'0007'


;----- TMR0L Bits -----------------------------------------------------
TMR0L0           EQU  H'0000'
TMR0L1           EQU  H'0001'
TMR0L2           EQU  H'0002'
TMR0L3           EQU  H'0003'
TMR0L4           EQU  H'0004'
TMR0L5           EQU  H'0005'
TMR0L6           EQU  H'0006'
TMR0L7           EQU  H'0007'


;----- PR0 Bits -----------------------------------------------------
TMR0H0           EQU  H'0000'
TMR0H1           EQU  H'0001'
TMR0H2           EQU  H'0002'
TMR0H3           EQU  H'0003'
TMR0H4           EQU  H'0004'
TMR0H5           EQU  H'0005'
TMR0H6           EQU  H'0006'
TMR0H7           EQU  H'0007'

T0PR0            EQU  H'0000'
T0PR1            EQU  H'0001'
T0PR2            EQU  H'0002'
T0PR3            EQU  H'0003'
T0PR4            EQU  H'0004'
T0PR5            EQU  H'0005'
T0PR6            EQU  H'0006'
T0PR7            EQU  H'0007'



;----- TMR0H Bits -----------------------------------------------------
TMR0H0           EQU  H'0000'
TMR0H1           EQU  H'0001'
TMR0H2           EQU  H'0002'
TMR0H3           EQU  H'0003'
TMR0H4           EQU  H'0004'
TMR0H5           EQU  H'0005'
TMR0H6           EQU  H'0006'
TMR0H7           EQU  H'0007'

T0PR0            EQU  H'0000'
T0PR1            EQU  H'0001'
T0PR2            EQU  H'0002'
T0PR3            EQU  H'0003'
T0PR4            EQU  H'0004'
T0PR5            EQU  H'0005'
T0PR6            EQU  H'0006'
T0PR7            EQU  H'0007'



;----- T0CON0 Bits -----------------------------------------------------
MD16             EQU  H'0004'
OUT_T0CON0       EQU  H'0005'
EN               EQU  H'0007'

T0MD16           EQU  H'0004'
T0OUT            EQU  H'0005'
T0EN             EQU  H'0007'

OUTPS0           EQU  H'0000'
OUTPS1           EQU  H'0001'
OUTPS2           EQU  H'0002'
OUTPS3           EQU  H'0003'
T016BIT          EQU  H'0004'


;----- T0CON1 Bits -----------------------------------------------------
ASYNC            EQU  H'0004'

CKPS0_T0CON1     EQU  H'0000'
CKPS1_T0CON1     EQU  H'0001'
CKPS2_T0CON1     EQU  H'0002'
CKPS3            EQU  H'0003'
T0ASYNC          EQU  H'0004'
CS0_T0CON1       EQU  H'0005'
CS1_T0CON1       EQU  H'0006'
CS2_T0CON1       EQU  H'0007'


T0CKPS0          EQU  H'0000'
T0CKPS1          EQU  H'0001'
T0CKPS2          EQU  H'0002'
T0CKPS3          EQU  H'0003'
T0CS0            EQU  H'0005'
T0CS1            EQU  H'0006'
T0CS2            EQU  H'0007'


;----- LATA Bits -----------------------------------------------------
LATA0            EQU  H'0000'
LATA1            EQU  H'0001'
LATA2            EQU  H'0002'
LATA3            EQU  H'0003'
LATA4            EQU  H'0004'
LATA5            EQU  H'0005'
LATA6            EQU  H'0006'
LATA7            EQU  H'0007'


;----- LATB Bits -----------------------------------------------------
LATB0            EQU  H'0000'
LATB1            EQU  H'0001'
LATB2            EQU  H'0002'
LATB3            EQU  H'0003'
LATB4            EQU  H'0004'
LATB5            EQU  H'0005'
LATB6            EQU  H'0006'
LATB7            EQU  H'0007'


;----- LATC Bits -----------------------------------------------------
LATC0            EQU  H'0000'
LATC1            EQU  H'0001'
LATC2            EQU  H'0002'
LATC3            EQU  H'0003'
LATC4            EQU  H'0004'
LATC5            EQU  H'0005'
LATC6            EQU  H'0006'
LATC7            EQU  H'0007'


;----- LATD Bits -----------------------------------------------------
LATD0            EQU  H'0000'
LATD1            EQU  H'0001'
LATD2            EQU  H'0002'
LATD3            EQU  H'0003'
LATD4            EQU  H'0004'
LATD5            EQU  H'0005'
LATD6            EQU  H'0006'
LATD7            EQU  H'0007'


;----- LATE Bits -----------------------------------------------------
LATE0            EQU  H'0000'
LATE1            EQU  H'0001'
LATE2            EQU  H'0002'


;----- TRISA Bits -----------------------------------------------------
TRISA0           EQU  H'0000'
TRISA1           EQU  H'0001'
TRISA2           EQU  H'0002'
TRISA3           EQU  H'0003'
TRISA4           EQU  H'0004'
TRISA5           EQU  H'0005'
TRISA6           EQU  H'0006'
TRISA7           EQU  H'0007'


;----- TRISB Bits -----------------------------------------------------
TRISB0           EQU  H'0000'
TRISB1           EQU  H'0001'
TRISB2           EQU  H'0002'
TRISB3           EQU  H'0003'
TRISB4           EQU  H'0004'
TRISB5           EQU  H'0005'
TRISB6           EQU  H'0006'
TRISB7           EQU  H'0007'


;----- TRISC Bits -----------------------------------------------------
TRISC0           EQU  H'0000'
TRISC1           EQU  H'0001'
TRISC2           EQU  H'0002'
TRISC3           EQU  H'0003'
TRISC4           EQU  H'0004'
TRISC5           EQU  H'0005'
TRISC6           EQU  H'0006'
TRISC7           EQU  H'0007'


;----- TRISD Bits -----------------------------------------------------
TRISD0           EQU  H'0000'
TRISD1           EQU  H'0001'
TRISD2           EQU  H'0002'
TRISD3           EQU  H'0003'
TRISD4           EQU  H'0004'
TRISD5           EQU  H'0005'
TRISD6           EQU  H'0006'
TRISD7           EQU  H'0007'


;----- TRISE Bits -----------------------------------------------------
TRISE0           EQU  H'0000'
TRISE1           EQU  H'0001'
TRISE2           EQU  H'0002'


;----- PORTA Bits -----------------------------------------------------
RA0              EQU  H'0000'
RA1              EQU  H'0001'
RA2              EQU  H'0002'
RA3              EQU  H'0003'
RA4              EQU  H'0004'
RA5              EQU  H'0005'
RA6              EQU  H'0006'
RA7              EQU  H'0007'


;----- PORTB Bits -----------------------------------------------------
RB0              EQU  H'0000'
RB1              EQU  H'0001'
RB2              EQU  H'0002'
RB3              EQU  H'0003'
RB4              EQU  H'0004'
RB5              EQU  H'0005'
RB6              EQU  H'0006'
RB7              EQU  H'0007'


;----- PORTC Bits -----------------------------------------------------
RC0              EQU  H'0000'
RC1              EQU  H'0001'
RC2              EQU  H'0002'
RC3              EQU  H'0003'
RC4              EQU  H'0004'
RC5              EQU  H'0005'
RC6              EQU  H'0006'
RC7              EQU  H'0007'


;----- PORTD Bits -----------------------------------------------------
RD0              EQU  H'0000'
RD1              EQU  H'0001'
RD2              EQU  H'0002'
RD3              EQU  H'0003'
RD4              EQU  H'0004'
RD5              EQU  H'0005'
RD6              EQU  H'0006'
RD7              EQU  H'0007'


;----- PORTE Bits -----------------------------------------------------
RE0              EQU  H'0000'
RE1              EQU  H'0001'
RE2              EQU  H'0002'
RE3              EQU  H'0003'


;----- INTCON0 Bits -----------------------------------------------------
INT0EDG          EQU  H'0000'
INT1EDG          EQU  H'0001'
INT2EDG          EQU  H'0002'
IPEN_INTCON0     EQU  H'0005'
GIEL             EQU  H'0006'
GIE              EQU  H'0007'

GIEH             EQU  H'0007'


;----- INTCON1 Bits -----------------------------------------------------
STAT0_INTCON1    EQU  H'0006'
STAT1_INTCON1    EQU  H'0007'


;----- IVTLOCK Bits -----------------------------------------------------
IVTLOCKED        EQU  H'0000'


;----- IVTBASEL Bits -----------------------------------------------------
BASE0            EQU  H'0000'
BASE1            EQU  H'0001'
BASE2            EQU  H'0002'
BASE3            EQU  H'0003'
BASE4            EQU  H'0004'
BASE5            EQU  H'0005'
BASE6            EQU  H'0006'
BASE7            EQU  H'0007'


;----- IVTBASEH Bits -----------------------------------------------------
BASE8            EQU  H'0000'
BASE9            EQU  H'0001'
BASE10           EQU  H'0002'
BASE11           EQU  H'0003'
BASE12           EQU  H'0004'
BASE13           EQU  H'0005'
BASE14           EQU  H'0006'
BASE15           EQU  H'0007'


;----- IVTBASEU Bits -----------------------------------------------------
BASE16           EQU  H'0000'
BASE17           EQU  H'0001'
BASE18           EQU  H'0002'
BASE19           EQU  H'0003'
BASE20           EQU  H'0004'


;----- STATUS Bits -----------------------------------------------------
C                EQU  H'0000'
DC               EQU  H'0001'
Z                EQU  H'0002'
OV_STATUS        EQU  H'0003'
N                EQU  H'0004'
NOT_PD           EQU  H'0005'
NOT_TO           EQU  H'0006'

PD               EQU  H'0005'
TO               EQU  H'0006'


;----- PCON0 Bits -----------------------------------------------------
NOT_BOR          EQU  H'0000'
NOT_POR          EQU  H'0001'
NOT_RI           EQU  H'0002'
NOT_RMCLR        EQU  H'0003'
NOT_RWDT         EQU  H'0004'
NOT_WDTWV        EQU  H'0005'
STKUNF           EQU  H'0006'
STKOVF           EQU  H'0007'

BOR              EQU  H'0000'
POR              EQU  H'0001'
RI               EQU  H'0002'
RMCLR            EQU  H'0003'
RWDT             EQU  H'0004'
WDTWV            EQU  H'0005'


;----- PCON1 Bits -----------------------------------------------------
NOT_MEMV         EQU  H'0001'

MEMV             EQU  H'0001'


;----- STKPTR Bits -----------------------------------------------------
STKPTR0          EQU  H'0000'
STKPTR1          EQU  H'0001'
STKPTR2          EQU  H'0002'
STKPTR3          EQU  H'0003'
STKPTR4          EQU  H'0004'

SP0              EQU  H'0000'
SP1              EQU  H'0001'
SP2              EQU  H'0002'
SP3              EQU  H'0003'
SP4              EQU  H'0004'



;==========================================================================
;
;       RAM Definitions
;
;==========================================================================
       __MAXRAM  H'3FFF'
       __BADRAM  H'2000'-H'37FF'
       __BADRAM  H'3800'-H'387F'
       __BADRAM  H'3891'-H'389C'
       __BADRAM  H'38A0'-H'395A'
       __BADRAM  H'396A'-H'3975'
       __BADRAM  H'397E'-H'397F'
       __BADRAM  H'398B'-H'398F'
       __BADRAM  H'399B'-H'399F'
       __BADRAM  H'39AB'-H'39BF'
       __BADRAM  H'39C8'-H'39CF'
       __BADRAM  H'39D1'-H'39D7'
       __BADRAM  H'39E2'
       __BADRAM  H'39E4'
       __BADRAM  H'39E7'-H'39EE'
       __BADRAM  H'39F0'
       __BADRAM  H'39F5'-H'39F6'
       __BADRAM  H'39F8'-H'39FF'
       __BADRAM  H'3A23'-H'3A3F'
       __BADRAM  H'3A48'-H'3A4F'
       __BADRAM  H'3A58'-H'3A59'
       __BADRAM  H'3A5C'-H'3A5F'
       __BADRAM  H'3A68'-H'3A69'
       __BADRAM  H'3A6C'-H'3A6F'
       __BADRAM  H'3A75'-H'3A79'
       __BADRAM  H'3A7C'-H'3A7F'
       __BADRAM  H'3A88'-H'3ABE'
       __BADRAM  H'3AE7'
       __BADRAM  H'3AEA'-H'3BC8'
       __BADRAM  H'3BE0'-H'3BE8'
       __BADRAM  H'3C00'-H'3C55'
       __BADRAM  H'3C7F'-H'3CE4'
       __BADRAM  H'3CE7'-H'3CF9'
       __BADRAM  H'3CFF'-H'3D0F'
       __BADRAM  H'3D1D'-H'3D53'
       __BADRAM  H'3D67'-H'3D69'
       __BADRAM  H'3D7D'-H'3DCF'
       __BADRAM  H'3DD1'
       __BADRAM  H'3DD3'
       __BADRAM  H'3DD5'
       __BADRAM  H'3DD7'
       __BADRAM  H'3DD9'
       __BADRAM  H'3DE3'-H'3DE7'
       __BADRAM  H'3DFB'-H'3E9B'
       __BADRAM  H'3E9D'
       __BADRAM  H'3E9F'-H'3EB7'
       __BADRAM  H'3EC2'
       __BADRAM  H'3EC4'-H'3EC8'
       __BADRAM  H'3ECB'-H'3ED6'
       __BADRAM  H'3ED8'-H'3EDD'
       __BADRAM  H'3EF2'
       __BADRAM  H'3F00'-H'3F11'
       __BADRAM  H'3F24'-H'3F37'
       __BADRAM  H'3F5B'-H'3F5D'
       __BADRAM  H'3F63'
       __BADRAM  H'3F67'
       __BADRAM  H'3F6B'
       __BADRAM  H'3F6F'
       __BADRAM  H'3F80'-H'3F91'
       __BADRAM  H'3FBF'-H'3FC1'
       __BADRAM  H'3FC7'-H'3FC9'
       __BADRAM  H'3FCF'-H'3FD1'
       __BADRAM  H'3FF2'

;==========================================================================
;
;   IMPORTANT: For the PIC18 devices, the __CONFIG directive has been
;              superseded by the CONFIG directive.  The following settings
;              are available for this device.
;
;   External Oscillator Selection:
;     FEXTOSC = LP         LP (crystal oscillator) optimized for 32.768 kHz; PFM set to low power
;     FEXTOSC = XT         XT (crystal oscillator) above 100 kHz, below 8 MHz; PFM set to medium power
;     FEXTOSC = HS         HS (crystal oscillator) above 8 MHz; PFM set to high power
;     FEXTOSC = RESERVED   Reserved (DO NOT USE)
;     FEXTOSC = OFF        Oscillator not enabled
;     FEXTOSC = ECL        EC (external clock) below 100 kHz; PFM set to low power
;     FEXTOSC = ECM        EC (external clock) for 500 kHz to 8 MHz; PFM set to medium power
;     FEXTOSC = ECH        EC (external clock) above 8 MHz; PFM set to high power
;
;   Reset Oscillator Selection:
;     RSTOSC = HFINTOSC_64MHZHFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1
;     RSTOSC = RESERVED_1  Reserved
;     RSTOSC = EXTOSC_4PLL EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits
;     RSTOSC = RESERVED_2  Reserved
;     RSTOSC = SOSC        Secondary Oscillator
;     RSTOSC = LFINTOSC    Low-Frequency Oscillator
;     RSTOSC = HFINTOSC_1MHZHFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1
;     RSTOSC = EXTOSC      EXTOSC operating per FEXTOSC bits (device manufacturing default)
;
;   Clock out Enable bit:
;     CLKOUTEN = ON        CLKOUT function is enabled
;     CLKOUTEN = OFF       CLKOUT function is disabled
;
;   PRLOCKED One-Way Set Enable bit:
;     PR1WAY = OFF         PRLOCK bit can be set and cleared repeatedly
;     PR1WAY = ON          PRLOCK bit can be cleared and set only once
;
;   Clock Switch Enable bit:
;     CSWEN = OFF          The NOSC and NDIV bits cannot be changed by user software
;     CSWEN = ON           Writing to NOSC and NDIV is allowed
;
;   Fail-Safe Clock Monitor Enable bit:
;     FCMEN = OFF          Fail-Safe Clock Monitor disabled
;     FCMEN = ON           Fail-Safe Clock Monitor enabled
;
;   MCLR Enable bit:
;     MCLRE = INTMCLR      If LVP = 0, MCLR pin function is port defined function; If LVP =1, RE3 pin fuction is MCLR
;     MCLRE = EXTMCLR      If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
;
;   Power-up timer selection bits:
;     PWRTS = PWRT_1       PWRT set at 1ms
;     PWRTS = PWRT_16      PWRT set at 16ms
;     PWRTS = PWRT_64      PWRT set at 64ms
;     PWRTS = PWRT_OFF     PWRT is disabled
;
;   Multi-vector enable bit:
;     MVECEN = OFF         Interrupt contoller does not use vector table to prioritze interrupts
;     MVECEN = ON          Multi-vector enabled, Vector table used for interrupts
;
;   IVTLOCK bit One-way set enable bit:
;     IVT1WAY = OFF        IVTLOCK bit can be cleared and set repeatedly
;     IVT1WAY = ON         IVTLOCK bit can be cleared and set only once
;
;   Low Power BOR Enable bit:
;     LPBOREN = ON         ULPBOR enabled
;     LPBOREN = OFF        ULPBOR disabled
;
;   Brown-out Reset Enable bits:
;     BOREN = OFF          Brown-out Reset disabled
;     BOREN = ON           Brown-out Reset enabled according to SBOREN
;     BOREN = NOSLP        Brown-out Reset enabled while running, disabled in Sleep; SBOREN is ignored
;     BOREN = SBORDIS      Brown-out Reset enabled , SBOREN bit is ignored
;
;   Brown-out Reset Voltage Selection bits:
;     BORV = VBOR_2P85     Brown-out Reset Voltage (VBOR) set to 2.8V
;     BORV = VBOR_2P7      Brown-out Reset Voltage (VBOR) set to 2.7V
;     BORV = VBOR_245      Brown-out Reset Voltage (VBOR) set to 2.45V
;     BORV = VBOR_190      Brown-out Reset Voltage (VBOR) set to 1.90V
;
;   ZCD Disable bit:
;     ZCD = ON             ZCD always enabled
;     ZCD = OFF            ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
;
;   PPSLOCK bit One-Way Set Enable bit:
;     PPS1WAY = OFF        PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence)
;     PPS1WAY = ON         PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
;
;   Stack Full/Underflow Reset Enable bit:
;     STVREN = OFF         Stack full/underflow will not cause Reset
;     STVREN = ON          Stack full/underflow will cause Reset
;
;   Debugger Enable bit:
;     DEBUG = ON           Background debugger enabled
;     DEBUG = OFF          Background debugger disabled
;
;   Extended Instruction Set Enable bit:
;     XINST = ON           Extended Instruction Set and Indexed Addressing Mode enabled
;     XINST = OFF          Extended Instruction Set and Indexed Addressing Mode disabled
;
;   WDT Period selection bits:
;     WDTCPS = WDTCPS_0    Divider ratio 1:32
;     WDTCPS = WDTCPS_1    Divider ratio 1:64
;     WDTCPS = WDTCPS_2    Divider ratio 1:128
;     WDTCPS = WDTCPS_3    Divider ratio 1:256
;     WDTCPS = WDTCPS_4    Divider ratio 1:512
;     WDTCPS = WDTCPS_5    Divider ratio 1:1024
;     WDTCPS = WDTCPS_6    Divider ratio 1:2048
;     WDTCPS = WDTCPS_7    Divider ratio 1:4096
;     WDTCPS = WDTCPS_8    Divider ratio 1:8192
;     WDTCPS = WDTCPS_9    Divider ratio 1:16384
;     WDTCPS = WDTCPS_10   Divider ratio 1:32768
;     WDTCPS = WDTCPS_11   Divider ratio 1:65536
;     WDTCPS = WDTCPS_12   Divider ratio 1:131072
;     WDTCPS = WDTCPS_13   Divider ratio 1:262144
;     WDTCPS = WDTCPS_14   Divider ratio 1:524299
;     WDTCPS = WDTCPS_15   Divider ratio 1:1048576
;     WDTCPS = WDTCPS_16   Divider ratio 1:2097152
;     WDTCPS = WDTCPS_17   Divider ratio 1:4194304
;     WDTCPS = WDTCPS_18   Divider ratio 1:8388608
;     WDTCPS = WDTCPS_19   Divider ratio 1:32
;     WDTCPS = WDTCPS_20   Divider ratio 1:32
;     WDTCPS = WDTCPS_21   Divider ratio 1:32
;     WDTCPS = WDTCPS_22   Divider ratio 1:32
;     WDTCPS = WDTCPS_23   Divider ratio 1:32
;     WDTCPS = WDTCPS_24   Divider ratio 1:32
;     WDTCPS = WDTCPS_25   Divider ratio 1:32
;     WDTCPS = WDTCPS_26   Divider ratio 1:32
;     WDTCPS = WDTCPS_27   Divider ratio 1:32
;     WDTCPS = WDTCPS_28   Divider ratio 1:32
;     WDTCPS = WDTCPS_29   Divider ratio 1:32
;     WDTCPS = WDTCPS_30   Divider ratio 1:32
;     WDTCPS = WDTCPS_31   Divider ratio 1:65536; software control of WDTPS
;
;   WDT operating mode:
;     WDTE = OFF           WDT Disabled; SWDTEN is ignored
;     WDTE = SWDTEN        WDT enabled/disabled by SWDTEN bit
;     WDTE = NSLEEP        WDT enabled while sleep=0, suspended when sleep=1; SWDTEN ignored
;     WDTE = ON            WDT enabled regardless of sleep
;
;   WDT Window Select bits:
;     WDTCWS = WDTCWS_0    window delay = 87.5; no software control; keyed access required
;     WDTCWS = WDTCWS_1    window delay = 75 percent of time; no software control; keyed access required
;     WDTCWS = WDTCWS_2    window delay = 62.5 percent of time; no software control; keyed access required
;     WDTCWS = WDTCWS_3    window delay = 50 percent of time; no software control; keyed access required
;     WDTCWS = WDTCWS_4    window delay = 37.5 percent of time; no software control; keyed access required
;     WDTCWS = WDTCWS_5    window delay = 25 percent of time; no software control; keyed access required
;     WDTCWS = WDTCWS_6    window always open (100%); no software control; keyed access required
;     WDTCWS = WDTCWS_7    window always open (100%); software control; keyed access not required
;
;   WDT input clock selector:
;     WDTCCS = LFINTOSC    WDT reference clock is the 31.0 kHz LFINTOSC
;     WDTCCS = MFINTOSC    WDT reference clock is the 32kHz MFINTOSC output
;     WDTCCS = SOSC        WDT reference clock is SOSC
;     WDTCCS = SC          Software Control
;
;   Boot Block Size selection bits:
;     BBSIZE = BBSIZE_32768Boot Block size is 32768 words
;     BBSIZE = BBSIZE_16384Boot Block size is 16384 words
;     BBSIZE = BBSIZE_8192 Boot Block size is 8192 words
;     BBSIZE = BBSIZE_4096 Boot Block size is 4096 words
;     BBSIZE = BBSIZE_2048 Boot Block size is 2048 words
;     BBSIZE = BBSIZE_1024 Boot Block size is 1024 words
;     BBSIZE = BBSIZE_512  Boot Block size is 512 words
;
;   Boot Block enable bit:
;     BBEN = ON            Boot block enabled
;     BBEN = OFF           Boot block disabled
;
;   Storage Area Flash enable bit:
;     SAFEN = ON           SAF enabled
;     SAFEN = OFF          SAF disabled
;
;   Application Block write protection bit:
;     WRTAPP = ON          Application Block write protected
;     WRTAPP = OFF         Application Block not write protected
;
;   Configuration Register Write Protection bit:
;     WRTB = ON            Configuration registers (300000-30000Bh) write-protected
;     WRTB = OFF           Configuration registers (300000-30000Bh) not write-protected
;
;   Boot Block Write Protection bit:
;     WRTC = ON            Boot Block (000000-0007FFh) write-protected
;     WRTC = OFF           Boot Block (000000-0007FFh) not write-protected
;
;   Data EEPROM Write Protection bit:
;     WRTD = ON            Data EEPROM write-protected
;     WRTD = OFF           Data EEPROM not write-protected
;
;   SAF Write protection bit:
;     WRTSAF = ON          SAF Write Protected
;     WRTSAF = OFF         SAF not Write Protected
;
;   Low Voltage Programming Enable bit:
;     LVP = OFF            HV on MCLR/VPP must be used for programming
;     LVP = ON             Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored
;
;   PFM and Data EEPROM Code Protection bit:
;     CP = ON              PFM and Data EEPROM code protection enabled
;     CP = OFF             PFM and Data EEPROM code protection disabled
;
;==========================================================================
;==========================================================================
;
;       Configuration Bits
;
;   NAME            Address
;   CONFIG1L        300000h
;   CONFIG1H        300001h
;   CONFIG2L        300002h
;   CONFIG2H        300003h
;   CONFIG3L        300004h
;   CONFIG3H        300005h
;   CONFIG4L        300006h
;   CONFIG4H        300007h
;   CONFIG5L        300008h
;   CONFIG5H        300009h
;
;==========================================================================

; The following is an assignment of address values for all of the
; configuration registers for the purpose of table reads
_CONFIG1L       EQU  H'300000'
_CONFIG1H       EQU  H'300001'
_CONFIG2L       EQU  H'300002'
_CONFIG2H       EQU  H'300003'
_CONFIG3L       EQU  H'300004'
_CONFIG3H       EQU  H'300005'
_CONFIG4L       EQU  H'300006'
_CONFIG4H       EQU  H'300007'
_CONFIG5L       EQU  H'300008'
_CONFIG5H       EQU  H'300009'

;----- CONFIG1L Options --------------------------------------------------
_FEXTOSC_LP_1L       EQU  H'F8'; LP (crystal oscillator) optimized for 32.768 kHz; PFM set to low power
_FEXTOSC_XT_1L       EQU  H'F9'; XT (crystal oscillator) above 100 kHz, below 8 MHz; PFM set to medium power
_FEXTOSC_HS_1L       EQU  H'FA'; HS (crystal oscillator) above 8 MHz; PFM set to high power
_FEXTOSC_RESERVED_1L EQU  H'FB'; Reserved (DO NOT USE)
_FEXTOSC_OFF_1L      EQU  H'FC'; Oscillator not enabled
_FEXTOSC_ECL_1L      EQU  H'FD'; EC (external clock) below 100 kHz; PFM set to low power
_FEXTOSC_ECM_1L      EQU  H'FE'; EC (external clock) for 500 kHz to 8 MHz; PFM set to medium power
_FEXTOSC_ECH_1L      EQU  H'FF'; EC (external clock) above 8 MHz; PFM set to high power

_RSTOSC_HFINTOSC_64MHZ_1L EQU  H'8F'; HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1
_RSTOSC_RESERVED_1_1L EQU  H'9F'; Reserved
_RSTOSC_EXTOSC_4PLL_1L EQU  H'AF'; EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits
_RSTOSC_RESERVED_2_1L EQU  H'BF'; Reserved
_RSTOSC_SOSC_1L      EQU  H'CF'; Secondary Oscillator
_RSTOSC_LFINTOSC_1L  EQU  H'DF'; Low-Frequency Oscillator
_RSTOSC_HFINTOSC_1MHZ_1L EQU  H'EF'; HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1
_RSTOSC_EXTOSC_1L    EQU  H'FF'; EXTOSC operating per FEXTOSC bits (device manufacturing default)

;----- CONFIG1H Options --------------------------------------------------
_CLKOUTEN_ON_1H      EQU  H'FE'; CLKOUT function is enabled
_CLKOUTEN_OFF_1H     EQU  H'FF'; CLKOUT function is disabled

_PR1WAY_OFF_1H       EQU  H'FD'; PRLOCK bit can be set and cleared repeatedly
_PR1WAY_ON_1H        EQU  H'FF'; PRLOCK bit can be cleared and set only once

_CSWEN_OFF_1H        EQU  H'F7'; The NOSC and NDIV bits cannot be changed by user software
_CSWEN_ON_1H         EQU  H'FF'; Writing to NOSC and NDIV is allowed

_FCMEN_OFF_1H        EQU  H'DF'; Fail-Safe Clock Monitor disabled
_FCMEN_ON_1H         EQU  H'FF'; Fail-Safe Clock Monitor enabled

;----- CONFIG2L Options --------------------------------------------------
_MCLRE_INTMCLR_2L    EQU  H'FE'; If LVP = 0, MCLR pin function is port defined function; If LVP =1, RE3 pin fuction is MCLR
_MCLRE_EXTMCLR_2L    EQU  H'FF'; If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 

_PWRTS_PWRT_1_2L     EQU  H'F9'; PWRT set at 1ms
_PWRTS_PWRT_16_2L    EQU  H'FB'; PWRT set at 16ms
_PWRTS_PWRT_64_2L    EQU  H'FD'; PWRT set at 64ms
_PWRTS_PWRT_OFF_2L   EQU  H'FF'; PWRT is disabled

_MVECEN_OFF_2L       EQU  H'F7'; Interrupt contoller does not use vector table to prioritze interrupts
_MVECEN_ON_2L        EQU  H'FF'; Multi-vector enabled, Vector table used for interrupts

_IVT1WAY_OFF_2L      EQU  H'EF'; IVTLOCK bit can be cleared and set repeatedly
_IVT1WAY_ON_2L       EQU  H'FF'; IVTLOCK bit can be cleared and set only once

_LPBOREN_ON_2L       EQU  H'DF'; ULPBOR enabled
_LPBOREN_OFF_2L      EQU  H'FF'; ULPBOR disabled

_BOREN_OFF_2L        EQU  H'3F'; Brown-out Reset disabled
_BOREN_ON_2L         EQU  H'7F'; Brown-out Reset enabled according to SBOREN
_BOREN_NOSLP_2L      EQU  H'BF'; Brown-out Reset enabled while running, disabled in Sleep; SBOREN is ignored
_BOREN_SBORDIS_2L    EQU  H'FF'; Brown-out Reset enabled , SBOREN bit is ignored

;----- CONFIG2H Options --------------------------------------------------
_BORV_VBOR_2P85_2H   EQU  H'FC'; Brown-out Reset Voltage (VBOR) set to 2.8V
_BORV_VBOR_2P7_2H    EQU  H'FD'; Brown-out Reset Voltage (VBOR) set to 2.7V
_BORV_VBOR_245_2H    EQU  H'FE'; Brown-out Reset Voltage (VBOR) set to 2.45V
_BORV_VBOR_190_2H    EQU  H'FF'; Brown-out Reset Voltage (VBOR) set to 1.90V

_ZCD_ON_2H           EQU  H'FB'; ZCD always enabled
_ZCD_OFF_2H          EQU  H'FF'; ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON

_PPS1WAY_OFF_2H      EQU  H'F7'; PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence)
_PPS1WAY_ON_2H       EQU  H'FF'; PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle

_STVREN_OFF_2H       EQU  H'EF'; Stack full/underflow will not cause Reset
_STVREN_ON_2H        EQU  H'FF'; Stack full/underflow will cause Reset

_DEBUG_ON_2H         EQU  H'DF'; Background debugger enabled
_DEBUG_OFF_2H        EQU  H'FF'; Background debugger disabled

_XINST_ON_2H         EQU  H'7F'; Extended Instruction Set and Indexed Addressing Mode enabled
_XINST_OFF_2H        EQU  H'FF'; Extended Instruction Set and Indexed Addressing Mode disabled

;----- CONFIG3L Options --------------------------------------------------
_WDTCPS_WDTCPS_0_3L  EQU  H'E0'; Divider ratio 1:32
_WDTCPS_WDTCPS_1_3L  EQU  H'E1'; Divider ratio 1:64
_WDTCPS_WDTCPS_2_3L  EQU  H'E2'; Divider ratio 1:128
_WDTCPS_WDTCPS_3_3L  EQU  H'E3'; Divider ratio 1:256
_WDTCPS_WDTCPS_4_3L  EQU  H'E4'; Divider ratio 1:512
_WDTCPS_WDTCPS_5_3L  EQU  H'E5'; Divider ratio 1:1024
_WDTCPS_WDTCPS_6_3L  EQU  H'E6'; Divider ratio 1:2048
_WDTCPS_WDTCPS_7_3L  EQU  H'E7'; Divider ratio 1:4096
_WDTCPS_WDTCPS_8_3L  EQU  H'E8'; Divider ratio 1:8192
_WDTCPS_WDTCPS_9_3L  EQU  H'E9'; Divider ratio 1:16384
_WDTCPS_WDTCPS_10_3L EQU  H'EA'; Divider ratio 1:32768
_WDTCPS_WDTCPS_11_3L EQU  H'EB'; Divider ratio 1:65536
_WDTCPS_WDTCPS_12_3L EQU  H'EC'; Divider ratio 1:131072
_WDTCPS_WDTCPS_13_3L EQU  H'ED'; Divider ratio 1:262144
_WDTCPS_WDTCPS_14_3L EQU  H'EE'; Divider ratio 1:524299
_WDTCPS_WDTCPS_15_3L EQU  H'EF'; Divider ratio 1:1048576
_WDTCPS_WDTCPS_16_3L EQU  H'F0'; Divider ratio 1:2097152
_WDTCPS_WDTCPS_17_3L EQU  H'F1'; Divider ratio 1:4194304
_WDTCPS_WDTCPS_18_3L EQU  H'F2'; Divider ratio 1:8388608
_WDTCPS_WDTCPS_19_3L EQU  H'F3'; Divider ratio 1:32
_WDTCPS_WDTCPS_20_3L EQU  H'F4'; Divider ratio 1:32
_WDTCPS_WDTCPS_21_3L EQU  H'F5'; Divider ratio 1:32
_WDTCPS_WDTCPS_22_3L EQU  H'F6'; Divider ratio 1:32
_WDTCPS_WDTCPS_23_3L EQU  H'F7'; Divider ratio 1:32
_WDTCPS_WDTCPS_24_3L EQU  H'F8'; Divider ratio 1:32
_WDTCPS_WDTCPS_25_3L EQU  H'F9'; Divider ratio 1:32
_WDTCPS_WDTCPS_26_3L EQU  H'FA'; Divider ratio 1:32
_WDTCPS_WDTCPS_27_3L EQU  H'FB'; Divider ratio 1:32
_WDTCPS_WDTCPS_28_3L EQU  H'FC'; Divider ratio 1:32
_WDTCPS_WDTCPS_29_3L EQU  H'FD'; Divider ratio 1:32
_WDTCPS_WDTCPS_30_3L EQU  H'FE'; Divider ratio 1:32
_WDTCPS_WDTCPS_31_3L EQU  H'FF'; Divider ratio 1:65536; software control of WDTPS

_WDTE_OFF_3L         EQU  H'9F'; WDT Disabled; SWDTEN is ignored
_WDTE_SWDTEN_3L      EQU  H'BF'; WDT enabled/disabled by SWDTEN bit
_WDTE_NSLEEP_3L      EQU  H'DF'; WDT enabled while sleep=0, suspended when sleep=1; SWDTEN ignored
_WDTE_ON_3L          EQU  H'FF'; WDT enabled regardless of sleep

;----- CONFIG3H Options --------------------------------------------------
_WDTCWS_WDTCWS_0_3H  EQU  H'F8'; window delay = 87.5; no software control; keyed access required
_WDTCWS_WDTCWS_1_3H  EQU  H'F9'; window delay = 75 percent of time; no software control; keyed access required
_WDTCWS_WDTCWS_2_3H  EQU  H'FA'; window delay = 62.5 percent of time; no software control; keyed access required
_WDTCWS_WDTCWS_3_3H  EQU  H'FB'; window delay = 50 percent of time; no software control; keyed access required
_WDTCWS_WDTCWS_4_3H  EQU  H'FC'; window delay = 37.5 percent of time; no software control; keyed access required
_WDTCWS_WDTCWS_5_3H  EQU  H'FD'; window delay = 25 percent of time; no software control; keyed access required
_WDTCWS_WDTCWS_6_3H  EQU  H'FE'; window always open (100%); no software control; keyed access required
_WDTCWS_WDTCWS_7_3H  EQU  H'FF'; window always open (100%); software control; keyed access not required

_WDTCCS_LFINTOSC_3H  EQU  H'C7'; WDT reference clock is the 31.0 kHz LFINTOSC
_WDTCCS_MFINTOSC_3H  EQU  H'CF'; WDT reference clock is the 32kHz MFINTOSC output
_WDTCCS_SOSC_3H      EQU  H'D7'; WDT reference clock is SOSC
_WDTCCS_SC_3H        EQU  H'FF'; Software Control

;----- CONFIG4L Options --------------------------------------------------
_BBSIZE_BBSIZE_32768_4L EQU  H'F9'; Boot Block size is 32768 words
_BBSIZE_BBSIZE_16384_4L EQU  H'FA'; Boot Block size is 16384 words
_BBSIZE_BBSIZE_8192_4L EQU  H'FB'; Boot Block size is 8192 words
_BBSIZE_BBSIZE_4096_4L EQU  H'FC'; Boot Block size is 4096 words
_BBSIZE_BBSIZE_2048_4L EQU  H'FD'; Boot Block size is 2048 words
_BBSIZE_BBSIZE_1024_4L EQU  H'FE'; Boot Block size is 1024 words
_BBSIZE_BBSIZE_512_4L EQU  H'FF'; Boot Block size is 512 words

_BBEN_ON_4L          EQU  H'F7'; Boot block enabled
_BBEN_OFF_4L         EQU  H'FF'; Boot block disabled

_SAFEN_ON_4L         EQU  H'EF'; SAF enabled
_SAFEN_OFF_4L        EQU  H'FF'; SAF disabled

_WRTAPP_ON_4L        EQU  H'7F'; Application Block write protected
_WRTAPP_OFF_4L       EQU  H'FF'; Application Block not write protected

;----- CONFIG4H Options --------------------------------------------------
_WRTB_ON_4H          EQU  H'FE'; Configuration registers (300000-30000Bh) write-protected
_WRTB_OFF_4H         EQU  H'FF'; Configuration registers (300000-30000Bh) not write-protected

_WRTC_ON_4H          EQU  H'FD'; Boot Block (000000-0007FFh) write-protected
_WRTC_OFF_4H         EQU  H'FF'; Boot Block (000000-0007FFh) not write-protected

_WRTD_ON_4H          EQU  H'FB'; Data EEPROM write-protected
_WRTD_OFF_4H         EQU  H'FF'; Data EEPROM not write-protected

_WRTSAF_ON_4H        EQU  H'F7'; SAF Write Protected
_WRTSAF_OFF_4H       EQU  H'FF'; SAF not Write Protected

_LVP_OFF_4H          EQU  H'DF'; HV on MCLR/VPP must be used for programming
_LVP_ON_4H           EQU  H'FF'; Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored

;----- CONFIG5L Options --------------------------------------------------
_CP_ON_5L            EQU  H'FE'; PFM and Data EEPROM code protection enabled
_CP_OFF_5L           EQU  H'FF'; PFM and Data EEPROM code protection disabled

;----- CONFIG5H Options --------------------------------------------------

;----- DEVID Equates --------------------------------------------------
_DEVID1          EQU  H'3FFFFE'
_DEVID2          EQU  H'3FFFFF'

;----- IDLOC Equates --------------------------------------------------
_IDLOC0          EQU  H'200000'
_IDLOC1          EQU  H'200001'
_IDLOC2          EQU  H'200002'
_IDLOC3          EQU  H'200003'
_IDLOC4          EQU  H'200004'
_IDLOC5          EQU  H'200005'
_IDLOC6          EQU  H'200006'
_IDLOC7          EQU  H'200007'
_IDLOC8          EQU  H'200008'
_IDLOC9          EQU  H'200009'
_IDLOC10         EQU  H'20000A'
_IDLOC11         EQU  H'20000B'
_IDLOC12         EQU  H'20000C'
_IDLOC13         EQU  H'20000D'
_IDLOC14         EQU  H'20000E'
_IDLOC15         EQU  H'20000F'

        LIST
