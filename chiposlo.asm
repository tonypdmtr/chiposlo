;*******************************************************************************
; C H I P O S L O
;
; COMPACT HEXADECIMAL INTERPRETIVE PROGRAMMING AND OPERATING SYSTEM
; WITH LOGICAL OPERATORS
;
; DREAM-6800 OPERATING SYSTEM WITH CHIP8 LANGUAGE INTERPRETER
;
; ORIGINATED BY MICHAEL J BAUER, DEAKIN UNIVERSITY, 1978
; MODIFICATIONS BY TOBIAS V LANGHOFF, UNIVERSITY OF OSLO, 2020
; ADAPTED TO ASM11 AND MC68HC11 USING OWN CODING STYLE BY TONY PAPADIMITRIOU, 2020
;
; www.mjbauer.biz/DREAM6800.htm
;*******************************************************************************
                    #Uses     811e2.inc
; (1) UPON RELOCATION, THE DATA AT LOC $C133
; MUST BE CHANGED ACCORDINGLY (SEE RANDOM)
; (2) IF DISPLAY AREA MOVED, THE DATA AT LOC $C227
; IS CHANGED TO HIGH-ORDER BYTE OF BUFFER ADRS.
          ;-------------------------------------- ; SCRATCHPAD RAM ASSIGNMENTS (PAGE 0)
IRQV                equ       $0000               ; INTERRUPT VECTOR
BEGA                equ       $0002               ; BEGIN ADRS FOR LOAD/DUMP
ENDA                equ       $0004               ; ENDING ADRS FOR LOAD/DUMP
ADRS                equ       $0006               ; ADRS FOR GO AND MEMOD
RND                 equ       $000D               ; RANDOM BYTE (SEED)
N                   equ       $000E               ; TEMP
ATEMP               equ       $000F               ; TEMP
XTEMP               equ       $0012               ; 2-BYTE SAVE FOR X, SP
ZHI                 equ       $0014               ; TEMP ADRS
ZLO                 equ       $0015
KEYCOD              equ       $0017               ; KEYCODE TEMP
BADRED              equ       $0018               ; KEY BAD-READ FLAG
BLOC                equ       $001C               ; DISPLAY POINTER (BYTE LOC'N)
PATNH               equ       $001E               ; PATTERN TEMP
PATNL               equ       $001F
TIMER               equ       $0020               ; RTC TIMER VALUE
TONE                equ       $0021               ; DURATION COUNT FOR TONE
PC                  equ       $0022               ; PSEUDO PRGM-COUNTER
PSP                 equ       $0024               ; PSEUDO STACK-PTR
I                   equ       $0026               ; CHIP8 MEMORY POINTER
PIR                 equ       $0028               ; PSEUDO INST-REG
VXLOC               equ       $002A               ; POINTS TO VX
VX                  equ       $002E               ; VARIABLE X (ALSO X-COORD)
VY                  equ       $002F               ; VARIABLE Y (ALSO Y-COORD)
          ;-------------------------------------- ; CHIP8 VARIABLES (TABLE)
VO                  equ       $0030
VF                  equ       $003F
          ;-------------------------------------- ; CHIPOSLO MODIFICATIONS
VXTMP               equ       $000A               ; VARIABLE X (ALSO X-COORD)
ALUSUB              equ       $0040               ; 8XYN SUBROUTINE TEMP (5 BYTES)
RNDX                equ       $0047               ; RANDOM POINTER
DDPAT               equ       $0050               ; DIGIT PATTERN TEMP (5 BYTES)
          ;-------------------------------------- ; CHIP8 SUBROUTINE STACK
STACK               equ       $005F
          ;-------------------------------------- ; OPERATING-SYSTEM STACK
STACKTOP            def       $007F               ; STACK TOP (MONITOR)
          ;--------------------------------------
          ; CHIP8 GRAPHIC DISPLAY AREA
          ; (1/4K RAM BLOCK MAPPED ONTO T.V. SCREEN BY DMA.)
          ; IN FORMAT 64X32 DOTS
          ;--------------------------------------
DISBUF              equ       $0100               ; DISPLAY BUFFER AREA
ENDBUF              equ       $0200
PIAA                equ       $8010               ; PORT-A FOR KEYPAD
PIAB                equ       $8012               ; PORT-B FOR TAPE, RTC, TONE

;*******************************************************************************
                    #ROM                          ; CHIP8 INTERPRETER MAINLINE
;*******************************************************************************

CHIP8               proc
                    bsr       ERASE               ; NORMAL ENTRY POINT
                    ldx       #$0200              ; RESET PSEUDO-PC
                    stx       PC
                    ldx       #STACK              ; RESET STACK PTR
                    stx       PSP
Fetch@@             ldx       PC                  ; POINT TO NEXT INSTR
                    ldx       ,x                  ; COPY TO PIR
                    stx       PIR
                    stx       ZHI                 ; SAVE ADRS (MMM)
                    jsr       SKIP2               ; BUMP PRGM-CTR
                    ldb       ZHI                 ; MASK OFF ADRS
                    andb      #$0F
                    stb       ZHI
                    bsr       FINDV               ; EXTRACT VX ALSO
                    stb       VX                  ; STASH VX
                    stb       VXTMP               ; STASH VX FOR ALU
                    stx       VXLOC               ; SAVE LOCATION OF VX
                    ldb       PIR+1               ; FIND Y
                    tba                           ; STASH KK
                    lsrb:4
                    bsr       FINDV               ; EXTRACT VY
                    stb       VY                  ; STASH VY
          ;-------------------------------------- ; EXEC
                    ldx       #JUMTAB-2           ; POINT TO JUMP TABLE
                    ldb       PIR                 ; EXTRACT MSD
                    andb      #$F0
Loop@@              inx:2                         ; FIND ROUTINE ADRS
                    subb      #16
                    bcc       Loop@@              ; BRANCH IF HIGHER OR SAME
                    ldx       ,x                  ; LOAD ROUTINE ADRS
                    jsr       ,x                  ; PERFORM ROUTINE
                    bra       Fetch@@             ; NEXT INSTR...

;*******************************************************************************

FINDV               proc
                    ldx       #VO-1               ; POINT TO VARIABLES TABLE
Loop@@              inx                           ; FIND LOCN VX
                    decb
                    bpl       Loop@@
                    ldb       ,x                  ; FETCH VX FROM TABLE
                    rts

;-------------------------------------------------------------------------------
; JUMP TABLE (ROUTINE ADDRESSES)
;-------------------------------------------------------------------------------

JUMTAB              fdb       EXCALL              ; ERASE, RET, CALL, NOP
                    fdb       GOTO                ; GOTO MMM
                    fdb       DOSUB               ; DO MMM
                    fdb       SKFEQ               ; SKF VX=KK
                    fdb       SKFNE               ; SKF VX#KK
                    fdb       SKFEV               ; SKF VX=VY
                    fdb       PUTVX               ; Vx=KK
                    fdb       LETVK               ; VX=VX+KK
                    fdb       LETVV               ; VX=[VX][+-&!]VY
                    fdb       SKFNV               ; SKF VX#VY
                    fdb       LETI                ; I=MMM
                    fdb       GOTOV               ; GOTO MMM+VO
                    fdb       RANDV               ; VX-RND.KK
                    fdb       SHOW                ; SHOW N@VX, VY
                    fdb       SKFKEY              ; SKF VX[=#]KEY
                    fdb       MISC                ; (MINOR JUMP TABL)

;*******************************************************************************
; ERASE, RETURN, CALL (MLS), OR NOP INTRN:

EXCALL              proc
                    ldb       PIR                 ; GET INSTR REG
                    bne       CALL
                    cmpa      #$EE
                    beq       RETDO
                    cmpa      #$E0
                    bne       Done@@              ; NOP, FETCH
;                   bra       ERASE
Done@@              equ       :AnRTS

;*******************************************************************************

ERASE               proc
                    clra                          ; WRITE ZEROS TO SCREEN
                    ldx       #DISBUF             ; POINT TO DISPLAY BUFF
;                   bra       FILL

;*******************************************************************************

FILL                proc
Loop@@              sta       ,x                  ; FILL SCREEN WITH ACC-A
                    inx
                    cpx       #ENDBUF             ; DONE?
                    bne       Loop@@
                    rts

;*******************************************************************************

RETDO               proc
                    tsx                           ; SAVE REAL SP
                    lds       PSP
                    pula
                    sta       PC                  ; PULL PC
                    pula
                    sta       PC+1
                    sts       PSP                 ; SAVE CHIP8 SP
                    txs                           ; RESTORE SP
                    rts

;*******************************************************************************

CALL                proc
                    ldx       ZHI                 ; GET OPRND ADRS(MMM)
                    jmp       ,x                  ; PERFORM MLS

;*******************************************************************************

GOTOV               proc
                    lda       VO                  ; 16-BIT ADD VO TO ADRS
                    clrb
                    adda      ZLO
                    sta       ZLO
                    adcb      ZHI
                    stb       ZHI
;                   bra       GOTO

;*******************************************************************************

GOTO                proc
                    ldx       ZHI                 ; MOVE ADRS TO PC
                    stx       PC
                    rts                           ; FETCH

;*******************************************************************************

LETI                proc
                    ldx       ZHI                 ; MOVE ADRS TO MI PTR
                    stx       I
                    rts                           ; FETCH

;*******************************************************************************

DOSUB               proc
                    tsx                           ; SAVE SP
                    lds       PSP
                    lda       PC+1                ; PUSH PC
                    psha
                    lda       PC
                    psha
                    sts       PSP                 ; SAVE CHIP SP
                    txs                           ; RESTORE REAL SP
                    bra       GOTO                ; JUMP TO ADRS(MMM)

;*******************************************************************************
; CONDITIONAL SKIP ROUTINES

SKFEV               proc
                    lda       VY                  ; GET VY
                    bra       SKFEQ

;*******************************************************************************

SKFNV               proc
                    lda       VY
                    bra       SKFNE

;*******************************************************************************

SKFEQ               proc
                    cmpa      VX
                    bne       Done@@
;                   bra       SKIP2
Done@@              equ       :AnRTS

;*******************************************************************************

SKIP2               proc
                    ldx       PC                  ; ADD 2 TO PC
                    inx:2
                    stx       PC
                    rts

;*******************************************************************************

SKFKEY              proc
                    jsr       KEYINP              ; INTERROGATE KEYBOARD
                    tst       BADRED              ; KEY DOWN?
                    beq       _1@@
                    ldb       #$A1                ; WHAT INSTRN?
                    cmpb      PIR+1               ; SKF VX#KEY
                    beq       SKIP2
                    rts                           ; NO KEY GO FETCH

_1@@                cmpa      #$9E
                    beq       SKFEQ
;                   bra       SKFNE

;*******************************************************************************

SKFNE               proc
                    cmpa      VX
                    bne       SKIP2
                    rts

;*******************************************************************************

LETVK               proc
                    adda      VX
                    bra       PUTVX

;*******************************************************************************

RANDV               proc
                    bsr       RANDOM              ; GET RANDOM BYTE
                    anda      PIR+1
                    bra       PUTVX

;*******************************************************************************
; ARITHMETIC/LOGIC ROUTINES
; CONSTRUCTS A TEMPORARY SUBROUTINE IN RAM

LETVV               proc
                    tab
                    lda       VY
                    andb      #$0F                ; EXTRACT N
                    beq       PUTVX               ; 8XY0: VX=VY
                    ldx       #$0A39              ; (OP) VX / CLV, RTS
                    cmpb      #$05                ; 8XY5: VX=VX-VY, INVERTED CARRY
                    bne       _1@@
                    lda       VX
                    ldx       #$2F7E              ; (OP) VY, JMP (INVC)
_1@@                cmpb      #$07                ; 8XY7: VX=VY-VX, INVERTED CARRY
                    bne       _2@@
                    ldx       #$0A7E              ; (OP) VX, JMP (INVC)
_2@@                stx       ALUSUB+1            ; STASH CONSTRUCTED ROUTINE
                    ldx       #INVC
                    stx       ALUSUB+3            ; STASH ADDRESS TO INVC
FindOp@@            inx
                    decb
                    bne       FindOp@@
                    ldb       3,x                 ; FIND OPCODE IN TABLE
                    stb       ALUSUB              ; STASH OPCODE
                    clr       VF                  ; CLEAR VF
                    jsr       ALUSUB
                    rol       VF                  ; VF=CARRY
;                   bra       PUTVX

;*******************************************************************************

PUTVX               proc
                    ldx       VXLOC               ; REPLACE VX
                    sta       ,x
                    rts

;*******************************************************************************
; INVERT CARRY FLAG

INVC                proc
                    rolb
                    incb
                    rorb
                    rts

;-------------------------------------------------------------------------------
; TABLE WITH ALU OPCODES FOR 8XYN INSTRS
;-------------------------------------------------------------------------------
;                   org       $C12B               ;(seems redundant also)

JUMP8              ;fcb       $96                 ; LDAA VY 8XY0, HANDLED ABOVE
                    fcb       $9A                 ; ORAA VY 8XY1
                    fcb       $94                 ; ANDA VY 8XY2
                    fcb       $98                 ; EORA VY 8XY3
                    fcb       $9B                 ; ADDA VY 8XY4
                    fcb       $90                 ; SUBA VY 8XY5
                    fcb       $44                 ; ASLA 8XY6
                    fcb       $90                 ; SUBA VX 8XY7
;                   fcb       $48                 ; LSRA    8XYE, BYTE FOUND BELOW

;*******************************************************************************
; RANDOM BYTE GENERATOR

RANDOM              proc
                    lda       #$C0                ; HIGH-ORDER BYTE OF RNDX =
                    sta       RNDX                ; =MSB OF CHIP8 START ADRS
                    inc       RNDX+1
                    ldx       RNDX                ; POINT TO NEXT PROGRAM BYTE
                    lda       RND                 ; GET SEED (LAST VALUE)
                    adda      ,x                  ; MANGLE IT
                    eora      $FF,x
                    sta       RND                 ; STASH IT
                    rts

;-------------------------------------------------------------------------------
; JUMP TABLE FOR MISCELLANEOUS INSTRNS [FXZZ]
;-------------------------------------------------------------------------------

MINJMP              fcb       $07                 ; VX=TIMER
                    fdb       VTIME
                    fcb       $0A                 ; VX=KEY
                    fdb       VKEY
                    fcb       $15                 ; TIMER=VX
                    fdb       TIMEV
                    fcb       $18                 ; TONE=VX
                    fdb       TONEV
                    fcb       $1E                 ; I=I+VX
                    fdb       LETIV
                    fcb       $29                 ; I=DSPL,VX
                    fdb       LETDSP
                    fcb       $33                 ; MI=DEQ,VX
                    fdb       LETDEQ
                    fcb       $55                 ; MI=VO:VX
                    fdb       STORV
                    fcb       $65                 ; VO:VX=MI
                    fdb       LOADV

;*******************************************************************************

MISC                proc
                    ldx       #MINJMP             ; POINT TO TABLE
                    ldb       #9                  ; DO 9 TIMES
Loop@@              lda       ,x                  ; GET TABLE OPCODE
                    cmpa      PIR+1
                    beq       _1@@
                    inx:3
                    decb
                    bne       Loop@@
                    jmp       Start               ; BAD OPCODE, RETURN TO MON.
_1@@                ldx       1,x                 ; GET ROUTINE ADRS FROM TABLE
                    lda       VX                  ; GET VX
                    jmp       ,x                  ; GO TO ROUTINE

;*******************************************************************************

VTIME               proc
                    lda       TIMER
                    bra       PUTVX

;*******************************************************************************

VKEY                proc
                    jsr       GETKEY
                    bra       PUTVX

;*******************************************************************************

TIMEV               proc
                    sta       TIMER
                    rts

;*******************************************************************************

TONEV               proc
                    tab                           ; SET DURATION=VX
                    jmp       BTONE

;*******************************************************************************

LETIV               proc
                    clrb                          ; 16-BIT ADD VX TO I
                    adda      I+1
                    sta       I+1
                    adcb      I
                    stb       I
                    rts

;*******************************************************************************
; COPY COMPRESSED DIGIT PATTERN (FROM TABLE)
; TO  5-BYTE ARRAY (DDPAT), & SET I FOR 'SHOW',

LETDSP              proc
                    ldx       #HEXTAB-2           ; POINT TO HEX DIGIT PATTERNS ,
                    anda      #$0F                ; ISOLATE LS DIGIT
_1@@                inx:2                         ; SEARCH TABLE.....
                    deca                          ; (A=VX)
                    bpl       _1@@
                    ldx       ,x                  ; MOVE PATNH
                    stx       PATNH
                    ldx       #DDPAT              ; POINT PATTERN ARRAY(5)
                    stx       I                   ; SET MI POINTER
                    ldb       #5                  ; DO 5 TIMES
Loop@@              lda       PATNH
                    anda      #$E0                ; EXTRACT 3 BITS
                    sta       4,x
                    dex
                    lda       #3                  ; DO 3 TIMES
_2@@                rol       PATNL               ; MOVE NEXT 3 BITS
                    rol       PATNH
                    deca
                    bne       _2@@                ; CONT (3)....
                    decb
                    bne       Loop@@              ; CONT (5)....
                    rts

;-------------------------------------------------------------------------------
; HEXADECIMAL DIGIT PATTERNS (3X5  MATRIX)
;-------------------------------------------------------------------------------

HEXTAB              fdb       $F6DF               ; 0
                    fdb       $4925               ; 1
                    fdb       $F39F               ; 2
                    fdb       $E79F               ; 3
                    fdb       $3ED9               ; 4
                    fdb       $E7CF               ; 5
                    fdb       $F7CF               ; 6
                    fdb       $249F               ; 7
                    fdb       $F7DF               ; 8
                    fdb       $E7DF               ; 9
                    fdb       $B7DF               ; A
                    fdb       $D7DD               ; B
                    fdb       $F24F               ; C
                    fdb       $D6DD               ; D
                    fdb       $F3CF               ; E
                    fdb       $934F               ; F

;*******************************************************************************

LETDEQ              proc
                    ldx       I                   ; GET MI POINTER
                    ldb       #100                ; N=100
                    bsr       DECI                ; CALC 100'S DIGIT
                    ldb       #10                 ; N=10
                    bsr       DECI                ; CALC l0'S DIGIT
                    ldb       #1
;                   bra       DECI

;*******************************************************************************

DECI                proc
                    stb       N
                    clrb
Loop@@              cmpa      N                   ; DO UNTIL A<N ...
                    bcs       Done@@              ; BRANCH IF LOWER NOT SAME.
                    incb
                    suba      N
                    bra       Loop@@              ; END-DO...
Done@@              stb       ,x                  ; STASH
                    inx                           ; FOR NEXT DIGIT
                    rts

;*******************************************************************************

STORV               proc
                    sei                           ; KILL IRQ FOR DATA STACK
                    sts       XTEMP               ; SAVE SP
                    lds       #VO-1               ; POINT TO VARIABLES TABLE
                    ldx       I                   ; FOINT MI
                    bra       MOVX                ; TRANSFER NB BYTES

;*******************************************************************************

LOADV               proc
                    sei                           ; KILL IRQ
                    sts       XTEMP
                    lds       I                   ; POINT MI
                    des
                    ldx       #VO                 ; POINT TO VO
;                   bra       MOVX                pro

;*******************************************************************************

MOVX                proc
                    ldb       VXLOC+1             ; CALC. X (AS IN VX)
                    andb      #$0F                ; LOOP (X+l) TIMES.....
Loop@@              pula                          ; GET NEXT V
                    sta       ,x                  ; COPY IT
                    inx
                    inc       I+1                 ; I=I+X+1(ASSUMES SAME PAGE)
                    decb
                    bpl       Loop@@              ; CONTINUE...
                    lds       XTEMP               ; RESTORE SP
                    cli                           ; RESTORE IRQ
                    rts

;*******************************************************************************
; DISPLAY ROUTINES

SHOW                proc
                    tab                           ; GET N (OPCODE LSB)
                    clr       VF                  ; CLEAR OVERLAP FLAG
                    nop                           ; PADDING FOR FALLTHROUGH
;                   bra       SHOWI

;*******************************************************************************

SHOWI               proc
                    ldx       I                   ; POINT TO PATTERN BYTES
                    lda       #$01                ; SET DISPLAY ADRS MSB =
                    sta       BLOC                ; = DISBUF HIGH-ORDER BYTE.
                    andb      #$0F                ; COMPUTE NO. OF BYTES (N)
                    bne       Loop@@              ; IF N=0, MAKE N=16
                    ldb       #16
Loop@@              pshb                          ; DO N TIMES,...,.
                    stx       ZHI                 ; SAVE MI POINTER
                    lda       ,x                  ; FETCH NEW PATTERN BYTE
                    sta       PATNH
                    clr       PATNL
                    ldb       VX                  ; DETERMINE OFFSET BIT COUNT
                    andb      #7
_1@@                beq       _2@@                ; SHIFT INTO MATCHING POS'N
                    lsr       PATNH
                    ror       PATNL
                    decb
                    bne       _1@@
_2@@                ldb       VX                  ; GET X COORD
                    bsr       DISLOC              ; FIND WHERE IS FIRST DISP BYTE
                    lda       PATNH
                    bsr       SHOWUP
                    ldb       VX
                    addb      #8                  ; FIND WHERE IS ADJACENT BYTE
                    bsr       DISLOC
                    lda       PATNL
                    bsr       SHOWUP
                    inc       VY
                    ldx       ZHI                 ; POINT NEXT PATTERN BYTE
                    inx
                    pulb
                    decb
                    bne       Loop@@              ; CONT.....
                    rts

;*******************************************************************************

SHOWUP              proc
                    tab                           ; UPDATE DISPLAY BYTE
                    eorb      ,x                  ; X-OR WITH EXISTING DISPLAY
                    ora       ,x                  ; OR ALSO FOR OVERLAP TEST
                    stb       ,x                  ; STORE XORED BYTE
                    cba
                    beq       Done@@              ; XOR SAME AS OR ELSE....
                    lda       #1                  ; SET OVERLAP FLAG (VF)
                    sta       VF
Done@@              rts

;*******************************************************************************
; COMPUTE ADRS OF DISPLAY BYTE AT COORDS(B, VY):

DISLOC              proc
                    lda       VY                  ; FETCH Y COORD
                    anda      #$1F                ; MASK TO 5 BITS FOR WRAP-ROUN
                    asla:3                        ; LEFT JUSTIFY
                    andb      #$3F                ; MASK X COORD TO 6 BITS
                    lsrb:3                        ; DROP 3 LS BITS
                    aba                           ; BUILD BYTE
                    sta       BLOC+1              ; DISP LOC'N LSB COMPLETED
                    ldx       BLOC                ; POINT TO DISP BYTE AT (VX,VY)
                    rts

;*******************************************************************************
; KEYPAD ROUTINES

PAINZ               proc
                    ldb       #$F0                ; INITIALIZE PORT
;                   bra       PAINV

;*******************************************************************************

PAINV               proc
                    ldx       #PIAA               ; (ENTRY PT FOR INV. DDR)
                    clr       1,x                 ; RESET & SELECT DDR
                    stb       ,x                  ; SET DATA DIRECTION
                    ldb       #$06                ; SET O/P REG & SETUP CTRL
                    stb       1,x
                    clr       ,x                  ; OUTPUT ZEROS & RE5ET FLAGS
                    rts

;*******************************************************************************
; KEYPAD INPUT SERVICE ROUTINE

KEYINP              proc
                    bsr       PAINZ               ; RESET KEYPAD PORT
                    clr       BADRED              ; RESET BAD-READ FLAG
                    bsr       DEL333              ; DELAY FOR DEBOUNCE
                    ldb       ,x                  ; INPUT ROW DATA
                    bsr       KBILD               ; FORM CODE BITS 0,1
                    sta       KEYCOD
                    ldb       #$0F                ; SET DDR FOR...
                    bsr       PAINV               ; INVERSE ROW/COL DIR N
                    ldb       ,x                  ; INPUT COLUM DATA
                    lsrb:4                        ; RIGHT JUSTIFY
                    bsr       KBILD               ; FORM CODE BITS 2,3
                    asla:2
                    adda      KEYCOD
                    sta       KEYCOD              ; BUILD COMPLETE KEYCODE
                    rts

;*******************************************************************************

KBILD               proc
                    cmpb      #$0F                ; CHECK KEY STATUS
                    bne       Go@@                ; KEY IS DOWN, GO DECODE IT
                    stb       BADRED              ; NO KEY, SET BAD-READ FLAG
Go@@                lda       #-1
Loop@@              inca                          ; (A=RESULT)
                    lsrb                          ; SHIFT DATA BIT TO CARRY
                    bcs       Loop@@              ; FOUND ZERO BIT ?
                    rts

;*******************************************************************************
; GETKEY WAIT FOR KEYDOWN, THEN INPUTS

GETKEY              proc
                    stx       XTEMP               ; SAVE X FOR CALLING ROUTINE
Loop@@              bsr       PAINZ               ; RESET PORT, CLEAR FLAGS
_1@@                lda       1,x                 ; INPUT STATUS (HEX KEY DOWN?)
                    bmi       HexKey@@            ; YES FETCH IT IN
                    asla                          ; TRY CA2 FLAG
                    bpl       _1@@                ; FN XEY DOWN? (A<0?)
                    tst       ,x                  ; YES: RESET FLAG IN PIA
                    bra       Done@@              ; RETURN WITHOUT CODE
HexKey@@            bsr       KEYINP              ; DECODE THE KEYPAD
                    tst       BADRED              ; WAS IT A BAD READ?
                    bne       Loop@@              ; YES, TRY AGAIN
Done@@              bsr       BLEEP               ; O.K. ACKNOWLEDGE
                    ldx       XTEMP               ; RESTORE CALLER'S X-REG
                    rts                           ; RETURN (WITH A<O FOR FN KEY)

;*******************************************************************************
; TONE GENERATING ROUTINES

BLEEP               proc
                    ldb       #4
;                   bra       BTONE

;*******************************************************************************

BTONE               proc
                    stb       TONE                ; SET DURATION (RTC CYCLES)
                    ldb       #$41                ; TURN AUDIO ON
                    stb       PIAB
_1@@                tst       TONE                ; WAIT FOR RTC TIME-OUT
                    bne       _1@@
                    ldb       #1                  ; TURN AUDIO OFF
                    stb       PIAB
                    rts

;*******************************************************************************
; SOFTWARE DELAY ROUTINE FOR SERIAL I/O:

DEL333              proc
                    bsr       DEL167              ; DELAY FOR 3.33 MILLISEC
;                   bra       DEL167

;*******************************************************************************
                              #Cycles
DEL167              proc
                    psha
                    lda       #200                ; DELAY FOR 1.67 MILLISEC
                              #Cycles
Loop@@              deca
                    nop
                    bne       Loop@@
                              #temp :cycles
                    pula
                    rts

;DELAY@@            equ       167*BUS_KHZ/100-:cycles-:ocycles/:temp

;*******************************************************************************
; TAPE INPUT/OUTPUT ROUTINES
; INITIALIZE TAPE, TONE, RTC, & DMA
; A=$3F FOR DISPLAY/DMA ON; A=$37 FOR OFF:

PBINZ               proc
                    ldx       #PIAB
                    ldb       #$3B                ; SELECT DDR (DMA ON)
                    stb       1,x
                    ldb       #$7F                ; WRITE DDR
                    stb       ,x
                    sta       1,x                 ; WRITE CTRL REG
                    ldb       #1                  ; OUTPUT FOR T0NE OFF, AND...
                    stb       ,x                  ; TAPE DATA-OUT HIGH (MARKING)
                    rts

;*******************************************************************************
; INPUT ONE BYTE FROM TAPE PORT

INBYT               proc
                    bsr       XCHG                ; EXCHANGE X FOR PIA ADRS
_1@@                lda       ,x
                    bmi       _1@@                ; LOOK FOR START BIT
                    bsr       DEL167              ; DELAY HALF BIT-TIME (300BD)
                    ldb       #9                  ; DO 9 TIMES....
Loop@@              sec                           ; ENSURE PB0 MARKING
                    rol       ,x                  ; INPUT & SHIFT NEXT BIT
                    rora                          ; INTO ACC-A
                    bsr       DEL333              ; WAIT 1 BIT-TIME
                    decb
                    bne       Loop@@              ; CONT....
                    ldx       XTEMP               ; RESTORE X
                    rts

;*******************************************************************************

XCHG                proc
                    stx       XTEMP               ; SAVE X-REG
                    ldx       #PIAB
                    rts

;*******************************************************************************
; OUTPUT ONE BYTE TO TAPE PORT

OUTBYT              proc
                    bsr       XCHG
                    psha
                    dec       ,x                  ; RESET START BIT
                    ldb       #10                 ; DO 10 TIMES....
Loop@@              bsr       DEL333              ; DELAY 1 BIT-TIME
                    sta       ,x                  ; NEXT BIT TO OUT LINE (PB0)
                    sec
                    rora
                    decb
                    bne       Loop@@              ; CONT....
                    pula                          ; RESTORE A
                    ldx       XTEMP               ; RESTORE X
                    rts

GETKEE              bra       GETKEY              ; FOR INTERLINKING

;*******************************************************************************
; TAPE LOAD AND DUMP ROUTINES

LODUMX              proc
                    lda       #$37                ; KILL DISPLAY (DMA OFF)
                    bsr       PBINZ
                    ldx       BEGA                ; POINT TO FIRST LOAD/DUMP ADR
                    rts

;*******************************************************************************

DUMP                proc
                    bsr       LODUMX
Loop@@              lda       ,x                  ; FETCH RAM BYTE
                    bsr       OUTBYT
                    inx
                    cpx       ENDA                ; (ENDA = LAST ADRS+1)
                    bne       Loop@@
                    bra       Start

;*******************************************************************************

LOAD                proc
                    bsr       LODUMX
Loop@@              bsr       INBYT
                    sta       ,x                  ; STASH BYTE IN RAM
                    inx
                    cpx       ENDA                ; DONE?
                    bne       Loop@@              ; CONT....
;                   bra       Start

;*******************************************************************************
; MONITOR ENTRY POINT

Start               proc
                    lds       #STACKTOP           ; RESET SP TO TOP
                    ldx       #RTC_Handler        ; SETUP IRQ VECTOR FOR RTC
                    stx       IRQV
                    lda       #$3F                ; SETUP I/O PORT: DISPLAY ON.
                    bsr       PBINZ
                    bsr       SHOADR              ; PROMPT
                    cli                           ; ENABLE RELATIVE TIME CLOCK
Cmd@@               bsr       GETKEE              ; INPUT SOMETHING
                    tsta
                    bpl       Addr@@              ; IF HEX, GET AN ADDRESS
                    bsr       GETKEE              ; IF FN, GET A COMMAND
                    anda      #3
                    beq       MEMOD               ; 0 = MEM0RY MODIFY
                    deca
                    beq       LOAD                ; 1 = TAPE LOAD
                    deca
                    beq       DUMP                ; 2 = TAPE DUMP
          ;-------------------------------------- ; GO
                    ldx       ADRS                ; FETCH ADRS FOR GO
                    jmp       ,x
          ;--------------------------------------
Addr@@              bsr       BYT1                ; BUILD ADRS MS BYTE
                    sta       ADRS
                    bsr       BYTIN               ; INPUT & BUILD LSB
                    sta       ADRS+1
                    bsr       SHOADR              ; DISPLAY RESULTANT ADRS
                    bra       Cmd@@

;*******************************************************************************

BYTIN               proc
                    bsr       GETKEE              ; INPUT 2 HEX DIGITS
;                   bra       BYT1                pro

;*******************************************************************************

BYT1                proc
                    asla:4                        ; LEFT JUSTIFY FIRST DIGIT
                    sta       ATEMP               ; HOLD IT
                    bsr       GETKEE              ; INPUT ANOTHER DIGIT
                    adda      ATEMP               ; BUILD A BYTE
                    rts

;*******************************************************************************
; MEMORY MODIFY ROUTINE

MEMOD               proc
Loop@@              bsr       SHOADR              ; SHOW CURRENT ADRS
                    ldx       ADRS                ; SHOW DATA AT ADRS
                    bsr       SHODAT
                    bsr       GETKEE              ; WAIT FOR INPUT
                    tsta
                    bmi       _1@@                ; FN KEY; NEXT ADRS
                    bsr       BYT1                ; HEX KEY; NEW DATA BYTE
                    sta       ,x                  ; DEPOSIT IT
_1@@                inx
                    stx       ADRS                ; BUMP ADRS
                    bra       Loop@@

;*******************************************************************************

SHOADR              proc
                    lda       #$10                ; SET CURSOR HOME POSITION
                    bsr       CURS1
                    ldx       #DISBUF+200         ; ILLUMINATE LAST 7 ROWS
                    lda       #$FF
                    jsr       FILL
                    ldx       #ADRS               ; POINT TO ADRS MS BYTE
                    bsr       SHODAT
                    inx                           ; FOINT TO ADRS LS BYTE
                    bsr       SHODAT
                    bsr       CURSR               ; MOVE CURSOR RIGHT
                    rts

;*******************************************************************************

SHODAT              proc
                    lda       ,x                  ; FETCH DATA @ X
                    psha
                    lsra:4                        ; ISOLATE MS DIGIT
                    bsr       DIGOUT              ; SHOW ONE DIGIT
                    pula
;                   bra       DIGOUT

;*******************************************************************************

DIGOUT              proc
                    stx       XTEMP               ; SAVE X
                    jsr       LETDSP              ; POINT TO DIGIT PATTERN
                    ldb       #5                  ; SHOW 5-BYTE PATTERN
                    jsr       SHOWI
;                   bra       CURSR

;*******************************************************************************

CURSR               proc
                    lda       #4                  ; SHIFT CURSOR RIGHT 4 DOTS
                    adda      VX
;                   bra       CURS1

;*******************************************************************************

CURS1               proc
                    sta       VX                  ; SET X COORD
                    lda       #$1A                ; SET Y COORD
                    sta       VY
                    ldx       XTEMP               ; RESTORE X_REG
                    rts

;*******************************************************************************
; REAL TIME CLOCK INTERRUPT SERVICE ROUTINE

RTC_Handler         proc
                    dec       TIMER
                    dec       TONE
                    tst       PIAB                ; CLEAR IRQ FLAG IN PIA
AnRTI               rti

;*******************************************************************************

IRQ_Handler         proc
                    ldx       IRQV                ; INDIRECT JUMP VIA IRQV
                    jmp       ,x

;*******************************************************************************
                    #VECTORS                      ; RESTART AND INTERRUPT TRAPS
;*******************************************************************************

                    @vector   Virq,IRQ_Handler    ; (ALLOWS USER-WRITTEN ISR)
                    @vector   Vswi                ; SWI ROUIINE AT $0080 (OPTION)
                    @vector   Vxirq               ; NMI ROUTINE AT $0083 (OPTION)
                    @vector   Vreset,Start

                    end
