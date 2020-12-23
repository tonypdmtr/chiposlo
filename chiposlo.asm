;1025 bytes, RAM:   336, CRC: $C4FA
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
                    #ListOff
                    #Uses     811e2.inc
                    #ListOn

; (1) UPON RELOCATION, THE DATA AT LOC $C133
;     MUST BE CHANGED ACCORDINGLY (SEE RANDOM)
; (2) IF DISPLAY AREA MOVED, THE DATA AT LOC $C227
;     IS CHANGED TO HIGH-ORDER BYTE OF BUFFER ADRS.

;*******************************************************************************
                    #RAM                          ; SCRATCHPAD RAM ASSIGNMENTS (PAGE 0)
;*******************************************************************************
                    org       0

.irq                rmb       2         ; Volatile vector for IRQ
.load_dump_begin    rmb       2         ; BEGIN ADRS FOR LOAD/DUMP
.load_dump_end      rmb       2         ; ENDING ADRS FOR LOAD/DUMP
.go_memod           rmb       2         ; ADRS FOR GO AND MEMOD
                    rmb       2
vx_temp             rmb       1         ; VARIABLE X (ALSO X-COORD)
                    rmb       2
seed                rmb       1         ; RANDOM BYTE (SEED)
                    rmb       1
atemp               rmb       1         ; TEMP
                    rmb       2
xtemp               rmb       2          ; 2-BYTE SAVE FOR X, SP
temp_adrs           rmb       2
                    rmb       1
keycode_temp        rmb       1          ; KEYCODE TEMP
bad_read_flag       rmb       1          ; KEY BAD-READ FLAG
                    rmb       3
.display            rmb       2          ; DISPLAY POINTER (BYTE LOC'N)
pattern_temp        rmb       2
rtc_timer           rmb       1          ; RTC TIMER VALUE
tone_duration       rmb       1          ; DURATION COUNT FOR TONE
pseudo_pc           rmb       2          ; PSEUDO PRGM-COUNTER
pseudo_sp           rmb       2          ; PSEUDO STACK-PTR
.chip8_mem          rmb       2          ; CHIP8 MEMORY POINTER
pseudo_ir           rmb       2          ; PSEUDO INST-REG
.vx                 rmb       2          ; POINTS TO VX
                    rmb       2
vx                  rmb       1          ; VARIABLE X (ALSO X-COORD)
vy                  rmb       1          ; VARIABLE Y (ALSO Y-COORD)
          ;-------------------------------------- ; CHIP8 VARIABLES (TABLE)
VO                  rmb       15
overlap_flag        rmb       1
          ;-------------------------------------- ; CHIPOSLO MODIFICATIONS
alu_sub             rmb       5                   ; 8XYN SUBROUTINE TEMP (5 BYTES)
                    rmb       2
.random             rmb       2                   ; RANDOM POINTER
                    rmb       7
digit_pattern       rmb       5                   ; DIGIT PATTERN TEMP (5 BYTES)
          ;-------------------------------------- ; CHIP8 SUBROUTINE STACK
STACK               equ       $005F
          ;-------------------------------------- ; OPERATING-SYSTEM STACK
STACKTOP            def       $007F               ; STACK TOP (MONITOR)
          ;--------------------------------------
          ; CHIP8 GRAPHIC DISPLAY AREA
          ; (1/4K RAM BLOCK MAPPED ONTO T.V. SCREEN BY DMA.)
          ; IN FORMAT 64X32 DOTS
          ;--------------------------------------
                    org       $0100
DISBUF              rmb       256                 ; DISPLAY BUFFER AREA
ENDBUF              equ       *

PIAA                equ       $8010,1             ; PORT-A FOR KEYPAD
PIAB                equ       $8012,1             ; PORT-B FOR TAPE, RTC, TONE

;*******************************************************************************
                    #ROM                          ; CHIP8 INTERPRETER MAINLINE
;*******************************************************************************

                    #spauto

CHIP8               proc
                    bsr       ERASE               ; NORMAL ENTRY POINT
                    ldx       #$0200              ; RESET PSEUDO-PC
                    stx       pseudo_pc
                    ldx       #STACK              ; RESET STACK PTR
                    stx       pseudo_sp
Fetch@@             ldx       pseudo_pc           ; POINT TO NEXT INSTR
                    ldx       ,x                  ; COPY TO pseudo_ir
                    stx       pseudo_ir
                    stx       temp_adrs           ; SAVE ADRS (MMM)
                    jsr       SKIP2               ; BUMP PRGM-CTR
                    ldb       temp_adrs           ; MASK OFF ADRS
                    andb      #$0F
                    stb       temp_adrs
                    bsr       FINDV               ; EXTRACT VX ALSO
                    stb       vx                  ; STASH VX
;                   stb       vx_temp             ; STASH VX FOR ALU
                    stx       .vx                 ; SAVE LOCATION OF VX
                    ldb       pseudo_ir+1         ; FIND Y
                    tba                           ; STASH KK
                    lsrb:4
                    bsr       FINDV               ; EXTRACT VY
                    stb       vy                  ; STASH VY
          ;-------------------------------------- ; EXEC
                    ldx       #JUMTAB-2           ; POINT TO JUMP TABLE
                    ldb       pseudo_ir           ; EXTRACT MSD
                    andb      #$F0
Loop@@              inx:2                         ; FIND ROUTINE ADRS
                    subb      #16
                    bcc       Loop@@              ; BRANCH IF HIGHER OR SAME
                    ldx       ,x                  ; LOAD ROUTINE ADRS
                    jsr       ,x                  ; PERFORM ROUTINE
                    bra       Fetch@@             ; NEXT INSTR...

;*******************************************************************************

                    #spauto

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

                    #spauto

EXCALL              proc
                    ldb       pseudo_ir           ; GET INSTR REG
                    bne       CALL
                    cmpa      #$EE
                    beq       RETDO
                    cmpa      #$E0
                    bne       Done@@              ; NOP, FETCH
;                   bra       ERASE
Done@@              equ       :AnRTS

;*******************************************************************************

                    #spauto

ERASE               proc
                    clra                          ; WRITE ZEROS TO SCREEN
                    ldx       #DISBUF             ; POINT TO DISPLAY BUFF
;                   bra       FILL

;*******************************************************************************

                    #spauto

FILL                proc
Loop@@              sta       ,x                  ; FILL SCREEN WITH ACC-A
                    inx
                    cpx       #ENDBUF             ; DONE?
                    bne       Loop@@
                    rts

;*******************************************************************************

                    #spauto

RETDO               proc
                    tsx                           ; SAVE REAL SP
                    lds       pseudo_sp
                    pula
                    sta       pseudo_pc           ; PULL PC
                    pula
                    sta       pseudo_pc+1
                    sts       pseudo_sp           ; SAVE CHIP8 SP
                    txs                           ; RESTORE SP
                    rts

;*******************************************************************************

                    #spauto

CALL                proc
                    ldx       temp_adrs           ; GET OPRND ADRS(MMM)
                    jmp       ,x                  ; PERFORM MLS

;*******************************************************************************

                    #spauto

GOTOV               proc
                    lda       VO                  ; 16-BIT ADD VO TO ADRS
                    clrb
                    adda      temp_adrs+1
                    sta       temp_adrs+1
                    adcb      temp_adrs
                    stb       temp_adrs
;                   bra       GOTO

;*******************************************************************************

                    #spauto

GOTO                proc
                    ldx       temp_adrs           ; MOVE ADRS TO PC
                    stx       pseudo_pc
                    rts                           ; FETCH

;*******************************************************************************

                    #spauto

LETI                proc
                    ldx       temp_adrs           ; MOVE ADRS TO MI PTR
                    stx       .chip8_mem
                    rts                           ; FETCH

;*******************************************************************************

                    #spauto

DOSUB               proc
                    tsx                           ; SAVE SP
                    lds       pseudo_sp
                    lda       pseudo_pc+1         ; PUSH PC
                    psha
                    lda       pseudo_pc
                    psha
                    sts       pseudo_sp           ; SAVE CHIP SP
                    txs                           ; RESTORE REAL SP
                    bra       GOTO                ; JUMP TO ADRS(MMM)

;*******************************************************************************
; CONDITIONAL SKIP ROUTINES

                    #spauto

SKFEV               proc
                    lda       vy                  ; GET VY
                    bra       SKFEQ

;*******************************************************************************

                    #spauto

SKFNV               proc
                    lda       vy
                    bra       SKFNE

;*******************************************************************************

                    #spauto

SKFEQ               proc
                    cmpa      vx
                    bne       Done@@
;                   bra       SKIP2
Done@@              equ       :AnRTS

;*******************************************************************************

                    #spauto

SKIP2               proc
                    ldx       pseudo_pc           ; ADD 2 TO PC
                    inx:2
                    stx       pseudo_pc
                    rts

;*******************************************************************************

                    #spauto

SKFKEY              proc
                    jsr       KEYINP              ; INTERROGATE KEYBOARD
                    tst       bad_read_flag       ; KEY DOWN?
                    beq       _1@@
                    ldb       #$A1                ; WHAT INSTRN?
                    cmpb      pseudo_ir+1         ; SKF VX#KEY
                    beq       SKIP2
                    rts                           ; NO KEY GO FETCH

_1@@                cmpa      #$9E
                    beq       SKFEQ
;                   bra       SKFNE

;*******************************************************************************

                    #spauto

SKFNE               proc
                    cmpa      vx
                    bne       SKIP2
                    rts

;*******************************************************************************

                    #spauto

LETVK               proc
                    adda      vx
                    bra       PUTVX

;*******************************************************************************

                    #spauto

RANDV               proc
                    bsr       RANDOM              ; GET RANDOM BYTE
                    anda      pseudo_ir+1
                    bra       PUTVX

;*******************************************************************************
; ARITHMETIC/LOGIC ROUTINES
; CONSTRUCTS A TEMPORARY SUBROUTINE IN RAM

                    #spauto

LETVV               proc
                    tab
                    lda       vy
                    andb      #$0F                ; EXTRACT N
                    beq       PUTVX               ; 8XY0: VX=VY
                    ldx       #$0A39              ; (OP) VX / CLV, RTS
                    cmpb      #$05                ; 8XY5: VX=VX-VY, INVERTED CARRY
                    bne       _1@@
                    lda       vx
                    ldx       #$2F7E              ; (OP) VY, JMP (INVC)
_1@@                cmpb      #$07                ; 8XY7: VX=VY-VX, INVERTED CARRY
                    bne       _2@@
                    ldx       #$0A7E              ; (OP) VX, JMP (INVC)
_2@@                stx       alu_sub+1           ; STASH CONSTRUCTED ROUTINE
                    ldx       #INVC
                    stx       alu_sub+3           ; STASH ADDRESS TO INVC
FindOp@@            inx
                    decb
                    bne       FindOp@@
                    ldb       3,x                 ; FIND OPCODE IN TABLE
                    stb       alu_sub             ; STASH OPCODE
                    clr       overlap_flag
                    jsr       alu_sub
                    rol       overlap_flag        ; overlap_flag=CARRY
;                   bra       PUTVX

;*******************************************************************************

                    #spauto

PUTVX               proc
                    ldx       .vx                 ; REPLACE VX
                    sta       ,x
                    rts

;*******************************************************************************
; INVERT CARRY FLAG

                    #spauto

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

                    #spauto

RANDOM              proc
                    lda       #$C0                ; HIGH-ORDER BYTE OF .random =
                    sta       .random             ; =MSB OF CHIP8 START ADRS
                    inc       .random+1
                    ldx       .random             ; POINT TO NEXT PROGRAM BYTE
                    lda       seed                ; GET SEED (LAST VALUE)
                    adda      ,x                  ; MANGLE IT
                    eora      $FF,x
                    sta       seed                ; STASH IT
                    rts

;-------------------------------------------------------------------------------
; JUMP TABLE FOR MISCELLANEOUS INSTRNS [FXZZ]
;-------------------------------------------------------------------------------

?                   macro
                    fcb       ~1~
                    fdb       ~2~
                    endm

MINJMP              @?        $07,VTIME           ; VX=TIMER
                    @?        $0A,VKEY            ; VX=KEY
                    @?        $15,TIMEV           ; TIMER=VX
                    @?        $18,TONEV           ; TONE=VX
                    @?        $1E,LETIV           ; I=I+VX
                    @?        $29,LETDSP          ; I=DSPL,VX
                    @?        $33,LETDEQ          ; MI=DEQ,VX
                    @?        $55,STORV           ; MI=VO:VX
                    @?        $65,LOADV           ; VO:VX=MI

;*******************************************************************************

                    #spauto

MISC                proc
                    ldx       #MINJMP             ; POINT TO TABLE
                    ldb       #9                  ; DO 9 TIMES
Loop@@              lda       ,x                  ; GET TABLE OPCODE
                    cmpa      pseudo_ir+1
                    beq       _1@@
                    inx:3
                    decb
                    bne       Loop@@
                    jmp       Start               ; BAD OPCODE, RETURN TO MON.
_1@@                ldx       1,x                 ; GET ROUTINE ADRS FROM TABLE
                    lda       vx                  ; GET VX
                    jmp       ,x                  ; GO TO ROUTINE

;*******************************************************************************

                    #spauto

VTIME               proc
                    lda       rtc_timer
                    bra       PUTVX

;*******************************************************************************

                    #spauto

VKEY                proc
                    jsr       GETKEY
                    bra       PUTVX

;*******************************************************************************

                    #spauto

TIMEV               proc
                    sta       rtc_timer
                    rts

;*******************************************************************************

                    #spauto

TONEV               proc
                    tab                           ; SET DURATION=VX
                    jmp       BTONE

;*******************************************************************************

                    #spauto

LETIV               proc
                    clrb                          ; 16-BIT ADD VX TO I
                    adda      .chip8_mem+1
                    sta       .chip8_mem+1
                    adcb      .chip8_mem
                    stb       .chip8_mem
                    rts

;*******************************************************************************
; COPY COMPRESSED DIGIT PATTERN (FROM TABLE)
; TO  5-BYTE ARRAY (digit_pattern), & SET I FOR 'SHOW',

                    #spauto

LETDSP              proc
                    ldx       #HexTable@@-2       ; POINT TO HEX DIGIT PATTERNS ,
                    anda      #$0F                ; ISOLATE LS DIGIT
_1@@                inx:2                         ; SEARCH TABLE.....
                    deca                          ; (A=VX)
                    bpl       _1@@
                    ldx       ,x                  ; MOVE pattern_temp
                    stx       pattern_temp
                    ldx       #digit_pattern      ; POINT PATTERN ARRAY(5)
                    stx       .chip8_mem          ; SET MI POINTER
                    ldb       #5                  ; DO 5 TIMES
Loop@@              lda       pattern_temp
                    anda      #$E0                ; EXTRACT 3 BITS
                    sta       4,x
                    dex
                    lda       #3                  ; DO 3 TIMES
_2@@                rol       pattern_temp+1      ; MOVE NEXT 3 BITS
                    rol       pattern_temp
                    deca
                    bne       _2@@                ; CONT (3)....
                    decb
                    bne       Loop@@              ; CONT (5)....
                    rts

;-------------------------------------------------------------------------------
; HEXADECIMAL DIGIT PATTERNS (3x5  MATRIX)
;-------------------------------------------------------------------------------

HexTable@@          fdb       $F6DF               ; 0
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

                    #spauto

LETDEQ              proc
                    ldx       .chip8_mem          ; GET MI POINTER
                    ldb       #100                ; N=100
                    bsr       DECI                ; CALC 100'S DIGIT
                    ldb       #10                 ; N=10
                    bsr       DECI                ; CALC l0'S DIGIT
                    ldb       #1
;                   bra       DECI

;*******************************************************************************

                    #spauto

DECI                proc
                    pshb      tmp@@
                    tsy
                    clrb
Loop@@              cmpa      tmp@@,spy           ; DO UNTIL A<N ...
                    bcs       Done@@              ; BRANCH IF LOWER NOT SAME.
                    incb
                    suba      tmp@@,spy
                    bra       Loop@@              ; END-DO...
Done@@              stb       ,x                  ; STASH
                    inx                           ; FOR NEXT DIGIT
                    ins::ais
                    rts

;*******************************************************************************

                    #spauto

STORV               proc
                    sei                           ; KILL IRQ FOR DATA STACK
                    sts       xtemp               ; SAVE SP
                    lds       #VO-1               ; POINT TO VARIABLES TABLE
                    ldx       .chip8_mem          ; FOINT MI
                    bra       MOVX                ; TRANSFER NB BYTES

;*******************************************************************************

                    #spauto

LOADV               proc
                    sei                           ; KILL IRQ
                    sts       xtemp
                    lds       .chip8_mem          ; POINT MI
                    des
                    ldx       #VO                 ; POINT TO VO
;                   bra       MOVX                pro

;*******************************************************************************

                    #spauto

MOVX                proc
                    ldb       .vx+1               ; CALC. X (AS IN VX)
                    andb      #$0F                ; LOOP (X+l) TIMES.....
Loop@@              pula                          ; GET NEXT V
                    sta       ,x                  ; COPY IT
                    inx
                    inc       .chip8_mem+1        ; I=I+X+1(ASSUMES SAME PAGE)
                    decb
                    bpl       Loop@@              ; CONTINUE...
                    lds       xtemp               ; RESTORE SP
                    cli                           ; RESTORE IRQ
                    rts

;*******************************************************************************
; DISPLAY ROUTINES

                    #spauto

SHOW                proc
                    tab                           ; GET N (OPCODE LSB)
                    clr       overlap_flag        ; CLEAR OVERLAP FLAG
                    nop                           ; PADDING FOR FALLTHROUGH
;                   bra       SHOWI

;*******************************************************************************

                    #spauto

SHOWI               proc
                    ldx       .chip8_mem          ; POINT TO PATTERN BYTES
                    lda       #1                  ; SET DISPLAY ADRS MSB =
                    sta       .display            ; = DISBUF HIGH-ORDER BYTE.
                    andb      #$0F                ; COMPUTE NO. OF BYTES (N)
                    bne       Loop@@              ; IF N=0, MAKE N=16
                    ldb       #16
Loop@@              pshb                          ; DO N TIMES,...,.
                    stx       temp_adrs           ; SAVE MI POINTER
                    lda       ,x                  ; FETCH NEW PATTERN BYTE
                    sta       pattern_temp
                    clr       pattern_temp+1
                    ldb       vx                  ; DETERMINE OFFSET BIT COUNT
                    andb      #7
_1@@                beq       _2@@                ; SHIFT INTO MATCHING POS'N
                    lsr       pattern_temp
                    ror       pattern_temp+1
                    decb
                    bne       _1@@
_2@@                ldb       vx                  ; GET X COORD
                    bsr       DISLOC              ; FIND WHERE IS FIRST DISP BYTE
                    lda       pattern_temp
                    bsr       SHOWUP
                    ldb       vx
                    addb      #8                  ; FIND WHERE IS ADJACENT BYTE
                    bsr       DISLOC
                    lda       pattern_temp+1
                    bsr       SHOWUP
                    inc       vy
                    ldx       temp_adrs           ; POINT NEXT PATTERN BYTE
                    inx
                    pulb
                    decb
                    bne       Loop@@              ; CONT.....
                    rts

;*******************************************************************************

                    #spauto

SHOWUP              proc
                    tab                           ; UPDATE DISPLAY BYTE
                    eorb      ,x                  ; X-OR WITH EXISTING DISPLAY
                    ora       ,x                  ; OR ALSO FOR OVERLAP TEST
                    stb       ,x                  ; STORE XORED BYTE
                    cba
                    beq       Done@@              ; XOR SAME AS OR ELSE....
                    lda       #1                  ; SET OVERLAP FLAG (overlap_flag)
                    sta       overlap_flag
Done@@              rts

;*******************************************************************************
; COMPUTE ADRS OF DISPLAY BYTE AT COORDS(B, VY):

                    #spauto

DISLOC              proc
                    lda       vy                  ; FETCH Y COORD
                    anda      #$1F                ; MASK TO 5 BITS FOR WRAP-ROUN
                    asla:3                        ; LEFT JUSTIFY
                    andb      #$3F                ; MASK X COORD TO 6 BITS
                    lsrb:3                        ; DROP 3 LS BITS
                    aba                           ; BUILD BYTE
                    sta       .display+1          ; DISP LOC'N LSB COMPLETED
                    ldx       .display            ; POINT TO DISP BYTE AT (VX,VY)
                    rts

;*******************************************************************************
; KEYPAD ROUTINES

                    #spauto

PAINZ               proc
                    ldb       #$F0                ; INITIALIZE PORT
;                   bra       PAINV

;*******************************************************************************

                    #spauto

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

                    #spauto

KEYINP              proc
                    bsr       PAINZ               ; RESET KEYPAD PORT
                    clr       bad_read_flag       ; RESET BAD-READ FLAG
                    bsr       DEL333              ; DELAY FOR DEBOUNCE
                    ldb       ,x                  ; INPUT ROW DATA
                    bsr       KBILD               ; FORM CODE BITS 0,1
                    sta       keycode_temp
                    ldb       #$0F                ; SET DDR FOR...
                    bsr       PAINV               ; INVERSE ROW/COL DIR N
                    ldb       ,x                  ; INPUT COLUM DATA
                    lsrb:4                        ; RIGHT JUSTIFY
                    bsr       KBILD               ; FORM CODE BITS 2,3
                    asla:2
                    adda      keycode_temp
                    sta       keycode_temp        ; BUILD COMPLETE KEYCODE
                    rts

;*******************************************************************************

                    #spauto

KBILD               proc
                    cmpb      #$0F                ; CHECK KEY STATUS
                    bne       Go@@                ; KEY IS DOWN, GO DECODE IT
                    stb       bad_read_flag       ; NO KEY, SET BAD-READ FLAG
Go@@                lda       #-1
Loop@@              inca                          ; (A=RESULT)
                    lsrb                          ; SHIFT DATA BIT TO CARRY
                    bcs       Loop@@              ; FOUND ZERO BIT ?
                    rts

;*******************************************************************************
; GETKEY WAIT FOR KEYDOWN, THEN INPUTS

                    #spauto

GETKEY              proc
                    stx       xtemp               ; SAVE X FOR CALLING ROUTINE
Loop@@              bsr       PAINZ               ; RESET PORT, CLEAR FLAGS
_1@@                lda       1,x                 ; INPUT STATUS (HEX KEY DOWN?)
                    bmi       HexKey@@            ; YES FETCH IT IN
                    asla                          ; TRY CA2 FLAG
                    bpl       _1@@                ; FN XEY DOWN? (A<0?)
                    tst       ,x                  ; YES: RESET FLAG IN PIA
                    bra       Done@@              ; RETURN WITHOUT CODE
HexKey@@            bsr       KEYINP              ; DECODE THE KEYPAD
                    tst       bad_read_flag       ; WAS IT A BAD READ?
                    bne       Loop@@              ; YES, TRY AGAIN
Done@@              bsr       BLEEP               ; O.K. ACKNOWLEDGE
                    ldx       xtemp               ; RESTORE CALLER'S X-REG
                    rts                           ; RETURN (WITH A<O FOR FN KEY)

;*******************************************************************************
; TONE GENERATING ROUTINES

                    #spauto

BLEEP               proc
                    ldb       #4
;                   bra       BTONE

;*******************************************************************************

                    #spauto

BTONE               proc
                    stb       tone_duration       ; SET DURATION (RTC CYCLES)
                    ldb       #$41                ; TURN AUDIO ON
                    stb       PIAB
_1@@                tst       tone_duration       ; WAIT FOR RTC TIME-OUT
                    bne       _1@@
                    ldb       #1                  ; TURN AUDIO OFF
                    stb       PIAB
                    rts

;*******************************************************************************
; SOFTWARE DELAY ROUTINE FOR SERIAL I/O:

                    #spauto

DEL333              proc
                    bsr       DEL167              ; DELAY FOR 3.33 MILLISEC
;                   bra       DEL167

;*******************************************************************************

                    #spauto
                              #Cycles
DEL167              proc
                    pshx
                    ldx       #DELAY@@            ; DELAY FOR 1.67 MILLISEC
                              #Cycles
Loop@@              dex
                    bne       Loop@@
                              #temp :cycles
                    pulx
                    rts

DELAY@@             equ       167*BUS_KHZ/100-:cycles-:ocycles/:temp

;*******************************************************************************
; TAPE INPUT/OUTPUT ROUTINES
; INITIALIZE TAPE, tone_duration, RTC, & DMA
; A=$3F FOR DISPLAY/DMA ON; A=$37 FOR OFF:

                    #spauto

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

                    #spauto

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
                    ldx       xtemp               ; RESTORE X
                    rts

;*******************************************************************************

                    #spauto

XCHG                proc
                    stx       xtemp               ; SAVE X-REG
                    ldx       #PIAB
                    rts

;*******************************************************************************
; OUTPUT ONE BYTE TO TAPE PORT

                    #spauto

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
                    ldx       xtemp               ; RESTORE X
                    rts

;*******************************************************************************

GETKEE              bra       GETKEY              ; FOR INTERLINKING

;*******************************************************************************
; TAPE LOAD AND DUMP ROUTINES

                    #spauto

LODUMX              proc
                    lda       #$37                ; KILL DISPLAY (DMA OFF)
                    bsr       PBINZ
                    ldx       .load_dump_begin    ; POINT TO FIRST LOAD/DUMP ADR
                    rts

;*******************************************************************************

                    #spauto

DUMP                proc
                    bsr       LODUMX
Loop@@              lda       ,x                  ; FETCH RAM BYTE
                    bsr       OUTBYT
                    inx
                    cpx       .load_dump_end      ; (.load_dump_end = LAST ADRS+1)
                    bne       Loop@@
                    bra       Start

;*******************************************************************************

                    #spauto

LOAD                proc
                    bsr       LODUMX
Loop@@              bsr       INBYT
                    sta       ,x                  ; STASH BYTE IN RAM
                    inx
                    cpx       .load_dump_end      ; DONE?
                    bne       Loop@@              ; CONT....
;                   bra       Start

;*******************************************************************************
; MONITOR ENTRY POINT

                    #spauto

Start               proc
                    lds       #STACKTOP           ; RESET SP TO TOP
                    ldx       #RTC_Handler        ; SETUP IRQ VECTOR FOR RTC
                    stx       .irq
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
                    ldx       .go_memod           ; FETCH ADRS FOR GO
                    jmp       ,x
          ;--------------------------------------
Addr@@              bsr       BYT1                ; BUILD ADRS MS BYTE
                    sta       .go_memod
                    bsr       BYTIN               ; INPUT & BUILD LSB
                    sta       .go_memod+1
                    bsr       SHOADR              ; DISPLAY RESULTANT ADRS
                    bra       Cmd@@

;*******************************************************************************

                    #spauto

BYTIN               proc
                    bsr       GETKEE              ; INPUT 2 HEX DIGITS
;                   bra       BYT1                pro

;*******************************************************************************

                    #spauto

BYT1                proc
                    asla:4                        ; LEFT JUSTIFY FIRST DIGIT
                    sta       atemp               ; HOLD IT
                    bsr       GETKEE              ; INPUT ANOTHER DIGIT
                    adda      atemp               ; BUILD A BYTE
                    rts

;*******************************************************************************
; MEMORY MODIFY ROUTINE

                    #spauto

MEMOD               proc
Loop@@              bsr       SHOADR              ; SHOW CURRENT ADRS
                    ldx       .go_memod           ; SHOW DATA AT ADRS
                    bsr       SHODAT
                    bsr       GETKEE              ; WAIT FOR INPUT
                    tsta
                    bmi       _1@@                ; FN KEY; NEXT ADRS
                    bsr       BYT1                ; HEX KEY; NEW DATA BYTE
                    sta       ,x                  ; DEPOSIT IT
_1@@                inx
                    stx       .go_memod           ; BUMP ADRS
                    bra       Loop@@

;*******************************************************************************

                    #spauto

SHOADR              proc
                    lda       #$10                ; SET CURSOR HOME POSITION
                    bsr       CURS1
                    ldx       #DISBUF+200         ; ILLUMINATE LAST 7 ROWS
                    lda       #$FF
                    jsr       FILL
                    ldx       #.go_memod          ; POINT TO ADRS MS BYTE
                    bsr       SHODAT
                    inx                           ; FOINT TO ADRS LS BYTE
                    bsr       SHODAT
                    bra       CURSR               ; MOVE CURSOR RIGHT

;*******************************************************************************

                    #spauto

SHODAT              proc
                    lda       ,x                  ; FETCH DATA @ X
                    psha
                    lsra:4                        ; ISOLATE MS DIGIT
                    bsr       DIGOUT              ; SHOW ONE DIGIT
                    pula
;                   bra       DIGOUT

;*******************************************************************************

                    #spauto

DIGOUT              proc
                    stx       xtemp               ; SAVE X
                    jsr       LETDSP              ; POINT TO DIGIT PATTERN
                    ldb       #5                  ; SHOW 5-BYTE PATTERN
                    jsr       SHOWI
;                   bra       CURSR

;*******************************************************************************

                    #spauto

CURSR               proc
                    lda       #4                  ; SHIFT CURSOR RIGHT 4 DOTS
                    adda      vx
;                   bra       CURS1

;*******************************************************************************

                    #spauto

CURS1               proc
                    sta       vx                  ; SET X COORD
                    lda       #$1A                ; SET Y COORD
                    sta       vy
                    ldx       xtemp               ; RESTORE X_REG
                    rts

;*******************************************************************************
; REAL TIME CLOCK INTERRUPT SERVICE ROUTINE

                    #spauto

RTC_Handler         proc
                    dec       rtc_timer
                    dec       tone_duration
                    tst       PIAB                ; CLEAR IRQ FLAG IN PIA
AnRTI               rti

;*******************************************************************************

                    #spauto

IRQ_Handler         proc
                    ldx       .irq                ; INDIRECT JUMP VIA .irq
                    jmp       ,x

;*******************************************************************************
                    #VECTORS                      ; RESTART AND INTERRUPT TRAPS
;*******************************************************************************

                    @vector   Virq,IRQ_Handler    ; (ALLOWS USER-WRITTEN ISR)
                    @vector   Vswi                ; SWI ROUIINE AT $0080 (OPTION)
                    @vector   Vxirq               ; NMI ROUTINE AT $0083 (OPTION)
                    @vector   Vreset,Start

                    end
