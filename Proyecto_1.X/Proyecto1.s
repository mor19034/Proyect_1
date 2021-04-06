;Archivo: Codigo_proyecto1
;Dispositivo: PIC16F887
;Autor:    Pablo Moreno
   
;Programa:	
;de resultado en displays, con hexadecimales y decimales
;Hardware: 4 displays de 7 segmentos, 3 push buttoms, 12 leds y 8 transistores
; 
;Creado: 23/03/2021  
;Ultima modificacion 26/03/2021

    
PROCESSOR 16F887
#include <xc.inc>
#include "Macros_proyecto1.s"
; CONFIG1
; CONFIGURATION WORD1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIGURATION WORD2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
;--------------------------------VARIABLES--------------------------------------  
  PSECT udata_shr ;comoon memory
    W_TEMP: DS 1  ; 1 byte
    STATUS_TEMP:DS 1
    cont_big:	DS 1
    cont_small: DS 1
  PSECT udata_bank0 ;memory bank 0
//<editor-fold defaultstate="collapsed" desc="Variables">
T_semaforo1:DS 1 
T_semaforo2:DS 1 
T_semaforo3:DS 1 
T_modo:	    DS 1    
flags:	    DS 1
nibbles:    DS 2
display1:   DS 2
display2:   DS 2
display3:   DS 2
display4:   DS 2
decena:	    DS 1
unidad:	    DS 1
decena2:    DS 1
unidad2:    DS 1
decena3:    DS 1    
unidad3:    DS 1
unidad4:    DS 1 
decena4:    DS 1
resta:	    DS 1
resta2:	    DS 1
resta3:	    DS 1
resta4:	    DS 1
titileo:    DS 1
valor_sema: DS 1
bandera_sem:DS 1
reset_flag: DS 1
verde:	    DS 1
verde_tilt: DS 1
amarillo:   DS 1
var:	    DS 1
mood_flags: DS 2
N_semaforo1:DS 1 ;variable para nuevo tiempo de semaforo 1
N_semaforo2:DS 1 ;variable para nuevo tiempo de semaforo 2
N_semaforo3:DS 1 ;variable para nuevo tiempo de semaforo 3
    //</editor-fold>
;--------------------------vector reset-----------------------------------------   
   PSECT resVect, class=code, abs, delta=2
   ORG 00h		;posicion 0000h para el reset
    resetVec:
	PAGESEL main
	goto main
;----------------------------vector interrupcion--------------------------------
   PSECT intVect, class=code, abs, delta=2
     ORG 04h		    ;Posicion 0004h para las interrupciones
    
PUSH:			    ;se guardan los valores de W y Status
    movwf	W_TEMP
    swapf	STATUS, W   ;se rotan los nibles de STATUS
    movwf	STATUS_TEMP
     
interrupcion:	
    btfsc	T0IF	    ;Reviso si la bandera de TMR0 se enciende
    call	int_timer0  ;Ir a interrupcion
	
    btfsc	TMR1IF	    ;Revisar si hay overflow en timer1
    call	int_timer1  ;Ir a interrupcion
	
POP:			    ;Status y W regresan a su estado normal 
    swapf	STATUS_TEMP, W
    movwf	STATUS
    swapf	W_TEMP, F
    swapf	W_TEMP, W
    retfie
;-----------------------------Subrutina de interrupcio--------------------------
//<editor-fold defaultstate="collapsed" desc="interrupcion_timer0_multiplexear">
int_timer0:
    limpiar_timer0	;Se reinicia TMR0 
    clrf	PORTD
    
    btfsc	flags, 0
    goto	display_2
	
    btfsc	flags, 1
    goto	display_3
	
    btfsc	flags, 2
    goto	display_4
	
    btfsc	flags, 3
    goto	display_5
	
    btfsc	flags, 4
    goto	display_6
    
    btfsc	flags, 5
    goto	display_7
    
    btfsc	flags, 6
    goto	display_8
;VIA 1    
display_1:			
    movf	display1, W
    movwf	PORTC
    bsf		PORTD, 1
    goto	proximo_display2
    
display_2:
    movf	display1+1, W
    movwf	PORTC
    bsf		PORTD, 0
    goto	proximo_display3
;VIA 2	
display_3:
    movf	display2, W
    movwf	PORTC
    bsf		PORTD, 3
    goto	proximo_display4
	
display_4:
    movf	display2+1, W
    movwf	PORTC
    bsf		PORTD, 2
    goto	proximo_display5
;VIA 3	
display_5:
    movf	display3, W
    movwf	PORTC
    bsf		PORTD, 5
    goto	proximo_display6
	
display_6: 
    movf	display3+1, W
    movwf	PORTC
    bsf		PORTD, 4
    goto	proximo_display7
;COFIGURACION    
display_7:
    movf	display4, W
    movwf	PORTC
    bsf		PORTD, 7
    goto	proximo_display8
	
display_8: 
    movf	display4+1, W
    movwf	PORTC
    bsf		PORTD, 6
    goto	proximo_display1

proximo_display2: 
    movlw	1
    xorwf	flags, F
    return
	
proximo_display3: 
    movlw	3
    xorwf	flags, F
    return
proximo_display4:
    movlw	6
    xorwf	flags, F
    return	
proximo_display5:
    movlw	12
    xorwf	flags, F
    return
	
proximo_display6:
    movlw	24
    xorwf	flags, F
    return
proximo_display7:
    movlw	48
    xorwf	flags, F
    return
proximo_display8:
    movlw	96
    xorwf	flags, F
    return    
proximo_display1:
    clrf	flags, F
    return//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="interrupcion timer1">
int_timer1:
    limpiar_timer1		;Se limpia la bandera de overflow de timer1
    decf    T_semaforo1, F	;Se decrementa el tiempo de semaforo 1
    decf    T_semaforo2, F	;Se decrementa el tiempo de semaforo 2
    decf    T_semaforo3, F	;Se decrementa el tiempo de semaforo 3
    return//</editor-fold>

;------------------------------codigo principal---------------------------------    
     PSECT code, delta=2, abs
    ORG 100h			;posicion para el codigo
    
//<editor-fold defaultstate="collapsed" desc="Tabla de conversion">
tabla:
    clrf	PCLATH
    bsf	PCLATH, 0   ;	   PCLATH = 01
    andlw	0x0f
    addwf	PCL	    ;	   PC = PCLATH + PCL + W
    retlw	00111111B   ;0	   Regresamos con la literal en W 
    retlw	00000110B   ;1
    retlw	01011011B   ;2
    retlw	01001111B   ;3
    retlw	01100110B   ;4
    retlw	01101101B   ;5
    retlw	01111101B   ;6
    retlw	00000111B   ;7
    retlw	01111111B   ;8
    retlw	01100111B   ;9
    retlw	01110111B   ;A
    retlw	01111100B   ;b
    retlw	00111001B   ;C
    retlw	01011110B   ;d
    retlw	01111001B   ;E
    retlw	01110001B   ;F//</editor-fold>

	
;----------------------------configruacion princiapl---------------------------- 
main: 
   call port_io
   call conf_reloj
   call	conf_timer0
   call	conf_timer1
   tiempos_semaforos	;Se colocan los tiempos de los semaforos
   call conf_int_Enbale
   banksel PORTA
;------------------------------loop principal-----------------------------------    
loop:  
    btfsc   PORTB,1
    call    cambiarmodo
    
    call    division
    
    call    semaforos
    
    call    preparar_display
    
    btfss   mood_flags,0
    goto    moodA
    goto    moodB
//<editor-fold defaultstate="collapsed" desc="Cambio de modo">
cambiarmodo:
    btfsc	PORTB,1
    goto	$-1
    incf	mood_flags
    return
    //</editor-fold>    

//<editor-fold defaultstate="collapsed" desc="incremento y decremento">
incr:
    btfsc   PORTB,2
    goto    $-1
    incf    var
    call    limite_superior
    
    movf    var,W
    return
    
decr:
    btfsc   PORTB,3
    goto    $-1
    decf    var
    call    limite_inferior
    
    movf    var,W
    return
    
    //</editor-fold>  

//<editor-fold defaultstate="collapsed" desc="limites">
limite_inferior:
 
    movlw   9
    subwf   var,W
    
    btfsc   STATUS,2
    goto    ufl
    
    return
    
ufl:
    movlw   20
    movwf   var
    return
    
limite_superior:
    movlw    21
    subwf    var,W
   
    btfsc    STATUS,2
    goto     ofl
   
    return
   
ofl:
    movlw   10
    movwf   var
    return
    //</editor-fold>   
 
//<editor-fold defaultstate="collapsed" desc="revisar bandera de modos">
moodA:
    btfsc   mood_flags,1
    goto    moodD
    goto    moodC
    goto    loop
    
moodB:
    btfsc   mood_flags,1
    goto    moodE
    goto    moodF
    goto    loop
    
moodC:
    btfsc   mood_flags,2
    goto    mood4
    goto    mood0
    goto    loop
    
moodD:
    btfsc   mood_flags,2
    goto    moodm
    goto    mood2
    goto    loop
  
moodE:
    btfsc   mood_flags,2
    goto    moodm
    goto    mood3
    goto    loop
    
moodF:
    btfsc   mood_flags,2
    goto    moodm
    goto    mood1
    goto    loop
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="MODOS">
mood0:
    bcf		    PORTE,0
    bcf		    PORTE,1
    bcf		    PORTE,2
    
    clrf	    display4
    clrf	    display4+1
    goto	    loop
    
mood1:
    movf	    N_semaforo1,W
    movwf	    var
    bsf		    PORTE,0
    bcf		    PORTE,1
    bcf		    PORTE,2	
    
    btfsc	    PORTB,2
    call	    incr
    
    btfsc	    PORTB,3
    call	    decr
    
    movf	    var,W
    movwf	    N_semaforo1
    movwf	    resta4
    
    
    call	    division
    
    movwf   unidad4,W
    call    tabla
    movwf   display4
    
    movwf   decena4,W
    call    tabla
    movwf   display4+1
    
    goto	    loop
mood2:
    movf    N_semaforo2,W
    movwf   var
    bcf	    PORTE,0
    bsf	    PORTE,1
    bcf	    PORTE,2
    
    btfsc   PORTB,2
    call    incr
    
    btfsc   PORTB,3
    call    decr
    
    movf    var,W
    movwf   N_semaforo2
    movwf   resta4
    
    
    call    division
    
    movwf   unidad4,W
    call    tabla
    movwf   display4
    
    movwf   decena4,W
    call    tabla
    movwf   display4+1
    
    goto    loop
    
mood3:
    movf    N_semaforo3,W
    movwf   var
    bcf	    PORTE,0
    bcf	    PORTE,1
    bsf	    PORTE,2
    
    btfsc   PORTB,2
    call    incr
    
    btfsc   PORTB,3
    call    decr
    
    movf    var,W
    movwf   N_semaforo3
    movwf   resta4  
    
    call division
    
    movwf   unidad4,W
    call    tabla
    movwf   display4
    
    movwf   decena4,W
    call    tabla
    movwf   display4+1
    
    goto    loop
    
mood4:
    bsf	    PORTE,0
    bsf	    PORTE,1
    bsf	    PORTE,2
    
    btfsc   PORTB,2
    call    aceptar
    
    btfsc   PORTB,3
    call    rechazar
    
    goto loop
    
    
moodm:
    clrf    mood_flags
    goto    loop
    
aceptar:
    btfsc   PORTB,2
    goto    $-1
    clrf    mood_flags
    
    movf    N_semaforo1,W
    movwf   T_semaforo1
    
    movf    N_semaforo2,W
    movwf   T_semaforo2
    
    movf    N_semaforo3,W
    movwf   T_semaforo3
    goto    loop
    
rechazar:
    btfsc   PORTB,3
    goto    $-1
    clrf    mood_flags
    goto    loop
   //</editor-fold>

;-------------------------------subrutinas--------------------------------------
//<editor-fold defaultstate="collapsed" desc="datos de operaciones para displays">
preparar_display:	    ;Resultados de operaciones se mueven a variable
    ;Via 1
    movf    unidad, W    
    call    tabla
    movwf   display1
    
    movf    decena, W 
    call    tabla
    movwf   display1+1
  
    ;Via 2
    movf    unidad2, W  
    call    tabla
    movwf   display2
    
    movf    decena2, W  
    call    tabla
    movwf   display2+1
    
    ;Via 3
    movf    unidad3, W 
    call    tabla
    movwf   display3
   
    movf    decena3, W 
    call    tabla
    movwf   display3+1
    
    return//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="conversion a decimales">
;Semaforo 1  
division:    
;Semaforo 1    
    clrf decena    ;Limpiar variables 
    clrf unidad
    clrf resta
    bcf	STATUS, 0   ;bajar la bandera de acarreo
    movf T_semaforo1, 0    ;Se mueve T_semaforo1  a resta 
    movwf resta
    movlw 10        ;Mover valor 10 a W
    subwf resta, F  ;Restamos W y resta; el resultado va a resta
    btfsc STATUS, 0 ;Si hay acarreo, incrementa decena
    incf decena	    ;Incrementar decenas
    btfsc STATUS, 0    ;Si hay acarreo, repito el proceso de division
    goto $-5        ;hasta que ya no hayan decenas
    movlw 10        ;se hace para que sea 10+ resta(que tiene valor negativo)
    addwf resta	    ;entonces se obtienen las unidades restantes
    movf resta, W   ;mover el resultado de la operacion a "resta"
    movwf unidad
  
;Semaforo 2    
    clrf decena2    ;Limpiar variables 
    clrf unidad2
    clrf resta2
    bcf	STATUS, 0
    movf T_semaforo2, 0    ;Se mueve T_semaforo1  a resta 
    movwf resta2
    movlw 10        ;Mover valor 10 a W
    subwf resta2, F  ;Restamos W y resta; el resultado va a resta
    btfsc STATUS, 0 ;Si hay acarreo, incrementa decena
    incf decena2	    ;Incrementar decenas
    btfsc STATUS, 0    ;Si hay acarreo, repito el proceso de division
    goto $-5        ;hasta que ya no hayan decenas
    movlw 10        ;se hace para que sea 10+ resta(que tiene valor negativo)
    addwf resta2	    ;entonces se obtienen las unidades restantes
    movf resta2, W   ;mover el resultado de la operacion a "resta"
    movwf unidad2  

;Semaforo 3     
    clrf decena3    ;Limpiar variables 
    clrf unidad3
    clrf resta3
    bcf	STATUS, 0
    movf T_semaforo3, 0    ;Se mueve T_semaforo3  a resta3 
    movwf resta3
    movlw 10        ;Mover valor 10 a W
    subwf resta3, F  ;Restamos W y resta3; el resultado va a resta3
    btfsc STATUS, 0 ;Si hay acarreo, incrementa decena3
    incf decena3	    ;Incrementar decenas3
    btfsc STATUS, 0    ;Si hay acarreo, repito el proceso de division
    goto $-5        ;hasta que ya no hayan decenas
    movlw 10        ;se hace para que sea 10+ resta3(que tiene valor negativo)
    addwf resta3    ;entonces se obtienen las unidades restantes
    movf resta3, W  ;mover el resultado de la operacion a "resta3"
    movwf unidad3
    
;modos     
    clrf decena4    ;Limpiar variables 
    clrf unidad4
    clrf resta4
    bcf	STATUS, 0
    movf T_modo, 0    ;Se mueve T_semaforo3  a resta3 
    movwf resta4
    movlw 10        ;Mover valor 10 a W
    subwf resta4, F  ;Restamos W y resta3; el resultado va a resta3
    btfsc STATUS, 0 ;Si hay acarreo, incrementa decena3
    incf decena4	    ;Incrementar decenas3
    btfsc STATUS, 0    ;Si hay acarreo, repito el proceso de division
    goto $-5        ;hasta que ya no hayan decenas
    movlw 10        ;se hace para que sea 10+ resta3(que tiene valor negativo)
    addwf resta4    ;entonces se obtienen las unidades restantes
    movf resta4, W  ;mover el resultado de la operacion a "resta3"
    movwf unidad4
    
    return //</editor-fold>
semaforos:    
    btfsc   bandera_sem, 0
    goto    sema1_titileo
    
    btfsc   bandera_sem, 1
    goto    sema1_amarillo

    btfsc   bandera_sem, 2
    goto    sema1_rojo    
    
    btfsc   bandera_sem, 3
    goto    sema2_titileo    
    
    btfsc   bandera_sem, 4
    goto    sema2_amarillo
    
    btfsc   bandera_sem, 5
    goto    sema2_rojo     
    
    btfsc   bandera_sem, 6
    goto    sema3_titileo   

    btfsc   bandera_sem, 7
    goto    sema3_amarillo
;--------------------------subrutinas para semaforo 1---------------------------
//<editor-fold defaultstate="collapsed" desc="semaforo 1 configuracion">    
;------------subrutina para encneder la luz verde de semaforo 1-----------------    
sema1_verde: 
    ;configuracion incial para todos los semaforos
    bcf	    STATUS, 2
    bcf	    PORTA, 0	; Rojo semaforo 1 apagado
    bcf	    PORTA, 1	; Amarillo semaforo 1 apagado
    bsf	    PORTA, 2	; Verde semaforo 1 encendido
    bsf	    PORTA, 3	; Rojo semaforo 2 apagado
    bcf	    PORTA, 4	; Amarillo semaforo 2 encendido
    bcf	    PORTA, 5	; Verde semaforo 2 apagado
    bsf	    PORTA, 6	; Rojo semaforo 3 encendido
    bcf	    PORTA, 7	; Amarillo semaforo 3 apagado
    bcf	    PORTB, 4	; Verde semaforo 3 apagado
    
    movlw   6	
    subwf   T_semaforo1, W ;se revisa si quedan 6 segundos de via
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+3
    bcf	    PORTA, 2	;Cuando ya se alcanzaron los 6 seg, la led verde titilea
    bsf	    bandera_sem, 0
    return
;-------------subrutina para titilear luz verde de semaforo 1-------------------    
sema1_titileo:
    bsf	    PORTA, 2	;Se vuelve a encneder la luz verde de semaforo1
    call    delay_1	;Se hace un delay para dar efecto de parpadeo
    bcf	    PORTA, 2	;Se apaga la luz verde de semaforo1
    movlw   3
    subwf   T_semaforo1, W ;Revisamos si quedan 3 segundos de tiempo
    btfss   STATUS, 2	;si quedan mas de 3 segundos, se repite el proceso
    goto    $+4	    ;una vez que quedan 3 segundos, se apaga bandera_sem,0
    bcf	    bandera_sem, 0  ;asi ya no se reite el proceso
    bsf	    bandera_sem, 1  ;procedemos a colocar en amarillo el semaforo 1.
    bsf	    PORTA, 1 ;se enciende la luz amarilla
    return
;------subrutina para encender luz amarilla y al terminar encender luz roja-----
;---------------------------------del semaforo 1--------------------------------    
sema1_amarillo:
    movlw   0
    subwf   T_semaforo1, W ;se revisa si acaba el tiempo de via
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+5	;mientras no se acabe el tiempo de via, me quedo en loop
    bcf	    bandera_sem, 1 ;asi ya no se reite el proceso
    bsf	    bandera_sem, 2 ;procedemos a colocar en rojo el semaforo 1.
    bcf	    PORTA, 1 ;apago el led amarillo
    call    tope ;el semaforo 1 ahora tiene un nuevo tiempo de espera
    return//</editor-fold>

;--------------------------subrutinas para semaforo 2---------------------------    
//<editor-fold defaultstate="collapsed" desc="semaforo 2 configuracion">
;------------------se enciende la luz verde del semaforo 2----------------------   
sema1_rojo:
    bsf	    PORTA, 5	;se enciende el led verde del semaforo 2
    bcf	    PORTA, 3	;se apaga la luz roja del semaforo 2
    movlw   6	
    subwf   T_semaforo2, W ;se revisa si quedan 6 segundos de via en semaforo 2
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+4
    bcf	    PORTA, 5	;Cuando ya se alcanzaron los 6 seg, la led verde titilea
    bcf	    bandera_sem, 2 ;asi ya no se reite el proceso
    bsf	    bandera_sem, 3 ;se procede a hacer titilear luz verde de semaforo 2
    return
;-------------subrutina para titilear luz verde de semaforo 2-------------------    
sema2_titileo:
    bsf	    PORTA, 5	;Se vuelve a encneder la luz verde de semaforo 2
    call    delay_1	;Se hace un delay para dar efecto de parpadeo
    bcf	    PORTA, 5	;Se apaga la luz verde de semaforo 2
    movlw   3
    subwf   T_semaforo2, W ;Revisamos si quedan 3 segundos de tiempo
    btfss   STATUS, 2	;si quedan mas de 3 segundos, se repite el proceso
    goto    $+4	    ;una vez que quedan 3 segundos, se apaga bandera_sem,0
    bcf	    bandera_sem, 3  ;asi ya no se reite el proceso
    bsf	    bandera_sem, 4  ;procedemos a colocar en amarillo el semaforo 2.
    bsf	    PORTA, 4 ;se enciende la luz amarilla de semaforo 2
    return
;------subrutina para encender luz amarilla y al terminar encender luz roja-----
;---------------------------------del semaforo 2--------------------------------    
sema2_amarillo:
    movlw   0
    subwf   T_semaforo2, W ;se revisa si acaba el tiempo de via semaforo 2
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+5	;mientras no se acabe el tiempo de via, me quedo en loop
    bcf	    bandera_sem, 4 ;asi ya no se reite el proceso
    bsf	    bandera_sem, 5 ;procedemos a colocar en rojo el semaforo 2.
    bcf	    PORTA, 4 ;apago el led amarillo
    call    tope ;el semaforo 2 ahora tiene un nuevo tiempo de espera
    return//</editor-fold>
    
;--------------------------subrutinas para semaforo 3---------------------------    
;------------------se enciende la luz verde del semaforo 3----------------------   
//<editor-fold defaultstate="collapsed" desc="Semaforo 3 configuracion">
sema2_rojo:
    bsf	    PORTB, 0	;se enciende el led verde del semaforo 3
    bcf	    PORTA, 6	;apagar luz roja de semaforo 3
    movlw   6	
    subwf   T_semaforo3, W ;se revisa si quedan 6 segundos de via en semaforo 3
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+4
    bcf	    PORTB, 0	;Cuando ya se alcanzaron los 6 seg, la led verde titilea
    bcf	    bandera_sem, 5 ;asi ya no se reite el proceso
    bsf	    bandera_sem, 6 ;se procede a hacer titilear luz verde de semaforo 2
    return
;-------------subrutina para titilear luz verde de semaforo 3-------------------    
sema3_titileo:
    bsf	    PORTB, 0	;Se vuelve a encneder la luz verde de semaforo 3
    call    delay_1	;Se hace un delay para dar efecto de parpadeo
    bcf	    PORTB, 0	;Se apaga la luz verde de semaforo 3
    movlw   3
    subwf   T_semaforo3, W ;Revisamos si quedan 3 segundos de tiempo
    btfss   STATUS, 2	;si quedan mas de 3 segundos, se repite el proceso
    goto    $+4	    ;una vez que quedan 3 segundos, se apaga bandera_sem,6
    bcf	    bandera_sem, 6  ;asi ya no se reite el proceso
    bsf	    bandera_sem, 7  ;procedemos a colocar en amarillo el semaforo 3.
    bsf	    PORTA, 7 ;se enciende la luz amarilla de semaforo 3
    return
;------subrutina para encender luz amarilla y al terminar encender luz roja-----
;---------------------------------del semaforo 2--------------------------------    
sema3_amarillo:
    movlw   0
    subwf   T_semaforo3, W ;se revisa si acaba el tiempo de via semaforo 2
    btfss   STATUS, 2	;Si quedan mas de 6 segundos, se repite la operacion
    goto    $+4	;mientras no se acabe el tiempo de via, me quedo en loop
    bcf	    bandera_sem, 7 ;asi ya no se reite el proceso
    bcf	    PORTA, 7 ;apago el led amarillo
    call    tope ;el semaforo 3 ahora tiene un nuevo tiempo de espera
    return//</editor-fold>

    
//<editor-fold defaultstate="collapsed" desc="tope de semaforos">
tope:
    ;semaforo 1
    movlw   0	
    subwf   T_semaforo1, W ;revisar cuando el semaforo 1 llegue a 0
    btfsc   STATUS, 2	;cuando el semaforo llegue a 0 
    call    nuevo_tiempo1 
    movwf   T_semaforo1, F ;cargar nuevo tiempo a semaforo 1
    bsf	    PORTA, 0	;prender led rojo semaforo 1
    ;semaforo 2 
    movlw   0
    subwf   T_semaforo2, W ;revisar cuando el semaforo 2 llegue a 0
    btfsc   STATUS, 2
    call    nuevo_tiempo2
    movwf   T_semaforo2, F
    bsf	    PORTA, 3	;prender led rojo semaforo 2
    ;semaforo 3
    movlw   0
    subwf   T_semaforo3, W ;
    btfsc   STATUS, 2
    call    nuevo_tiempo3
    movwf   T_semaforo3, F
    bsf	    PORTA, 6	;prender led rojo  
    
    ;semaforo 1 nuevo tiempo
nuevo_tiempo1:  ;sumar los tiempos de semaforo 2 y 3 
    movf    T_semaforo2, W
    addwf   T_semaforo3, W
    return
    ;semaforo 2 nuevo tiempo
nuevo_tiempo2: ;sumar los tiempos de semaforo 1 y 3 
    movf    T_semaforo1, W
    addwf   T_semaforo3, W
    return    
    ;semaforo 3 nuevo tiempo
nuevo_tiempo3: ;sumar los tiempos de semaforo 1 y 2 
    movf    T_semaforo1, W
    addwf   T_semaforo2, W
    return  //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="delay de titileo">
;un delay de 'como de 200ms' 500ms    
delay_1:
    movlw   249		    ;valor inicial del contador
    movlw   cont_small	
    decfsz  cont_small, F   ;decrementar contador
    goto    $-1		    ;ejecutar linea anterior
    return//</editor-fold>
 
//<editor-fold defaultstate="collapsed" desc="Displays de modos">
;multi_modos:
  
    ;return
   //</editor-fold>
   
;-------------------------seleccion de puertos----------------------------------
//<editor-fold defaultstate="collapsed" desc="Configuracion de puertos">
port_io:
    banksel	ANSEL	    ;Se selecciona el banco 3
    clrf	ANSEL	    ;Digital I/O
    clrf	ANSELH
   
    banksel	TRISA	    ;banco 1
   
    clrf	TRISA 
    movlw	00001110B
    movwf	TRISB
    clrf	TRISC	    ;puerto c como salida
    clrf	TRISD	    ;primeras dos terminales de D como salidas
    clrf	TRISE
    
    banksel	PORTA	    ;Se limpian todos los puertos  
    clrf	PORTA
    clrf	PORTB
    clrf	PORTC   
    clrf	PORTD
    clrf	PORTE
    return//</editor-fold>
;------------------------------configuraciones----------------------------------    
//<editor-fold defaultstate="collapsed" desc="configuracion reloj">
conf_reloj:
    banksel OSCCON
    bcf	    IRCF0	;el oscilador interno esta a una frecuencia de 1MHz
    bcf	    IRCF1
    bsf	    IRCF2
    bsf	    SCS		;Utilizamos reloj interno 
    return//</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="configuracion timer 0">
conf_timer0:
    banksel TRISA
    bcf	    T0CS	;ciclo de intruccion FOSC/4
    bcf	    PSA		;El preescalar se asigna a timer0
    bsf	    PS0		;valor del preescalar de seleccion de bits 1:256
    bsf	    PS1
    bsf	    PS2
    
    banksel PORTA
    limpiar_timer0
    return//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="configuracion timer 1">

conf_timer1:
    banksel PORTA	;timer oon interrupcion cada segundo
    bsf	    TMR1ON	;se ahbilita el timer1
    bsf	    T1CKPS0	;preesccalar 1:8
    bsf	    T1CKPS1
    
    banksel PORTA
    limpiar_timer1
    return//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="configuracion de interrupciones">
conf_int_Enbale:
    banksel PORTA
    bsf	    GIE		;Interrupciones globales habilitadas
    bsf	    PEIE
    
    bsf	    T0IE	;Se habilita la interrupcion del TMR0
    bcf	    T0IF	;Bajamos la bandera de TMR0
    
    banksel TRISA
    bsf	    TMR1IE	;interrupciones overflow de TMR1 habilitada
    banksel PORTA
    bcf	    TMR1IF	;Se baja la bandera de overflow de timer1
    return//</editor-fold>
 
END