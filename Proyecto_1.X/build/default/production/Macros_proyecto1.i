# 1 "Macros_proyecto1.s"
# 1 "<built-in>" 1
# 1 "Macros_proyecto1.s" 2
;---------aqui estan todos los macros que se pueden utilizar--------------------
limpiar_timer0 macro
    banksel PORTA ;tiempo = 4*1/FOSC * (256-N)*preesclar
    movlw 254 ;Valor N que se carga a TMR0
    movwf TMR0 ;Se mueve el valor al registro de TMR0
    bcf T0IF ;Se baja la bandera de overflow del timer0
    endm

limpiar_timer1 macro;Tarda un segundo la interrupcion
    banksel PORTA ;N = 65536 - tiempo_desbordamiento/(preesclar*1/(Fosc/4))
    movlw 238 ;Valor a cargar en TMR1L para cumplir con 34286
    movwf TMR1L
    movlw 133 ;Valor a cargar en TMR1H para cumplir con 34286
    movwf TMR1H
    bcf TMR1IF ;Limpiar bandera de overflow de timer1
    endm

;limpiar_timer2 macro
; banksel TRISA ;PR2 = Ttmr2IF/(preescalar*posescalar**(1/(FOSC/4))
; movlw 245 ;valor para que TMR2 sume cada 250ms
; movwf PR2 ;Se envia el valor al registro PR2
;
; banksel PORTA
; bcf TMR2IF ;Limpiar bandera de overflow de timer2
; endm

tiempos_semaforos macro
    movlw 10
    movwf T_semaforo1
    movlw 20
    movwf T_semaforo2
    movlw 30
    movwf T_semaforo3
    endm