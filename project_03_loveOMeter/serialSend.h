/*
 * serialSend.h
 *
 * Created: August 1, 2019
 * Author : Diego
 * Specification for serialSend.c
 */

#ifndef _SERIALSEND

    #define _SERIALSEND

    // give error if CPU frequcency if not defined
    #ifndef F_CPU
    #  error "serialPrint requires F_CPU to be defined"
    #endif
    
    // give error if BAUD rate if not defined
    #ifndef BAUD
    #  error "serialPrint requires BAUD to be defined"
    #endif
    
    // include avr input and output if not defined
    #ifndef _AVR_IO_H_
        #include <avr/io.h>
    #endif
    
    // include setbaud if not defined
    #ifndef UBRR_VALUE 
        #include <util/setbaud.h>
    #endif

// function prototypes
void usart_init();
void serialSendChar( char c );

#endif
