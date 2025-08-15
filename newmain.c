/*
 * Sistema de monitoreo ambiental con PIC
 * - Sensor DHT11 (Temperatura y Humedad)
 * - LDR (Sensor de luz)
 * - Visualización en LCD
 * - Control de LED basado en luz ambiental
 */

#include <xc.h>         // Librería para el microcontrolador PIC
#include <stdio.h>      // Librería estándar de entrada/salida
#include <string.h>     // Librería para manejo de cadenas

#define _XTAL_FREQ 8000000UL  // Frecuencia del oscilador: 8 MHz

// =============================================================================
// CONFIGURACIÓN DEL MICROCONTROLADOR
// =============================================================================

#pragma config FOSC = HS    // Oscilador de alta velocidad
#pragma config WDTE = OFF   // Watchdog Timer desactivado
#pragma config PWRTE = OFF  // Power-up Timer desactivado
#pragma config BOREN = ON   // Brown-out Reset activado
#pragma config LVP = OFF    // Programación en bajo voltaje desactivada
#pragma config CPD = OFF    // Protección de memoria EEPROM desactivada
#pragma config WRT = OFF    // Protección de escritura en memoria Flash desactivada
#pragma config CP = OFF     // Protección de código en memoria Flash desactivada

// =============================================================================
// DEFINICIÓN DE PINES Y VARIABLES GLOBALES
// =============================================================================

#define LED RD0               // LED conectado a RD0
#define DHT11_DATA RB1        // Pin de datos del DHT11 en RB1

// Variables globales
unsigned char TOUT = 0;       // Flag para timeout
unsigned char CheckSum;       // Checksum de verificación DHT11
unsigned char i;              // Variable contador
unsigned char T_Byte1, T_Byte2; // Bytes de temperatura
unsigned char RH_Byte1, RH_Byte2; // Bytes de humedad relativa
char message[] = "00.0";      // Buffer para mostrar valores en LCD

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================

unsigned int adc_read(void);          // Lectura del ADC
void StartSignal(void);               // Inicia comunicación con DHT11
unsigned char CheckResponse(void);    // Verifica respuesta del DHT11
unsigned char ReadByte(void);         // Lee un byte del DHT11
void lcd_init(void);                  // Inicializa el LCD
void lcd_cmd(unsigned char cmd);      // Envía comando al LCD
void lcd_data(unsigned char dat);     // Envía dato al LCD
void lcd_out(unsigned char row, unsigned char col, const char *str); // Escribe en LCD

// =============================================================================
// FUNCIÓN PRINCIPAL
// =============================================================================

void main(void) {
    // Configuración de puertos
    TRISD = 0x00;       // PORTD como salida (LED)
    TRISA0 = 1;         // RA0 como entrada (LDR)
    TRISB = 0b00000010; // RB1 como entrada (DHT11), otros como salida (LCD)
    
    // Configuración del ADC para el LDR
    ADCON1 = 0xC0;      // Justificado a la derecha, AN0 como analógico
    ADCON0 = 0x81;      // ADC encendido, Fosc/64
    
    // Configuración del DHT11
    CMCON = 7;          // Comparadores desactivados
    INTCONbits.GIE = 1;     // Interrupciones globales habilitadas
    INTCONbits.PEIE = 1;    // Interrupciones periféricas habilitadas
    PIE1bits.TMR2IE = 1;    // Interrupción del Timer2 habilitada
    T2CON = 0;          // Timer2 apagado, prescaler 1:1
    PIR1bits.TMR2IF = 0;    // Bandera de interrupción del Timer2 limpiada
    
    // Inicialización del LCD
    lcd_init();
    lcd_cmd(0x01);      // Limpiar LCD
    lcd_cmd(0x0C);      // Cursor apagado
    lcd_out(1, 1, "System Starting");
    lcd_out(2, 1, "Please wait...");
    __delay_ms(2000);   // Espera 2 segundos
    
    // Bucle principal infinito
    while(1) {
        // Lectura del LDR y control del LED
        unsigned int ldr_value = adc_read();
        if(ldr_value > 150) {
            LED = 0;    // LED OFF cuando hay mucha luz
        } else {
            LED = 1;    // LED ON cuando está oscuro
        }
        
        // Lectura del DHT11
        PIE1bits.TMR2IE = 1;  // Habilitar interrupción del Timer2
        StartSignal();         // Enviar señal de inicio al DHT11
        
        if(CheckResponse()) {  // Verificar si el DHT11 responde
            // Leer los 5 bytes de datos (2 humedad, 2 temp, 1 checksum)
            RH_Byte1 = ReadByte();
            RH_Byte2 = ReadByte();
            T_Byte1 = ReadByte();
            T_Byte2 = ReadByte();
            CheckSum = ReadByte();
            
            // Verificar checksum
            if(CheckSum == ((RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2) & 0xFF)) {
                // Mostrar humedad en el LCD
                lcd_cmd(0x01);  // Limpiar LCD
                lcd_out(1, 1, "Humidity:");
                message[0] = (char)(RH_Byte1/10 + 48); // Decenas
                message[1] = (char)(RH_Byte1%10 + 48);  // Unidades
                message[3] = (char)(RH_Byte2%10 + 48);  // Decimal
                lcd_out(1, 11, message);
                lcd_out(1, 15, "%");
                
                // Mostrar temperatura en el LCD
                lcd_out(2, 1, "Temp:");
                message[0] = (char)(T_Byte1/10 + 48);  // Decenas
                message[1] = (char)(T_Byte1%10 + 48);  // Unidades
                message[3] = (char)(T_Byte2%10 + 48);  // Decimal
                lcd_out(2, 7, message);
                lcd_data(0xDF); // Símbolo de grados
                lcd_out(2, 11, "C");
                
                // Mostrar estado de la luz
                lcd_out(2, 13, (ldr_value > 150) ? "Light" : "Dark ");
            } else {
                // Error de checksum
                lcd_cmd(0x01);
                lcd_out(1, 1, "Checksum Error");
                __delay_ms(500);
            }
        } else {
            // Error de comunicación con DHT11
            lcd_cmd(0x01);
            lcd_out(1, 1, "DHT11 Error");
            lcd_out(2, 1, "Retrying...");
            __delay_ms(500);
        }
        PIE1bits.TMR2IE = 0;  // Deshabilitar interrupción del Timer2
        __delay_ms(1500);      // Esperar 1.5 segundos entre lecturas
    }
}

// =============================================================================
// FUNCIONES PARA EL DHT11
// =============================================================================

void StartSignal() {
    TRISB1 = 0;     // Configurar RB1 como salida
    DHT11_DATA = 0; // Poner línea en bajo
    __delay_ms(18); // Esperar 18ms (pulso de inicio)
    DHT11_DATA = 1; // Poner línea en alto
    __delay_us(30); // Esperar 30?s
    TRISB1 = 1;     // Configurar RB1 como entrada
}

unsigned char CheckResponse() {
    TOUT = 0;       // Resetear flag de timeout
    TMR2 = 0;       // Resetear Timer2
    T2CONbits.TMR2ON = 1;  // Encender Timer2
    
    // Esperar mientras DHT11 mantiene la línea baja
    while(!DHT11_DATA && !TOUT);
    if (TOUT) return 0; // Timeout ocurrido
    
    // Esperar mientras DHT11 mantiene la línea alta
    TMR2 = 0;       // Resetear Timer2
    while(DHT11_DATA && !TOUT);
    if (TOUT) return 0; // Timeout ocurrido
    
    T2CONbits.TMR2ON = 0; // Apagar Timer2
    return 1;        // Respuesta correcta
}

unsigned char ReadByte() {
    unsigned char num = 0;
    TRISB1 = 1;      // Asegurar que RB1 es entrada
    
    for (i=0; i<8; i++) {
        while(!DHT11_DATA);  // Esperar inicio del bit
        __delay_us(40);      // Esperar 40?s
        
        // Si después de 40?s la línea sigue alta, es un 1
        if(DHT11_DATA)       
            num |= (1 << (7-i)); // Almacenar el bit
        
        while(DHT11_DATA);   // Esperar fin del bit
    }
    return num; // Retornar byte leído
}

// =============================================================================
// RUTINA DE INTERRUPCIÓN
// =============================================================================

void __interrupt() ISR(void) {
    if(PIR1bits.TMR2IF) {  // Verificar si es interrupción del Timer2
        TOUT = 1;          // Activar flag de timeout
        T2CONbits.TMR2ON = 0; // Apagar Timer2
        PIR1bits.TMR2IF = 0;  // Limpiar bandera de interrupción
    }
}

// =============================================================================
// FUNCIONES PARA EL LCD
// =============================================================================

void lcd_init() {
    TRISB = 0x00;  // Todos los pines del LCD como salidas
    __delay_ms(15); // Esperar 15ms para inicialización
    
    // Configurar LCD en modo 8 bits, 2 líneas, 5x7 puntos
    lcd_cmd(0x38);
    __delay_ms(5);
    lcd_cmd(0x38);
    __delay_us(100);
    lcd_cmd(0x38);
    
    lcd_cmd(0x0C); // Display ON, cursor OFF
    lcd_cmd(0x01); // Limpiar display
    __delay_ms(2);
    lcd_cmd(0x06); // Auto-incremento del cursor
}

void lcd_cmd(unsigned char cmd) {
    PORTB = cmd;    // Enviar comando al puerto
    RB2 = 0;        // RS = 0 (modo comando)
    RB3 = 1;        // EN = 1 (habilitar)
    __delay_us(10); // Esperar 10?s
    RB3 = 0;        // EN = 0 (deshabilitar)
    __delay_ms(2);  // Esperar 2ms
}

void lcd_data(unsigned char dat) {
    PORTB = dat;    // Enviar dato al puerto
    RB2 = 1;        // RS = 1 (modo dato)
    RB3 = 1;        // EN = 1 (habilitar)
    __delay_us(10); // Esperar 10?s
    RB3 = 0;        // EN = 0 (deshabilitar)
    __delay_us(100); // Esperar 100?s
}

void lcd_out(unsigned char row, unsigned char col, const char *str) {
    unsigned char address;
    
    // Calcular dirección según fila
    if(row == 1) address = 0x80 + col - 1; // Primera fila
    else address = 0xC0 + col - 1;         // Segunda fila
    
    lcd_cmd(address); // Posicionar cursor
    
    // Escribir cada carácter de la cadena
    while(*str) lcd_data(*str++);
}

// =============================================================================
// FUNCIÓN PARA LEER EL ADC (LDR)
// =============================================================================

unsigned int adc_read() {
    ADCON0bits.GO = 1;       // Iniciar conversión ADC
    while(ADCON0bits.GO);    // Esperar hasta que la conversión termine
    return ((ADRESH << 8) + ADRESL); // Retornar valor de 10 bits
}