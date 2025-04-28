#include "stm32l053xx.h"

//Definiciones para LCD
#define RS 0x01  // PC0
#define RW 0x02  // PC1
#define EN 0x08  // PC3

//Variables para display 7  segmentos
volatile uint8_t display_index = 0;
#define NUM_0 (0x3F)  //0
#define NUM_1 (0x06)  //1
#define NUM_2 (0x5B)  //2
#define NUM_3 (0x4F)  //3
#define NUM_4 (0x66)  //4
#define NUM_5 (0x6D)  //5
#define NUM_6 (0x7D)  //6
#define NUM_7 (0x07)  //7
#define NUM_8 (0x7F)  //8
#define NUM_9 (0x67)  //9

//Funciones a utilizar
void delay_ms(uint32_t n);
void LCD_command(unsigned char command);
void LCD_data(char data);
void PORTS_init(void);
void TIM2config(void);


//Variables globales
volatile uint8_t lcd_step = 0;


void delay_ms(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        TIM2->CNT = 0;                // Reiniciar contador
        while (!(TIM2->SR & 0x0001));  // Esperar que UIF=1
        TIM2->SR &= ~(1<<0);           // Limpiar UIF
        while (!(TIM2->SR & 0x0001));  // Esperar otra vez
        TIM2->SR &= ~(1<<0);           // Limpiar UIF
    }
}



int main(void) {
	//1. Habilitar HSI 16 MHz como SYSCLK
    RCC->CR |= (1<<0);   // HSI ON
    RCC->CFGR |= (1<<0); // HSI16 como SYSCLK


    //2. Inicializar  puertos y timers
    PORTS_init();   //Configuracion de todos los puertos
    TIM2config();   //Configuracion del TIM2 a  200 ms
    TIM21config();  //Configuracion del TIM21 a 2 ms

    //3. Inicializar LCD
    LCD_INIT();


    //while (1);
}


//Envia comando al LCD
void LCD_command(unsigned char command) {
    GPIOC->BSRR = (RS | RW) << 16; // RS=0, RW=0
    GPIOC->ODR &= ~(0xFF0);
    GPIOC->ODR |= (command << 4);
    GPIOC->BSRR = EN;
    for (volatile int i = 0; i < 800; i++) __NOP();  // Pausa de 1ms
    GPIOC->BSRR = EN << 16;
    for (volatile int i = 0; i < 1600; i++) __NOP(); // Pausa de 2ms
}

void LCD_data(char data) {
    GPIOC->BSRR = RS;      // RS=1
    GPIOC->BSRR = RW << 16; // RW=0
    GPIOC->ODR &= ~(0xFF0);
    GPIOC->ODR |= (data << 4);
    GPIOC->BSRR = EN;
    for (volatile int i = 0; i < 800; i++) __NOP();  // Pausa de 1ms
    GPIOC->BSRR = EN << 16;
    for (volatile int i = 0; i < 1600; i++) __NOP(); // Pausa de 2ms
}

void PORTS_init(void) {
    RCC->IOPENR |= (1<<2); // Habilitar GPIOC y los pines para el LCD
    for (int i = 0; i <= 11; i++) {
        GPIOC->MODER &= ~(0x3 << (2*i));
        GPIOC->MODER |=  (0x1 << (2*i));
    }

    RCC->IOPENR |= (1<<0); //Enable clock GPIOA
    RCC->IOPENR |= (1<<1); //Enable clock GPIOB

    GPIOB->MODER &= ~(1<<1); //Config PB0 como output Salida  0
	GPIOB->MODER &= ~(1<<3); //Config PB1 como output Salida  1
	GPIOB->MODER &= ~(1<<5); //Config PB2 como output Salida  2
	GPIOB->MODER &= ~(1<<7); //Config PB3 como output Salida  3
	GPIOB->MODER &= ~(1<<9); //Config PB4 como output Salida  4
	GPIOB->MODER &= ~(1<<11); //Config PB5 como output Salida 5
	GPIOB->MODER &= ~(1<<13); //Config PB6 como output Salida 6
	GPIOB->MODER &= ~(1<<15); //Config PB7 como output (Push 1)
	GPIOB->MODER &= ~(1<<17); //Config PB8 como output (Push 2)
	GPIOB->MODER &= ~(1<<19); //Config PB9 como output (Push 3)
	GPIOB->MODER &= ~(1<<21); //Config PB10 como output (Push 4)
	GPIOB->MODER &= ~(1<<23); //Config PB11 como output (Push 4)
	GPIOB->MODER &= ~(1<<25); //Config PB12 como output (Push 4)

}

void LCD_INIT(void) {
    delay_ms(50);  // Esperar estabilización inicial
    LCD_command(0x30);
    delay_ms(5);
    LCD_command(0x30);
    delay_ms(1);
    LCD_command(0x30);
    delay_ms(1);
    LCD_command(0x38); // 8 bits, 2 líneas
    delay_ms(1);
    LCD_command(0x06); // Modo entrada (incremento)
    delay_ms(1);
    LCD_command(0x01); // Clear Display
    delay_ms(2);
    LCD_command(0x0C); // Display ON, cursor OFF
}


void TIM2config(void) {
    RCC->APB1ENR |= (1<<0);   // Habilitar reloj a TIM2
    TIM2->PSC = 16000 - 1;    // Prescaler: 16 MHz / 16000 = 1 kHz (1ms)
    TIM2->ARR = 200 - 1;        // Cada 200 * 1 ms = 200 ms
    TIM2->CNT = 0;
    TIM2->DIER |= (1<<0);     // Habilitar interrupción
    TIM2->CR1 = (1<<0);       // Enable contador
    NVIC_EnableIRQ(TIM2_IRQn);// Permitir interrupción
}

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~(1<<0); // Limpiar UIF

    switch (lcd_step) {
        case 0:
            LCD_command(0x01);  // Clear Display
            LCD_command(0x80);  // Cursor a línea 1
            lcd_step++;
            break;

        case 1: LCD_data('1'); lcd_step++; break;
        case 2: LCD_data('.'); lcd_step++; break;
        case 3: LCD_data('U'); lcd_step++; break;
        case 4: LCD_data('s'); lcd_step++; break;
        case 5: LCD_data('u'); lcd_step++; break;
        case 6: LCD_data('a'); lcd_step++; break;
        case 7: LCD_data('r'); lcd_step++; break;
        case 8: LCD_data('i'); lcd_step++; break;
        case 9: LCD_data('o'); lcd_step++; break;
        case 10: LCD_data(' '); lcd_step++; break;
        case 11: LCD_data('y'); lcd_step++; break;
        case 12: LCD_data(' '); lcd_step++; break;
        case 13: LCD_data('P'); lcd_step++; break;
        case 14: LCD_data('I'); lcd_step++; break;
        case 15: LCD_data('N'); lcd_step++; break;

        case 16:
            LCD_command(0x01);  // Clear
            LCD_command(0x80);  // Cursor a línea 1
            lcd_step++;
            break;

        case 17: LCD_data('2'); lcd_step++; break;
        case 18: LCD_data('.'); lcd_step++; break;
        case 19: LCD_data('C'); lcd_step++; break;
        case 20: LCD_data('e'); lcd_step++; break;
        case 21: LCD_data('l'); lcd_step++; break;
        case 22: LCD_data('u'); lcd_step++; break;
        case 23: LCD_data('l'); lcd_step++; break;
        case 24: LCD_data('a'); lcd_step++; break;
        case 25: LCD_data('r'); lcd_step++; break;

        case 26:
            LCD_command(0x01);  // Clear
            LCD_command(0x80);  // Cursor a línea 1
            lcd_step++;
            break;

        case 27: LCD_data('3'); lcd_step++; break;
        case 28: LCD_data('.'); lcd_step++; break;
        case 29: LCD_data('S'); lcd_step++; break;
        case 30: LCD_data('e'); lcd_step++; break;
        case 31: LCD_data('r'); lcd_step++; break;
        case 32: LCD_data('i'); lcd_step++; break;
        case 33: LCD_data('a'); lcd_step++; break;
        case 34: LCD_data('l'); lcd_step++; break;

        case 35:
            lcd_step = 0; // Reiniciar para mostrar desde 1 otra vez
            break;

        default:
            break;
    }
}

void TIM21config (void) {
	RCC->APB2ENR |= (1<<2); //Se encuentra operando a 1s
	TIM21->PSC = 16000-1;
	TIM21->ARR = 2-1;
	TIM21->CNT = 0;
	TIM21->CR1 = (1<<0);
	TIM21->DIER |= (1<<0);  //Enable Mode Interrupt
	NVIC_EnableIRQ(TIM21_IRQn);
}

void TIM21_IRQHandler() {
	GPIOB->ODR = 0x0000;   // Apagar los displays antes de escribir
    switch (display_index) {
        case 0:
            GPIOB->ODR |= NUM_0 | (1<<7);  // Número 4 en display 1
            break;
        case 1:
            GPIOB->ODR |= NUM_0 | (1<<8);  // Número 0 en display 2
            break;
        case 2:
            GPIOB->ODR |= NUM_0 | (1<<9);  // Número 4 en display 3
            break;
        case 3:
            GPIOB->ODR |= NUM_0 | (1<<10); // Número 5 en display 4
            break;
        case 4:
            GPIOB->ODR |= NUM_0 | (1<<11); // Número 5 en display 5
            break;
        case 5:
            GPIOB->ODR |= NUM_0 | (1<<12); // Número 5 en display 6
            break;
        default:
            display_index = 0; // Reiniciar ciclo
            return;
    }

    display_index++;
    if (display_index > 5) display_index = 0; // Ciclar
	TIM21->SR &= ~(1<<0);  //Clear UIF flag

}
