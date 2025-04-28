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
void USART2config();
void USART2Init();

//Variables globales
volatile uint8_t lcd_step = 0; //LCD
volatile uint8_t current_menu = 0;  // 0 = Menú Principal, 1 = Menú Nuevo; SERIAL


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

    //4. Inicializar USART
    USART2Init();


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
    TIM2->SR &= ~(1<<0); // Limpiar bandera de actualización (UIF)

    if (current_menu == 0) {
        // Menú principal
        switch (lcd_step) {
            case 0: LCD_command(0x01); LCD_command(0x80); break;
            case 1: LCD_data('1'); break;
            case 2: LCD_data('.'); break;
            case 3: LCD_data('U'); break;
            case 4: LCD_data('s'); break;
            case 5: LCD_data('u'); break;
            case 6: LCD_data('a'); break;
            case 7: LCD_data('r'); break;
            case 8: LCD_data('i'); break;
            case 9: LCD_data('o'); break;
            case 10: LCD_data(' '); break;
            case 11: LCD_data('y'); break;
            case 12: LCD_data(' '); break;
            case 13: LCD_data('P'); break;
            case 14: LCD_data('I'); break;
            case 15: LCD_data('N'); break;
            case 16: LCD_command(0x01); LCD_command(0x80); break;
            case 17: LCD_data('2'); break;
            case 18: LCD_data('.'); break;
            case 19: LCD_data('C'); break;
            case 20: LCD_data('e'); break;
            case 21: LCD_data('l'); break;
            case 22: LCD_data('u'); break;
            case 23: LCD_data('l'); break;
            case 24: LCD_data('a'); break;
            case 25: LCD_data('r'); break;
            case 26: LCD_command(0x01); LCD_command(0x80); break;
            case 27: LCD_data('3'); break;
            case 28: LCD_data('.'); break;
            case 29: LCD_data('S'); break;
            case 30: LCD_data('e'); break;
            case 31: LCD_data('r'); break;
            case 32: LCD_data('i'); break;
            case 33: LCD_data('a'); break;
            case 34: LCD_data('l'); break;
            case 35: LCD_command(0x01); LCD_command(0x80); break;
            case 36: lcd_step = 0; break; // Reiniciar al menú principal
            default: break;
        }
    }
    else if (current_menu == 1) {
        // Menú secundario (cuando recibes '3')
        switch (lcd_step) {
            case 0: LCD_command(0x01); LCD_command(0x80); break;
            case 1: LCD_data('1'); break;
            case 2: LCD_data('.'); break;
            case 3: LCD_data('R'); break;
            case 4: LCD_data('e'); break;
            case 5: LCD_data('t'); break;
            case 6: LCD_data('i'); break;
            case 7: LCD_data('r'); break;
            case 8: LCD_data('o'); break;
            case 9: LCD_data(' '); break;
            case 10: LCD_data('s'); break;
            case 11: LCD_data('i'); break;
            case 12: LCD_data('n'); break;
            case 13: LCD_data(' '); break;
            case 14: LCD_data('P'); break;
            case 15: LCD_data('I'); break;
            case 16: LCD_data('N'); break;
            case 17: LCD_command(0x01); LCD_command(0x80); break;
            case 18: LCD_data('2'); break;
            case 19: LCD_data('.'); break;
            case 20: LCD_data('E'); break;
            case 21: LCD_data('n'); break;
            case 22: LCD_data('v'); break;
            case 23: LCD_data('i'); break;
            case 24: LCD_data('o'); break;
            case 25: LCD_data(' '); break;
            case 26: LCD_data('r'); break;
            case 27: LCD_data('e'); break;
            case 28: LCD_data('m'); break;
            case 29: LCD_data('e'); break;
            case 30: LCD_data('s'); break;
            case 31: LCD_data('a'); break;
            case 32: LCD_data('s'); break;
            case 33: LCD_command(0x01); LCD_command(0x80); break;
            case 34: LCD_data('3'); break;
            case 35: LCD_data('.'); break;
            case 36: LCD_data('C'); break;
            case 37: LCD_data('o'); break;
            case 38: LCD_data('n'); break;
            case 39: LCD_data('s'); break;
            case 40: LCD_data('u'); break;
            case 41: LCD_data('l'); break;
            case 42: LCD_data('t'); break;
            case 43: LCD_data('a'); break;
            case 44: LCD_data(' '); break;
            case 45: LCD_data('s'); break;
            case 46: LCD_data('a'); break;
            case 47: LCD_data('l'); break;
            case 48: LCD_data('d'); break;
            case 49: LCD_data('o'); break;
            case 50: lcd_step = 0; break; // Reiniciar al nuevo menú
            default: break;
        }
    }

    lcd_step++; // ⚡ Incrementar SOLO al final
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


//Funciones de USART2
void USART2Init(void) {
	RCC->APB1ENR |= (1<<17); //USART CLK ENABLE
	RCC->IOPENR |= (1<<0); //GPIOA CLK ENABLE
	//ALTERNATE FUNCTION PA2(TX) Y PA3(RX)
	GPIOA->MODER &= ~(1<<4);  //PA2 as AF
	GPIOA->MODER &= ~(1<<6);  //PA3 as AF
	GPIOA->AFR[0] |= (1<<10); //PA2 AS  AF4
	GPIOA->AFR[0] |= (1<<14); //PA3 AS AF4
	USART2->BRR = 139; //USART2 @115200 bps with 16Mhz clock HSi
	USART2->CR1 = 0;     // Apagar USART2 primero
	USART2->ICR = 0xFFFFFFFF; // Limpiar cualquier error previo
	USART2->CR1 |= (1<<2) | (1<<3); // Habilitar RX y TX
	USART2->CR1 |= (1<<5);          // Habilitar interrupción por RXNE
	USART2->CR1 |= (1<<0);          // Habilitar USART2
	NVIC_EnableIRQ(USART2_IRQn);    // Habilitar interrupción en NVIC
}

void USART2_write (uint8_t ch)
{
	while (!(USART2->ISR & 0X0080)){}
	USART2->TDR = ch;
}

uint8_t USART2_read (void)
{
	while( !(USART2->ISR & 0x0020) ){}
	return USART2->RDR;
}

void USART2_Putstring(uint8_t* stringptr) {
	while(*stringptr != 0x00) {
		USART2_write(*stringptr);
		stringptr++;
	}
}

void USART2_PutstringE(uint8_t* stringptr) {
	while(*stringptr != 0x00) {
		USART2_write(*stringptr);
		stringptr++;
	}
	USART2_write(0x0A);
	USART2_write(0x0D);
}

void USART2_IRQHandler(void) {
    if (USART2->ISR & (1<<5)) { // RXNE (dato recibido)
    	uint8_t received = USART2->RDR;

        if (received == '51') {
        	TIM2->CR1 &= ~(1<<0);    // 🔥 Detener TIM2
        	TIM2->SR &= ~(1<<0);     // 🔥 Limpiar UIF por si acaso
        	current_menu = 1;    // Cambiar de
            lcd_step = 0;
            // Enviar el nuevo menú por serial
            USART2_PutstringE("1. Retiro sin PIN");
            USART2_PutstringE("2. Envio remesas");
            USART2_PutstringE("3. Consulta saldo");
            TIM2->CR1 |= (1<<0);
        }
    }
}



