#include <stdint.h>
#include "stm32f042.h"

void initSPI()
{
    // Turn on the clock for the SPI interface
    RCC_APB2ENR |= BIT12;
    // Turn on PORT A
	RCC_AHBENR |= BIT17;
    // Configure the pins
    // SPI1 MOSI is on pin 13 (PA7) AF0.  This is labelled A6 on the Nucleo board
    GPIOA_MODER |= BIT15;
    GPIOA_MODER &= ~BIT14;
    GPIOA_AFRL &= ~(BIT31+BIT30+BIT29+BIT28);
    // Start with zero in the control registers.
    SPI1_CR1 = SPI1_CR2 = 0;
    SPI1_CR1 |= BIT0; // set CPHA to ensure MOSI is low when idle  
    SPI1_CR1 |= BIT2; // select master mode
    SPI1_CR1 |= (3 << 3); // select divider of 16.  48MHz/16 = 3MHz.  3 bits per WS2812 bit so: 1 Million WSbits/second : Within range of  1.53MHz to 540kHz    
    SPI1_CR1 |= BIT8+BIT9; // select software slave management and drive SSI high (not relevant in TI mode and not output to pins)
    SPI1_CR2 |= (7 << 8); // select 8 bit data transfer size   
    SPI1_CR2 |= BIT1; // enable transmit DMA
    SPI1_CR1 |= BIT6; // enable SPI1
    // DMA configuration: SPI1 TX is on DMA channel 3
    RCC_AHBENR |= BIT0; // enable clocks for DMA controller
    DMA_CPAR3 = (unsigned long)(&(SPI1_DR));
    DMA_CMAR3 = 0;  // Don't know what to send yet
    DMA_CNDTR3 = 0; // No bytes yet
    // Select high priority, 8 bits, Memory increment mode (only), 	Read from memory, enable
    DMA_CCR3 = BIT13+BIT12+BIT7+BIT4;       
}
void writeSPI(uint8_t *DMABuffer,unsigned Count)
{	
	DMA_CCR3 &= ~BIT0; // disable DMA
	DMA_CNDTR3 = Count; // Tell the DMA controller how many bytes are to be sent
	DMA_CMAR3 = (unsigned long)DMABuffer; // Set DMA source
	DMA_IFCR = 0x0f; // clear any pending interrupts 
	DMA_CCR3 |= BIT0; // re-enable DMA
}

