
/* 
 * WS2812B: Drive several WS2812Bs using SPI and DMA
 * Serial interface is included for debugging purposes
 * and to prove that WS2812B code is tolerant of interrupts
 * in the MCU
*/


#include "stm32f042.h"
#include "serial.h"
#include "spi.h"

uint8_t DMABuffer[27]; // This will be used as a DMA Buffer for the SPI bus

void delay(int);

void delay(int dly)
{
  while( dly--);
}
void configPins()
{
	// Power up PORTB
	RCC_AHBENR |= BIT18;
	GPIOB_MODER |= BIT6; // make bit3  an output
	GPIOB_MODER &= ~BIT7; // make bit3  an output	
}	

void writeDMABuffer(int DeviceNumber, unsigned long Value)
{
    // have to expand each bit to 3 bits
    // Can then output 110 for WS2812B logic '1'
    // and 100 for WS2812B logic '0'
    uint32_t Encoding=0;
    uint8_t SPI_Data[9];
    int Index;
    
    // Process the GREEN byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & BIT23)
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;
        
    }    
    SPI_Data[0] = ((Encoding >> 16) & 0xff);
    SPI_Data[1] = ((Encoding >> 8) & 0xff);
    SPI_Data[2] = (Encoding & 0xff);
    
    // Process the RED byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & BIT23)
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;
        
    }    
    SPI_Data[3] = ((Encoding >> 16) & 0xff);
    SPI_Data[4] = ((Encoding >> 8) & 0xff);
    SPI_Data[5] = (Encoding & 0xff);
    
    // Process the BLUE byte
    Index=0;
    Encoding=0;
    while (Index < 8)
    {
        Encoding = Encoding << 3;
        if (Value & BIT23)
        {
            Encoding |= 0b110;
        }
        else
        {
            Encoding |= 0b100;
        }
        Value = Value << 1;
        Index++;
        
    }    
    SPI_Data[6] = ((Encoding >> 16) & 0xff);
    SPI_Data[7] = ((Encoding >> 8) & 0xff);
    SPI_Data[8] = (Encoding & 0xff);
    
    Index=0;
	while(Index < 9)
    {
		DMABuffer[Index+DeviceNumber*9]=SPI_Data[Index];
		Index++;		
	}              
}
unsigned long getRainbow()
{   // Cycle through the colours of the rainbow (non-uniform brightness however)
	// Inspired by : http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/
	static unsigned Red = 255;
	static unsigned Green = 0;
	static unsigned Blue = 0;
	static int State = 0;
	switch (State)
	{
		case 0:{
			Green++;
			if (Green == 255)
				State = 1;
			break;
		}
		case 1:{
			Red--;
			if (Red == 0)
				State = 2;
			break;
		}
		case 2:{
			Blue++;
			if (Blue == 255)
				State = 3;			
			break;
		}
		case 3:{
			Green--;
			if (Green == 0)
				State = 4;
			break;
		}
		case 4:{
			Red++;
			if (Red == 255)
				State = 5;
			break;
		}
		case 5:{
			Blue --;
			if (Blue == 0)
				State = 0;
			break;
		}		
	}
	return (Green << 16) + (Red << 8) + Blue;
}
void latchWS2812B()
{
	delay(300); // This is about 80us at this clock speed; enough to
			    // be considered as a latch signal by the WS2812B's	
}			   
int main()
{
	unsigned Index=0;
	unsigned Colour[3];
	unsigned swap;	
	initUART(9600);  // Set serial port to 9600,n,8,1
	configPins(); // Set up the pin to drive the onboard LDE
	initSPI(); // set up the SPI bus
    Colour[0]=0xff0000;
    Colour[1]=0x00ff00;
    Colour[2]=0x0000ff;
    
	while(1)
	{	
		
		// Output a long serial message to ensure interrupts are 
		// happening at the same time as DMA transfers
		eputs("Interrupts are happening!\r\n");  
		writeDMABuffer(0,Colour[0]); // Output a colour Format: GGRRBB					
		writeDMABuffer(1,Colour[1]); // Output a colour Format: GGRRBB					
		writeDMABuffer(2,Colour[2]); // Output a colour Format: GGRRBB)
		// Now send out the bits to the SPI bus
		writeSPI(DMABuffer,sizeof(DMABuffer));
		latchWS2812B(); // latch the values to the LED's
	
	    Index++;
	    // The following causes each of the WS2812B's to ramp up
	    // one colour component to maximum and then reset again
	    if (Index > 5) 
	    {
			Index = 0;
			swap=Colour[2];
			Colour[2]=Colour[1];
			Colour[1]=Colour[0];
			Colour[0]=swap;
		}
		while(usart_tx_busy()); // wait for last Serial TX to finish
	} 
	return 0;
}








