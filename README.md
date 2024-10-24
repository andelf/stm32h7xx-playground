# STM32H750

Board: https://github.com/WeActStudio/MiniSTM32H7xx

- Blue LED: PE3
- Button: PC13

XTAL: 25MHz, 32.768kHz

## QSPI BANK1

- IO3: PD13
- IO2: PE2
- IO1: PD12
- IO0: PD11
- CLK: PB2
- CS: PB6

## SPI Flash

- MISO: PB4
- MOSI: PD7
- CLK: PB3
- CS: PD6 (SB3 焊接点)

## MicroSD

- D0: PC8
- D1: PC9
- D2: PC10
- D3: PC11
- CK: PC12
- CMD: PD2

## TFT LCD

- SDA(MOSI): PE14
- DC: PE13
- SCL: PE12
- CS: PE11
- LED: PE10 (TIM)

## DVP

DCMI DO -> PC6
DCMI D1 -> PC7
DCMI D2 ->PEO
DCMI D3 > PE1
DCMI D4 -> PE4
DCMI D5 -> PD3
DCMI D6 -> PE5
DCMI D7 -> PE6
DCMI VSYNC -> PB7
DCMI HSYNC ->PA4
DCMI PIXCLK ->PA6
RCC MCO 1 ->PA8
I2C1 SCL-> PB8
I2C1 SDA -> PB9
