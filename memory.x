MEMORY
{
     /* BANK_1 */
    FLASH : ORIGIN = 0x08000000, LENGTH =  128K
    SPIFLASH :  ORIGIN = 0x90000000, LENGTH =  8M
    RAM   : ORIGIN = 0x24000000, LENGTH =  512K /* SRAM */
}


/* add a new section for SPIFLASH */

SECTIONS
{
    .spiflash : {
        *(.spiflash .spi_flash)
    } > SPIFLASH
}


