#include "xensiv_bgt60trxx_platform.h"
#include "xensiv_bgt60trxx_conf.h"
#include <stdlib.h>
#include <string.h>

void xensiv_bgt60trxx_platform_rst_set(const SPI_HandleTypeDef* iface, bool val){
    HAL_GPIO_WritePin(RST_M_R_GPIO_Port,RST_M_R_Pin,val);
}

void xensiv_bgt60trxx_platform_spi_cs_set(const SPI_HandleTypeDef* iface, bool val){
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,val);
}

int32_t xensiv_bgt60trxx_platform_spi_transfer(const SPI_HandleTypeDef* iface, uint8_t* tx_data, uint8_t* rx_data, uint32_t len){
    HAL_StatusTypeDef commstatus = HAL_OK;

    if(rx_data == NULL){
        commstatus = HAL_SPI_Transmit(iface,tx_data,len,HAL_MAX_DELAY/100);
    }
    else if(tx_data == NULL){
        commstatus = HAL_SPI_Receive (iface, rx_data, len,HAL_MAX_DELAY/100);
    }
    else if (rx_data && tx_data){
        commstatus = HAL_SPI_TransmitReceive(iface, tx_data,rx_data, len,HAL_MAX_DELAY/100);
    }
    else{
        commstatus = HAL_ERROR;
    }
    return commstatus;
    
}

int32_t xensiv_bgt60trxx_platform_spi_fifo_read(SPI_HandleTypeDef* iface, uint8_t* rx_data, uint32_t len){

    HAL_StatusTypeDef commstatus = HAL_OK;
    uint8_t keephigh[N_BYTES];
    // uint32_t numbytes = N_BYTES;
    // uint8_t buffer[numbytes]; //1536 is the number of bytes in a burst when the fifo limit is set to 1024(num of 12 bit samples)
    memset(keephigh, 0xFF, N_BYTES);
    commstatus = HAL_SPI_TransmitReceive_DMA(iface,keephigh,rx_data,N_BYTES);


    return commstatus;
}



void xensiv_bgt60trxx_platform_delay(uint32_t ms){
    HAL_Delay(ms);
}


uint32_t xensiv_bgt60trxx_platform_word_reverse(uint32_t x){ //ARM (little endian) BGT60UTR11AIP (big endian)
    return (((x & 0x000000ffUL) << 24) |
           ((x & 0x0000ff00UL) <<  8) |
           ((x & 0x00ff0000UL) >>  8) |
           ((x & 0xff000000UL) >> 24));
}

void xensiv_bgt60trxx_platform_assert(bool expr){
    if(!expr){
        NVIC_SystemReset();
        while(1){
            HAL_Delay(300);
        }
    }

}