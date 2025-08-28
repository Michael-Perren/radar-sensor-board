#include "xensiv_bgt60trxx_platform.h"
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
    //xensiv_bgt60trxx_platform_spi_cs_set(iface,false);
    
    if(rx_data == NULL){
        commstatus = HAL_SPI_Transmit(iface,tx_data,len,HAL_MAX_DELAY);
    }
    else if(tx_data == NULL){
        commstatus = HAL_SPI_Receive (iface, rx_data, len, HAL_MAX_DELAY);
    }
    else if (rx_data && tx_data){
        commstatus = HAL_SPI_TransmitReceive (iface, tx_data,rx_data, len, HAL_MAX_DELAY);
    }
    else{
        commstatus = HAL_ERROR;
    }
    //xensiv_bgt60trxx_platform_spi_cs_set(iface,true);
    return commstatus;
    
}

int32_t xensiv_bgt60trxx_platform_spi_fifo_read(SPI_HandleTypeDef* iface, uint16_t* rx_data, uint32_t len){ 
    HAL_StatusTypeDef commstatus = HAL_OK;
    uint8_t buffer[1536];
    uint8_t keephigh[1536];
    memset(keephigh, 0xFF, 1536);
    // for(size_t i = 0; i < (len/2); ++i){ //len is the number of 12 bit samples.
    commstatus = HAL_SPI_TransmitReceive (iface,keephigh,buffer,1536, HAL_MAX_DELAY/100);
    //     rx_data[i*2] = (buffer[0] << 4) | (buffer[1] >> 4);
    //     rx_data[(i*2)+1] = ((buffer[1] & 0x0f) << 8) | (buffer[2]);
    // }
    for (size_t i = 0, j = 0; i < 1024; i += 2, j += 3) {
        uint8_t b0 = buffer[j+0];
        uint8_t b1 = buffer[j+1];
        uint8_t b2 = buffer[j+2];
        rx_data[i+0] = ((uint16_t)b0 << 4) | (b1 >> 4);
        rx_data[i+1] = ((uint16_t)(b1 & 0x0F) << 8) | b2;
        if(rx_data[i] < 1000){
            rx_data[i] += 4095;
        }
        if(rx_data[i+1]<1000){
            rx_data[i+1] += 4095;
        }
    }
    return commstatus;
}

// int32_t xensiv_bgt60trxx_platform_spi_fifo_read(const SPI_HandleTypeDef* iface,
//                                                 uint16_t *rx_data,
//                                                 uint32_t len)
// {
    
//     if (!iface || !rx_data || !len) return 1;
//     //assert_spi_dsize_12(iface); // FIFO phase uses 12-bit frames

//     // Prepare a small dummy TX buffer filled with 0x0FFF (MOSI held high)
//     enum { CHUNK = 1024 };
//     uint16_t dummy[CHUNK];
//     for (unsigned i = 0; i < CHUNK; ++i) dummy[i] = 0x0FFF;

//     while (len) {
//         uint32_t n = (len > CHUNK) ? CHUNK : len;

//         // In HAL, when DataSize > 8, the size parameter counts **half-words**
//         HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
//             iface,
//             (uint8_t*)dummy,                 // TX buffer (n half-words, all 0x0FFF)
//             (uint8_t*)rx_data,               // RX buffer (n half-words)
//             n,                               // number of 12-bit frames (half-words)
//             HAL_MAX_DELAY);

//         if (st != HAL_OK) return 1;

//         rx_data   += n;
//         len  -= n;
//     }
//     return 0;
// }

void xensiv_bgt60trxx_platform_delay(uint32_t ms){
    HAL_Delay(ms);
}


uint32_t xensiv_bgt60trxx_platform_word_reverse(uint32_t x){
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