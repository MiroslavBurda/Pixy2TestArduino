#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/uart.h>

#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"
#include "pixy2/i2c.hpp"

using namespace pixy2;

void setup() {
  
   uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));

    spi_bus_config_t busCfg;
    busCfg.mosi_io_num = GPIO_NUM_25; //13 
    busCfg.miso_io_num = GPIO_NUM_26; //12
    busCfg.sclk_io_num = GPIO_NUM_27; //14
    busCfg.quadwp_io_num = -1;
    busCfg.quadhd_io_num = -1;
    busCfg.flags = SPICOMMON_BUSFLAG_MASTER | (1<<2);  // (1<<2) je SPICOMMON_BUSFLAG_GPIO_PINS

    spi_host_device_t SPI2_HOST = (spi_host_device_t)1;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, 0)); 

    auto linkRes = LinkSpi::addSpiDevice(SPI2_HOST);
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());
    GetBlocksContext blocksCtx;
    while(true) {
        auto err = pixy.getColorBlocks(1, 4, blocksCtx); // cervena
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error1: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
 
        const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
        uart_write_bytes(UART_NUM_0, (const char*)lorrisHeader, sizeof(lorrisHeader));
        if(blocksCtx.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, (const char*)blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
        }
    
        // int poziceX = blocksCtx.blocks[0]->x;
        // int poziceY = blocksCtx.blocks[0]->y;
        // int poziceX2 = blocksCtx.blocks[1]->x;
        // int poziceY2 = blocksCtx.blocks[1]->y;

        //    printf("i: %i, %i, %i, %i\n", poziceX, poziceY, poziceX2, poziceY2);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    } 
}

void loop(){

}
