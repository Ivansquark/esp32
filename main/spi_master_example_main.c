
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"

//#define CONFIG_IDF_TARGET_ESP32
#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
//SPI_DMA_DISABLED = 0;

#define PIN_NUM_MISO 19 //25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18 //19
#define PIN_NUM_CS   22
#endif

#define ECHO_TEST_TXD 1
#define ECHO_TEST_RXD 3
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      0
#define ECHO_UART_BAUD_RATE     115200
#define ECHO_TASK_STACK_SIZE    2048
#define BUF_SIZE 1024

#define BLINK_GPIO 2

static void echo_task(void *arg);

uint32_t lcd_get_id(spi_device_handle_t spi)
{
    //get_id cmd
    //lcd_cmd(spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    return *(uint32_t*)t.rx_data;
}

void delay(volatile uint32_t val) {
    while(val--);
}
volatile uint8_t arr[8] ={0x00, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x55};
volatile uint8_t receivedArr[4] = {0};
void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=1*320*2+8
    };
    spi_device_interface_config_t devcfg={
        //.clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .duty_cycle_pos = 0,
        .clock_speed_hz=1*1000*1000,           //Clock out at 10 MHz. 2MHz
        .input_delay_ns = 0,
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .flags = 0,
        .queue_size=8,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,  //Specify pre-transfer callback to handle D/C line
        .post_cb = NULL
    };

    ret=spi_bus_initialize(VSPI_HOST, &buscfg, DMA_CHAN);
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);

    uint8_t* buffer = heap_caps_malloc(32, MALLOC_CAP_DMA); // for DMA transaction

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=4*8;
    t.rxlength = 4*8;
    t.tx_buffer = arr;
    t.rx_buffer = receivedArr;

    //xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    uint8_t setFlag = 0;
    while(true) {
        if(setFlag) {
            gpio_set_level(BLINK_GPIO, 1); setFlag = 0;
        } else {
            gpio_set_level(BLINK_GPIO, 0); setFlag = 1;
        }
        spi_device_polling_transmit(spi, &t);
        arr[0] = 1; arr[1] = 2; arr[2] = 3; arr[3] = *((uint8_t*)t.rx_buffer+3);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void echo_task(void *arg)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t setFlag = 0;
    while (1) {
        if(setFlag) {
            gpio_set_level(BLINK_GPIO, 1); setFlag = 0;
        } else {
            gpio_set_level(BLINK_GPIO, 0); setFlag = 1;
        }

        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
    }
}
