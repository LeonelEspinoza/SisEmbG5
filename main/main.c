#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define BUF_SIZE (128) // buffer size
#define TXD_PIN 1  // UART TX pin
#define RXD_PIN 3  // UART RX pin
#define UART_NUM UART_NUM_0   // UART port number
#define BAUD_RATE 115200   // Baud rate

#define REDIRECT_LOGS 1 // if redirect ESP log to another UART

// Function for sending things to UART1
static int uart1_printf(const char *str, va_list ap) {
    char *buf;
    vasprintf(&buf, str, ap);
    uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    free(buf);
    return 0;
}

// Setup of UART connections 0 and 1, and try to redirect logs to UART1 if asked
static void uart_setup() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Redirect ESP log to UART1
    if (REDIRECT_LOGS) {
        esp_log_set_vprintf(uart1_printf);
    }
}

// Write message through UART_num with an \0 at the end
// int serial_write(const char *msg, int len){

//     char *send_with_end = (char *)malloc(sizeof(char) * (len + 1));
//     memcpy(send_with_end, msg, len);
//     send_with_end[len] = '\0';

//     int result = uart_write_bytes(UART_NUM, send_with_end, len+1);

//     free(send_with_end);

//     vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
//     return result;
// }

// Read UART_num for input with timeout of 1 sec
int serial_read(char *buffer, int size){
    int len = uart_read_bytes(UART_NUM, (uint8_t*)buffer, size, pdMS_TO_TICKS(1000));
    return len;
}

// Main
void app_main()
{
    uart_setup(); // Uart setup

    srand(time(0));  // Initialize random seed

    // Waiting for an BEGIN to initialize data sending
    char dataResponse1[6];
    printf("Beginning initialization... \n");
    while (1)
    {
        int rLen = serial_read(dataResponse1, 6);
        if (rLen > 0)
        {
            if (strcmp(dataResponse1, "BEGIN") == 0)
            {
                uart_write_bytes(UART_NUM,"OK\0",3);
                break;
            }
        }
    }

    // Data sending, can be stopped receiving an END between sendings
    char dataResponse2[4];
    printf("Beginning sending... \n");
    while (1)
    {
        float data[3];
        for (int i = 0; i < 3; i++)
        {
            data[i] = (float)rand() / RAND_MAX;  // Generate random float number between 0 and 1
        }
        const char* dataToSend = (const char*)data;


        int len = sizeof(float)*3;

        uart_write_bytes(UART_NUM, dataToSend, len);

        int rLen = serial_read(dataResponse2, 4);
        if (rLen > 0)
        {
            if (strcmp(dataResponse2, "END") == 0)
            {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }

    // Data sending stopped, sending just OK with \0 at the end
    while (1)
    {
        uart_write_bytes(UART_NUM,"OK\0",3);
    }
}
